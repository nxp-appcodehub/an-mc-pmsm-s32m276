/*******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.3
*   RTD Version  : 7.0.0
*   Brief description  :
*   - application entry function (main)
*   - interrupt service routine
*   - state machine functions
*
*   Copyright 2024, 2026 NXP
*
*   NXP Proprietary. This software is owned or controlled by NXP and may only be
*   used strictly in accordance with the applicable license terms. By expressly
*   accepting such terms or by downloading, installing, activating and/or otherwise
*   using the software, you are agreeing that you have read, and that you agree to
*   comply with and are bound by, such license terms. If you do not agree to be
*   bound by the applicable license terms, then you may not retain, install,
*   activate or otherwise use the software.
*******************************************************************************/
/******************************************************************************
 * main.c
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file, updated AE fault handling and AE Init with regards to fault handling
 * rev 1.2 updated AEC_DPGAConfig() function call - now also the voltage detector limits are configured in graphical config tool, not manually via API call
 * rev 1.3 Added support for RTD 7.0.0
 *
 ********************************************************************************/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/

#ifdef __cplusplus
extern "C" {
#endif


/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "Siul2_Port_Ip.h"
#include "Siul2_Dio_Ip.h"
#include "Siul2_Icu_Ip.h"
#include "Clock_Ip.h"
#include "Lpuart_Uart_Ip.h"
#include "freemaster.h"
#include "freemaster_s32_lpuart.h"
#include "IntCtrl_Ip.h"
#include "Emios_Mcl_Ip.h"
#include "Emios_Pwm_Ip.h"
#include "Trgmux_Ip.h"
#include "Adc_Sar_Ip.h"
#include "Bctu_Ip.h"
#include <actuate_s32m.h>
#include <meas_s32m.h>
#include "motor_structure.h"
#include "pospe_sensor.h"
#include "state_machine.h"
#include "config\PMSM_appconfig.h"
#include "Peripherals\peripherals_config.h"
#include "amclib.h"
#include "Lcu_Ip.h"
#include "Lpspi_Ip.h"
#include "Aec_Ip.h"
#include "CDD_GDU.h"
#include "Dpga_Ip.h"
#include "S32M27x_HVI_AE.h"
#include "S32M27x_PMC_AE.h"

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
/* Filters */
#define FILTER_LAMBDA_CALC(x)   MLIB_Div(1.0F, (tFloat)(x))
#define FILTER_LAMBDA_FW_SPEED  (1.0F)
#define FILTER_SAMPLE_NO_MEAS   (8)
#define FILTER_SAMPLE_NO_DCB    (8)


/* DCB ripple eliminator */
#define ELIM_DCD_RIP_MOD_INDEX  (0.866025403784439F)
#define U_DCB_NORMAL  (12.0F)

/* Application */
#define SPEED_RPM_INC           200
#define SPEED_RPM_DEC           200
#define	SPEED_UP_CNT            1000
#define	SPEED_DOWN_CNT          1000
#define BTN_FACTOR				0.98F
#define SPEED_LIM_RAD_BTN		WEL_MAX*BTN_FACTOR
#define	APP_OFF_CNT             5000
#define MAX_TH_DIF_OPEN_ESTIM	(0.01F)
#define SPEED_RAD_INC           (float)((SPEED_RPM_INC/N_MAX)*WEL_MAX)
#define SPEED_RAD_DEC           (float)((SPEED_RPM_DEC/N_MAX)*WEL_MAX)
#define LED_FLASH_CNT (1250)
#define FOC_TIMEOUT 10000       /* Timeout for reaching demanded speed [sapling periods]  */
#define ALIGN_D_FACTOR 0.9      /* Defines how many % of alignment time is reserved for D axis */
#define N_HOLD  0.015F       /* PWM period * N_HOLD =50us *0.015 = 0.75us */
#define ZERO_PULSE 0.04F      /* PWM period * ZERO_PULSE = 50us *0.04 = 2us */
#define SAMPLING_PULSE 0.03F  /* PWM period * SAMPLING_PULSE = 50us *0.03 = 1.5us */

/*****************************************************************************
* Define Motor with or without Encoder sensor
*
* ENCODER  0    PM motor is NOT equipped with Encoder sensor, motor position/speed is estimated only by eBEMF observer
* ENCODER  1    PM motor is equipped with Encoder sensor, motor position/speed can either be estimated by eBEMF observer
*               or obtained by Encoder sensor. To switch between Sensorless and Encoder mode use switchSensor variable.
*
* Note          Default PM motor of the MTRDEVKSPNK344 (SUNRISE 42BLY3A78-24110) is NOT equipped with Encoder sensor,
*               thus ENCODER 0 is used as default for MTRDEVKSPNK344
******************************************************************************/
#define ENCODER   0


/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/

/* Motor control measurements */
Bctu_Ip_FifoResultType measuredValues[16];
uint8_t mCount = 0;

volatile int exit_code = 0;

tFloat speed_fb = 0.0F;

/* Application and FOC control */
pmsmDrive_t       drvFOC;         /* Field Oriented Control Variables */
driveStates_t     cntrState;      /* Responsible for stateMachine state propagation */
appFaultStatus_t  tempfaults;     /* Temporary faults to be indicated inhere */
appFaultStatus_t  permFaults;     /* Permanent faults to be indicated inhere */
fm_scale_t        fmScale;        /* Scales for freeMASTER interface */
measModule_t      meas;           /* Variables measured by ADC modules */
tPos_mode         pos_mode;       /* Variable defining position mode */
tBool             statePWM;       /* Status of the PWM update */
tBool             fieldWeakOnOff; /* Enable/Disable Field Weakening */
switchSensor_t    switchSensor;   /* Position sensor selector */
encoderPospe_t    encoderPospe;   /* Encoder position and speed*/



/* Open Loop and Closed loop speed ramp variants */
volatile tFloat OL_SpeedRampInc = 0.0F, CL_SpeedRampInc = 0.0F, CL_SpeedRampDec = 0.0F;
volatile tFloat FW_PropGainControl = 0.0F, FW_IntegGainControl = 0.0F;
volatile tFloat UDQVectorSum = 0.0F;

/*Double switching variables*/
float Nhold;          /*The minimal "sample&hold" time of the ADC*/
float ZeroPulse;     /* The minimal pulse inserted to the middle of the PWM period of double-switching signal.*/
float SamplingPulse; /* Represents at least the minimal conversion time of ADC */

/* Measurement/Actuate Variables */
tADCresults  ADCResults;
tFloat       duty_cycle;

AEFaultStatus_t	AEFaultStatus;

uint32 SPI_rx_data =0;
tU16  	AEHealthStatus = 0;
tBool	ReadAEStatus = 0;

tU32	LEDCnt = 0;


/*==================================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
==================================================================================================*/


// static void BoardButtons(void);
tBool AutomaticMode(void);
void  BoardButtons(void);
tBool CalcOpenLoop(openLoopPospe_t *openLoop, tFloat speedReqRamp);
tBool FocFastLoop(void);
tBool FaultDetection(void);
tBool FocSlowLoop(void);
void  MCAT_Init(void);
void CurrentTrigOutput(void);

/*==================================================================================================
*                                    FUNCTIONS DEFINITION
==================================================================================================*/





/* reserved for future use */
void Gdu_Notif (uint32 InterruptFlags)
{
	(void)(InterruptFlags);
}

/* reserved for future use */
void Dpga_Callback (Dpga_Ip_EventType Event)
{
	(void)(Event);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : main
 * Description   : It initializes peripherals, Application Extension and Feemaster communication.
 *                 On the background it maintains freemaster communication.
 *
 *END**************************************************************************/

int main(void)
{
    StatusType status;
    Aec_Ip_StatusType AEStatus = 0;
	Siul2_Port_Ip_PinSettingsConfig TempPinSettings[1];

    /***********************************************************************************************
    * Clock Driver
    ***********************************************************************************************/
    Clock_Ip_Init(&Clock_Ip_aClockConfig[0]);
    /* Initialize osif. */
    OsIf_Init(NULL_PTR);

    /***********************************************************************************************
    * Interrupt from analog extension
    ***********************************************************************************************/
    /* Initialize ICU channel for AE interrupt. */
    Siul2_Icu_Ip_Init(0U, &Siul2_Icu_Ip_0_Config_PB);
    /* Enable ICU edge detect for  interrupt. */
    Siul2_Icu_Ip_EnableInterrupt(0U, 11U);
    Siul2_Icu_Ip_EnableNotification(0U, 11U);


    /***********************************************************************************************
    * Port Driver
    ***********************************************************************************************/
    Siul2_Port_Ip_Init(NUM_OF_CONFIGURED_PINS_PortContainer_0_BOARD_InitPeripherals, g_pin_mux_InitConfigArr_PortContainer_0_BOARD_InitPeripherals);

    //ADC interleave for PTA6 / ADC1_S18 not set through PINS tool, thus current configuration is read and interleave configured manually
    Siul2_Port_Ip_GetPinConfiguration(PORTA_L_HALF, &TempPinSettings[0], 6);
    TempPinSettings[0].adcInterleaves[0] = DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC1_S18_MASK;
    TempPinSettings[0].adcInterleaves[1] = DCM_GPR_DCMRWF4_MUX_MODE_EN_ADC1_S18_MASK;
    Siul2_Port_Ip_Init(1, TempPinSettings);
    /***********************************************************************************************
    * Triggermux Driver
    ***********************************************************************************************/
    Trgmux_Ip_Init(&Trgmux_Ip_Sa_xTrgmuxInitPB);

#if ENCODER
    /* Siul2_Port_Ip_SetInputBuffer is needed due to missing feature in Configuration Tools. */
    /* It is a known issue and will be fix in the later RTD releases */

    /* Assign eMIOS0_CH6 input to TRGMUX_OUT_37 */
    Siul2_Port_Ip_SetInputBuffer(NULL_PTR, NULL_PTR, 1, 54, 3U);
    //this effectively functions as:
    //IP_SIUL2->IMCR[54] = SIUL2_IMCR_SSS(3U);
    /* Assign eMIOS0_CH7 input to TRGMUX_OUT_38 */
    Siul2_Port_Ip_SetInputBuffer(NULL_PTR, NULL_PTR, 1, 55, 4U);
    //this effectively functions as:
    //IP_SIUL2->IMCR[55] = SIUL2_IMCR_SSS(4U);

#endif
    /***********************************************************************************************
    * UART Driver
    ***********************************************************************************************/
    /* Initializes an UART driver*/
    Lpuart_Uart_Ip_Init(0U, &Lpuart_Uart_Ip_xHwConfigPB_0);

    /***********************************************************************************************
    * ADC Driver
    ***********************************************************************************************/
    do {
        status = (StatusType)Adc_Sar_Ip_Init(1U, &AdcHwUnit_1);
    } while (status != E_OK);

    do {
        status = (StatusType)Adc_Sar_Ip_DoCalibration(1U);
    } while (status != E_OK);


    /***********************************************************************************************
    * LCU Driver
    ***********************************************************************************************/
    Lcu_Ip_Init(&Lcu_Ip_Sa_xLcuInitPB);
    CurrentTrigOutput();
#if ENCODER
    Lcu_Ip_SyncOutputValueType EncLcuEnable[6U];
    EncLcuEnable[0].LogicOutputId = LCU_LOGIC_OUTPUT_8;
    EncLcuEnable[0].Value = 1U;
    EncLcuEnable[1].LogicOutputId = LCU_LOGIC_OUTPUT_9;
    EncLcuEnable[1].Value = 1U;
    EncLcuEnable[2].LogicOutputId = LCU_LOGIC_OUTPUT_10;
    EncLcuEnable[2].Value = 1U;
    EncLcuEnable[3].LogicOutputId = LCU_LOGIC_OUTPUT_11;
    EncLcuEnable[3].Value = 1U;
    Lcu_Ip_SetSyncOutputEnable(EncLcuEnable, 4U);
#endif
    /***********************************************************************************************
    * SPI Driver
    ***********************************************************************************************/
    Lpspi_Ip_Init(&Lpspi_Ip_PhyUnitConfig_SpiPhyUnit_0_Instance_1);

    /***********************************************************************************************
    * Application Extension Driver
    ***********************************************************************************************/
    AECConfig();

   	AEStatus = 1;
   	//Check status
   	AEStatus = Aec_Ip_SpiRead((uint32_t)&IP_AEC_AE->VERID, 16, &SPI_rx_data);

   	while (!(AEStatus == 0))//Check status loop till status OK
   	{
   			//global health status error (clock miss, wrong answer, faults)
   		AEStatus = Aec_Ip_SpiRead((uint32_t) &IP_AEC_AE->VERID, 16, &SPI_rx_data);
   			// Update global variable again
   	}

   	// configure AE modules
	AEC_PMCConfig(1,0);
	AEC_ResetConfig();
	AEC_HVMConfig();
	Aec_Ip_SpiRead((uint32_t) &IP_PMC_AE->MONITOR, 32, &AEFaultStatus.PMCMonitorRegister);
	Aec_Ip_SpiWrite((uint32_t) &IP_PMC_AE->MONITOR, 32, AEFaultStatus.PMCMonitorRegister);//reset flags which may have appeared during reset in PMC
	AEC_GDUConfig();
	Aec_Ip_SpiRead((uint32_t) &IP_GDU_AE->INTF, 8, &AEFaultStatus.GDUInterruptFlag);
	Aec_Ip_SpiWrite((uint32_t) &IP_GDU_AE->INTF, 8, AEFaultStatus.GDUInterruptFlag);//reset flags which may have appeared during reset
	AEC_DPGAConfig();

#if ENCODER
   	AEC_VDDE_Enable(1);
#endif

   	AEStatus = Aec_Ip_SpiRead((uint32_t)&IP_AEC_AE->VERID, 16, &SPI_rx_data);
	AEHealthStatus = Aec_Ip_DecodeGlobalHealthStatus();

	if ((AEStatus == 0) && ((AEHealthStatus & 0xC0) == 0))
	{
		//Do nothing since communication is ok and no alarm event or fault detected by global health status bit 6 and 7
	}
	else
	{
		AEFaultStatus.AEIntFlag = 1;
		ReadAEStatus = 1; //In case AE communication is not ok or fault is detected by global health status, raise a flag as if AE fault was detected by PTD3 input
	}

    /***********************************************************************************************
    *Configure and enable interrupts
    ***********************************************************************************************/
    IntCtrl_Ip_Init(&IntCtrlConfig_0); // Initialize intrerrupts after AE init to avoid false alarms

    /***********************************************************************************************
    * FreeMASTER
    ***********************************************************************************************/
    /* Set FreeMASTER serial base address. */
	FMSTR_SerialSetBaseAddress((FMSTR_ADDR)IP_LPUART_0_BASE);
    /* Freemaster initalization      */
    FMSTR_Init();

    /***********************************************************************************************
    * Application
    ***********************************************************************************************/
    /* MCAT variables initialization */
    MCAT_Init();
    /* Clear measured variables      */
    MEAS_Clear(&meas);

    /* Application starts from init state */
    cntrState.state   = init;
    cntrState.event   = e_init;

    /* Default setting of a FOC Control Mode */
    cntrState.usrControl.FOCcontrolMode   = speedControl;

    /***********************************************************************************************
    * eMios Driver
    ***********************************************************************************************/
    /* Enable eMIOS clock at last to ensure the correct trigger order */
    Emios_Mcl_Ip_Init(0U, &Emios_Mcl_Ip_Sa_0_Config);

    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch1);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch2);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch3);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch4);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch5);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch17);

    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch9);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch10);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch11);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch12);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch13);
    Emios_Pwm_Ip_InitChannel(0U, &Emios_Pwm_Ip_Sa_I0_Ch14);

#if ENCODER
    Emios_Icu_Ip_Init(0U, &eMios_Icu_Ip_0_Config_PB);

    Emios_Icu_Ip_SetInitialCounterValue(0U, 6U, (uint32_t)0x1U);
    Emios_Icu_Ip_SetInitialCounterValue(0U, 7U, (uint32_t)0x1U);

    Emios_Icu_Ip_SetMaxCounterValue(0U, 6U, (uint32_t)(4*ENC_PULSES));
    Emios_Icu_Ip_SetMaxCounterValue(0U, 7U, (uint32_t)(4*ENC_PULSES));

    Emios_Icu_Ip_EnableEdgeCount(0u, 6U);
    Emios_Icu_Ip_EnableEdgeCount(0u, 7U);
#endif
    /***********************************************************************************************
    * BCTU Driver
    ***********************************************************************************************/
    Bctu_Ip_Init(0U, &BctuHwUnit_0);

    /*Enable eMIOS clock at last to ensure the correct trigger order*/
    Emios_Mcl_Ip_ConfigureGlobalTimebase(0U, TRUE);

    ACTUATE_EnableOutput();

    while (1)
    {
     	FMSTR_Poll();


		if (ReadAEStatus) //AE status is read when this flag is set in PORTD_Notif notification function
		{

			//fault status already read in PTD3 ISR
			Aec_Ip_SpiRead((uint32_t) &IP_AEC_AE->EVENTS_STATUS,16, &AEFaultStatus.AEEventStatus);

			Aec_Ip_SpiWrite((uint32_t) &IP_AEC_AE->FAULTS_STATUS,16, AEFaultStatus.AEFaultStatus);//Clear W1C bits
			Aec_Ip_SpiWrite((uint32_t) &IP_AEC_AE->EVENTS_STATUS,16, AEFaultStatus.AEEventStatus);//Clear W1C bits

			Aec_Ip_SpiRead((uint32_t) &IP_PMC_AE->MONITOR, 32, &AEFaultStatus.PMCMonitorRegister);
			Aec_Ip_SpiWrite((uint32_t) &IP_PMC_AE->MONITOR, 32, AEFaultStatus.PMCMonitorRegister);//Clear W1C bits

			Aec_Ip_SpiRead((uint32_t) &IP_GDU_AE->INTF,8, &AEFaultStatus.GDUInterruptFlag);
			Aec_Ip_SpiWrite((uint32_t) &IP_GDU_AE->INTF,8, AEFaultStatus.GDUInterruptFlag);//Clear W1C bits

			Aec_Ip_SpiRead((uint32_t) &IP_DPGA_AE->INTF,8, &AEFaultStatus.DPGAInterruptFlag);
			Aec_Ip_SpiWrite((uint32_t) &IP_DPGA_AE->INTF,8, AEFaultStatus.DPGAInterruptFlag);//Clear W1C bits

			Aec_Ip_SpiRead((uint32_t) &IP_HVI_AE->INTF,32, &AEFaultStatus.HVMInterruptFlags);
			Aec_Ip_SpiWrite((uint32_t) &IP_HVI_AE->INTF,32, AEFaultStatus.HVMInterruptFlags);//Clear W1C bits

			ReadAEStatus = 0;
		}

        switch(cntrState.state)
		{
			case ready:
				Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 1U);
				LEDCnt = 0;
				break;

			case run:

				if (LEDCnt == 1000000)
				{
					Siul2_Dio_Ip_TogglePins(LED_PORT, 1 << LED_PIN);
					LEDCnt = 0;
				}

				break;

			case fault:

				if (LEDCnt == 50000)
				{
					Siul2_Dio_Ip_TogglePins(LED_PORT, 1 << LED_PIN);
					LEDCnt = 0;
				}
				if (LEDCnt > 50000)
				{
					LEDCnt = 0;
				}

				break;

			default:
				Siul2_Dio_Ip_WritePin(LED_PORT, LED_PIN, 0U);
				LEDCnt = 0;
				break;

		}

		LEDCnt++;

    }

    return (0U);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : AE_INT_Handler
 * Description   : INT pin IRQ handler.
 *
 *END**************************************************************************/
void AE_INT_Handler(void)
{
	//Set flag to read AE status - this is done in main() in the background loop
	/*This SW example stops the motor in case of any fault/event reported by AE through NMI/PTD3 line
	 * in case any application specific AE fault/event arbitration is desired, it needs to be implemented in addition
	 */

	Aec_Ip_SpiRead((uint32_t) &IP_AEC_AE->FAULTS_STATUS,16, &AEFaultStatus.AEFaultStatus);//read fault status
	//if there are any pending faults or events this is informed by Global Heald Status, in that case the rest of the
	//relevant status words is read in the background loop in main()

	AEHealthStatus = Aec_Ip_DecodeGlobalHealthStatus();
	AEHealthStatus &= (AEC_STATUS_EVENT_NOTIFY | AEC_STATUS_FAULT_NOTIFY );

	if (AEHealthStatus !=0)
	{
		//If there is a pending Fault or event in the AE, the flag is set and is further processed in FaultDetection()
		AEFaultStatus.AEIntFlag = 1;
		ReadAEStatus = 1;
	}
}



/*FUNCTION**********************************************************************
 *
 * Function Name : Bctu_FIFO1_WatermarkNotification
 * Description   : BCTU FIFO water mark notification function.
 *
 *END**************************************************************************/
void Bctu_FIFO1_WatermarkNotification(void)
{
    static tBool getFcnStatus;
    /* Board buttons to control the application from board */
    cntrState.usrControl.btSpeedUp = Siul2_Dio_Ip_ReadPin(BTN_INC_SW0_PORT, BTN_INC_SW0_PIN);
    cntrState.usrControl.btSpeedDown = Siul2_Dio_Ip_ReadPin(BTN_DEC_SW1_PORT, BTN_DEC_SW1_PIN);
    /* Board buttons control logic */
    BoardButtons();

    /* User accessible switch for stopping the application. */
    if (cntrState.usrControl.switchAppOnOff ^ cntrState.usrControl.switchAppOnOffState)
    {
        cntrState.usrControl.switchAppOnOffState  = cntrState.usrControl.switchAppOnOff;
        cntrState.event = (cntrState.usrControl.switchAppOnOff) ? e_app_on: e_app_off;
    }

    Siul2_Dio_Ip_SetPins(TST_GPIO_D16_PORT, (1 << TST_GPIO_D16_PIN));

    mCount = 0;
    while (Bctu_Ip_GetFifoCount(0U, 0U))
    {
        Bctu_Ip_GetFifoResult(0U, 0U, &measuredValues[mCount++]);

    }

    MEAS_SaveAdcRawResult();

    getFcnStatus    =    true;

    /* DCB voltage, DCB current and phase currents measurement */
    getFcnStatus  = MEAS_GetUdcVoltage(&meas, &drvFOC.uDcbFilter);
    getFcnStatus &= MEAS_Get3PhCurrent(&meas, &drvFOC.iAbcFbck, drvFOC.svmSector);
    drvFOC.fltUdcb = meas.measured.fltUdcb.filt;
    #if ENCODER
    /* Get rotor position and speed from Encoder sensor */
    POSPE_GetPospeElEnc(&encoderPospe);
    #endif
    /* Fault detection routine, must be executed prior application state machine */
    getFcnStatus &= FaultDetection();

    if (getFcnStatus)    cntrState.event = e_fault;

    /* Execute State table with newly measured data */
    StateTable[cntrState.event][cntrState.state]();
    Siul2_Dio_Ip_ClearPins(TST_GPIO_D16_PORT, (1 << TST_GPIO_D16_PIN));
    FMSTR_Recorder(0);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CurrentTrigOutput
 * Description   : BCTU trigger enable.
 *
 *END**************************************************************************/
void CurrentTrigOutput(void)
{
    Lcu_Ip_SyncOutputValueType lcuEnable[2U];
    lcuEnable[0].LogicOutputId = LCU_LOGIC_OUTPUT_6;
    lcuEnable[0].Value = 1U;
    lcuEnable[1].LogicOutputId = LCU_LOGIC_OUTPUT_7;
    lcuEnable[1].Value = 1U;
    Lcu_Ip_SetSyncOutputEnable(lcuEnable, 2U);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MCAT_Init
 * Description   : Initializes application parameters generated from MCAT tool
 *                 (src/config/PMSM_appconfig.h).
 *
 *END**************************************************************************/
void MCAT_Init(void)
{
    /*------------------------------------
     * Freemaster parameters
     * ----------------------------------*/
    fmScale.speed_n_m           = FM_SPEED_RPM_MEC_SCALE;
    fmScale.position            = FM_POSITION_DEG_SCALE;

    /*------------------------------------
     * Speed ramp parameters
     * ----------------------------------*/
    OL_SpeedRampInc             = OL_START_RAMP_INC;
    CL_SpeedRampInc             = SPEED_RAMP_UP;
    CL_SpeedRampDec             = SPEED_RAMP_DOWN;

    /*------------------------------------
     * Alignment parameters
     * ----------------------------------*/
    drvFOC.alignCntrInitValue   = ALIGN_DURATION;
    drvFOC.alignVoltage         = ALIGN_VOLTAGE;

    /*------------------------------------
     * DC Bus ripple elimination parameters
     * ----------------------------------*/
    drvFOC.elimDcbRip.fltModIndex            = ELIM_DCD_RIP_MOD_INDEX;

    /*------------------------------------
     * Current Loop parameters
     * ----------------------------------*/
    /* D-axis PI controller */
    drvFOC.CurrentLoop.pPIrAWD.fltCC1sc      = D_CC1SC;
    drvFOC.CurrentLoop.pPIrAWD.fltCC2sc      = D_CC2SC;
    drvFOC.CurrentLoop.pPIrAWD.fltLowerLimit = -CLOOP_LIMIT;
    drvFOC.CurrentLoop.pPIrAWD.fltUpperLimit = CLOOP_LIMIT;
    /* Q-axis PI controller */
    drvFOC.CurrentLoop.pPIrAWQ.fltCC1sc      = Q_CC1SC;
    drvFOC.CurrentLoop.pPIrAWQ.fltCC2sc      = Q_CC2SC;
    drvFOC.CurrentLoop.pPIrAWQ.fltLowerLimit = -CLOOP_LIMIT;
    drvFOC.CurrentLoop.pPIrAWQ.fltUpperLimit = CLOOP_LIMIT;
    /* Requirement and feedback pointers */
    drvFOC.CurrentLoop.pIDQReq               = &drvFOC.iDQReqInLoop;
    drvFOC.CurrentLoop.pIDQFbck              = &drvFOC.iDQFbck;

    /*------------------------------------
     * Speed Loop parameters
     * ----------------------------------*/
    /* Speed ramp */
    drvFOC.FwSpeedLoop.pRamp.fltRampUp        = OL_SpeedRampInc;
    drvFOC.FwSpeedLoop.pRamp.fltRampDown      = OL_SpeedRampInc;
    /*  Q-axis Controller parameters */
    drvFOC.FwSpeedLoop.pPIpAWQ.fltPropGain    = SPEED_PI_PROP_GAIN;
    drvFOC.FwSpeedLoop.pPIpAWQ.fltIntegGain   = SPEED_PI_INTEG_GAIN;
    drvFOC.FwSpeedLoop.pPIpAWQ.fltUpperLimit  = SPEED_LOOP_HIGH_LIMIT;
    drvFOC.FwSpeedLoop.pPIpAWQ.fltLowerLimit  = SPEED_LOOP_LOW_LIMIT;
    drvFOC.FwSpeedLoop.pFilterW.fltLambda     = POSPE_SPEED_FILTER_MA_LAMBDA;
    /* Field weakening FilterMA */
    drvFOC.FwSpeedLoop.pFilterFW.fltLambda    = FILTER_LAMBDA_FW_SPEED;
    /* Field weakening PI controller */
    drvFOC.FwSpeedLoop.pPIpAWFW.fltPropGain   = SPEED_PI_PROP_GAIN;
    drvFOC.FwSpeedLoop.pPIpAWFW.fltIntegGain  = SPEED_PI_INTEG_GAIN;
    FW_PropGainControl                        = SPEED_PI_PROP_GAIN;
    FW_IntegGainControl                       = SPEED_PI_INTEG_GAIN;
    drvFOC.FwSpeedLoop.pPIpAWFW.fltUpperLimit = 0.0F;
    drvFOC.FwSpeedLoop.pPIpAWFW.fltLowerLimit = -FLOAT_PI_DIVBY_2;
    /* Input/output pointers */
    drvFOC.FwSpeedLoop.pIQFbck                = &(drvFOC.iDQFbck.fltArg2);
    drvFOC.FwSpeedLoop.pUQLim                 = &(drvFOC.CurrentLoop.pPIrAWQ.fltUpperLimit);
    drvFOC.FwSpeedLoop.pUQReq                 = &(drvFOC.uDQReq.fltArg2);
    /* Field weakening PI controller */
    drvFOC.FwSpeedLoop.fltUmaxDivImax         = MLIB_Div(U_DCB_MAX, I_MAX);
    /* Field weakening on/off */
    fieldWeakOnOff                            = false;

    /*------------------------------------
     * Back-EMF observer parameters
     * ----------------------------------*/
    /* back-EMF observer parameters - D-axis */
    drvFOC.pospeSensorless.bEMFObs.pParamD.fltCC1sc         = BEMF_DQ_CC1_GAIN;
    drvFOC.pospeSensorless.bEMFObs.pParamD.fltCC2sc         = BEMF_DQ_CC2_GAIN;
    drvFOC.pospeSensorless.bEMFObs.pParamD.fltUpperLimit    = FLOAT_MAX;
    drvFOC.pospeSensorless.bEMFObs.pParamD.fltLowerLimit    = FLOAT_MIN;
    /* back-EMF observer parameters - Q-axis */
    drvFOC.pospeSensorless.bEMFObs.pParamQ.fltCC1sc         = BEMF_DQ_CC1_GAIN;
    drvFOC.pospeSensorless.bEMFObs.pParamQ.fltCC2sc         = BEMF_DQ_CC2_GAIN;
    drvFOC.pospeSensorless.bEMFObs.pParamQ.fltUpperLimit    = FLOAT_MAX;
    drvFOC.pospeSensorless.bEMFObs.pParamQ.fltLowerLimit    = FLOAT_MIN;
    /* back-EMF observer parameters - Scale constants */
    drvFOC.pospeSensorless.bEMFObs.fltIGain                 = I_Gain;
    drvFOC.pospeSensorless.bEMFObs.fltUGain                 = U_Gain;
    drvFOC.pospeSensorless.bEMFObs.fltEGain                 = E_Gain;
    drvFOC.pospeSensorless.bEMFObs.fltWIGain                = WI_Gain;

    /*------------------------------------
     * Angle Tracker Observer parameters
     * ----------------------------------*/
    /* ATO observer - Controller parameters */
    drvFOC.pospeSensorless.TrackObsrv.pParamPI.fltCC1sc      = TO_CC1SC;
    drvFOC.pospeSensorless.TrackObsrv.pParamPI.fltCC2sc      = TO_CC2SC;
    drvFOC.pospeSensorless.TrackObsrv.pParamPI.fltUpperLimit = FLOAT_MAX;
    drvFOC.pospeSensorless.TrackObsrv.pParamPI.fltLowerLimit = FLOAT_MIN;
    /* ATO observer - Integrator parameters */
    drvFOC.pospeSensorless.TrackObsrv.pParamInteg.fltC1      = TO_THETA_GAIN;

    #if ENCODER
    /*------------------------------------
     * Encoder ATO parameters
     * ----------------------------------*/
    /* Encoder ATO observer - Controller parameters */
    encoderPospe.TrackObsrv.pParamPI.fltCC1sc                = POSPE_ENC_TO_CC1;
    encoderPospe.TrackObsrv.pParamPI.fltCC2sc                = POSPE_ENC_TO_CC2;
    encoderPospe.TrackObsrv.pParamPI.fltUpperLimit           = FLOAT_MAX;
    encoderPospe.TrackObsrv.pParamPI.fltLowerLimit           = FLOAT_MIN;
    /* Encoder observer - Integrator parameters */
    encoderPospe.TrackObsrv.pParamInteg.fltC1                = POSPE_ENC_TO_INTEG_GAIN;
    #endif

    /*------------------------------------
     * Sensorless algorithm parameters
     * ----------------------------------*/
    drvFOC.pospeSensorless.wRotElMatch_1  = MLIB_Div(MLIB_Mul(MERG_SPEED_1_TRH, MLIB_Mul(FLOAT_2_PI, MOTOR_PP)), 60);
    drvFOC.pospeSensorless.wRotElMatch_2  = MLIB_Div(MLIB_Mul(MERG_SPEED_2_TRH, MLIB_Mul(FLOAT_2_PI, MOTOR_PP)), 60);
    drvFOC.pospeSensorless.iQUpperLimit   = SPEED_LOOP_HIGH_LIMIT;
    drvFOC.pospeSensorless.iQLowerLimit   = SPEED_LOOP_LOW_LIMIT;

    /*------------------------------------
     * Open loop parameters
     * ----------------------------------*/
    drvFOC.pospeOpenLoop.integ.f32C1      = SCALAR_INTEG_GAIN;
    drvFOC.pospeOpenLoop.integ.u16NShift  = SCALAR_INTEG_SHIFT;
    drvFOC.pospeOpenLoop.iQUpperLimit     = OL_START_I;
    drvFOC.pospeOpenLoop.iQLowerLimit     = MLIB_Neg(drvFOC.pospeOpenLoop.iQUpperLimit);

    /*------------------------------------
     * DC bus filter parameters
     * ----------------------------------*/
    /* DCBus 1st order filter; Fcut = 100Hz, Ts = 100e-6 */
    drvFOC.uDcbFilter.fltLambda           = FILTER_LAMBDA_CALC(FILTER_SAMPLE_NO_DCB);

    /*------------------------------------
     * Measurement parameters
     * ----------------------------------*/
    /* Number of samples = 2^u16CalibSamples */
    meas.param.u16CalibSamples              = FILTER_SAMPLE_NO_MEAS;
    /* Measurement filter parameters*/
    meas.offset.fltPhA.filtParam.fltLambda  = FILTER_LAMBDA_CALC(meas.param.u16CalibSamples);
    meas.offset.fltPhB.filtParam.fltLambda  = FILTER_LAMBDA_CALC(meas.param.u16CalibSamples);
    meas.offset.fltPhC.filtParam.fltLambda  = FILTER_LAMBDA_CALC(meas.param.u16CalibSamples);
    meas.offset.fltIdcb.filtParam.fltLambda  = FILTER_LAMBDA_CALC(meas.param.u16CalibSamples);
    /*------------------------------------
     * Double switching parameters
     * ----------------------------------*/
    /*The minimal "sample&hold" time of the ADC*/
    Nhold = N_HOLD;
    /* The minimal pulse inserted to the middle of the PWM period of double-switching signal.*/
    ZeroPulse = ZERO_PULSE;
    /* Represents at least the minimal conversion time of ADC */
    SamplingPulse = SAMPLING_PULSE;
    /*------------------------------------
     * LED flashing parameters
     * ----------------------------------*/
    cntrState.usrControl.ledFlashing         = LED_FLASH_CNT;

    /* Default state set according to the selected sensor */
    if (switchSensor == encoder)
    {
        cntrState.usrControl.controlMode    = automatic;
        pos_mode                            = encoder1;
    }
    else
    {
        cntrState.usrControl.controlMode    = automatic;
        pos_mode                            = force;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : StateFault
 * Description   : Function of the application state machine.
 *
 *END**************************************************************************/
void StateFault(void)
{

	//tU32 temp_fault_event;

	/*-----------------------------------------------------
    Application State Machine - state identification
    ----------------------------------------------------- */
    /* Entering state fault */
    cntrState.state   = fault;
    cntrState.event   = e_fault;

    /* Turn off PWM */
    statePWM = ACTUATE_DisableOutput();

	// Indicate State Machine fault state invoked by irrelevant event call
	if((permFaults.mcu.R == 0)&&(permFaults.motor.R == 0)&&(permFaults.stateMachine.R == 0)&&(permFaults.AEModule == 0))
	{
		permFaults.stateMachine.B.FOCError 		= 1;
	}

    /* Disable user application switch */
    cntrState.usrControl.switchAppOnOff      = false;
    cntrState.usrControl.switchAppOnOffState = false;

    if (cntrState.usrControl.switchFaultClear)
    {

        permFaults.mcu.R              = 0;        /* Clear mcu faults */
        permFaults.motor.R            = 0;        /* Clear motor faults */
        permFaults.stateMachine.R     = 0;        /* Clear state machine faults */
        permFaults.AEModule 		  = 0;


        AEFaultStatus.AEEventStatus		 = 0;
		AEFaultStatus.AEFaultStatus		 = 0;
		AEFaultStatus.DPGAInterruptFlag	 = 0;
		AEFaultStatus.GDUInterruptFlag 	 = 0;
		AEFaultStatus.HVMInterruptFlags  = 0;
		AEFaultStatus.PMCMonitorRegister = 0;

		Aec_Ip_SpiRead((uint32_t)&IP_AEC_AE->VERID, 16, &SPI_rx_data);
		AEHealthStatus = Aec_Ip_DecodeGlobalHealthStatus();
		AEHealthStatus &= (AEC_STATUS_EVENT_NOTIFY | AEC_STATUS_FAULT_NOTIFY );

		//Check for AE health status - in case of pending faults set flag to read AE status in background loop and set AEfaultflag	if (AEHealthStatus !=0)
		if (AEHealthStatus !=0)
		{
			AEFaultStatus.AEIntFlag = 1;
			ReadAEStatus = 1;
		}

        /* When all Faults cleared prepare for transition to next state. */
        cntrState.usrControl.readFault             = true;
        cntrState.usrControl.switchFaultClear      = false;
        cntrState.event                            = e_fault_clear;

    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : StateInit
 * Description   : Function of the application state machine.
 *                 It clears application states variables.
 *
 *END**************************************************************************/
void StateInit(void)
{
    /*------------------------------------
     * Local user variables
     * ----------------------------------*/
    tBool InitFcnStatus = FALSE;

    /*------------------------------------
     * Application state machine variables
     * ----------------------------------*/
    cntrState.state     = init;
    cntrState.event     = e_init;

    /*------------------------------------
     * Turn off PWM output
     * ----------------------------------*/
    statePWM = ACTUATE_DisableOutput();

    /*------------------------------------
     * Reset state of user control variables
     * ----------------------------------*/
    cntrState.usrControl.switchAppOnOff      = false;
    cntrState.usrControl.switchAppOnOffState = false;
    cntrState.usrControl.switchFaultClear    = false;
    cntrState.usrControl.switchAppReset      = false;
    cntrState.usrControl.ledCounter          = 0;

    /*------------------------------------
     * Reset state of Speed control variables
     * ----------------------------------*/
    drvFOC.pospeControl.speedLoopCntr        = 0;

    /*------------------------------------
     * Reset state of Align control variables
     * ----------------------------------*/
    drvFOC.alignCntr               = drvFOC.alignCntrInitValue;

    /*------------------------------------
     * Clear Currents variables
     * ----------------------------------*/
    /* Current of ABC phases */
    drvFOC.iAbcFbck.fltArg1        = 0.0F;
    drvFOC.iAbcFbck.fltArg2        = 0.0F;
    drvFOC.iAbcFbck.fltArg3        = 0.0F;
    /* Current of alpha/beta phases */
    drvFOC.iAlBeFbck.fltArg1       = 0.0F;
    drvFOC.iAlBeFbck.fltArg2       = 0.0F;
    /* Current of DQ phases */
    drvFOC.iDQFbck.fltArg1         = 0.0F;
    drvFOC.iDQFbck.fltArg2         = 0.0F;
    /* iDQ requirement of inner loop */
    drvFOC.iDQReqInLoop.fltArg1    = 0.0F;
    drvFOC.iDQReqInLoop.fltArg2    = 0.0F;
    /* iDQ requirement of outer loop */
    drvFOC.iDQReqOutLoop.fltArg1   = 0.0F;
    drvFOC.iDQReqOutLoop.fltArg2   = 0.0F;

    /*------------------------------------
     * Clear voltage variables
     * ----------------------------------*/
    /* Voltage of alpha/beta phases */
    drvFOC.uAlBeReq.fltArg1        = 0.0F;
    drvFOC.uAlBeReq.fltArg2        = 0.0F;
    /* Voltage of alpha/beta phases after DC Bus ripple elimination */
    drvFOC.uAlBeReqDCB.fltArg1     = 0.0F;
    drvFOC.uAlBeReqDCB.fltArg2     = 0.0F;
    /* Voltage of DQ phases */
    drvFOC.uDQReq.fltArg1          = 0.0F;
    drvFOC.uDQReq.fltArg2          = 0.0F;

    /*------------------------------------
     * Clear Transformation angle
     * ----------------------------------*/
    drvFOC.thTransform.fltArg1     = 0.0F;
    drvFOC.thTransform.fltArg2     = 0.0F;

    /*------------------------------------
     * Clear SVC-PWM variables
     * ----------------------------------*/
    drvFOC.svmSector               = 1;
    drvFOC.pwmflt.fltArg1          = 0.0F;
    drvFOC.pwmflt.fltArg2          = 0.0F;
    drvFOC.pwmflt.fltArg3          = 0.0F;

    /*------------------------------------
     * Clear DC Bus ripple elimination variables
     * ----------------------------------*/
    drvFOC.elimDcbRip.fltArgDcBusMsr = 0.0F;

    /*------------------------------------
     * Clear position/speed variables
     * ----------------------------------*/
    drvFOC.pospeControl.wRotEl           = 0.0F;
    /* Sensorless position/speed */
    drvFOC.pospeSensorless.wRotEl        = 0.0F;
    drvFOC.pospeSensorless.thRotEl       = 0.0F;
    drvFOC.pospeSensorless.DQtoGaDeError = 0.0F;
    /* Open loop position/speed */
    drvFOC.pospeOpenLoop.integ.f32InK1   = 0.0F;
    drvFOC.pospeOpenLoop.integ.f32State  = 0.0F;
    drvFOC.pospeOpenLoop.thRotEl         = 0.0F;
    drvFOC.pospeOpenLoop.wRotEl          = 0.0F;

    /*------------------------------------
     * FOC function re-init
     * ----------------------------------*/
    /* Clear AMCLIB_CurrentLoop state variables */
    AMCLIB_CurrentLoopInit(&drvFOC.CurrentLoop);
    /* Clear DCBus filter and set state */
    GDFLIB_FilterMAInit(&drvFOC.uDcbFilter);
    drvFOC.uDcbFilter.fltAcc = U_DCB_NORMAL;
    /* Clear AMCLIB_FWSpeedLoopInit state variables */
    AMCLIB_FWSpeedLoopInit(&drvFOC.FwSpeedLoop);
    /* Clear back-EMF observer state variables */
    AMCLIB_BemfObsrvDQInit(&drvFOC.pospeSensorless.bEMFObs);
    /* Clear ATO observer state variables */
    AMCLIB_TrackObsrvInit(&drvFOC.pospeSensorless.TrackObsrv);
    #if ENCODER
    /* Clear ATO observer state variables */
    AMCLIB_TrackObsrvInit(&encoderPospe.TrackObsrv);
    #endif

    /*------------------------------------
     * Clear Measurement
     * ----------------------------------*/
    InitFcnStatus = MEAS_Clear(&meas);

    /*------------------------------------
     * Set default mode of operation
     * ----------------------------------*/
    cntrState.usrControl.controlMode    = automatic;
    pos_mode                            = force;
    switchSensor                        = sensorless;

    if (!InitFcnStatus)
    {
        /* Fault in initialization state */
        tempfaults.stateMachine.B.InitError = 1;       /* Mark the initialization fault */
        cntrState.event                     = e_fault; /* prepare for transition to fault state. */
    }
    else
    {
        /* Initialization phase successfully done */
        cntrState.event   = e_init_done;               /* prepare for transition to next state. */
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : StateReady
 * Description   : Function of the application state machine.
 *
 *END**************************************************************************/
void StateReady(void)
{
    /*-----------------------------------------------------
    Application State Machine - state identification
    ----------------------------------------------------- */

    /* Turn off PWM output */
    statePWM = ACTUATE_DisableOutput();

    if (cntrState.loadDefSetting)
    {
        cntrState.loadDefSetting = false;
        MCAT_Init();
    }

    cntrState.state   = ready;
    cntrState.event   = e_ready;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : StateCalib
 * Description   : Function of the application state machine.
 *
 *END**************************************************************************/
void StateCalib(void)
{
    /*-----------------------------------------------------
    Application State Machine - state identification
    ----------------------------------------------------- */
    tBool           CalibStatus;

    cntrState.state    = calib;
    cntrState.event    = e_calib;
    CalibStatus        = false;

    /* Turn on actuator output */
    statePWM = ACTUATE_EnableOutput();

    CalibStatus = MEAS_CalibCurrentSense(&meas, drvFOC.svmSector);

    /* Apply 0.5 duty cycle */
    drvFOC.pwmflt.fltArg1 = 0.5F;
    drvFOC.pwmflt.fltArg2 = 0.5F;
    drvFOC.pwmflt.fltArg3 = 0.5F;

    statePWM = ACTUATE_SetDutycycle(&drvFOC.pwmflt, 2);

    /* Exit the calibration state when DC calibration is done for all sectors */
    if (CalibStatus)
    {
        /* Calibration sequence has successfully finished */
        cntrState.event = e_calib_done;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : StateAlign
 * Description   : Function of the application state machine.
 *
 *END**************************************************************************/
void StateAlign(void)
{
    /*-----------------------------------------------------
    Application State Machine - state identification
    ----------------------------------------------------- */
    tBool           AlignStatus;

    cntrState.state   = align;
    cntrState.event   = e_align;

    /* Align sequence is at the beginning */
    AlignStatus     = true;
    if(drvFOC.alignCntr>(tU16)(drvFOC.alignCntrInitValue*ALIGN_D_FACTOR))
    {
    	/* Moving a rotor to prevent sticking in -D(180 deg) position */
        drvFOC.uDQReq.fltArg1      = 0;
        drvFOC.uDQReq.fltArg2      = drvFOC.alignVoltage;
    }else
    {
    	/* Alignment to D(0 deg) axis */
        drvFOC.uDQReq.fltArg1      = drvFOC.alignVoltage;
        drvFOC.uDQReq.fltArg2      = 0;
    }

    drvFOC.thTransform.fltArg1 = GFLIB_Sin(0);
    drvFOC.thTransform.fltArg2 = GFLIB_Cos(0);

    GMCLIB_ParkInv(&(drvFOC.uAlBeReq), &(drvFOC.thTransform), &(drvFOC.uDQReq));

    if (--(drvFOC.alignCntr) <= 0)
    {
        drvFOC.CurrentLoop.pIDQReq->fltArg1   = 0;
        drvFOC.CurrentLoop.pIDQReq->fltArg2   = 0;

        AMCLIB_CurrentLoopInit(&drvFOC.CurrentLoop);

        drvFOC.uDQReq.fltArg1           = 0;
        drvFOC.uDQReq.fltArg2           = 0;

        drvFOC.CurrentLoop.pPIrAWD.fltInErrK1 = 0;
        drvFOC.CurrentLoop.pPIrAWD.fltAcc     = 0;

        drvFOC.CurrentLoop.pPIrAWQ.fltInErrK1 = 0;
        drvFOC.CurrentLoop.pPIrAWQ.fltAcc     = 0;

        drvFOC.pwmflt.fltArg1           = 0.5F;
        drvFOC.pwmflt.fltArg2           = 0.5F;
        drvFOC.pwmflt.fltArg3           = 0.5F;
        /* Clear Encoder position and speed variables */
        #if ENCODER
        POSPE_ClearPospeElEnc(&encoderPospe);
        /* position range <-pi,pi> is <0,4*ENC_PULSES>  */
        encoderPospe.counterCwOffset  = Emios_Icu_Ip_GetEdgeNumbers(0U, 6U) + (2*ENC_PULSES);
        encoderPospe.counterCcwOffset = Emios_Icu_Ip_GetEdgeNumbers(0U, 7U);
        #endif
        if (!AlignStatus)
        {
            tempfaults.stateMachine.B.AlignError = 1;
        }
        else
        {
            cntrState.event = e_align_done;

        }
    }

    drvFOC.elimDcbRip.fltArgDcBusMsr = meas.measured.fltUdcb.raw;
    GMCLIB_ElimDcBusRip(&drvFOC.uAlBeReqDCB, &drvFOC.uAlBeReq, &drvFOC.elimDcbRip);

    drvFOC.svmSector = GMCLIB_SvmStd(&(drvFOC.pwmflt), &drvFOC.uAlBeReqDCB);

    statePWM = ACTUATE_SetDutycycle(&drvFOC.pwmflt, drvFOC.svmSector);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : StateRun
 * Description   : Function of the application state machine.
 *
 *END**************************************************************************/
void StateRun(void)
{
    static tBool stateRunStatus;

    stateRunStatus = false;

    /* Application State Machine - state identification */
    cntrState.state   = run;
    cntrState.event   = e_run;

    /* Calculate positions */
    CalcOpenLoop(&drvFOC.pospeOpenLoop, drvFOC.FwSpeedLoop.pRamp.fltState);

    /* Start calculation of the Bemf Observer in tracking mode */
    if (pos_mode != force)
    /* SENSORLESS CALCULATION - BACK-EMF OBSERVER
       INPUT  - Stator currents and voltages
              - Electrical angular velocity
              - Estimated rotor flux angle
       OUTPUT   - Phase error between synchronous and quasi-synchronous reference frame */
    drvFOC.pospeSensorless.DQtoGaDeError = AMCLIB_BemfObsrvDQ(&drvFOC.iAlBeFbck, &drvFOC.uAlBeReq,
                                                            drvFOC.pospeSensorless.wRotEl,
                                                            drvFOC.pospeSensorless.thRotEl,
                                                            &drvFOC.pospeSensorless.bEMFObs);

    /* SENSORLESS CALCULATION - ANGLE TRACKING OBSERVER
       INPUT  - Phase error between synchronous and quasi-synchronous reference frame
       OUTPUT   - Estimated rotor position and velocity */
    AMCLIB_TrackObsrv(drvFOC.pospeSensorless.DQtoGaDeError, &drvFOC.pospeSensorless.thRotEl, &drvFOC.pospeSensorless.wRotEl, &drvFOC.pospeSensorless.TrackObsrv);

    drvFOC.pospeOpenLoop.thDifOpenLEstim = MLIB_Sub(drvFOC.pospeSensorless.thRotEl, drvFOC.pospeOpenLoop.thRotEl);

    /*-----------------------------------------------------
    Get positions according to selected mode
    ----------------------------------------------------- */
    /* Selecting 0 will disable the "AUTOMATIC MODE"
       where the transition from open loop to sensorless in performed automatically
       Selecting 1 will enable the "USER mode"
       where user decide whether to switch to force mode, tracking mode, sensorless mode */
    #if ENCODER
    if (switchSensor == encoder && cntrState.usrControl.FOCcontrolMode != scalarControl)
    {
        pos_mode = encoder1;
    }

    else if (cntrState.usrControl.controlMode == automatic)
    {
        AutomaticMode();
    }
    #else
    if (cntrState.usrControl.controlMode == automatic)
    {
        AutomaticMode();
    }
    #endif


    /* user decide whether to switch to force mode, tracking mode, sensorless mode */
    switch (pos_mode)
    {
    case force:
        drvFOC.FwSpeedLoop.pPIpAWQ.fltUpperLimit  = drvFOC.pospeOpenLoop.iQUpperLimit;
        drvFOC.FwSpeedLoop.pPIpAWQ.fltLowerLimit  = MLIB_Neg(drvFOC.FwSpeedLoop.pPIpAWQ.fltUpperLimit);

        drvFOC.pospeControl.thRotEl = drvFOC.pospeOpenLoop.thRotEl;
        drvFOC.pospeControl.wRotEl  = 0;

        drvFOC.FwSpeedLoop.pRamp.fltRampDown = OL_SpeedRampInc;
        drvFOC.FwSpeedLoop.pRamp.fltRampUp   = OL_SpeedRampInc;

        drvFOC.pospeSensorless.wRotEl  = 0;
        drvFOC.pospeSensorless.thRotEl = 0;

        /* Clear back-EMF observer state variables */
        AMCLIB_BemfObsrvDQInit(&drvFOC.pospeSensorless.bEMFObs);

        /* Load back-EMF observer state variables from tracking mode */
        drvFOC.pospeSensorless.TrackObsrv.pParamPI.fltAcc      = drvFOC.pospeOpenLoop.wRotEl;
        drvFOC.pospeSensorless.TrackObsrv.pParamInteg.fltState = drvFOC.pospeOpenLoop.thRotEl;
        break;

    case tracking:
        drvFOC.FwSpeedLoop.pPIpAWQ.fltUpperLimit = drvFOC.pospeOpenLoop.iQUpperLimit;
        drvFOC.FwSpeedLoop.pPIpAWQ.fltLowerLimit = MLIB_Neg(drvFOC.FwSpeedLoop.pPIpAWQ.fltUpperLimit);

        drvFOC.pospeControl.thRotEl = drvFOC.pospeOpenLoop.thRotEl;
        drvFOC.pospeControl.wRotEl  = 0;

        drvFOC.FwSpeedLoop.pRamp.fltRampDown = OL_SpeedRampInc;
        drvFOC.FwSpeedLoop.pRamp.fltRampUp   = OL_SpeedRampInc;
        break;

    case sensorless1:
        drvFOC.FwSpeedLoop.pPIpAWQ.fltUpperLimit = drvFOC.pospeSensorless.iQUpperLimit;
        drvFOC.FwSpeedLoop.pPIpAWQ.fltLowerLimit = drvFOC.pospeSensorless.iQLowerLimit;

        drvFOC.pospeControl.thRotEl = drvFOC.pospeSensorless.thRotEl;
        drvFOC.pospeControl.wRotEl  = drvFOC.pospeSensorless.wRotEl;

        drvFOC.FwSpeedLoop.pRamp.fltRampDown = CL_SpeedRampDec;
        drvFOC.FwSpeedLoop.pRamp.fltRampUp   = CL_SpeedRampInc;

        drvFOC.pospeOpenLoop.integ.f32State  = MLIB_ConvertPU_F32FLT(MLIB_Div(drvFOC.pospeSensorless.thRotEl, FLOAT_PI));
        break;

    #if ENCODER
    case encoder1:
        drvFOC.FwSpeedLoop.pPIpAWQ.fltUpperLimit  = drvFOC.pospeSensorless.iQUpperLimit;
        drvFOC.FwSpeedLoop.pPIpAWQ.fltLowerLimit  = drvFOC.pospeSensorless.iQLowerLimit;

        // Quadrature decoder mode
        drvFOC.pospeControl.thRotEl = encoderPospe.thRotEl.filt;
        drvFOC.pospeControl.wRotEl  = encoderPospe.wRotEl.raw;

        drvFOC.FwSpeedLoop.pRamp.fltRampDown    = CL_SpeedRampDec;
        drvFOC.FwSpeedLoop.pRamp.fltRampUp      = CL_SpeedRampInc;

        drvFOC.pospeOpenLoop.integ.f32State = MLIB_ConvertPU_F32FLT(MLIB_Div(encoderPospe.thRotEl.filt, FLOAT_PI));
        break;
    #endif

    default:
        pos_mode = sensorless1;
    }

    /*-----------------------------------------------------
    Calculate Field Oriented Control FOC
    ----------------------------------------------------- */
    if (++drvFOC.pospeControl.speedLoopCntr >= SPEED_LOOP_CNTR)
    {
        drvFOC.pospeControl.speedLoopCntr = 0;

        stateRunStatus = FocSlowLoop();

        if (!stateRunStatus)
        {
            tempfaults.stateMachine.B.RunError = 1;
        }
    }

    stateRunStatus = FocFastLoop();

    if (!stateRunStatus)
    {
        tempfaults.stateMachine.B.RunError = 1;
    }

    /* Voltage vector sum calculation to check if DC bus voltage is used appropriately */
    UDQVectorSum = GFLIB_Sqrt(MLIB_Add(MLIB_Mul(drvFOC.uDQReq.fltArg1, drvFOC.uDQReq.fltArg1), MLIB_Mul(drvFOC.uDQReq.fltArg2, drvFOC.uDQReq.fltArg2)));

    statePWM = ACTUATE_SetDutycycle(&drvFOC.pwmflt, drvFOC.svmSector);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FocSlowLoop
 * Description   : Field Oriented Control - slow loop calculations
 *
 *END**************************************************************************/
tBool FocSlowLoop(void)
{
    if (cntrState.usrControl.FOCcontrolMode != speedControl)
    {
        /* required speed for open loop start-up in sensorless mode = MERG_SPEED_1_TRH*1,5 */
        /* wRotElReq = MERG_SPEED_1_TRH * 9.55 * 1.5 / pp = MERG_SPEED_1_TRH * 4.775 = ((MERG_SPEED_1_TRH*Frac16(0.596875)) << 3; */
        if ((cntrState.usrControl.FOCcontrolMode == voltageControl) && (drvFOC.uDQReq.fltArg2 == 0))
            drvFOC.pospeControl.wRotElReq = 0;
        else if ((cntrState.usrControl.FOCcontrolMode == currentControl) && (drvFOC.CurrentLoop.pIDQReq->fltArg2 == 0))
            drvFOC.pospeControl.wRotElReq = 0;
        else if (cntrState.usrControl.FOCcontrolMode != scalarControl)
            drvFOC.pospeControl.wRotElReq = MLIB_Mul(0.75F, MERG_SPEED_1_TRH);
    }

    if (fieldWeakOnOff)
    {
        drvFOC.FwSpeedLoop.pPIpAWFW.fltPropGain     = FW_PropGainControl;
        drvFOC.FwSpeedLoop.pPIpAWFW.fltIntegGain    = FW_IntegGainControl;
    }
    else
    {
        drvFOC.FwSpeedLoop.pPIpAWFW.fltPropGain     = 0.0F;
        drvFOC.FwSpeedLoop.pPIpAWFW.fltIntegGain    = 0.0F;
        drvFOC.FwSpeedLoop.pPIpAWFW.fltIntegPartK_1 = 0.0F;
    }

    AMCLIB_FWSpeedLoop(drvFOC.pospeControl.wRotElReq, drvFOC.pospeControl.wRotEl, &drvFOC.iDQReqOutLoop, &drvFOC.FwSpeedLoop);

    /* Speed FOC control mode */
    if (cntrState.usrControl.FOCcontrolMode == speedControl)
    {
        /* In Speed control mode, FOC Outer Loop (Speed Loop & Field Weakening) output is interconnected with FOC Inner Loop (Current Loop) input */
        if((pos_mode == sensorless1) || (pos_mode ==encoder1))
        {
             drvFOC.iDQReqInLoop.fltArg1 = drvFOC.iDQReqOutLoop.fltArg1;
             drvFOC.iDQReqInLoop.fltArg2 = drvFOC.iDQReqOutLoop.fltArg2;
        }else{
        	drvFOC.iDQReqInLoop.fltArg1 = drvFOC.pospeOpenLoop.iQUpperLimit;
        	drvFOC.iDQReqInLoop.fltArg2 = 0.0F;;
        }
    }

    return (true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : FocFastLoop
 * Description   : Field Oriented Control - fast loop calculations
 *
 *END**************************************************************************/
tBool FocFastLoop(void)
{

    GMCLIB_Clark(&drvFOC.iAlBeFbck, &drvFOC.iAbcFbck);

    /* Scalar control mode */
    if (cntrState.usrControl.FOCcontrolMode == scalarControl)
    {
        /* generated electrical position for scalar control purpose */

        /* Required voltage = VHzRatio * Required Frequency */
        drvFOC.scalarControl.UmReq     = MLIB_Mul(drvFOC.scalarControl.VHzRatioReq, drvFOC.pospeControl.wRotElReq);

        /* thRotEl is calculated in CalcOpenLoop executed in focSlowLoop */
        drvFOC.thTransform.fltArg1     = GFLIB_Sin(drvFOC.pospeControl.thRotEl);
        drvFOC.thTransform.fltArg2     = GFLIB_Cos(drvFOC.pospeControl.thRotEl);



        drvFOC.uDQReq.fltArg1          = 0;
        drvFOC.uDQReq.fltArg2          = drvFOC.scalarControl.UmReq;

        /* enable Bemf observer */
        cntrState.usrControl.controlMode = manual;
        pos_mode = tracking;
    }

    /* DQ Voltage FOC control mode */
    if (cntrState.usrControl.FOCcontrolMode == voltageControl)
    {
        if (drvFOC.uDQReq.fltArg2 != 0)
        {
            drvFOC.thTransform.fltArg1 = GFLIB_Sin(drvFOC.pospeControl.thRotEl);
            drvFOC.thTransform.fltArg2 = GFLIB_Cos(drvFOC.pospeControl.thRotEl);
        }
        else
        {
            drvFOC.thTransform.fltArg1 = GFLIB_Sin(0.0F);
            drvFOC.thTransform.fltArg2 = GFLIB_Cos(0.0F);
        }

        GMCLIB_Park(&drvFOC.iDQFbck, &drvFOC.thTransform, &drvFOC.iAlBeFbck);
    }

    /* DQ Current and Speed FOC control mode */
    if ((cntrState.usrControl.FOCcontrolMode == currentControl) || (cntrState.usrControl.FOCcontrolMode == speedControl))
    {
        if (cntrState.usrControl.FOCcontrolMode == speedControl)
        {
            drvFOC.thTransform.fltArg1     = GFLIB_Sin(drvFOC.pospeControl.thRotEl);
            drvFOC.thTransform.fltArg2     = GFLIB_Cos(drvFOC.pospeControl.thRotEl);
        }
        else
        {
            if (drvFOC.iDQReqOutLoop.fltArg2 != 0)
            {
                drvFOC.thTransform.fltArg1 = GFLIB_Sin(drvFOC.pospeControl.thRotEl);
                drvFOC.thTransform.fltArg2 = GFLIB_Cos(drvFOC.pospeControl.thRotEl);
            }
            else
            {
                drvFOC.thTransform.fltArg1 = GFLIB_Sin(0.0F);
                drvFOC.thTransform.fltArg2 = GFLIB_Cos(0.0F);
            }
        }

        GMCLIB_Park(&drvFOC.iDQFbck, &drvFOC.thTransform, &drvFOC.iAlBeFbck);

        /* 90% of available DCbus recalculated to phase voltage = 0.90*uDCB/sqrt(3) */
        AMCLIB_CurrentLoop(drvFOC.fltUdcb, &drvFOC.uDQReq, &drvFOC.CurrentLoop);
    }

    GMCLIB_ParkInv(&drvFOC.uAlBeReq, &drvFOC.thTransform, &drvFOC.uDQReq);

    drvFOC.elimDcbRip.fltArgDcBusMsr = meas.measured.fltUdcb.raw;
    GMCLIB_ElimDcBusRip(&drvFOC.uAlBeReqDCB, &drvFOC.uAlBeReq, &drvFOC.elimDcbRip);

    drvFOC.AlBeReqDCBLim.fltLimit = 0.9;
    GFLIB_VectorLimit_FLT (&drvFOC.uAlBeReqDCBLim, &drvFOC.uAlBeReqDCB, &drvFOC.AlBeReqDCBLim);

    drvFOC.svmSector = GMCLIB_SvmStd(&(drvFOC.pwmflt), &drvFOC.uAlBeReqDCBLim);

    return (true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CalcOpenLoop
 * Description   : Generator of open loop position and speed.
 *
 *END**************************************************************************/
tBool CalcOpenLoop(openLoopPospe_t *openLoop, tFloat speedReqRamp)
{
    openLoop->wRotEl  = speedReqRamp;
    openLoop->thRotEl = MLIB_Mul(MLIB_ConvertPU_FLTF32(GFLIB_IntegratorTR_F32(MLIB_ConvertPU_F32FLT(MLIB_Div(speedReqRamp, WEL_MAX)),
                                &(openLoop->integ))), FLOAT_PI);

    return (true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : AutomaticMode
 * Description   : Automatic selection of the mode based on demanded speed.
 *
 *END**************************************************************************/
tBool AutomaticMode(void)
{
	static tBool trackingToSensorless 	= false;

	if ((MLIB_Abs(drvFOC.pospeOpenLoop.wRotEl) > drvFOC.pospeSensorless.wRotElMatch_2))
	{
		// Just once the sensorless is entered, the speed PI controller needs to be reset to avoid step current changes
		if(trackingToSensorless)
		{
			drvFOC.iDQReqInLoop.fltArg1 = 0.0F;
			drvFOC.iDQReqInLoop.fltArg2 = 0.0F;
			drvFOC.FwSpeedLoop.pPIpAWQ.fltIntegPartK_1 = 0.0F;
			drvFOC.FwSpeedLoop.pPIpAWFW.fltIntegPartK_1 = 0.0F;

			trackingToSensorless		= false;
		}

		pos_mode = sensorless1;
	}
	else if ((MLIB_Abs(drvFOC.pospeOpenLoop.wRotEl) >= drvFOC.pospeSensorless.wRotElMatch_1 &&
			 MLIB_Abs(drvFOC.pospeOpenLoop.wRotEl) < drvFOC.pospeSensorless.wRotElMatch_2))
	{
		pos_mode = tracking;
		trackingToSensorless			= true;
	}
	else
	{
		pos_mode 						= force;
	}

	return(true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BoardButtons
 * Description   : Board buttons to control the motor speed and ON/OFF/FAULT state.
 *
 *END**************************************************************************/
void BoardButtons(void)
{
    /* Turn the application on and increase the rotor velocity */
    if (cntrState.usrControl.btSpeedUp && (!cntrState.usrControl.btSpeedDown))
    {
        if (cntrState.usrControl.cntSpeedUp > SPEED_UP_CNT)
        {
            cntrState.usrControl.cntSpeedUp = 0;
            cntrState.usrControl.switchAppOnOff = true;
            drvFOC.pospeControl.wRotElReq = MLIB_Add(drvFOC.pospeControl.wRotElReq, SPEED_RAD_INC);
        }
        else
        {
            cntrState.usrControl.cntSpeedUp++;
            cntrState.usrControl.cntSpeedDown = 0;
            cntrState.usrControl.cntAppOff = 0;
        }
        if(drvFOC.pospeControl.wRotElReq > SPEED_LIM_RAD_BTN)	drvFOC.pospeControl.wRotElReq = SPEED_LIM_RAD_BTN;
    }

    /* Turn the application ON and decrease the rotor velocity */
    if ((!cntrState.usrControl.btSpeedUp) && cntrState.usrControl.btSpeedDown)
    {
        if (cntrState.usrControl.cntSpeedDown > SPEED_DOWN_CNT)
        {
            cntrState.usrControl.cntSpeedDown = 0;
            cntrState.usrControl.switchAppOnOff = true;
            drvFOC.pospeControl.wRotElReq = MLIB_Sub(drvFOC.pospeControl.wRotElReq, SPEED_RAD_DEC);
        }
        else
        {
            cntrState.usrControl.cntSpeedDown++;
            cntrState.usrControl.cntSpeedUp = 0;
            cntrState.usrControl.cntAppOff = 0;
        }
        if(drvFOC.pospeControl.wRotElReq < -SPEED_LIM_RAD_BTN)	drvFOC.pospeControl.wRotElReq = -SPEED_LIM_RAD_BTN;
    }

    /* Turn the application off and clear application faults */
    if (cntrState.usrControl.btSpeedUp && cntrState.usrControl.btSpeedDown)
    {
        /* Turn the application off */
        if ((cntrState.usrControl.cntAppOff > APP_OFF_CNT) && (cntrState.state != fault))
        {
            cntrState.usrControl.cntAppOff = 0;
            cntrState.usrControl.switchAppOnOff = false;
            drvFOC.pospeControl.wRotElReq = 0;
        }

        /* Clear application faults */
        else if ((cntrState.usrControl.cntAppOff > APP_OFF_CNT) && (cntrState.state == fault))
        {
            cntrState.usrControl.cntAppOff = 0;
            cntrState.usrControl.switchFaultClear = true;
            drvFOC.pospeControl.wRotElReq = 0;
        }
        else
        {
            cntrState.usrControl.cntAppOff++;
            cntrState.usrControl.cntSpeedUp = 0;
            cntrState.usrControl.cntSpeedDown = 0;
        }
    }
}

/***************************************************************************//*!
 *
 * @brief   Fault Detection function
 *
 * @param   none
 *
 * @return  none
 *
 ******************************************************************************/
__attribute__((section (".ramcode")))
tBool FaultDetection()
{
	tBool faultDetectiontEvent;

	faultDetectiontEvent = false;

	//-----------------------------
	// Actual Faults
	//-----------------------------
	// TRIP:   Phase A over-current detected
	tempfaults.motor.B.OverPhaseACurrent = (drvFOC.iAbcFbck.fltArg1 > MLIB_Mul(I_PH_OVER, 0.9F)) ? true : false;

	// TRIP:   Phase B over-current detected
	tempfaults.motor.B.OverPhaseBCurrent = (drvFOC.iAbcFbck.fltArg2 > MLIB_Mul(I_PH_OVER, 0.9F)) ? true : false;

	// TRIP:   Phase C over-current detected
	tempfaults.motor.B.OverPhaseCCurrent = (drvFOC.iAbcFbck.fltArg3 > MLIB_Mul(I_PH_OVER, 0.9F)) ? true : false;

	// TRIP:   DC-bus over-voltage
	tempfaults.motor.B.OverDCBusVoltage  = (meas.measured.fltUdcb.raw > U_DCB_TRIP) ? true : false;

	// TRIP:   DC-bus under-voltage
	tempfaults.motor.B.UnderDCBusVoltage = (meas.measured.fltUdcb.raw < MLIB_Div(U_DCB_UNDER,0.91F)) ? true : false;

	// Activate braking resistor, if there is DC-bus over-voltage
	/*
	if(tempfaults.motor.B.OverDCBusVoltage)
	{
		// Activate braking resistor
		GPIO_HAL_SetPins(PTD, 1<<14);
	}
	else
	{
		// Deactivate braking resistor
		GPIO_HAL_ClearPins(PTD, 1<<14);
	}
	 */

	//-----------------------------
	// Pending Faults
	//-----------------------------
	if (cntrState.state != fault)
	{
		// Fault:   Phase A over-current detected
		permFaults.motor.B.OverPhaseACurrent   = (drvFOC.iAbcFbck.fltArg1 > I_PH_OVER) ? true : permFaults.motor.B.OverPhaseACurrent;

		// Fault:   Phase B over-current detected
		permFaults.motor.B.OverPhaseBCurrent   = (drvFOC.iAbcFbck.fltArg2 > I_PH_OVER) ? true : permFaults.motor.B.OverPhaseBCurrent;

		// Fault:   Phase C over-current detected
		permFaults.motor.B.OverPhaseCCurrent   = (drvFOC.iAbcFbck.fltArg3 > I_PH_OVER) ? true : permFaults.motor.B.OverPhaseCCurrent;

		// Fault:   DC-bus over-voltage
		permFaults.motor.B.OverDCBusVoltage    = (meas.measured.fltUdcb.raw > U_DCB_OVER) ? true : permFaults.motor.B.OverDCBusVoltage;

		// Fault:   DC-bus under-voltage
		permFaults.motor.B.UnderDCBusVoltage   = (meas.measured.fltUdcb.raw < U_DCB_UNDER) ? true : permFaults.motor.B.UnderDCBusVoltage;
	}

	// Check, whether back-EMF observer estimates rotor position properly
	if((pos_mode == sensorless1) && (cntrState.usrControl.FOCcontrolMode == speedControl) && (cntrState.state != fault)
			&& (MLIB_Abs(drvFOC.pospeOpenLoop.thDifOpenLEstim)>MAX_TH_DIF_OPEN_ESTIM))
	{
		drvFOC.pospeSensorless.sensorlessCnt++;
		if(drvFOC.pospeSensorless.sensorlessCnt > 10000)
		{
			drvFOC.pospeSensorless.sensorlessCnt 	= 0;
			permFaults.stateMachine.B.FOCError 		= 1;
		}
	}
	else
	{
		drvFOC.pospeSensorless.sensorlessCnt 		= 0;
	}

    /* BCTU trigger fault */
	if ((measuredValues[0].TriggerIdx != 23U) | (measuredValues[1].TriggerIdx != 23U) | (measuredValues[2].TriggerIdx != 23U) |
		(measuredValues[3].TriggerIdx != 23U) | (measuredValues[4].TriggerIdx != 23U))
	{
		permFaults.mcu.B.ADC_Trigger_Error = true;
		faultDetectiontEvent = true;
	}
	if ((measuredValues[0].AdcNum != 1U) | (measuredValues[1].AdcNum != 1U) | (measuredValues[2].AdcNum != 1U) |
		(measuredValues[3].AdcNum != 1U) | (measuredValues[4].AdcNum != 1U))
	{
		permFaults.mcu.B.ADC_Instance_Error = true;
		faultDetectiontEvent = true;
	}
	if ((measuredValues[0].ChanIdx != 32U) | (measuredValues[1].ChanIdx != 32U) | (measuredValues[2].ChanIdx != 42U) |
		(measuredValues[0].ChanIdx != 32U) | (measuredValues[1].ChanIdx != 32U))
	{
		permFaults.mcu.B.ADC_Channel_Error = true;
		faultDetectiontEvent = true;
	}

	if (AEFaultStatus.AEIntFlag)
	{

		permFaults.AEModule = 1;
		faultDetectiontEvent 	= true;

		if (ReadAEStatus == 0) //wait till all relevant AE status words have been read in the background loop
		{
			AEFaultStatus.AEIntFlag = false;

		}

	}

	if ((permFaults.motor.R != 0x0))
		faultDetectiontEvent 	= true;
	if ((permFaults.mcu.R != 0x0))
		faultDetectiontEvent 	= true;
	if ((permFaults.stateMachine.R != 0x0))
		faultDetectiontEvent 	= true;

	return faultDetectiontEvent;
}


#ifdef __cplusplus
}
#endif

/** @} */
