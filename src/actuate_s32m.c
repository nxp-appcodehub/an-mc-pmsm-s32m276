/*******************************************************************************
*   Project      : S32M27xEVB_PMSM_FOC_1Sh
*   Revision     : 1.2
*   RTD Version  : 7.0.0
*   Brief description  :
*   File contains definition of functions needed for inverter voltage control.
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
 * actuate_S32m.c
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 * rev 1.2 Added support for RTD 7.0.0
 *
 ********************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include <actuate_s32m.h>
#include <meas_s32m.h>
#include "gflib.h"
#include "gdflib.h"
#include "gmclib.h"
#include "Emios_Mcl_Ip.h"
#include "Lcu_Ip.h"

/******************************************************************************
| External variables
-----------------------------------------------------------------------------*/
extern float Nhold;         /*The minimal "sample&hold" time of the ADC*/
extern float ZeroPulse;     /* The minimal pulse inserted to the middle of the PWM period of double-switching signal.*/
extern float SamplingPulse; /* Represents at least the minimal conversion time of ADC */

/******************************************************************************
| Global variable definitions   (scope: module-local)
-----------------------------------------------------------------------------*/

uint32_t				minZeroVectorDPGAOffset = 400;
tBool					DPGAOffsetCompSwitch = 0;
/*==================================================================================================
*                                    FUNCTIONS DEFINITION
==================================================================================================*/

/**************************************************************************//*!
@brief Unmask PWM output and set 50% dutycyle

@param[in,out]  

@return
******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : ACTUATE_EnableOutput
 * Description   : It unmasks PWM output and set 1% dutycyle.
 *
 *END**************************************************************************/
tBool ACTUATE_EnableOutput(void)
{
	SWLIBS_3Syst_FLT fltpwm;
    tBool statePWM;
    Lcu_Ip_SyncOutputValueType lcuEnable[6U];
    /* Apply 0.5 duty cycle */
    fltpwm.fltArg1 = 0.5;
    fltpwm.fltArg2 = 0.5;
    fltpwm.fltArg3 = 0.5;
    statePWM = ACTUATE_SetDutycycle(&fltpwm, 2);
    lcuEnable[0].LogicOutputId = LCU_LOGIC_OUTPUT_0;
    lcuEnable[0].Value = 1U;
    lcuEnable[1].LogicOutputId = LCU_LOGIC_OUTPUT_1;
    lcuEnable[1].Value = 1U;
    lcuEnable[2].LogicOutputId = LCU_LOGIC_OUTPUT_2;
    lcuEnable[2].Value = 1U;
    lcuEnable[3].LogicOutputId = LCU_LOGIC_OUTPUT_3;
    lcuEnable[3].Value = 1U;
    lcuEnable[4].LogicOutputId = LCU_LOGIC_OUTPUT_4;
    lcuEnable[4].Value = 1U;
    lcuEnable[5].LogicOutputId = LCU_LOGIC_OUTPUT_5;
    lcuEnable[5].Value = 1U;
    Lcu_Ip_SetSyncOutputEnable(lcuEnable, 6U);

    return (statePWM);
}


/*FUNCTION**********************************************************************
 *
 * Function Name : ACTUATE_DisableOutput
 * Description   : It masks PWM output and set 1% dutycyle.
 *
 *END**************************************************************************/
tBool ACTUATE_DisableOutput(void)
{
	SWLIBS_3Syst_FLT fltpwm;
    tBool statePWM;
    Lcu_Ip_SyncOutputValueType lcuDisable[6U];
    lcuDisable[0].LogicOutputId = LCU_LOGIC_OUTPUT_0;
    lcuDisable[0].Value = 0U;
    lcuDisable[1].LogicOutputId = LCU_LOGIC_OUTPUT_1;
    lcuDisable[1].Value = 0U;
    lcuDisable[2].LogicOutputId = LCU_LOGIC_OUTPUT_2;
    lcuDisable[2].Value = 0U;
    lcuDisable[3].LogicOutputId = LCU_LOGIC_OUTPUT_3;
    lcuDisable[3].Value = 0U;
    lcuDisable[4].LogicOutputId = LCU_LOGIC_OUTPUT_4;
    lcuDisable[4].Value = 0U;
    lcuDisable[5].LogicOutputId = LCU_LOGIC_OUTPUT_5;
    lcuDisable[5].Value = 0U;
    Lcu_Ip_SetSyncOutputEnable(lcuDisable, 6U);
    /* Apply 0.5 duty cycle */
    fltpwm.fltArg1 = 0.5;
    fltpwm.fltArg2 = 0.5;
    fltpwm.fltArg3 = 0.5;
    statePWM = ACTUATE_SetDutycycle(&fltpwm, 2);
    
    return (statePWM);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACTUATE_SetDutycycle
 * Description   : It sets PWM dutycyle, the dutycyle will by updated on next reload event.
 *
 *END**************************************************************************/
tBool ACTUATE_SetDutycycle(SWLIBS_3Syst_FLT *fltpwm, tU16 sector)
{
    tBool statePwm = TRUE;
    SWLIBS_3Syst_F16 inPwm;
    GMCLIB_DOUBLESWITCHING_T_F16 params;
    GMCLIB_DOUBLESWITCHING_3PH_EDGES_T_F16 outEdges;
    GMCLIB_DOUBLESWITCHING_TRIGGERS_T_F16 outTriggers;
    /*pwm timebase period generated by config tool*/
    uint16 pwmPeriodHalf =(uint16) (Emios_Mcl_Ip_Sa_0_MasterBusConfig[0].defaultPeriod >> 1);
    uint16 pwmPeriod =(uint16) Emios_Mcl_Ip_Sa_0_MasterBusConfig[0].defaultPeriod;
    tFrac16	SamplingPulseDouble = 0;

    inPwm.f16Arg1 = FRAC16(fltpwm->fltArg1);
    inPwm.f16Arg2 = FRAC16(fltpwm->fltArg2);
    inPwm.f16Arg3 = FRAC16(fltpwm->fltArg3);
    params.f16MinZeroPulse = FRAC16(ZeroPulse);
    params.f16MinSamplingPulse = FRAC16(SamplingPulse);
    params.f16SampleNhold = FRAC16(Nhold);



#if   DOUBLE_SW_ADAPTIVE

    SamplingPulseDouble = params.f16MinSamplingPulse << 1;

    switch (sector)
    {
		case 1:		//duty A > duty B > duty C
				if((MLIB_Sub_F16(inPwm.f16Arg1, inPwm.f16Arg2) > SamplingPulseDouble) &&
				  (MLIB_Sub_F16(inPwm.f16Arg2, inPwm.f16Arg3) > SamplingPulseDouble))
				{
					params.f16MinZeroPulse = FRAC16(0.0F);
				}
				break;

		case 2:		//duty B > duty A > duty C
				if((MLIB_Sub_F16(inPwm.f16Arg2, inPwm.f16Arg1) > SamplingPulseDouble) &&
				  (MLIB_Sub_F16(inPwm.f16Arg1, inPwm.f16Arg3) > SamplingPulseDouble))
				{
					params.f16MinZeroPulse = FRAC16(0.0F);
				}
				break;

		case 3:		//duty B > duty C > duty A
				if((MLIB_Sub_F16(inPwm.f16Arg2, inPwm.f16Arg3) > SamplingPulseDouble) &&
				  (MLIB_Sub_F16(inPwm.f16Arg3, inPwm.f16Arg1) > SamplingPulseDouble))
				{
					params.f16MinZeroPulse = FRAC16(0.0F);
				}
				break;

		case 4:		//duty C > duty B > duty A
				if((MLIB_Sub_F16(inPwm.f16Arg3, inPwm.f16Arg2) > SamplingPulseDouble) &&
				  (MLIB_Sub_F16(inPwm.f16Arg2, inPwm.f16Arg1) > SamplingPulseDouble))
				{
					params.f16MinZeroPulse = FRAC16(0.0F);
				}
				break;

		case 5:		//duty C > duty A > duty B
				if((MLIB_Sub_F16(inPwm.f16Arg3, inPwm.f16Arg1) > SamplingPulseDouble) &&
				  (MLIB_Sub_F16(inPwm.f16Arg1, inPwm.f16Arg2) > SamplingPulseDouble))
				{
					params.f16MinZeroPulse = FRAC16(0.0F);
				}
				break;
		case 6:		//duty A > duty C > duty B
				if((MLIB_Sub_F16(inPwm.f16Arg1, inPwm.f16Arg3) > SamplingPulseDouble) &&
				  (MLIB_Sub_F16(inPwm.f16Arg3, inPwm.f16Arg2) > SamplingPulseDouble))
				{
					params.f16MinZeroPulse = FRAC16(0.0F);
				}
				break;

		default:
			break;
    }
#endif

    GMCLIB_DoubleSwitching_F16(&outEdges, &outTriggers, &inPwm, sector, &params);
    Emios_Mcl_Ip_ComparatorTransferDisable(0U,(uint32_t)0x27E3EU);

    uint16 pwmTrig1RegA = (uint16_t)((outTriggers.u16Trigger1 * pwmPeriod) >> 15);
    uint16 pwmTrig2RegA = (uint16_t)((outTriggers.u16Trigger2 * pwmPeriod) >> 15);
    uint16 pwmTrig3RegA = (uint16_t)((outTriggers.u16Trigger3 * pwmPeriod) >> 15);
    uint16 pwmTrig4RegA = (uint16_t)((outTriggers.u16Trigger4 * pwmPeriod) >> 15);

    uint16 pwmPhaseAHRegA = (uint16_t)((outEdges.pPhA.f16Edge1 * pwmPeriod) >> 15);
    uint16 pwmPhaseAHRegB = (uint16_t)((outEdges.pPhA.f16Edge2 * pwmPeriod) >> 15);
    uint16 pwmPhaseALRegA = (uint16_t)((outEdges.pPhA.f16Edge3 * pwmPeriod) >> 15);
    uint16 pwmPhaseALRegB = (uint16_t)((outEdges.pPhA.f16Edge4 * pwmPeriod) >> 15);
    uint16 pwmPhaseBHRegA = (uint16_t)((outEdges.pPhB.f16Edge1 * pwmPeriod) >> 15);
    uint16 pwmPhaseBHRegB = (uint16_t)((outEdges.pPhB.f16Edge2 * pwmPeriod) >> 15);
    uint16 pwmPhaseBLRegA = (uint16_t)((outEdges.pPhB.f16Edge3 * pwmPeriod) >> 15);
    uint16 pwmPhaseBLRegB = (uint16_t)((outEdges.pPhB.f16Edge4 * pwmPeriod) >> 15);
    uint16 pwmPhaseCHRegA = (uint16_t)((outEdges.pPhC.f16Edge1 * pwmPeriod) >> 15);
    uint16 pwmPhaseCHRegB = (uint16_t)((outEdges.pPhC.f16Edge2 * pwmPeriod) >> 15);
    uint16 pwmPhaseCLRegA = (uint16_t)((outEdges.pPhC.f16Edge3 * pwmPeriod) >> 15);
    uint16 pwmPhaseCLRegB = (uint16_t)((outEdges.pPhC.f16Edge4 * pwmPeriod) >> 15);

    Emios_Pwm_Ip_UpdateUCRegA(0U, 1U, pwmTrig1RegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 1U, pwmTrig1RegA + 10);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 2U, pwmTrig2RegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 2U, pwmTrig2RegA + 10);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 3U, pwmPeriodHalf);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 3U, pwmPeriodHalf + 10);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 4U, pwmTrig3RegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 4U, pwmTrig3RegA + 10);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 5U, pwmTrig4RegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 5U, pwmTrig4RegA + 10);


    Emios_Pwm_Ip_UpdateUCRegA(0U, 9U, pwmPhaseAHRegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 9U, pwmPhaseAHRegB);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 10U, pwmPhaseALRegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 10U, pwmPhaseALRegB);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 11U, pwmPhaseBHRegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 11U, pwmPhaseBHRegB);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 12U, pwmPhaseBLRegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 12U, pwmPhaseBLRegB);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 13U, pwmPhaseCHRegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 13U, pwmPhaseCHRegB);
    Emios_Pwm_Ip_UpdateUCRegA(0U, 14U, pwmPhaseCLRegA);
    Emios_Pwm_Ip_UpdateUCRegB(0U, 14U, pwmPhaseCLRegB);


		if ((pwmTrig4RegA > (pwmPeriod - minZeroVectorDPGAOffset + 20)) && (DPGAOffsetCompSwitch == 0))
		{
			Emios_Pwm_Ip_UpdateUCRegA(0U, 17U, 1);
			Emios_Pwm_Ip_UpdateUCRegB(0U, 17U, 20000);

			DPGAOffsetCompSwitch = 1;

		}

		if ((pwmTrig4RegA < (pwmPeriod - minZeroVectorDPGAOffset - 20)) && (DPGAOffsetCompSwitch == 1))
		{
			Emios_Pwm_Ip_UpdateUCRegA(0U, 17U, 1);
			Emios_Pwm_Ip_UpdateUCRegB(0U, 17U, 2);

			DPGAOffsetCompSwitch = 0;

		}


		Emios_Mcl_Ip_ComparatorTransferEnable(0U,(uint32_t)0x27E3EU);

    return(statePwm);
}
