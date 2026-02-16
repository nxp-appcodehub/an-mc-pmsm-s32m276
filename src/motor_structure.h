/*******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.2
*   RTD Version  : 7.0.0
*   Brief description  :
*   File contains declaration of application structures used for motor control.
*
*   Copyright 2024-2025 NXP
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
 * motor_structure.h
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 * rev 1.2 removed unused defines
 *
 ********************************************************************************/
#ifndef _MOTOR_STRUCTURE_H
#define _MOTOR_STRUCTURE_H

/******************************************************************************
* Includes
******************************************************************************/
#include "amclib.h"
#include "gdflib.h"
#include "gflib.h"
#include "gmclib.h"
#include "state_machine.h"

/*==================================================================================================
*                              STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/






/*------------------------------------------------------------------------*//*!
@brief  Structure containing position/speed module variables
*//*-------------------------------------------------------------------------*/
typedef struct
{
    tFloat                          thRotEl;          /* El. position entering to the control loop           */
    tFloat                          thRotElEnc;       /* El. position entering to the control loop (Encoder) */
    tFloat                          thRotElEnck1;     /* El. position entering to the control loop (Encoder) */
    tFloat                          thRotElEncErr;    /* El. position entering to the control loop (Encoder) */
    tFloat                          wRotEl;           /* El. speed entering to the control loop              */
    tFloat                          wRotElEnc;        /* El. speed entering to the control loop              */
    tFloat                          wRotElFilt;       /* Filtered El. speed entering to the control loop     */
    tFloat                          wRotElReq;        /* Required el. speed                                  */
    tFloat                          wRotElErr;        /* Error of the el. speed entering to speed controller */
    tS16                            speedLoopCntr;    /* rate between speed and current loop                 */
} pospeControl_t;

typedef struct
{
    tFloat                          thRotEl;
    tFloat                          wRotEl;
    GFLIB_INTEGRATOR_TR_T_F32       integ;
    tFloat                          iQUpperLimit;
    tFloat                          iQLowerLimit;
    tFloat                          thDifOpenLEstim;
}openLoopPospe_t;

typedef struct
{
    tFloat                          thRotEl;
    tFloat                          wRotEl;
    tFloat                          wRotElMatch_1;
    tFloat                          wRotElMatch_2;
    AMCLIB_BEMF_OBSRV_DQ_T_FLT      bEMFObs;
    AMCLIB_TRACK_OBSRV_T_FLT        TrackObsrv;
    tFloat                          DQtoGaDeError;
    tFloat                          iQUpperLimit;
    tFloat                          iQLowerLimit;
    uint16_t                        sensorlessCnt;
}sensorLessPospe_t;

typedef struct
{
    tFloat                          UmReq;                /* Required magnitude of supply voltage for scalar control */
    tFloat                          VHzRatioReq;          /* V/f ratio - gain                                        */
    tS16                            VHzRatioReq_Shift;    /* V/f ratio - shift                                       */
}scalarControl_t;


/*! General stucture for PMSM motor */
typedef struct{
    tU16                            alignCntr;          /* Alignment duration                                                                */
    tU16                            alignCntrInitValue; /* Alignment Initial duration                                                        */
    tFloat                          alignVoltage;       /* Alignment voltage                                                                 */
    tU16                            svmSector;          /* Space Vector Modulation sector                                                    */
    SWLIBS_2Syst_FLT                iDQFbck;            /* dq - axis current feedback                                                        */
    SWLIBS_2Syst_FLT                iAlBeFbck;          /* Alpha/Beta - axis current feedback                                                */
    SWLIBS_2Syst_FLT                iDQReqInLoop;       /* dq - axis required currents, FOC Inner Loop (Current Loop) input                  */
    SWLIBS_2Syst_FLT                iDQReqOutLoop;      /* dq - axis required currents, FOC Outer Loop (Speed Loop & Field Weakening) output */
    SWLIBS_2Syst_FLT                uDQReq;             /* dq - axis required voltages given by current PIs                                  */
    SWLIBS_2Syst_FLT                uAlBeReq;           /* Alpha/Beta required voltages                                                      */
    SWLIBS_2Syst_FLT                uAlBeReqDCB;        /* Alpha/Beta required voltages after DC Bus ripple elimination                      */
    SWLIBS_2Syst_FLT                uAlBeReqDCBLim;     /* Alpha/Beta required voltages after DC Bus ripple elimination with limits          */
    SWLIBS_2Syst_FLT                uAlBEReqDCBTest;
    SWLIBS_2Syst_FLT                thTransform;        /* Transformation angle - enters to Park transformation                              */
    SWLIBS_3Syst_FLT                iAbcFbck;           /* Three phases current feedback                                                     */
    SWLIBS_3Syst_FLT                pwmflt;             /* Three phase 16bit Duty-Cycles estimated from uAlBeReqDCB                          */
    GMCLIB_ELIMDCBUSRIP_T           elimDcbRip;         /* Predefined structure related to DC Bus voltage ripple elimination                 */
    GDFLIB_FILTER_MA_T              uDcbFilter;         /* DC bus voltage filter settings                                                    */
    tFloat                          fltUdcb;            /* DC bus voltage                                                                    */
    openLoopPospe_t                 pospeOpenLoop;      /* Open Loop Position generator                                                      */
    sensorLessPospe_t               pospeSensorless;    /* Sensorless position and speed including open loop matching                        */
    pospeControl_t                  pospeControl;       /* Position/Speed variables needed for control                                       */
    scalarControl_t                 scalarControl;      /* Scalar Control variables for MCAT purpose                                         */
    AMCLIB_CURRENT_LOOP_T_FLT       CurrentLoop;        /* Current loop function                                                             */
    AMCLIB_FW_SPEED_LOOP_T_FLT      FwSpeedLoop;        /* Speed loop plus field weakining function                                          */
    GFLIB_VECTORLIMIT_T_FLT         AlBeReqDCBLim;      /* limits for uAlBeReqDCB                                                            */
}pmsmDrive_t;

typedef enum
{
	force	 		= 0,
	tracking 		= 1,
	sensorless1 	= 2,
	encoder1     	= 3
}tPos_mode;

typedef union {
    uint16_t R;
    struct {
        uint16_t Alignment:1;
        uint16_t Sensorless:1;
        uint16_t StallCheckReq:1;
        uint16_t EnableCMT:1;
        uint16_t AfterCMT:1;
        uint16_t CloseLoop:1;
        uint16_t NewZC:1;
        uint16_t AdcSaved:1;
        uint16_t CurrentLimiting:1;
        uint16_t Fault:1;
        uint16_t Freewheeling:1;
        uint16_t Calib:1;
        uint16_t HallEvent:1;
        uint16_t Reserved:3;
    }B;
}tDriveStatus;

typedef union {
    uint8_t R;
    struct {
        uint8_t OverDCBusCurrent:1;
        uint8_t OverDCBusVoltage:1;
        uint8_t UnderDCBusVoltage:1;
        uint8_t PreDriverError:1;
        uint8_t StallError:1;
        uint8_t Reserved:3;
    }B;
}tFaultStatus;

typedef struct {
    tFloat BEMFVoltage;
    tFloat DCBVVoltage;
    tFloat DCBIVoltage;
    tFloat DCBIVoltageRaw;
    tFloat DCBIOffset;
}tADCresults;

typedef struct {
    uint8_t     InA;
    uint8_t     InB;
    uint8_t     InC;
    uint8_t     InABC;
    uint8_t     Sector;
    uint16_t    Period[6];
}tSensorHall;

typedef struct
{
    tBool                     AEIntFlag;
    uint32                    AEEventStatus;
    uint32                    AEFaultStatus;
    uint32                    HVMInterruptFlags;
    uint32                    PMCMonitorRegister;
    uint32                    GDUInterruptFlag;
    uint32                    DPGAInterruptFlag;
}AEFaultStatus_t;

typedef enum
{
    encoder    = 0,
    sensorless = 1
}switchSensor_t;

typedef enum CONTORL_MODE_e
{
    manual,
    automatic
}controlMode_t;

typedef enum
{
    scalarControl	=0,
    voltageControl	=1,
    currentControl	=2,
    speedControl	=3
}controlStructMode_t;

typedef struct
{
    tU32                ledCounter;
    tU32                ledFlashing;
    tU32                cntSpeedUp;
    tU32                cntSpeedDown;
    tU32                cntAppOff;
    tU8                 btSpeedUp;                     /* Variable to increase the speed command by given step                */
    tU8                 btSpeedDown;                   /* Variable to lower the speed command by given step                   */
    tU8                 btFlipFlop, btFlipFlopTemp;    /* Enable/Disable Control                                              */
    controlMode_t       controlMode;                   /* Handles whether the drive is in openLoop, sensorLess, resolver mode */
    controlStructMode_t FOCcontrolMode;                /* defines the cascade control mode: range(0-SC, 1-UC, 2-IC, 3-SC)     */
    tBool               switchAppOnOff;                /* raw value                                                           */
    tBool               switchAppOnOffState;           /* raw value                                                           */
    tBool               switchFaultClear;              /* raw value                                                           */
    tBool               switchAppReset;                /* raw value                                                           */
    tBool               readFault;                     /* Read fault status                                                   */
}userControl_t;

typedef struct
{
    AppStates       state;                  /*! raw value */
    AppEvents       event;                  /*! raw value */
    userControl_t   usrControl;				/* user action required */
    tBool			loadDefSetting;			/* load default application settings */
}driveStates_t;

typedef union
{
    tU16 R;
    struct
    {
        tU16 ADC_Instance_Error     : 1;     /* Wrong ADC instance used for conversion   */
        tU16 ADC_Channel_Error      : 1;     /* Wrong ADC channel used for conversion    */
        tU16 ADC_Trigger_Error      : 1;     /* Wrong trigger invoked conversion         */
        tU16                        : 13;    /* RESERVED                                 */
    }B;
}mcuFaultStatus_t;

typedef union
{
    tU16 R;
    struct
    {
        tU16 InitError              : 1;    /* Error during app initialization */
        tU16 CalibError             : 1;    /* Error during calibration        */
        tU16 AlignError             : 1;    /* Error during alignment          */
        tU16 RunError               : 1;    /* Error during run state          */
        tU16 FOCError               : 1;    /* Error during FOC calculation    */
        tU16 : 11;                          /* RESERVED                        */

    }B;
}stMachineFaultStatus_t;

typedef union
{
    tU16 R;
    struct
    {
        tU16 OverPhaseCCurrent      : 1;    /* OverCurrent fault flag         */
        tU16 OverPhaseBCurrent      : 1;    /* OverCurrent fault flag         */
        tU16 OverPhaseACurrent      : 1;    /* OverCurrent fault flag         */
        tU16 UnderDCBusVoltage      : 1;    /* Undervoltage fault flag        */
        tU16 OverDCBusVoltage       : 1;    /* Overvoltage fault flag         */
        tU16 : 11;                           /* RESERVED                       */
    }B;
}motorFaultStatus_t;

typedef struct
{
    tU32 speed_w_e;
    tU32 speed_n_m;
    tU32 speed_ramp;
    tU32 position;
}fm_scale_t;

/*! Application fault status user type */
typedef struct
{
	mcuFaultStatus_t 		mcu;
	tBool 					AEModule;
	stMachineFaultStatus_t	stateMachine;
	motorFaultStatus_t 		motor;
}appFaultStatus_t;    /* Application fault status user type*/

#endif /* _MOTOR_STRUCTURE */
