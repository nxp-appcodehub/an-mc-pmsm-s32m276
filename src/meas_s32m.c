/******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.1
*   RTD Version  : 7.0.0
*   Brief description  :
*   File contains definition of functions related to analog quantities capturing.
*
*   Copyright 2024 NXP
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
 * meas_s32m.c
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 *
 ********************************************************************************/
/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include <meas_s32m.h>
#include "motor_structure.h"

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/
#define I_MAXDC                           I_MAX
#define GAIN_CNST_I 					  MLIB_Div(MLIB_Mul(I_MAX, 2.0F), 16383.0F)

/*==================================================================================================
*                                      GLOBAL TYPEDEFS AND STRUCTURES
==================================================================================================*/
typedef struct ADC_RAW_DATA_T
{
    SWLIBS_2Syst_FLT  ph1;
    SWLIBS_2Syst_FLT  ph2;
    tFloat            dcOffset;
}ADC_RAW_DATA_T;

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
uint16_t adcRawResultArray[6] = {0x00001FFF, 0x00001FFF, 0x00001FFF, 0, 0x00001FFF,0x00001FFF};
tFloat MeasOffset = 0.0F;
uint16_t CalibCurrentA = 0x00001FFF;

extern volatile Bctu_Ip_FifoResultType measuredValues[16];
extern driveStates_t     cntrState;

/*==================================================================================================
*                                    FUNCTIONS DEFINITION
==================================================================================================*/
void GetAdcRawValues(measModule_t *ptr, ADC_RAW_DATA_T *rawData);


/*FUNCTION**********************************************************************
 *
 * Function Name : MEAS_Clear
 * Description   : Clears variables needed for both calibration as well as run time measurement.
 *                 It is not intended to be executed when application is in run mode.
 *
 *END**************************************************************************/
tBool MEAS_Clear(measModule_t *ptr)
{
    ptr->measured.fltPhA.filt   = 0.0F;
    ptr->measured.fltPhA.raw    = 0.0F;
    ptr->measured.fltPhB.filt   = 0.0F;
    ptr->measured.fltPhB.raw    = 0.0F;
    ptr->measured.fltPhC.filt   = 0.0F;
    ptr->measured.fltPhC.raw    = 0.0F;
    ptr->measured.fltUdcb.filt  = 0.0F;
    ptr->measured.fltUdcb.raw   = 0.0F;
    ptr->measured.fltTemp.filt  = 0.0F;
    ptr->measured.fltTemp.raw   = 0.0F;
    ptr->measured.fltIdcb.filt  = 0.0F;
    ptr->measured.fltIdcb.raw   = 0.0F;
    ptr->offset.fltIdcb.fltOffset  = I_MAX;
    ptr->flag.R                 = 0;
    ptr->flag.B.calibInitDone   = 0;
    ptr->flag.B.calibDone       = 0;

    return (TRUE);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MEAS_CalibCurrentSense
 * Description   : This function performs offset calibration for 3 phase current measurement
 *				   during the calibration phase of the application. It is not intended to be
 *				   executed when application is in run mode.
 *
 *
 *END**************************************************************************/
tBool MEAS_CalibCurrentSense(measModule_t *ptr, tU16 svmSector)
{
    ADC_RAW_DATA_T  rawValues;

    if (!(ptr->flag.B.calibInitDone))
    {
        ptr->calibCntr = 1 << (ptr->param.u16CalibSamples + 4); /* +4 in order to accommodate settling time of the filter */

        ptr->offset.fltIdcb.filtParam.fltAcc = I_MAX;
        ptr->flag.B.calibDone       = 0;
        ptr->flag.B.calibInitDone   = 1;
    }

    if (!(ptr->flag.B.calibDone))
    {
        GetAdcRawValues(ptr, &rawValues);
        /* --------------------------------------------------------------
         * DC Bus Current - DC offset data filtering using MA recursive filter
         * ------------------------------------------------------------ */
        ptr->offset.fltIdcb.fltOffset = GDFLIB_FilterMA(CalibCurrentA, &ptr->offset.fltIdcb.filtParam);

        if ((--ptr->calibCntr) <= 0)
        {
            ptr->flag.B.calibDone = 1;    /* end of DC offset calibration */
        }
    }
    return (ptr->flag.B.calibDone);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MEAS_Get3PhCurrent
 * Description   : This function performs measurement of three phase currents from
            	   single shunt resistor. Because a non-zero length PWM pulse width
            	   is required for successful current sample, this approach can not
            	   be utilized up to full PWM dutycycle.
 *
 *
 *END**************************************************************************/
tBool MEAS_Get3PhCurrent(measModule_t *ptr, SWLIBS_3Syst_FLT *i,  tU16 svmSector)
{
    ADC_RAW_DATA_T      rawValues;
    SWLIBS_3Syst_FLT    raw;
    SWLIBS_2Syst_FLT    avgValue;
    tFloat isub = 0.0F;


    GetAdcRawValues(ptr, &rawValues);

    avgValue.fltArg1    = rawValues.ph1.fltArg1;
    avgValue.fltArg2    = rawValues.ph2.fltArg1;

    switch (svmSector)
    {
    case 1:
        /* direct sensing of U, -W, calculation of V */
        raw.fltArg1 = avgValue.fltArg1;
        raw.fltArg3 = avgValue.fltArg2;

        i->fltArg1 = raw.fltArg1;
        i->fltArg3 = MLIB_Neg_FLT(raw.fltArg3);
        i->fltArg2 = MLIB_Sub(MLIB_Neg_FLT(i->fltArg3),i->fltArg1);

        break;

    case 2:
        /* direct sensing of V, -W, calculation of U */
        raw.fltArg2 = avgValue.fltArg1;
        raw.fltArg3 = avgValue.fltArg2;

        i->fltArg2 = raw.fltArg2;
        i->fltArg3 = MLIB_Neg_FLT(raw.fltArg3);
        i->fltArg1 = MLIB_Sub(MLIB_Neg_FLT(i->fltArg2),i->fltArg3);

       break;

    case 3:
        /* direct sensing of V, -U, calculation of W */
        raw.fltArg2 = avgValue.fltArg1;
        raw.fltArg1 = avgValue.fltArg2;

        i->fltArg2 = raw.fltArg2;
        i->fltArg1 = MLIB_Neg_FLT(raw.fltArg1);
        i->fltArg3 = MLIB_Sub(MLIB_Neg_FLT(i->fltArg2),i->fltArg1);

        break;

    case 4:
        /* direct sensing of W, -U, calculation of V */
        raw.fltArg3 = avgValue.fltArg1;
        raw.fltArg1 = avgValue.fltArg2;

        i->fltArg3 = raw.fltArg3;
        i->fltArg1 = MLIB_Neg_FLT(raw.fltArg1);
        i->fltArg2 = MLIB_Sub(MLIB_Neg_FLT(i->fltArg3),i->fltArg1);

        break;

    case 5:
        /* direct sensing of W, -V, calculation of U */
        raw.fltArg3 = avgValue.fltArg1;
        raw.fltArg2 = avgValue.fltArg2;

        i->fltArg3 = raw.fltArg3;
        i->fltArg2 = MLIB_Neg_FLT(raw.fltArg2);
        i->fltArg1 = MLIB_Sub(MLIB_Neg_FLT(i->fltArg3),i->fltArg2);
        break;

    case 6:
        /* direct sensing of U, -V, calculation of W */
        raw.fltArg1 = avgValue.fltArg1;
        raw.fltArg2 = avgValue.fltArg2;

        i->fltArg1 = raw.fltArg1;
        i->fltArg2 = MLIB_Neg_FLT(raw.fltArg2);
        i->fltArg3 = MLIB_Sub(MLIB_Neg_FLT(i->fltArg1),i->fltArg2);

        break;

    default:
        break;
    }

      return (TRUE);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MEAS_GetUdcVoltage
 * Description   : This function performs measurement of DCBus Voltage.
 *
 *END**************************************************************************/
tBool MEAS_GetUdcVoltage(measModule_t *ptr, GDFLIB_FILTER_MA_T *uDcbFilter)
{
    uint16_t DCBus_Voltage= 0;
    DCBus_Voltage = adcRawResultArray[3];
    ptr->measured.fltUdcb.raw = MLIB_Mul(((tFloat)MLIB_Div((tFloat)(DCBus_Voltage & 0x00003FFF), (tFloat)0x00003FFF)), U_DCB_MAX);
    ptr->measured.fltUdcb.filt  = GDFLIB_FilterMA(ptr->measured.fltUdcb.raw, uDcbFilter);

    return (TRUE);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : MEAS_SaveAdcRawResult
 * Description   : This function performs read ADC results FIFO registers.
 *
 *END**************************************************************************/
tBool MEAS_SaveAdcRawResult(void)
{
    adcRawResultArray[0] =  Bctu_Ip_GetConvData(0, 1);
    adcRawResultArray[1] =  measuredValues[0].AdcData;
    adcRawResultArray[2] =  measuredValues[1].AdcData;
    adcRawResultArray[3] =  measuredValues[2].AdcData;
    adcRawResultArray[4] =  measuredValues[3].AdcData;
    adcRawResultArray[5] =  measuredValues[4].AdcData;

    return (TRUE);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : GetAdcRawValues
 * Description   : This function performs measurement of IPH1 and IPH2.
 *
 *END**************************************************************************/
void GetAdcRawValues(measModule_t *ptr, ADC_RAW_DATA_T *rawData)
{
    tFloat PhaseCurrent1, PhaseCurrent2;


    PhaseCurrent1 = MLIB_Mul((tFloat)((adcRawResultArray[1] + adcRawResultArray[5])>>1), GAIN_CNST_I);
    PhaseCurrent2 = MLIB_Mul((tFloat)((adcRawResultArray[2] + adcRawResultArray[4])>>1), GAIN_CNST_I);

    MeasOffset = MLIB_Mul((tFloat)adcRawResultArray[0], GAIN_CNST_I);
    if ((DPGA_OFFSET_COMP) && (ptr->flag.B.calibDone)) ptr->offset.fltIdcb.fltOffset = MeasOffset;

#if	(DPGA_OFFSET_COMP == 1)    
	CalibCurrentA = MeasOffset;
#else
	CalibCurrentA = PhaseCurrent1;
#endif

    rawData->ph1.fltArg1 = MLIB_Sub(PhaseCurrent1, ptr->offset.fltIdcb.fltOffset);
    rawData->ph2.fltArg1 = MLIB_Sub(PhaseCurrent2, ptr->offset.fltIdcb.fltOffset);


}
