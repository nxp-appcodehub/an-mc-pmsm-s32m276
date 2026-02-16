/*******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.1
*   RTD Version  : 7.0.0
*   Brief description  :
*   File contains definition of functions needed for encoder sensor processing.
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
 * pospe_sensor.c
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 *
 ********************************************************************************/
/******************************************************************************
| Includes
-----------------------------------------------------------------------------*/
#include "pospe_sensor.h"

/*==================================================================================================
*                                      GLOBAL VARIABLES
==================================================================================================*/
volatile uint16_t counterCW,counterCCW;
tFloat Absolut_position;
/*==================================================================================================
*                                    FUNCTIONS DEFINITION
==================================================================================================*/

/*FUNCTION**********************************************************************
 *
 * Function Name : POSPE_GetPospeElEnc
 * Description   : Reads encoder edges to get rotor position and speed.
 *
 *END**************************************************************************/
tBool POSPE_GetPospeElEnc(encoderPospe_t *ptr)
{
    tBool statusPass;
    statusPass = TRUE;

    static tFrac32 f32ThRotMe, f32ThRotEl, f32ThRotMe_EMI;

    /* read encoder edges to get mechanical position */
    counterCW   = (uint16_t)((Emios_Icu_Ip_GetEdgeNumbers(0U, 6U))- ptr->counterCwOffset);     /* CW  counter */
    counterCCW  = (uint16_t)((Emios_Icu_Ip_GetEdgeNumbers(0U, 7U))- ptr->counterCcwOffset);    /* CCW counter */
    Absolut_position = (tFloat)(((uint16_t)(counterCW-counterCCW))&(uint16_t)(4*ENC_PULSES-1));
    /* Mechanical rotor position acquired from EMIOS0 - in fix point <-1,1) */
    f32ThRotMe_EMI = MLIB_ConvertPU_F32FLT(MLIB_Sub(MLIB_Div(Absolut_position, (2*ENC_PULSES)),1.0));

    AMCLIB_TrackObsrv(ptr->thRoErr, &(ptr->thRotMec), &(ptr->wRotMec.raw), &(ptr->TrackObsrv));

    /* Mechanical and electrical angular speed calculation - float */
    ptr->wRotEl.raw = MLIB_Mul(ptr->wRotMec.raw, MOTOR_PP);

    /* Estimated Mechanical rotor position - wrapping into the range <-pi,pi> - float */
    if (ptr->thRotMec > FLOAT_PI)
    {
        ptr->TrackObsrv.pParamInteg.fltState = MLIB_Sub(ptr->TrackObsrv.pParamInteg.fltState, FLOAT_2_PI);
        ptr->thRotMec = ptr->TrackObsrv.pParamInteg.fltState;
    }

    if (ptr->thRotMec < -FLOAT_PI)
    {
        ptr->TrackObsrv.pParamInteg.fltState = MLIB_Add(ptr->TrackObsrv.pParamInteg.fltState, FLOAT_2_PI);
        ptr->thRotMec = ptr->TrackObsrv.pParamInteg.fltState;
    }

    /* Mechanical rotor position - transformation in to the range <-1,1> - fix point */
    f32ThRotMe = MLIB_ConvertPU_F32FLT(MLIB_Div(ptr->thRotMec, FLOAT_PI));
    /* Rotor position - transformation from mechanical to electrical  - fix point */
    f32ThRotEl = MLIB_ShL_F32(MLIB_MulSat_F32(f32ThRotMe, MOTOR_PP_GAIN), MOTOR_PP_SHIFT);

    /* Electrical rotor position - <-pi, pi> range - floating point */
    ptr->thRotEl.filt = MLIB_Mul(MLIB_ConvertPU_FLTF32(f32ThRotEl), FLOAT_PI);

    /* Theta error = (Theta_eTimer - Theta_est), calculated in fix point due to the precision */
    ptr->thRoErr = MLIB_Mul(MLIB_ConvertPU_FLTF32(MLIB_Sub_F32(f32ThRotMe_EMI, f32ThRotMe)), FLOAT_PI);

    return (statusPass);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : POSPE_ClearPospeElEnc
 * Description   : Clears internal variables.
 *
 *END**************************************************************************/
tBool POSPE_ClearPospeElEnc(encoderPospe_t *ptr)
{
    tBool statusPass;
    statusPass = TRUE;

    /* Encoder's mechanical position and speed init */
    ptr->thRotMec       = 0.0F;
    ptr->thRoErr        = 0.0F;
    ptr->wRotMec.raw    = 0.0F;
    ptr->wRotMec.filt   = 0.0F;

    /* Encoder's electrical position and speed init */
    ptr->thRotEl.raw    = 0.0F;
    ptr->thRotEl.filt   = 0.0F;
    ptr->wRotEl.raw     = 0.0F;
    ptr->wRotEl.filt    = 0.0F;

    AMCLIB_TrackObsrvInit(&(ptr->TrackObsrv));

    return (statusPass);
}
