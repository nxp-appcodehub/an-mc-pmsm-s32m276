/*******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.1
*   RTD Version  : 7.0.0
*   Brief description  :
*   File contains declarations for functions needed for encoder sensor processing.
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
 * pospe_sensor.h
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 *
 ********************************************************************************/
#ifndef POSPE_SENSOR_H_
#define POSPE_SENSOR_H_

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "gflib.h"
#include "amclib.h"
#include "config\PMSM_appconfig.h"
#include "Emios_Icu_Ip.h"

/*==================================================================================================
*                              STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
typedef struct
{
    tFloat    raw;   /*! raw value */
    tFloat    filt;  /*! filtered value */
}pospeValue_t;

typedef struct
{
    tFloat                              thRotMec;
    pospeValue_t                        wRotMec;
    pospeValue_t                        thRotEl;
    pospeValue_t                        thRotElk1;
    pospeValue_t                        wRotEl;
    tFloat                              thRoErr;
    pospeValue_t                        thRotMecSin;
    pospeValue_t                        thRotMecCos;
    pospeValue_t                        thRotElSin;
    pospeValue_t                        thRotElCos;
    AMCLIB_TRACK_OBSRV_T_FLT            TrackObsrv;
    tFrac32                             s32MotorPpScale;
    tFrac16                             s16MotorPpScaleShift;
    tFloat                              fltMotorPP;
    uint16_t                            counterCwOffset;
    uint16_t                            counterCcwOffset;
}encoderPospe_t;

/*==================================================================================================
                                    FUNCTION PROTOTYPES
==================================================================================================*/
extern tBool POSPE_GetPospeElEnc(encoderPospe_t *ptr);
extern tBool POSPE_ClearPospeElEnc(encoderPospe_t *ptr);

#endif /* POSPE_SENSOR_H_ */
