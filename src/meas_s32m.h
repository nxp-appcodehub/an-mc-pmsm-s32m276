/******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.1
*   RTD Version  : 7.0.0
*   Brief description  :
*   File contains declarations for functions related to analog quantities capturing.
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
 * meas_s32m.h
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 *
 ********************************************************************************/

#ifndef _MEAS_S32K_H_
#define _MEAS_S32K_H_

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "Adc_Sar_Ip.h"
#include "Bctu_Ip.h"
#include "config\PMSM_appconfig.h"
#include "gdflib.h"
#include "gflib.h"
#include "gmclib.h"

/******************************************************************************
| Defines and macros            (scope: module-local)
-----------------------------------------------------------------------------*/
#define DPGA_OFFSET_COMP	1 // Activates runtime DPGA offset compensation

/*==================================================================================================
*                              STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

typedef struct
{
    tFloat    raw;   /*! raw value */
    tFloat    filt;  /*! filtered value */
}meas_t;

/*------------------------------------------------------------------------*//*!
@brief  Structure containing measured raw values
*//*-------------------------------------------------------------------------*/
typedef struct
{
    meas_t    fltPhA;     /* Measured phase A current */
    meas_t    fltPhB;     /* Measured phase B current */
    meas_t    fltPhC;     /* Measured phase C current */
    meas_t    fltIdcb;    /* Measured on DC bus current*/
    meas_t    fltUdcb;    /* Measured DC bus voltage */
    meas_t    fltTemp;    /* Measured temperature */
}measResult_t;

typedef struct
{
    tFloat                    fltOffset;   /* raw value */
    GDFLIB_FILTER_MA_T_FLT    filtParam;   /* filter parameters */
}offsetBasic_t;


/*------------------------------------------------------------------------*//*!
@brief  Structure containing variables for software DC offset calibration.
*//*-------------------------------------------------------------------------*/
typedef struct
{
    offsetBasic_t    fltPhA;         /* DC offset measured on phase A current */
    offsetBasic_t    fltPhB;         /* DC offset measured on phase B current */
    offsetBasic_t    fltPhC;         /* DC offset measured on phase C current */
    offsetBasic_t    fltIdcb;        /* DC offset measured on DC bus current */
    offsetBasic_t    fltUdcb;        /* DC offset measured on DC bus voltage */
    offsetBasic_t    fltTemp;        /* DC offset measured on temperature */
}offset_t;


/*------------------------------------------------------------------------*//*!
@brief  Structure containing variables to configure Calibration on the application
        level.
*//*-------------------------------------------------------------------------*/
typedef struct
{
    tU16     u16CalibSamples; /* Number of samples taken for calibration */
}calibParam_t;

/*------------------------------------------------------------------------*//*!
@brief  Union containing module operation flags.
*//*-------------------------------------------------------------------------*/
typedef union
{
    tU16 R;
    struct {
        tU16               :14; /* RESERVED */
        tU16 calibDone     :1;  /* DC offset calibration done */
        tU16 calibInitDone :1;  /* initial setup for DC offset calibration done */
    } B;
}calibFlags_t;

/*------------------------------------------------------------------------*//*!
@brief  Module structure containing measurement related variables.
*//*-------------------------------------------------------------------------*/
typedef struct
{
    measResult_t        measured;
    offset_t            offset;
    calibParam_t        param;
    calibFlags_t        flag;
    tU16                calibCntr;
}measModule_t;

/*==================================================================================================
                                     FUNCTION PROTOTYPES
==================================================================================================*/
extern tBool MEAS_Clear(measModule_t *ptr);
extern tBool MEAS_CalibCurrentSense(measModule_t *ptr, tU16 svmSector);
extern tBool MEAS_Get3PhCurrent(measModule_t *ptr, SWLIBS_3Syst_FLT *i, tU16 svmSector);
extern tBool MEAS_GetUdcVoltage(measModule_t *ptr, GDFLIB_FILTER_MA_T *uDcbFilter);
extern tBool MEAS_SaveAdcRawResult(void);

#endif /* _MEAS_S32K_H_ */
