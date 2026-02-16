/*******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.2
*   RTD Version  : 7.0.0
*   Brief description  :
*   File contains declarations for functions needed for inverter voltage control.
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
 * actuate_S32m.h
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 * rev 1.2 Added support for RTD 7.0.0
 *
 ********************************************************************************/
/*
 * actuate_S32k.h
 *
 *  Created on: July 18, 2024
 *  Author: nxf46115
 *
 *  rev0_0_1 (date: July 18, 2024) Initial release (nxf46115)
 */

#ifndef _ACTUATE_S32K_H_
#define _ACTUATE_S32K_H_


#define DOUBLE_SW_ADAPTIVE 	1

/*==================================================================================================
*                                        INCLUDE FILES
==================================================================================================*/
#include "Emios_Pwm_Ip.h"
#include "gflib.h"
#include "gdflib.h"
#include "gmclib.h"
//#include "Lcu_Ip.h"

/*==================================================================================================
                                     FUNCTION PROTOTYPES
==================================================================================================*/
extern tBool ACTUATE_EnableOutput(void);
extern tBool ACTUATE_DisableOutput(void);
extern tBool    ACTUATE_SetDutycycle(SWLIBS_3Syst_FLT *fltpwm, tU16 sector);

#endif /* _ACTUATES_S32K_H_ */
