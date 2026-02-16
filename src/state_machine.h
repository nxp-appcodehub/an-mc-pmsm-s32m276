/*******************************************************************************
*   Project      : S32M27xEVB_PMSM_FOC_1Sh
*   Revision     : 1.1
*   RTD Version  : 4.0.0
*   Brief description  :
*   File contains declarations for the application state machine.
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
 * state_machine.h
 *
 * rev 1.0 Initial release
 * rev 1.1 removed references to App_MemMap - now placing into TCM is done in linker file
 *
 ********************************************************************************/

#ifndef _STATE_MACHINE_FRAME_H
#define _STATE_MACHINE_FRAME_H

/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
#define APP_INIT                0   /* Application states */
#define APP_CALIB               1
#define APP_ALIGNMENT           2
#define APP_START               3
#define APP_RUN                 4
#define APP_STOP                5
#define APP_FAULT               6

/*==================================================================================================
*                              STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
typedef void (*tPointerFcn)(void);  /* pointer to a function */
typedef void (*tPointerStr)(void);  /* pointer to a structure */
typedef void (*PFCN_VOID_STATES)(void); /* pointer to function */

/*------------------------------------------------------------------------*//*!
@brief  Exported Variables
*//*-------------------------------------------------------------------------*/
/* Array with pointers to the state machine functions */
extern PFCN_VOID_STATES StateTable[12][6];
/* Array with pointers to the RGB Led state functions */
extern const tPointerFcn AppStateLed[];


/*==================================================================================================
                                    FUNCTION PROTOTYPES
==================================================================================================*/
/* Application control*/
extern void StateFault(void);
extern void StateInit(void);
extern void StateReady(void);
extern void StateCalib(void);
extern void StateAlign(void);
extern void StateRun(void);

typedef enum {
    init            = 0,
    fault           = 1,
    ready           = 2,
    calib           = 3,
    align           = 4,
    run             = 5
}AppStates;         /* Application state identification user type*/

typedef enum {
    e_fault         = 0,
    e_fault_clear   = 1,
    e_init          = 2,
    e_init_done     = 3,
    e_ready         = 4,
    e_app_on        = 5,
    e_calib         = 6,
    e_calib_done    = 7,
    e_align         = 8,
    e_align_done    = 9,
    e_run           = 10,
    e_app_off       = 11
}AppEvents;         /* Application event identification user type*/

#endif //_STATE_MACHINE_FRAME_H
