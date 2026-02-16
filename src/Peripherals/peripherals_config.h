/*******************************************************************************
*   Project      : S32M27xEVB_PMSM_FOC_1Sh
*   Revision     : 1.2
*   RTD Version  : 4.0.0
*   Brief description  :
*   Peripherals Configuration
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
 * peripherals_config.h
 *
 * rev 1.0 Initial release
 * rev 1.1 Removed AE W1C functions,removed references to App_MemMap - now placing into TCM is done in linker file
 * rev 1.2 Updated AEC_DPGAConfig
 *
 ********************************************************************************/


#ifndef PERIPHERALS_PERIPHERALS_CONFIG_H_
#define PERIPHERALS_PERIPHERALS_CONFIG_H_

# include "PlatformTypes.h"

#define bool	_Bool

#define INST_PORTD 	(3U)
#define INST_LPUART (0U)
#define INST_ADC0 	(0U)
#define INST_ADC1 	(1U)
#define INST_PDB0	(0U)
#define INST_PDB1	(1U)
#define INST_FTM2	(2U)
#define INST_FTM3	(3U)


void AECConfig(void);
void AEC_PMCConfig(bool VPREEXT, bool VPREINT);

void AEC_ResetConfig(void);
void AEC_HVMConfig(void);
void AEC_GDUConfig(void);
void AEC_DPGAConfig(void);
void AEC_VDDE_Enable(bool OCMonitorOn);
void AEC_VDDE_Disable(void);

#endif /* PERIPHERALS_PERIPHERALS_CONFIG_H_ */
