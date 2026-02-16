/*******************************************************************************
*   Project      : FRDM_A_S32M276_PMSM_FOC_1Sh
*   Revision     : 1.2
*   RTD Version  : 7.0.0
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
 * peripherals_config.c
 *
 * rev 1.0 Initial release
 * rev 1.1 Updated AEC_DPGAConfig, removed AE W1C functions,removed references to App_MemMap - now placing into TCM is done in linker file
 * rev 1.2 Updated AEC_PMCConfig - now the PMC AE monitor settings done in configuration tool propagate into AE PMC Monitor register upon function call
 * 		   Updated AEC_DPGAConfig - now the voltage deector is configured through the graphical config tool, notmanually in the API
 *
 ********************************************************************************/


#include "peripherals_config.h"

#include "Clock_Ip.h"
#include "IntCtrl_Ip.h"
#include "Trgmux_Ip.h"
#include "Lpuart_Uart_Ip.h"
#include "Power_Ip.h"
#include "Lpspi_Ip.h"
#include "AEC_Ip_Hw_Access.h"
#include "Dpga_Ip.h"
#include "CDD_Gdu.h"
#include "Hvm_Ip.h"
#include "Power_Ip_PBcfg.h"
#include "S32M27x_HVI_AE.h"

/******************************************************************************
 *
 * Function Name : AECConfig
 * Description   : AEC initialization.
 *
 ******************************************************************************/
void AECConfig(void)
{
	Aec_Ip_Init(&Aec_Ip_aConfigPB);
}

/******************************************************************************
 *
 * Function Name : AEC_DPGAConfig
 * Description   : DPGA initialization.
 *
 ******************************************************************************/
void AEC_DPGAConfig(void)
{
	Dpga_Ip_Init(0, &Dpga_Ip_Config);
}

/******************************************************************************
 *
 * Function Name : AEC_GDUConfig
 * Description   : GDU initialization.
 ******************************************************************************/
void AEC_GDUConfig(void)
{
	Gdu_Init(&GDU_CONFIG);
}

/******************************************************************************
 *
 * Function Name : AEC_ResetConfig
 * Description   : AE modules enable - write 1 to the respective register bits.
 *
 ******************************************************************************/
void AEC_ResetConfig(void)
{
	Power_Ip_AecResetConfig(&Power_Ip_HwIPsConfigPB);
}

/******************************************************************************
*
 * Function Name : AEC_PMCConfig
 * Description   : PMC init - configure the respective register bits.
 * RTD limitation - need to be done manually, not through the graphical tool
 * External or internal VPRE ballast transistor can be chosen by VPREEXT, VPREINT
 *
 ******************************************************************************/
void AEC_PMCConfig(bool VPREEXT, bool VPREINT )
{

		uint32 temp =0;

		Power_Ip_PmcAeConfig(&Power_Ip_HwIPsConfigPB);
		Aec_Ip_SpiRead((uint32)&IP_PMC_AE->CONFIG, 32, &temp);

	    temp = (temp & ~PMC_AE_CONFIG_VPREEXT_MASK & ~PMC_AE_CONFIG_VPREINT_MASK) |
	    			(PMC_AE_CONFIG_VPREEXT(VPREEXT)	|
	    			PMC_AE_CONFIG_VPREINT(VPREINT));

	    Aec_Ip_SpiWrite((uint32)&IP_PMC_AE->CONFIG, 32, temp);
	    Aec_Ip_SpiRead((uint32)&IP_PMC_AE->CONFIG, 32, &temp);

}

/******************************************************************************
 *
 * Function Name : AEC_HVMConfig
 * Description   : HVM init
 *
 ******************************************************************************/
void AEC_HVMConfig(void)
{
	Hvm_Ip_Init(&Hvm_Ip_aConfigPB);
}

/******************************************************************************
 *
 * Function Name : AEC_VDDE_Enable
 * Description   : Enable VDDE sensor supply, OCMonitorOn enables overcurrent detection
 *
 ******************************************************************************/
void AEC_VDDE_Enable(bool OCMonitorOn)
{
	uint32 temp;
	Aec_Ip_SpiRead((uint32_t) &IP_AEC_AE->IO_FUNCMUX_CFG, 32, &temp);
	temp = (temp & ~(AEC_AE_IO_FUNCMUX_CFG_VDDE_SEL_MASK) &
			       ~(AEC_AE_IO_FUNCMUX_CFG_VDDE_OCD_EN_MASK)&
			       ~(AEC_AE_IO_FUNCMUX_CFG_VDDE_DRV_MASK)) |
		   AEC_AE_IO_FUNCMUX_CFG_VDDE_SEL(1) |
		   AEC_AE_IO_FUNCMUX_CFG_VDDE_OCD_EN(OCMonitorOn)|
		   AEC_AE_IO_FUNCMUX_CFG_VDDE_DRV(1);
	Aec_Ip_SpiWrite((uint32_t) &IP_AEC_AE->IO_FUNCMUX_CFG, 32, temp);

	if (OCMonitorOn) //Enable OCMonitor event
	{
		Aec_Ip_SpiRead((uint32_t) &IP_AEC_AE->EVENTS_ENABLE, 16, &temp);
		temp = (temp & ~(AEC_AE_EVENTS_ENABLE_OCD_VDDE_EN_MASK)) |
				AEC_AE_EVENTS_ENABLE_OCD_VDDE_EN(OCMonitorOn);

		Aec_Ip_SpiWrite((uint32_t)&IP_AEC_AE->EVENTS_ENABLE, 16, temp);

	}
}

/******************************************************************************
 *
 * Function Name : AEC_VDDE_Disable
 * Description   : Disable VDDE sensor supply, disable OCMonitor
 *
 ******************************************************************************/
void AEC_VDDE_Disable(void)
{
	uint32 temp;
	Aec_Ip_SpiRead((uint32_t) &IP_AEC_AE->IO_FUNCMUX_CFG, 32, &temp);
	temp = temp &
		   ~(AEC_AE_IO_FUNCMUX_CFG_VDDE_SEL_MASK) &
		   ~(AEC_AE_IO_FUNCMUX_CFG_VDDE_OCD_EN_MASK)&
		   ~(AEC_AE_IO_FUNCMUX_CFG_VDDE_DRV_MASK);
	Aec_Ip_SpiWrite((uint32_t) &IP_AEC_AE->IO_FUNCMUX_CFG, 32, temp);

	Aec_Ip_SpiRead((uint32_t) &IP_AEC_AE->EVENTS_ENABLE, 16, &temp);
			temp = (temp & ~(AEC_AE_EVENTS_ENABLE_OCD_VDDE_EN_MASK));
			Aec_Ip_SpiWrite((uint32_t) &IP_AEC_AE->IO_FUNCMUX_CFG, 16, temp);	//Disable OCMonitor event

}

/******************************************************************************/
