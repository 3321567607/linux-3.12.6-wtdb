/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include "ddi_bc_internal.h"


/* file       ddi_bc_init.c */
/* brief      Contains the Battery Charger initialization function. */
/* date       06/2005 */
/*  */
/*  This file contains Battery Charger initialization function. */
/*  */



/* Includes and external references */
#include "ddi_bc.h"
#include "ddi_bc_internal.h"


/* [luheng] initialize charger, hw-charger is turned off, charger-state is DISABLED */
ddi_bc_Status_t ddi_bc_Init(ddi_bc_Cfg_t *pCfg)
{
	if (g_ddi_bc_State != DDI_BC_STATE_UNINITIALIZED)
		return DDI_BC_STATUS_ALREADY_INITIALIZED;

	if (pCfg->u16ChargingVoltage != DDI_BC_LIION_CHARGING_VOLTAGE)
		return DDI_BC_STATUS_CFG_BAD_CHARGING_VOLTAGE;

	if (pCfg->u8DieTempChannel > 7)
		return DDI_BC_STATUS_CFG_BAD_BATTERY_TEMP_CHANNEL;

	pCfg->u16ChargingThresholdCurrent = ddi_power_ExpressibleCurrent(pCfg->u16ChargingThresholdCurrent);
	pCfg->u16BatteryTempSafeCurrent   = ddi_power_ExpressibleCurrent(pCfg->u16BatteryTempSafeCurrent);
	pCfg->u16DieTempSafeCurrent       = ddi_power_ExpressibleCurrent(pCfg->u16DieTempSafeCurrent);

	g_ddi_bc_Configuration = *pCfg;

	ddi_power_SetChargerPowered(0); /* turn off charger. why???? */

	ddi_bc_RampReset();

	g_ddi_bc_State = DDI_BC_STATE_DISABLED;

	return DDI_BC_STATUS_SUCCESS;

}


/* End of file */

/*  @} */
