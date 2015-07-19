/*
 * Copyright (C) 2010 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */


/* Includes */


#include <linux/kernel.h>
#include "ddi_bc_internal.h"


/* Variables */


/* This structure holds the current Battery Charger configuration. */

ddi_bc_Cfg_t g_ddi_bc_Configuration;

extern uint32_t g_ddi_bc_u32StateTimer;
extern ddi_bc_BrokenReason_t ddi_bc_gBrokenReason;
extern bool bRestartChargeCycle;


/* Code */




/* brief Shut down the Battery Charger. */

/* fntype Function */

/* This function immediately shuts down the Battery Charger hardware and */
/* returns the state machine to the Uninitialized state. Use this function to */
/* safely mummify the battery charger before retiring it from memory. */


void ddi_bc_ShutDown()
{

	/* -------------------------------------------------------------------------- */
	/* Reset the current ramp. */
	/* -------------------------------------------------------------------------- */

	ddi_bc_RampReset();

	/* -------------------------------------------------------------------------- */
	/* Move to the Uninitialized state. */
	/* -------------------------------------------------------------------------- */

	g_ddi_bc_State = DDI_BC_STATE_UNINITIALIZED;

}

/* [luheng] call state handler specified by current state 'g_ddi_bc_State' */
ddi_bc_Status_t ddi_bc_StateMachine()
{
	int ret, oldstate;

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED)
		return DDI_BC_STATUS_NOT_INITIALIZED;

	oldstate = g_ddi_bc_State;
	ret = (stateFunctionTable[g_ddi_bc_State] ());
	if (oldstate != g_ddi_bc_State)
		pr_debug("Charger: transit from state %s to %s\n",
		    ddi_bc_state_names[oldstate], ddi_bc_state_names[g_ddi_bc_State]);
	return ret;

}



/* brief Get the Battery Charger's current state. */

/* fntype Function */

/* This function returns the current state. */

/* retval The current state. */


ddi_bc_State_t ddi_bc_GetState()
{
	/* -------------------------------------------------------------------------- */
	/* Return the current state. */
	/* -------------------------------------------------------------------------- */

	return g_ddi_bc_State;

}



/* brief Disable the Battery Charger. */

/* fntype Function */

/* This function forces the Battery Charger into the Disabled state. */

/* retval DDI_BC_STATUS_SUCCESS          If all goes well */
/* retval DDI_BC_STATUS_NOT_INITIALIZED  If the Battery Charger is not yet */
/*                                        initialized. */
ddi_bc_Status_t ddi_bc_SetDisable()
{
	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED)
		return DDI_BC_STATUS_NOT_INITIALIZED;

	if (g_ddi_bc_State == DDI_BC_STATE_BROKEN)
		return DDI_BC_STATUS_BROKEN;

	ddi_bc_RampReset();

	g_ddi_bc_u32StateTimer = 0;

	g_ddi_bc_State = DDI_BC_STATE_DISABLED;

	return DDI_BC_STATUS_SUCCESS;

}


/*
 * set:
 *     g_ddi_bc_u32StateTimer = 0;
 *     g_ddi_bc_State = DDI_BC_STATE_WAITING_TO_CHARGE;
 *
 * before this calling, g_ddi_bc_State should be DDI_BC_STATE_DISABLED, otherwise not set
 */
ddi_bc_Status_t ddi_bc_SetEnable()
{
	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED)
		return DDI_BC_STATUS_NOT_INITIALIZED;

	if (g_ddi_bc_State != DDI_BC_STATE_DISABLED)
		return DDI_BC_STATUS_NOT_DISABLED;

	g_ddi_bc_u32StateTimer = 0;

	g_ddi_bc_State = DDI_BC_STATE_WAITING_TO_CHARGE;

	return DDI_BC_STATUS_SUCCESS;

}



/* brief Report the reason for being in the broken state */

/* fntype Function */


/* retval  ddi_bc_BrokenReason_t enumeration */


ddi_bc_BrokenReason_t ddi_bc_GetBrokenReason(void)
{
	return ddi_bc_gBrokenReason;
}


/* End of file */

/* @} */
