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


/* addtogroup ddi_bc */
/*  @{ */
/*  */
/* Copyright (c) 2004-2005 SigmaTel, Inc. */
/*  */
/* file       ddi_bc_ramp.c */
/* brief      Contains the Battery Charger current ramp controller. */
/* date       06/2005 */
/*  */
/*  This file contains Battery Charger current ramp controller. */
/*  */



/* Includes and external references */


#include "ddi_bc.h"
#include "ddi_bc_internal.h"


/* Definitions */


/*  This is the control structure for the current ramp. */

typedef struct _ddi_bc_RampControl {

	/* < The accumulated time since we last changed the actual */
	/* < current setting in the hardware. If the time between */
	/* < steps is quite short, we may have to wait for several steps */
	/* < before we can actually change the hardware setting. */
	uint32_t u32AccumulatedTime;
	uint16_t u16Target;         /* target charging current, regardless of expressibility. */
	uint16_t u16Limit           /* current limit, regardless of expressibility. */;
	uint8_t dieTempAlarm:1;     /* if we are under a die temperature alarm. */
	uint8_t batteryTempAlarm:1; /* if we are under a battery temperature alarm. */
	uint8_t ambientTempAlarm:1; /* if we are under an ambient temperature alarm. */
} ddi_bc_RampControl_t;


/* Variables */


/*  This structure contains control information for the current ramp. */

static ddi_bc_RampControl_t g_RampControl;


/*  */
void ddi_bc_RampReset()
{

	g_RampControl.u32AccumulatedTime = 0;
	g_RampControl.u16Target = 0;
	ddi_bc_RampStep(0);
}


/*  */
/* brief Set the target charging current. */
uint16_t ddi_bc_RampSetTarget(uint16_t u16Target)
{

	g_RampControl.u16Target = u16Target;

	ddi_bc_RampStep(0);

	return ddi_bc_hwExpressibleCurrent(u16Target);

}


/* [luheng] return software ultimate charging current: g_RampControl.u16Target */
uint16_t ddi_bc_RampGetTarget(void)
{
	return g_RampControl.u16Target;
}


/*  */
/* brief Set the current limit. */
/*  */
/* fntype Function */
/*  */
/*  This function sets the current limit and implements it immediately. */
/*  */
/* param[in]  u16Limit  The current limit. */
/*  */
/* retval  The expressible version of the limit. */
/*  */

uint16_t ddi_bc_RampSetLimit(uint16_t u16Limit)
{
	g_RampControl.u16Limit = u16Limit;

	/* Step the ramp. Note that we don't care if this function returns an error. */
	/* We're stepping the ramp to make sure it takes immediate effect, if */
	/* possible. But, for example, if the Battery Charger is not yet */
	/* initialized, it doesn't matter. */
	ddi_bc_RampStep(0);

	return ddi_bc_hwExpressibleCurrent(u16Limit);

}

/* [luheng] return software all-circumstance charging current limit: g_RampControl.u16Limit */
uint16_t ddi_bc_RampGetLimit(void)
{
	return g_RampControl.u16Limit;
}


/* [luheng] update die-temp, batt-temp alarm if specified by g_ddi_bc_Configuration setting */
void ddi_bc_RampUpdateAlarms()
{
	int iStepTheRamp = 0;

	/* update die temp alarm */
	if (g_ddi_bc_Configuration.monitorDieTemp) {                  /* need to monitor die temp */
		int16_t i16Low;
		int16_t i16High;

		ddi_bc_hwGetDieTemp(&i16Low, &i16High);

		if (g_RampControl.dieTempAlarm) {                         /* previously alarmed */
			if (i16High < g_ddi_bc_Configuration.u8DieTempLow) {  /* safe, release alarm */
				g_RampControl.dieTempAlarm = 0;
				iStepTheRamp = !0;
				/* printk("Battery charger: releasing die temp alarm: [%d, %d] < %d\r\n",
				       (int32_t) i16Low, (int32_t) i16High,
				       (int32_t) g_ddi_bc_Configuration.u8DieTempLow); */

			}
		} else {                                                   /* previously not alarmed */
			if (i16High >= g_ddi_bc_Configuration.u8DieTempHigh) { /* danger, alarm now */
				g_RampControl.dieTempAlarm = 1;
				iStepTheRamp = !0;
				/* printk("Battery charger: declaring die temp alarm: [%d, %d] >= %d\r\n",
				       (int32_t) i16Low, (int32_t) i16High,
				       (int32_t) g_ddi_bc_Configuration.u8DieTempLow); */
			}
		}
	}

	/* update batt temp alarm */
	if (g_ddi_bc_Configuration.monitorBatteryTemp) {                /* need monitor batt temp */
		ddi_bc_Status_t status;
		uint16_t u16Reading;

		status = ddi_bc_hwGetBatteryTemp(&u16Reading);

		if (status == DDI_BC_STATUS_SUCCESS) {
			if (g_RampControl.batteryTempAlarm) {                  /* previously alarmed */
				if (u16Reading < g_ddi_bc_Configuration.u16BatteryTempLow) {
					g_RampControl.batteryTempAlarm = 0;            /* release alarm if now below lower margin */
					iStepTheRamp = !0;
				}
			} else {                                               /* previously not alarmed */
				if (u16Reading >= g_ddi_bc_Configuration.u16BatteryTempHigh) {
					g_RampControl.batteryTempAlarm = 1;            /* start alarm if now higher than high margin */
					iStepTheRamp = !0;
				}
			}
		}
	}

	if (iStepTheRamp)
		ddi_bc_RampStep(0);
}


/*  */
/* brief Reports the state of the die temperature alarm. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the state of the die temperature alarm. */
/*  */
/* retval  The state of the die temperature alarm. */
/*  */

int ddi_bc_RampGetDieTempAlarm(void)
{
	return g_RampControl.dieTempAlarm;
}


/*  */
/* brief Reports the state of the battery temperature alarm. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the state of the battery temperature alarm. */
/*  */
/* retval  The state of the battery temperature alarm. */
/*  */

int ddi_bc_RampGetBatteryTempAlarm(void)
{
	return g_RampControl.batteryTempAlarm;
}


/*  */
/* brief Reports the state of the ambient temperature alarm. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the state of the ambient temperature alarm. */
/*  */
/* retval  The state of the ambient temperature alarm. */
/*  */

int ddi_bc_RampGetAmbientTempAlarm(void)
{
	return g_RampControl.ambientTempAlarm;
}


/* [luheng] ramp up/down charging current to our ultimate target: g_RampControl.u16Target
 * 
 *    u32Time:    IN    time interval of this func, mili sec.
 * 
 * this func is called periodically to ramp up/down charging current to our ultimate target: 'g_RampControl.u16Target'.
 *     If we need to ramp down, go to the target directly.
 *     If we already at the target, do nothing.
 *     If we need to ramp up(currently under target), we cann't go to target directly, we can only ramp up
 *         step by step at the speed specified by 'g_ddi_bc_Configuration.u16CurrentRampSlope' in unit of mA/sec.
 *
 * The ultimate target 'g_RampControl.u16Target' is also confined under below limits:
 *     g_RampControl.u16Limit                              limit at all circumstances
 *     g_ddi_bc_Configuration.u16DieTempSafeCurrent        limit under die-temp-alarm
 *     g_ddi_bc_Configuration.u16BatteryTempSafeCurrent    limit under batt-temp-alarm
 */ 
ddi_bc_Status_t ddi_bc_RampStep(uint32_t u32Time)
{

	uint16_t u16MaxNow;
	uint16_t u16Target;
	uint16_t u16Cart;
	int32_t i32Delta;

	if (g_ddi_bc_State == DDI_BC_STATE_UNINITIALIZED)
		return DDI_BC_STATUS_NOT_INITIALIZED;

	u16MaxNow = ddi_bc_hwGetMaxCurrent();   /* current max charging current hw setting */

	u16Target = g_RampControl.u16Target;    /* target should not exceeds limit(780mA) */
	if (u16Target > g_RampControl.u16Limit)
		u16Target = g_RampControl.u16Limit;

	if (    g_RampControl.dieTempAlarm      /* target should not exceeds die-temp-safe-current if under die-temp alarm */
		 && (u16Target > g_ddi_bc_Configuration.u16DieTempSafeCurrent)
	   )
		u16Target = g_ddi_bc_Configuration.u16DieTempSafeCurrent;
	if (    g_RampControl.batteryTempAlarm  /* target should not exceeds bat-temp-safe-current if under bat-temp alarm */
		 && (u16Target > g_ddi_bc_Configuration.u16BatteryTempSafeCurrent)
	   )
		u16Target = g_ddi_bc_Configuration.u16BatteryTempSafeCurrent;

	/* now 'u16Target' is our ultimate taregt, convert it to expressible */
	u16Target = ddi_bc_hwExpressibleCurrent(u16Target);

	i32Delta = ((int32_t) u16Target) - ((int32_t) u16MaxNow);

	if (i32Delta == 0) {                    /* current == target, no need to ramp */
		g_RampControl.u32AccumulatedTime = 0;
		return DDI_BC_STATUS_SUCCESS;
	}

	if (i32Delta < 0) {                     /* to ramp \|/, go to target directly */
		ddi_bc_hwSetMaxCurrent(u16Target);
		ddi_bc_hwSetChargerPower(u16Target != 0);
		g_RampControl.u32AccumulatedTime = 0;
		return DDI_BC_STATUS_SUCCESS;
	}

                                            /* to ramp /|\ */
	/* we're to "buy" more current by "spending" time, sum up all our deposits */
	u32Time += g_RampControl.u32AccumulatedTime;

	/* 'u16Cart' is what we can buy with our deposits, non-expressible */
	u16Cart = (g_ddi_bc_Configuration.u16CurrentRampSlope * u32Time) / 1000;

	if ((u16MaxNow + u16Cart) < u16Target) { /* can't buy ultimate target, adjust temp target to what we can */
		u16Target = ddi_bc_hwExpressibleCurrent(u16MaxNow + u16Cart);

		if (u16Target == u16MaxNow) {        /* what we can buy is non-expressible, deposit our sum */
			g_RampControl.u32AccumulatedTime = u32Time;
			return DDI_BC_STATUS_SUCCESS;
		} /* else, 'u16Target' is what we can buy now, expressible */
	} /* else, 'u16Target' is the ultimate target and we can affort to buy it, expressible */

	/* now, the 'u16Tqarget' is what we can buy now in any case, consume all our deposits to buy it */
	ddi_bc_hwSetMaxCurrent(u16Target);       /* buy what we can */
	ddi_bc_hwSetChargerPower(u16Target != 0);

	g_RampControl.u32AccumulatedTime = 0;    /* dump all our deposits */

	return DDI_BC_STATUS_SUCCESS;
}


/* End of file */

/*  @} */
