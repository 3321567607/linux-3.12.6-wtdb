

#include "ddi_bc.h"
#include "ddi_bc_internal.h"

#include <linux/delay.h>


/* Definitions */

#define COND_CURR (g_ddi_bc_Configuration.u16ConditioningCurrent)
#define CHRG_CURR (g_ddi_bc_Configuration.u16ChargingCurrent)
#define CHRG_THSH (g_ddi_bc_Configuration.u16ChargingThresholdCurrent)

#define TransitionToWaitingToCharge() transtate(DDI_BC_STATE_WAITING_TO_CHARGE,         0,         0)
#define TransitionToConditioning()    transtate(DDI_BC_STATE_CONDITIONING,      COND_CURR,         0)
#define TransitionToToppingOff()      transtate(DDI_BC_STATE_TOPPING_OFF,       CHRG_CURR,         0)
#define TransitionToCharging()        transtate(DDI_BC_STATE_CHARGING,          CHRG_CURR, CHRG_THSH)
#define TransitionToBroken()          transtate(DDI_BC_STATE_BROKEN,                    0,         0)

#define LOOP_INTV (g_ddi_bc_Configuration.u32StateMachinePeriod)

ddi_bc_State_t g_ddi_bc_State = DDI_BC_STATE_UNINITIALIZED;

/* This table contains pointers to the functions that implement states. The */
/* table is indexed by state. Note that it's critically important for this */
/* table to agree with the state enumeration in ddi_bc.h. */
static ddi_bc_Status_t ddi_bc_Uninitialized(void);
static ddi_bc_Status_t ddi_bc_Broken(void);
static ddi_bc_Status_t ddi_bc_Disabled(void);
static ddi_bc_Status_t ddi_bc_WaitingToCharge(void);
static ddi_bc_Status_t ddi_bc_Conditioning(void);
static ddi_bc_Status_t ddi_bc_Charging(void);
static ddi_bc_Status_t ddi_bc_ToppingOff(void);

/* state machine handler, no handler for DDI_BC_STATE_DCDC_MODE_WAITING_TO_CHARGE??? */
ddi_bc_Status_t(*const (stateFunctionTable[])) (void) = {
    ddi_bc_Uninitialized,   /* DDI_BC_STATE_UNINITIALIZED */
    ddi_bc_Broken,          /* DDI_BC_STATE_BROKEN */
    ddi_bc_Disabled,        /* DDI_BC_STATE_DISABLED */
    ddi_bc_WaitingToCharge, /* DDI_BC_STATE_WAITING_TO_CHARGE */
    ddi_bc_Conditioning,    /* DDI_BC_STATE_CONDITIONING */
    ddi_bc_Charging,        /* DDI_BC_STATE_CHARGING */
    ddi_bc_ToppingOff       /* DDI_BC_STATE_TOPPING_OFF */
};

/* state name */
char *ddi_bc_state_names[] = {
    "DDI_BC_STATE_UNINITIALIZED",     /* DDI_BC_STATE_UNINITIALIZED */
    "DDI_BC_STATE_BROKEN",            /* DDI_BC_STATE_BROKEN */
    "DDI_BC_STATE_DISABLED",          /* DDI_BC_STATE_DISABLED */
    "DDI_BC_STATE_WAITING_TO_CHARGE", /* DDI_BC_STATE_WAITING_TO_CHARGE */
    "DDI_BC_STATE_CONDITIONING",      /* DDI_BC_STATE_CONDITIONING */
    "DDI_BC_STATE_CHARGING",          /* DDI_BC_STATE_CHARGING */
    "DDI_BC_STATE_TOPPING_OFF"        /* DDI_BC_STATE_TOPPING_OFF */
};

/* Used by states that need to watch the time. */
uint32_t g_ddi_bc_u32StateTimer;

/* Always attempt to charge on first 5V connection */
bool bRestartChargeCycle = true;

#ifdef CONFIG_POWER_SUPPLY_DEBUG
static uint16_t u16ExternalBatteryPowerVoltageCheck;
#endif

ddi_bc_BrokenReason_t ddi_bc_gBrokenReason = DDI_BC_BROKEN_UNINITIALIZED;

static void transtate(ddi_bc_State_t newstate, uint16_t charging_current, uint16_t stop_thresh)
{
	g_ddi_bc_u32StateTimer = 0;

	if (charging_current) {
		ddi_bc_RampSetTarget(charging_current);
		if (stop_thresh) {
			ddi_bc_hwSetCurrentThreshold(stop_thresh);
		}
	} else {
		ddi_bc_RampReset();
	}

	printk("Battery charger: %s -> %s!\n", ddi_bc_state_names[g_ddi_bc_State], ddi_bc_state_names[newstate]);
	g_ddi_bc_State = newstate;
}

/* [luheng]: voltage is not supplied to batt-pin in these states, just accumulate state-timer, update alarm and do nothing */
static ddi_bc_Status_t idle_state(void)
{
	ddi_bc_RampUpdateAlarms();
	g_ddi_bc_u32StateTimer += LOOP_INTV;
	return DDI_BC_STATUS_SUCCESS;
}
static ddi_bc_Status_t ddi_bc_Uninitialized(void) { return idle_state(); }
static ddi_bc_Status_t ddi_bc_Broken       (void) { return idle_state(); }
static ddi_bc_Status_t ddi_bc_Disabled     (void) { return idle_state(); }

/*
 * [luheng] State routine of 'waiting_to_charge'
 *     1) stay here, on 5v removed
 *     2) stay here, if bRestartChargeCycle is not set and if batt-volt still full
 *     3) -> conditioning, if batt-volt below conditioning thresh
 *     4) -> charging, if batt-volg is between conditioning thresh and full, or is full but bRestartChargeCycle is set.
 */
static ddi_bc_Status_t ddi_bc_WaitingToCharge(void)
{
	uint16_t u16BatteryVoltage;

	ddi_bc_RampUpdateAlarms();

	g_ddi_bc_u32StateTimer += LOOP_INTV;

	if (!ddi_bc_hwPowerSupplyIsPresent()) {
#ifdef CONFIG_POWER_SUPPLY_DEBUG
		u16ExternalBatteryPowerVoltageCheck = 0;
#endif
		return DDI_BC_STATUS_SUCCESS;
	}

	u16BatteryVoltage = ddi_bc_hwGetBatteryVoltage();

#ifdef CONFIG_POWER_SUPPLY_DEBUG
	if (u16ExternalBatteryPowerVoltageCheck) {
		if ((u16ExternalBatteryPowerVoltageCheck - u16BatteryVoltage) > 300) { /* ??? */
			ddi_bc_gBrokenReason = DDI_BC_BROKEN_EXTERNAL_BATTERY_VOLTAGE_DETECTED;
			TransitionToBroken();
			return DDI_BC_STATUS_BROKEN;
		} else { /* reset this check */
			u16ExternalBatteryPowerVoltageCheck = 0;
		}
	}
#endif

	if (!bRestartChargeCycle) {
		uint16_t x;
		x = u16BatteryVoltage + (u16BatteryVoltage / 20);
		if (x >= g_ddi_bc_Configuration.u16ChargingVoltage) /* full battery, charge completed */
			return DDI_BC_STATUS_SUCCESS;

	}

	bRestartChargeCycle = false;

	if (u16BatteryVoltage < g_ddi_bc_Configuration.u16ConditioningThresholdVoltage) {
		TransitionToConditioning();
	} else {
		TransitionToCharging();
	}

	return DDI_BC_STATUS_SUCCESS;
}


/* [luheng] State Routine for 'conditioning'
 * exit state criteria:
 *     1) -> 'waiting_to_charging' on 5v removed
 *     2) -> 'broken', if voltage rise over upper line before ramping done
 *     3) -> 'broken', if stay in this state timeout
 *     4) -> 'charging', if voltage rise over upper line properly
 *
 * Note: as voltage is supplied to batt-pin, LOOP_INTV is not added when under alarm
 */
static ddi_bc_Status_t ddi_bc_Conditioning(void)
{
	ddi_bc_RampUpdateAlarms();
	if (!ddi_bc_RampGetDieTempAlarm() && !ddi_bc_RampGetBatteryTempAlarm()) {
		g_ddi_bc_u32StateTimer += LOOP_INTV;
	}

	if (!ddi_bc_hwPowerSupplyIsPresent()) {
		TransitionToWaitingToCharge();
		return DDI_BC_STATUS_SUCCESS;
	}

	if (    (ddi_bc_hwGetBatteryVoltage() > g_ddi_bc_Configuration.u16ConditioningMaxVoltage)
	     && (ddi_power_GetMaxBatteryChargeCurrent() < g_ddi_bc_Configuration.u16ConditioningCurrent))
	{
		/* rise over upper level before ramping done, too quickly, must be no batt */
		ddi_bc_gBrokenReason = DDI_BC_BROKEN_NO_BATTERY_DETECTED;
		TransitionToBroken();
		return DDI_BC_STATUS_BROKEN;
	}

	if (ddi_bc_hwGetBatteryVoltage() >= g_ddi_bc_Configuration.u16ConditioningMaxVoltage) {
		TransitionToCharging();
		return DDI_BC_STATUS_SUCCESS;
	}

	if (g_ddi_bc_u32StateTimer >= g_ddi_bc_Configuration.u32ConditioningTimeout) {
		ddi_bc_gBrokenReason = DDI_BC_BROKEN_CHARGING_TIMEOUT;
		TransitionToBroken();
		return DDI_BC_STATUS_BROKEN;
	}

	ddi_bc_RampStep(LOOP_INTV);
	return DDI_BC_STATUS_SUCCESS;
}

/* [luheng] 
 * State Routine for 'charging' state.
 * 3 conditions to exit this tate:
 *     1) -> 'waiting_to_charging' on 5v removed
 *     2) -> 'topping_off', if charging-stop at target current has been detected 10 times(periods)
 *     3) -> 'broken', if charging timeout
 * Note:
 *     if under alarm, we're not supplying voltage to batt, so LOOP_INTV is not added under alarm.
 */
static ddi_bc_Status_t ddi_bc_Charging(void)
{
	static int iStatusCount;
	uint16_t u16ActualProgrammedCurrent;
	uint16_t u16CurrentRampTarget;

	ddi_bc_RampUpdateAlarms();

	if (!ddi_bc_RampGetDieTempAlarm() && !ddi_bc_RampGetBatteryTempAlarm()) {
		g_ddi_bc_u32StateTimer += LOOP_INTV;
	}
	
	if (!ddi_bc_hwPowerSupplyIsPresent()) { /* -> 'waiting_to_charging' if 5v is removed */
		TransitionToWaitingToCharge();
		return DDI_BC_STATUS_SUCCESS;
	}

	ddi_bc_hwSetCurrentThreshold(g_ddi_bc_Configuration.u16ChargingThresholdCurrent);

	u16ActualProgrammedCurrent = ddi_bc_hwGetMaxCurrent();
	u16CurrentRampTarget = ddi_bc_RampGetTarget();

	if (u16CurrentRampTarget > ddi_bc_RampGetLimit())
		u16CurrentRampTarget = ddi_bc_RampGetLimit();

	u16CurrentRampTarget = ddi_bc_hwExpressibleCurrent(u16CurrentRampTarget);

	if ((u16ActualProgrammedCurrent >= u16CurrentRampTarget) && !ddi_bc_hwGetChargeStatus()) {
		if ((++iStatusCount) >= 10) {
#ifdef CONFIG_POWER_SUPPLY_DEBUG
			u16ExternalBatteryPowerVoltageCheck = ddi_bc_hwGetBatteryVoltage();
#endif
			iStatusCount = 0;
			TransitionToToppingOff();   /* -> 'topping_off' if charger stopped at target current */
			return DDI_BC_STATUS_SUCCESS;
		} /* else, we doubt, keep counting */
	} else {
		iStatusCount = 0;
	}

	if (g_ddi_bc_u32StateTimer >= g_ddi_bc_Configuration.u32ChargingTimeout) {
		ddi_bc_gBrokenReason = DDI_BC_BROKEN_CHARGING_TIMEOUT;
		TransitionToBroken();           /* -> 'broken' is charging timeout */
		return DDI_BC_STATUS_BROKEN;
	}

	/* normal charging \|/, ramp if necessary */
	ddi_bc_RampStep(LOOP_INTV);
	return DDI_BC_STATUS_SUCCESS;
}


/* [luheng]
 * state routine for 'topping_off' state.
 * -> 'waiting_to_charge' on 2 conditions
 *     1) 5v is removed
 *     2) stayed in this state long enough (> g_ddi_bc_Configuration.u32TopOffPeriod)
 *
 * since we're supplying voltage to batt-pin, shouldn't we add LOOP_INTV to StateTimer under alarm just
 * as in 'Charging' & 'Conditioning' state????
 */
static ddi_bc_Status_t ddi_bc_ToppingOff(void)
{
	ddi_bc_RampUpdateAlarms();

	if (!ddi_bc_RampGetDieTempAlarm() && !ddi_bc_RampGetBatteryTempAlarm()) {
		g_ddi_bc_u32StateTimer += LOOP_INTV;
	}

	/* -> waiting_to_charge if 1) 5v removed; 2) has stayed in this state long enough */
	if (    (!ddi_bc_hwPowerSupplyIsPresent())
	     || (g_ddi_bc_u32StateTimer >= g_ddi_bc_Configuration.u32TopOffPeriod)
	   )
	{
		TransitionToWaitingToCharge();
		return DDI_BC_STATUS_SUCCESS;
	}

	ddi_bc_RampStep(LOOP_INTV);
	return DDI_BC_STATUS_SUCCESS;
}

