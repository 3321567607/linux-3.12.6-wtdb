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


/* Includes and external references */



/* Variables */



/* Code */



/*  */
/* brief Report if the battery charging hardware is available. */
/*  */
/* fntype Function */
/*  */
/*  This function reports if the battery charging hardware is available by */
/*  reading the corresponding laser fuse bit. */
/*  */
/* retval  Zero if the battery charging hardware is not available. Non-zero */
/*           otherwise. */
/*  */

int ddi_bc_hwBatteryChargerIsEnabled(void)
{
	/* TODO: replace ddi_bc_hwBatteryChargerIsEnabled with the function below in the code */
	return (int)ddi_power_GetBatteryChargerEnabled();
}


/*  */
/* brief Report the battery configuration. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the hardware battery configuration. */
/*  */
/* retval  A value that indicates the battery configuration. */
/*  */

ddi_bc_BatteryMode_t ddi_bc_hwGetBatteryMode(void)
{
	/* TODO: replace ddi_bc_hwGetBatteryMode() with the function below. */
	return (ddi_bc_BatteryMode_t) ddi_power_GetBatteryMode();
}



/* [luheng] read from reg current battery voltage */
uint16_t ddi_bc_hwGetBatteryVoltage(void)
{
	/* TODO: replace ddi_bc_hwGetBattery with function below */
	return ddi_power_GetBattery();
}


/* [luheng] check if 5v is present */
int ddi_bc_hwPowerSupplyIsPresent(void)
{
	return (int)ddi_power_Get5vPresentFlag();
}


/* [luheng] read from reg current max charging current setting */
uint16_t ddi_bc_hwGetMaxCurrent(void)
{
	return (uint16_t) ddi_power_GetMaxBatteryChargeCurrent();
}

/* [luheng] set max hw charging current */
uint16_t ddi_bc_hwSetMaxCurrent(uint16_t u16Limit)
{
	return ddi_power_SetMaxBatteryChargeCurrent(u16Limit);
}

/* [luheng] set hw stop-charging lower thresh, unit:mA */
uint16_t ddi_bc_hwSetCurrentThreshold(uint16_t u16Threshold)
{
	return ddi_power_SetBatteryChargeCurrentThreshold(u16Threshold);
}


/*  */
/* brief Report the charging current threshold. */
/*  */
/* fntype Function */
/*  */
/*  This function reports the charging current threshold. When the actual */
/*  current flow to the battery is less than this threshold, the */
/*  HW_POWER_STS.CHRGSTS flag is clear. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 180mA (see the data sheet for details). */
/*  */
/* retval  The charging current threshold, in mA. */
/*  */

uint16_t ddi_bc_hwGetCurrentThreshold(void)
{
	/* TODO: replace calls to ddi_bc_hwGetCurrentThreshold with function below */
	return ddi_power_GetBatteryChargeCurrentThreshold();
}


/*  */
/* brief Report if the charger hardware power is on. */
/*  */
/* fntype Function */
/*  */
/*  This function reports if the charger hardware power is on. */
/*  */
/* retval  Zero if the charger hardware is not powered. Non-zero otherwise. */
/*  */

int ddi_bc_hwChargerPowerIsOn(void)
{

	/* -------------------------------------------------------------------------- */
	/* Note that the bit we're looking at is named PWD_BATTCHRG. The "PWD" */
	/* stands for "power down". Thus, when the bit is set, the battery charger */
	/* hardware is POWERED DOWN. */
	/* -------------------------------------------------------------------------- */

	/* -------------------------------------------------------------------------- */
	/* Read the register and return the result. */
	/* -------------------------------------------------------------------------- */

	/* TODO: replace ddi_bc_hwChargerPowerIsOn with function below */
	return ddi_power_GetChargerPowered();
}

/* [luheng] turn on/off batt-charger. To turn on, also turn on 5v->charger_&_4p2 route */
void ddi_bc_hwSetChargerPower(int on)
{
	ddi_power_SetChargerPowered(on);
}

/* [luheng] read from hw reg if charger is actualling delivering current to batt */
int ddi_bc_hwGetChargeStatus(void)
{
	return ddi_power_GetChargeStatus();
}


/* [luheng] get current die temp in celsius degree. */
/*     pLow   The low  end of the temperature range. */
/*     pHigh  The high end of the temperature range. */
void ddi_bc_hwGetDieTemp(int16_t *pLow, int16_t *pHigh)
{
	ddi_power_GetDieTemp(pLow, pHigh);
}


/* [luheng] return external temp thermistor's ohm value attached on physical lradc chan-0 */
ddi_bc_Status_t ddi_bc_hwGetBatteryTemp(uint16_t *pReading)
{
	ddi_power_GetBatteryTemp(pReading);
  return DDI_BC_STATUS_SUCCESS;
}


/*  */
/* brief Convert a current in mA to a hardware setting. */
/*  */
/* fntype Function */
/*  */
/*  This function converts a current measurement in mA to a hardware setting */
/*  used by HW_POWER_BATTCHRG.STOP_ILIMIT or HW_POWER_BATTCHRG.BATTCHRG_I. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 780mA (see the data sheet for details). If the given */
/*  current cannot be expressed exactly, then the largest expressible smaller */
/*  value will be used. */
/*  */
/* param[in]  u16Current  The current of interest. */
/*  */
/* retval  The corresponding setting. */
/*  */

uint8_t ddi_bc_hwCurrentToSetting(uint16_t u16Current)
{
	return ddi_power_convert_current_to_setting(u16Current);
}


/*  */
/* brief Convert a hardware current setting to a value in mA. */
/*  */
/* fntype Function */
/*  */
/*  This function converts a setting used by HW_POWER_BATTCHRG.STOP_ILIMIT or */
/*  HW_POWER_BATTCHRG.BATTCHRG_I into an actual current measurement in mA. */
/*  */
/*  Note that the hardware current fields are 6 bits wide. The higher bits in */
/*  the 8-bit input parameter are ignored. */
/*  */
/* param[in]  u8Setting  A hardware current setting. */
/*  */
/* retval  The corresponding current in mA. */
/*  */

uint16_t ddi_bc_hwSettingToCurrent(uint8_t u8Setting)
{
	return ddi_power_convert_setting_to_current(u8Setting);
}


/*  */
/* brief Compute the actual current expressible in the hardware. */
/*  */
/* fntype Function */
/*  */
/*  Given a desired current, this function computes the actual current */
/*  expressible in the hardware. */
/*  */
/*  Note that the hardware has a minimum resolution of 10mA and a maximum */
/*  expressible value of 780mA (see the data sheet for details). If the given */
/*  current cannot be expressed exactly, then the largest expressible smaller */
/*  value will be used. */
/*  */
/* param[in]  u16Current  The current of interest. */
/*  */
/* retval  The corresponding current in mA. */
/*  */

uint16_t ddi_bc_hwExpressibleCurrent(uint16_t u16Current)
{
	/* TODO: replace the bc function with this one */
	return ddi_power_ExpressibleCurrent(u16Current);
}


/*  */
/* brief Checks to see if the DCDC has been manually enabled */
/*  */
/* fntype Function */
/*  */
/* retval  true if DCDC is ON, false if DCDC is OFF. */
/*  */

bool ddi_bc_hwIsDcdcOn(void)
{
	return ddi_power_IsDcdcOn();
}


/* End of file */

/*  @} */
