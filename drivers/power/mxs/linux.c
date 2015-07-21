/*
 * Linux glue to MXS battery state machine.
 *
 * Author: Steve Longerbeam <stevel@embeddedalley.com>
 *
 * Copyright (C) 2008 EmbeddedAlley Solutions Inc.
 * Copyright (C) 2008-2013 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/suspend.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/fiq.h>
#include "ddi_bc_internal.h"
#include "ddi_bc.h"
#include "mxs-battery.h"
#include "regs-lradc.h"
#include "regs-power.h"

struct mxs_info {
	struct device *dev;
	struct regulator *regulator;
	struct regulator *onboard_vbus5v;

	struct power_supply bat;
	struct power_supply ac;
	struct power_supply usb;

	ddi_bc_Cfg_t *sm_cfg;
	struct mutex sm_lock;
	struct timer_list sm_timer;
	struct work_struct sm_work;

	int irq_vdd5v;
	int irq_dcdc4p2_bo;
	int irq_batt_brnout;
	int irq_vddd_brnout;
	int irq_vdda_brnout;
	int irq_vddio_brnout;
	int irq_vdd5v_droop;

	int is_ac_online;
	int source_protection_mode;
	uint32_t sm_new_5v_connection_jiffies;
	uint32_t sm_new_5v_disconnection_jiffies;
	enum application_5v_status sm_5v_connection_status;




#define USB_ONLINE      0x01
#define USB_REG_SET     0x02
#define USB_SM_RESTART  0x04
#define USB_SHUTDOWN    0x08
#define USB_N_SEND      0x10
	int is_usb_online;
	int onboard_vbus5v_online;

	int powersource;
	int is_5v_irq_detected;
	u32	clks[10];
	struct clk *lradc_clk;
};

#define to_mxs_info(x) container_of((x), struct mxs_info, bat)


//#define  POWER_FIQ

/* #define DEBUG_IRQS */

/* There is no direct way to detect wall power presence, so assume the AC
 * power source is valid if 5V presents and USB device is disconnected.
 * If USB device is connected then assume that AC is offline and USB power
 * is online.
 */


#define is_ac_online() ddi_power_Get5vPresentFlag()

int mxs_pwr_irqs[NUMS_MXS_PWR_IRQS];
/*void __iomem *mxs_digctl_base;*/
void __iomem *mxs_rtc_base;
void __iomem *mxs_lradc_base;


static ddi_bc_Cfg_t battery_data = {
	.u32StateMachinePeriod           = 100,          /* ms, period of state machine */
	.u16CurrentRampSlope             = 75,           /* mA/s */
	.u16ConditioningThresholdVoltage = 2900,         /* mV, condition low thresh, use it's own charging cur */
	.u16ConditioningMaxVoltage       = 3000,         /* mV, condition exit thresh */
	.u16ConditioningCurrent          = 160,          /* mA, condition charging current */
	.u32ConditioningTimeout          = 4*60*60*1000, /* ms (4 hours) */
	.u16ChargingVoltage              = DDI_BC_LIION_CHARGING_VOLTAGE,	/* charging voltage, 4200 mV */
	/* FIXME: the current comparator could have h/w bugs in current
	 * detection through POWER_STS.CHRGSTS bit */
	.u16ChargingCurrent              = 500,          /* mA 600 */
	.u16ChargingThresholdCurrent     = 50,           /* mA 60, when below this thresh, stop charging */
	.u32ChargingTimeout              = 4*60*60*1000, /* ms (4 hours) */

	.u32TopOffPeriod                 = 30*60*1000,   /* ms (30 min), time stay in top_off state */

	.monitorDieTemp                  = 1,            /* need to Monitor the die */
	.u8DieTempHigh                   = 75,           /* deg centigrade */
	.u8DieTempLow                    = 65,           /* deg centigrade */
	.u16DieTempSafeCurrent           = 0,            /* mA, charging current when die temp alarms */
	.u8DieTempChannel                = 0,            /* LRADC logic channel 0 to monitor die temp*/

	.monitorBatteryTemp              = 0,            /* don't need to monitor the battery*/
	.u8BatteryTempChannel            = 1,            /* LRADC logic channel 1 to monitor batt temp */
	.u16BatteryTempHigh              = 642,          /* high margin of thermistor ohm value */
	.u16BatteryTempLow               = 497,          /* low margin of thermistor ohm value */
	.u16BatteryTempSafeCurrent       = 0,            /* mA */
};

void hw_lradc_set_delay_trigger(int trgr, u32 trgr_lradc, u32 delay_trgr, u32 loops, u32 delays)
{
	WR_LRADC_REG(BF_LRADC_DELAYn_TRIGGER_LRADCS(trgr_lradc), HW_LRADC_DELAYn_SET(trgr));
	WR_LRADC_REG(BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_trgr), HW_LRADC_DELAYn_SET(trgr));
	WR_LRADC_REG(BM_LRADC_DELAYn_LOOP_COUNT | BM_LRADC_DELAYn_DELAY, HW_LRADC_DELAYn_CLR(trgr));
	WR_LRADC_REG(BF_LRADC_DELAYn_LOOP_COUNT(loops), HW_LRADC_DELAYn_SET(trgr));
	WR_LRADC_REG(BF_LRADC_DELAYn_DELAY(delays), HW_LRADC_DELAYn_SET(trgr));
}

void init_protection(struct mxs_info *info)
{
	enum ddi_power_5v_status pmu_5v_status;
	uint16_t battery_voltage;

	pmu_5v_status = ddi_power_GetPmu5vStatus();
	battery_voltage = ddi_power_GetBattery();

	/* enable vddd/vdda/vddio BO INT, don't shutdown on them */
	ddi_power_InitOutputBrownouts();

	/* whitetiger don't care, cause we have both 5v & batt */
	if (info->powersource == NO_VDD5V_SOURCE) {
		ddi_power_EnableBatteryBoInterrupt(true);
		return;
	}

	if ((pmu_5v_status == existing_5v_connection) && ddi_power_check_4p2_bits()) { /* by 5v now */
		ddi_power_enable_5v_disconnect_detection(); /* start detecting 5v detach */
		ddi_power_init_4p2_protection();            /* enable VBUSDROOP INT */

		ddi_power_EnableBatteryBoInterrupt(false);
		info->sm_5v_connection_status = _5v_connected_verified;
	} else {                                                                        /* by batt now */
		info->sm_5v_connection_status = _5v_disconnected_verified;
		ddi_power_EnableBatteryBoInterrupt(true);

	}

	/* all brownouts are now handled software fiqs.  We
	 * can now disable the hardware protection mechanisms
	 *  because leaving them on yields ~2kV ESD level
	 *  versus ~4kV ESD levels when they are off.  This
	 *  difference is suspected to be cause by the fast
	 *  falling edge pswitch functionality being tripped
	 *  by ESD events.  This functionality is disabled
	 *  when PWD_OFF is disabled.
	 */
#ifdef DISABLE_HARDWARE_PROTECTION_MECHANISMS
	WR_PWR_REG(BM_POWER_RESET_PWD_OFF, HW_POWER_RESET_SET);
#endif
}


/*
 * [luheng] check and handle 5v detach/attach
 *
 *     This function is called by statemachine. It make sure detach/attach handling
 * happens after at least 500mA after the event, this allow the power supply become
 * stable before we take action. The event stamp is saved by event irq handler(irq_vdd5v)
 *
 *     Handling include:
 * 1) turn off/on 5v->charger_&_4p2->DCDC on detach/attach
 * 2) switch 5v detecting to detecting on/off on detach/attach
 * 3) enable vddio-BO INT, this is disabled in the INT handler
 * 4) reset batt charging structure.
 */
static void check_and_handle_5v_connection(struct mxs_info *info)
{
	switch (ddi_power_GetPmu5vStatus()) {
		case new_5v_connection:
			if (info->is_5v_irq_detected == 0) /* wait for irq_vdd5v handler to run first, Otherwise debounce checking wrong.*/
				break;

			info->is_5v_irq_detected = 0;
			ddi_power_enable_5v_disconnect_detection(); /* switch detecting 5v attach to detecting detach */
			info->sm_5v_connection_status = _5v_connected_unverified;
			/* \|/ Fall through */
		case existing_5v_connection:
			if (info->sm_5v_connection_status == _5v_connected_verified) /* this attach msg already handled, exit */
				break;
			/* wait 500ms before considering the 5v connection to be ready. Allow USB driver
			 * time to enumerate (coordination with USB driver to be added in the future). */
			if (jiffies_to_msecs(MXSDELTA(jiffies,info->sm_new_5v_connection_jiffies)) <= _5V_DEBOUNCE_TIME_MS)
				break;

			if (info->onboard_vbus5v) { /* irrelevant to whitetiger */
				if (regulator_is_enabled(info->onboard_vbus5v) > 0) {
					info->onboard_vbus5v_online = 1;
					pr_debug("When supply from onboard vbus 5v , DO NOT switch to 4p2 \n");
					break;
				}
			}
			ddi_power_Enable4p2(450); /* turn on 5v->charger_&_4p2->DCDC, max current 450mA */

			/* It is safe to turn on vddio-BO INT again */
			ddi_power_enable_vddio_interrupt(true);
			WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDDD_BO, HW_POWER_CTRL_SET);
			WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDDA_BO, HW_POWER_CTRL_SET);

			info->sm_5v_connection_status = _5v_connected_verified;
			dev_dbg(info->dev, "5v connection verified\n");
			break;

		case new_5v_disconnection:
			if (info->is_5v_irq_detected == 0) /* wait for irq_vdd5v handler run first.*/
				break;

			ddi_bc_SetDisable();
			ddi_bc_RampSetLimit(0);
			if (info->regulator)
				regulator_set_current_limit(info->regulator, 0, 0);
			info->is_ac_online = 0;
			info->onboard_vbus5v_online = 0;
			info->sm_5v_connection_status = _5v_disconnected_unverified; /* mark this detach unhandled */
			/* \|/Fall through */
		case existing_5v_disconnection:
			if (info->sm_5v_connection_status == _5v_disconnected_verified)
				break;                                 /* this detach msg already handled, exit */
			if (jiffies_to_msecs(MXSDELTA(jiffies,info->sm_new_5v_disconnection_jiffies)) <= _5V_DEBOUNCE_TIME_MS)
				break;                                 /* wait detach 500ms before handle */
			ddi_power_execute_5v_to_battery_handoff(); /* turn off 5v->charger_&_4p2->DCDC */
			info->is_5v_irq_detected = 0;
			ddi_power_enable_5v_connect_detection();   /* start detecting attach */
			ddi_power_enable_vddio_interrupt(true);	   /* It is safe to turn on vddio interrupts again */
			CLR_SET_PWR_REG_BITS(HW_POWER_5VCTRL, BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT,
				(0x20 << BP_POWER_5VCTRL_CHARGE_4P2_ILIMIT)); /* 5v->charger_&_4p2 max current: 400mA, why??? */
			info->sm_5v_connection_status = _5v_disconnected_verified;
			dev_dbg(info->dev, "5v disconnection handled\n");
			break;
	}
}

/* [luheng] adjust 4p2-batt cmptrip, 5v-droop behavior according to cur batt-vol */
static void handle_battery_voltage_changes(struct mxs_info *info)
{
	static bool xfer_enabled = -1;
	bool ready_xfer = false;

	ddi_power_handle_cmptrip();

	ready_xfer = ddi_power_IsBattRdyForXfer();

	if (ready_xfer != xfer_enabled) {
		xfer_enabled = ready_xfer;
		ddi_power_enable_5v_to_battery_xfer(xfer_enabled);
		printk("DCDC -> batt xfer is now %s\n", xfer_enabled ? "enabled" : "disabled");
	}
}


/* Power properties for 5v ac */
static enum power_supply_property mxs_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
static int mxs_power_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mxs_info *info;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
				info = container_of(psy, struct mxs_info, ac);
				val->intval = info->onboard_vbus5v_online ? 0 : is_ac_online();
			}
			break;
		default:
			return -EINVAL;
	}
	return 0;
}
/*
 * Battery properties
 */
static enum power_supply_property mxs_bat_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int mxs_bat_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mxs_info *info = to_mxs_info(psy);
	ddi_bc_State_t state;
	ddi_bc_BrokenReason_t reason;
	int temp_alarm;
	int16_t temp_lo, temp_hi;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:                           /* "status" property */
		state = ddi_bc_GetState();
		switch (state) {
			case DDI_BC_STATE_CONDITIONING:
			case DDI_BC_STATE_CHARGING:
			case DDI_BC_STATE_TOPPING_OFF:
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
				break;
			case DDI_BC_STATE_DISABLED:
				val->intval = (ddi_power_Get5vPresentFlag()
					&& !info->onboard_vbus5v_online) ?
					POWER_SUPPLY_STATUS_NOT_CHARGING :
				POWER_SUPPLY_STATUS_DISCHARGING;
				break;
			default:
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:                          /* "present" property */
		state = ddi_bc_GetState();
		switch (state) {
			case DDI_BC_STATE_WAITING_TO_CHARGE:
			case DDI_BC_STATE_DCDC_MODE_WAITING_TO_CHARGE:
			case DDI_BC_STATE_CONDITIONING:
			case DDI_BC_STATE_CHARGING:
			case DDI_BC_STATE_TOPPING_OFF:
			case DDI_BC_STATE_DISABLED:
				val->intval = 1;
				break;

			case DDI_BC_STATE_BROKEN:
				val->intval = !(ddi_bc_GetBrokenReason() ==
						DDI_BC_BROKEN_NO_BATTERY_DETECTED);
				break;

			default:
				val->intval = 0;
				break;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:                           /* "health" property */
		temp_alarm = ddi_bc_RampGetDieTempAlarm();
		if (temp_alarm) {
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		} else {
			state = ddi_bc_GetState();
			switch (state) {
				case DDI_BC_STATE_BROKEN:
					reason = ddi_bc_GetBrokenReason();
					val->intval =
					   (reason == DDI_BC_BROKEN_CHARGING_TIMEOUT) ?
						POWER_SUPPLY_HEALTH_DEAD :
						POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
					break;

				case DDI_BC_STATE_UNINITIALIZED:
					val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
					break;

				default:
					val->intval = POWER_SUPPLY_HEALTH_GOOD;
					break;
			}
		}
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:                       /* "technology" property */
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:                      /* "voltage_now" property, uV */
		val->intval = ddi_power_GetBattery() * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:                      /* "current_now" property, uA */
		val->intval = ddi_power_GetMaxBatteryChargeCurrent() * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:                             /* "temp" property, die temp in celsius */
		mutex_lock(&info->sm_lock);
		ddi_power_GetDieTemp(&temp_lo, &temp_hi);
		mutex_unlock(&info->sm_lock);
		val->intval = temp_lo + (temp_hi - temp_lo) / 2;

		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* [luheng] schedule sm_work per 100ms. */
static void state_machine_timer(unsigned long data)
{
	struct mxs_info *info = (struct mxs_info *)data;
	ddi_bc_Cfg_t *cfg = info->sm_cfg;
	int ret;

	/* schedule next call to state machine */
	mod_timer(&info->sm_timer, jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));

	ret = schedule_work(&info->sm_work);
	if (!ret)
		dev_dbg(info->dev, "state machine failed to schedule\n");

}
/*
 * [luheng] real state machine handler, schedule by timer per 100ms
 */
static void state_machine_work(struct work_struct *work)
{
	struct mxs_info *info = container_of(work, struct mxs_info, sm_work);

	mutex_lock(&info->sm_lock);

	handle_battery_voltage_changes(info); /* handle batt vol change */
	check_and_handle_5v_connection(info); /* handle 5v detach/attach */

	if (info->sm_5v_connection_status != _5v_connected_verified) {
		mod_timer(&info->sm_timer, jiffies + msecs_to_jiffies(100));
		goto out;
	}

	/* if we made it here, we have a verified 5v connection */
	if (info->is_ac_online || info->onboard_vbus5v_online)
		goto done;

	/* ac supply connected */
	dev_dbg(info->dev, "changed power connection to ac/5v.\n)");
	dev_dbg(info->dev, "5v current limit set to %u.\n", NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA);

	info->is_ac_online = 1;
	info->is_usb_online = 0;

	ddi_power_set_4p2_ilimit(NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA);
	ddi_bc_RampSetLimit(NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA /*mA*/);

	ddi_bc_SetEnable();

	dev_dbg(info->dev, "changed power connection to usb/5v present\n");

done:
	ddi_bc_StateMachine();
out:
	mutex_unlock(&info->sm_lock);
}

/* [luheng] init charger and schedule statemachine 100ms later */
static int bc_sm_restart(struct mxs_info *info)
{
	ddi_bc_Status_t bcret;
	int ret = 0;

	mutex_lock(&info->sm_lock);

	bcret = ddi_bc_Init(info->sm_cfg); /* init charger */
	if (bcret != DDI_BC_STATUS_SUCCESS) {
		dev_err(info->dev, "battery charger init failed: %d\n", bcret);
		ret = -EIO;
		goto out;
	}
	info->regulator = NULL;

	/* schedule first call to state machine */
	mod_timer(&info->sm_timer, jiffies + msecs_to_jiffies(100));
out:
	mutex_unlock(&info->sm_lock);
	return ret;
}

#ifndef POWER_FIQ
static irqreturn_t mxs_irq_dcdc4p2_bo(int irq, void *cookie)
{
	printk("%s shutting down...\n", __FUNCTION__);
	mdelay(500);
	ddi_power_handle_dcdc4p2_bo();
	return IRQ_HANDLED;
}

static irqreturn_t mxs_irq_batt_brnout(int irq, void *cookie)
{
	printk("%s shutting down...\n", __FUNCTION__);
	mdelay(500);
	ddi_power_shutdown();
	return IRQ_HANDLED;
}


static irqreturn_t mxs_irq_vddd_brnout(int irq, void *cookie)
{
	printk("%s shutting down...\n", __FUNCTION__);
	mdelay(500);
	ddi_power_shutdown();
	return IRQ_HANDLED;
}
static irqreturn_t mxs_irq_vdda_brnout(int irq, void *cookie)
{
	printk("%s shutting down...\n", __FUNCTION__);
	mdelay(500);
	ddi_power_shutdown();
	return IRQ_HANDLED;
}

static irqreturn_t mxs_irq_vdd5v_droop(int irq, void *cookie)
{
	ddi_power_handle_vdd5v_droop();
	printk("wangluheng %s...\n", __FUNCTION__);
	/*mdelay(5000);*/

	return IRQ_HANDLED;
}

#endif /* if POWER_FIQ */

static irqreturn_t mxs_irq_vddio_brnout(int irq, void *cookie)
{
	printk("wangluheng %s...\n", __FUNCTION__);
	/*mdelay(5000);*/
	ddi_power_handle_vddio_brnout();
	return IRQ_HANDLED;
}

/*
 * [luheng]
 * 5v detach/attach irq handler
 *   1. disable detach/attach irq, this will be re-enabled by real
 *      handler called from state-machine.
 *   2. mark the event not handled.
 *   3. save event stamp and schedule state-machine right away,
 *      state-machine will do real handling 500ms after the event stamp.
 */
static irqreturn_t mxs_irq_vdd5v(int irq, void *cookie)
{
	struct mxs_info *info = (struct mxs_info *)cookie;
	enum ddi_power_5v_status event = ddi_power_GetPmu5vStatus();

	if ((new_5v_connection == event) || (new_5v_disconnection == event)) {
			if (new_5v_connection == event)               /* save event stamp */
				info->sm_new_5v_connection_jiffies = jiffies;
			else
				info->sm_new_5v_disconnection_jiffies = jiffies;

			ddi_power_disable_5v_connection_irq();        /* disable detect irq */
			mod_timer(&info->sm_timer, jiffies + 1);      /* schedule state-machine right away */
			info->is_5v_irq_detected = 1;                 /* mark the event unhandled */
			printk("wangluheng new 5v connection detected\n");
	}
	return IRQ_HANDLED;
}

#ifdef POWER_FIQ
static int power_relinquish(void *data, int relinquish)
{
	return -1;
}

static struct fiq_handler power_fiq = {
	.name = "mxs-battery",
	.fiq_op = power_relinquish
};

static struct pt_regs fiq_regs;
extern char power_fiq_start[], power_fiq_end[];
extern void lock_vector_tlb(void *);
extern long power_fiq_count;
static struct proc_dir_entry *power_fiq_proc;
#endif

static int mxs_bat_init_regs(struct platform_device *pdev)
{
	struct device_node *np;

	np = pdev->dev.of_node;                                       mxs_pwr_base    = of_iomap(np, 0);
	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-digctl"); mxs_digctl_base = of_iomap(np, 0);
	np = of_find_compatible_node(NULL, NULL, "fsl,stmp3xxx-rtc"); mxs_rtc_base    = of_iomap(np, 0);
	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-lradc");  mxs_lradc_base  = of_iomap(np, 0);

	IRQ_BATT_BRNOUT    = platform_get_irq_byname(pdev, "batt_bo");
	IRQ_VDDD_BRNOUT    = platform_get_irq_byname(pdev, "vddd_bo");
	IRQ_VDDIO_BRNOUT   = platform_get_irq_byname(pdev, "vddio_bo");
	IRQ_VDDA_BRNOUT    = platform_get_irq_byname(pdev, "vdda_bo");
	IRQ_VDD5V_DROOP    = platform_get_irq_byname(pdev, "vdd5v_droop");
	IRQ_DCDC4P2_BRNOUT = platform_get_irq_byname(pdev, "dcdc4p2_bo");
	IRQ_VDD5V          = platform_get_irq_byname(pdev, "vdd5v");

	if (    (IRQ_BATT_BRNOUT  < 0) || (IRQ_VDDD_BRNOUT < 0)
	     || (IRQ_VDDIO_BRNOUT < 0) || (IRQ_VDDA_BRNOUT < 0)
	     || (IRQ_VDD5V_DROOP  < 0) || (IRQ_DCDC4P2_BRNOUT < 0) || (IRQ_VDD5V < 0)
	   )
	{
		printk("Battery: one of the irqs not specified in dtb!\n");
		return -ENXIO;
	}

#ifdef POWER_FIQ
	ret = claim_fiq(&power_fiq);
	if (ret) {
		pr_err("Can't claim fiq");
	} else {
		get_fiq_regs(&fiq_regs);
		set_fiq_handler(power_fiq_start, power_fiq_end-power_fiq_start);
		lock_vector_tlb((void *)0xffff0000);
		lock_vector_tlb(REGS_POWER_BASE);

		/* disable interrupts to be configured as FIQs */
		disable_irq(IRQ_DCDC4P2_BRNOUT);
		disable_irq(IRQ_BATT_BRNOUT);
		disable_irq(IRQ_VDDD_BRNOUT);
		disable_irq(IRQ_VDDA_BRNOUT);
		disable_irq(IRQ_VDD5V_DROOP);
		/* Enable these interrupts as FIQs */
		mxs_icoll_set_irq_fiq(IRQ_DCDC4P2_BRNOUT);
		mxs_icoll_set_irq_fiq(IRQ_BATT_BRNOUT);
		mxs_icoll_set_irq_fiq(IRQ_VDDD_BRNOUT);
		mxs_icoll_set_irq_fiq(IRQ_VDDA_BRNOUT);
		mxs_icoll_set_irq_fiq(IRQ_VDD5V_DROOP);
		/* enable FIQ functionality */
		enable_irq(IRQ_DCDC4P2_BRNOUT);
		enable_irq(IRQ_BATT_BRNOUT);
		enable_irq(IRQ_VDDD_BRNOUT);
		enable_irq(IRQ_VDDA_BRNOUT);
		enable_irq(IRQ_VDD5V_DROOP);
	}
#endif

	return 0;
}

static void mxs_free_irqs(struct platform_device *pdev)
{
	struct mxs_info *info = platform_get_drvdata(pdev);

	if (info) {
		if (info->irq_vdd5v >= 0) {
			devm_free_irq(&(pdev->dev), info->irq_vdd5v, info);
			info->irq_vdd5v = -1;
		}
		if (info->irq_vddio_brnout >= 0) {
			devm_free_irq(&(pdev->dev), info->irq_vddio_brnout, info);
			info->irq_vddio_brnout = -1;
		}
		if (info->irq_dcdc4p2_bo >= 0) {
			devm_free_irq(&(pdev->dev), info->irq_dcdc4p2_bo, info);
			info->irq_dcdc4p2_bo = -1;
		}
		if (info->irq_batt_brnout >= 0) {
			devm_free_irq(&(pdev->dev), info->irq_batt_brnout, info);
			info->irq_batt_brnout = -1;
		}
		if (info->irq_vddd_brnout >= 0) {
			devm_free_irq(&(pdev->dev), info->irq_vddd_brnout, info);
			info->irq_vddd_brnout = -1;
		}
		if (info->irq_vdda_brnout >= 0) {
			devm_free_irq(&(pdev->dev), info->irq_vdda_brnout, info);
			info->irq_vdda_brnout = -1;
		}
		if (info->irq_vdd5v_droop >= 0) {
			devm_free_irq(&(pdev->dev), info->irq_vdd5v_droop, info);
			info->irq_vdd5v_droop = -1;
		}
	}
}

static int mxs_init_irqs(struct platform_device *pdev)
{
	int ret = 0;
	struct mxs_info *info = platform_get_drvdata(pdev);

	if (NULL == info)
		return -EINVAL;

	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_VDD5V, mxs_irq_vdd5v,
			NULL, IRQF_SHARED, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_VDD5V\n");
		goto out;
	} else 
		info->irq_vdd5v = IRQ_VDD5V;

	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_VDDIO_BRNOUT, mxs_irq_vddio_brnout,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_VDDIO_BRNOUT\n");
		goto out;
	} else
		info->irq_vddio_brnout = IRQ_VDDIO_BRNOUT;
	
#ifndef POWER_FIQ
	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_DCDC4P2_BRNOUT, mxs_irq_dcdc4p2_bo,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_DCDC4P2_BRNOUT\n");
		goto out;
	} else
		info->irq_dcdc4p2_bo = IRQ_DCDC4P2_BRNOUT;

	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_BATT_BRNOUT, mxs_irq_batt_brnout,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_BATT_BRNOUT\n");
		goto out;
	} else
		info->irq_batt_brnout = IRQ_BATT_BRNOUT;

	/*if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_VDDD_BRNOUT, mxs_irq_vddd_brnout,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_VDDD_BRNOUT\n");
		goto out;
	} else
		info->irq_vddd_brnout = IRQ_VDDD_BRNOUT;*/

	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_VDDA_BRNOUT, mxs_irq_vdda_brnout,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_VDDA_BRNOUT\n");
		goto out;
	} else
		info->irq_vdda_brnout = IRQ_VDDA_BRNOUT;

	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_VDD5V_DROOP, mxs_irq_vdd5v_droop,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_VDD5V_DROOP\n");
		goto out;
	} else
		info->irq_vdd5v_droop  = IRQ_VDD5V_DROOP;
#endif

out:
	if (0 != ret)
		mxs_free_irqs(pdev);
	else
		printk("all batt irqs requested!\n");

	return ret;
}

static struct mxs_info *mxs_bat_init_info(struct platform_device *pdev)
{
	struct mxs_info *info = NULL;

	if (!(info = kzalloc(sizeof(*info), GFP_KERNEL)))
		return NULL;

	if (IS_ERR(info->lradc_clk =  clk_get(&pdev->dev, NULL))) {
		printk("%s: failed to get lradc clock!\n", __FUNCTION__);
		return NULL;
	}

	if (clk_prepare_enable(info->lradc_clk)) {
		printk("%s: failed to enable lradc clock!\n", __FUNCTION__);
		clk_put(info->lradc_clk);
		kfree(info);
		return NULL;
	}

	info->irq_vdd5v = info->irq_vddio_brnout = info->irq_dcdc4p2_bo = info->irq_batt_brnout =
		info->irq_vddd_brnout = info->irq_vdda_brnout = info->irq_vdd5v_droop = -1;

	info->powersource = NORMAL_SOURCE;

	platform_set_drvdata(pdev, info);

	mutex_init(&info->sm_lock);

	info->dev    = &pdev->dev;
	info->sm_cfg = &battery_data;
	/* initialize bat power_supply struct */
	info->bat.name           = "battery";
	info->bat.type           = POWER_SUPPLY_TYPE_BATTERY;
	info->bat.properties     = mxs_bat_props;
	info->bat.num_properties = ARRAY_SIZE(mxs_bat_props);
	info->bat.get_property   = mxs_bat_get_property;
	/* initialize ac power_supply struct */
	info->ac.name           = "ac";
	info->ac.type           = POWER_SUPPLY_TYPE_MAINS;
	info->ac.properties     = mxs_power_props;
	info->ac.num_properties = ARRAY_SIZE(mxs_power_props);
	info->ac.get_property   = mxs_power_get_property;

	init_timer(&info->sm_timer);
	info->sm_timer.data = (unsigned long)info;
	info->sm_timer.function = state_machine_timer;
	INIT_WORK(&(info->sm_work), state_machine_work);

	return info;
}

static int mxs_bat_probe(struct platform_device *pdev)
{
	struct mxs_info *info = NULL;
	int ret = 0;

	printk("%s: Entering\n", __FUNCTION__);

	if (NULL == (info = mxs_bat_init_info(pdev))) /* alloc & init mem, request lradc clock */
		return -ENOMEM;

	if ((ret = mxs_bat_init_regs(pdev))) {        /* init hw-module base address from dt */
		goto free_info;
	}

	if ((ret = ddi_power_init_battery())) {       /* enable batt vol measrement, enable 5v detect */
		printk(KERN_ERR "Aborting power driver initialization\n");
		goto free_info;
	}

	if ((ret = bc_sm_restart(info)))              /* init state machine and start it right away */
		goto free_info;

	if (mxs_init_irqs(pdev))                      /* request battery irqs */
		goto stop_sm;

	if ((ret = power_supply_register(&pdev->dev, &info->bat))) {
		dev_err(info->dev, "failed to register battery\n");
		goto stop_sm;
	}

	if ((ret = power_supply_register(&pdev->dev, &info->ac))) {
		dev_err(info->dev, "failed to register ac power supply\n");
		goto unregister_bat;
	}

	/* handoff protection handling from bootlets protection method
	 * to kernel protection method */
	init_protection(info);

	return 0;

unregister_bat:
	power_supply_unregister(&info->bat);

stop_sm:
	ddi_bc_ShutDown();

free_info:
	mxs_free_irqs(pdev);
	kfree(info);
	return ret;
}

static int mxs_bat_remove(struct platform_device *pdev)
{
	struct mxs_info *info = platform_get_drvdata(pdev);

	printk("%s...", __FUNCTION__);
	mxs_free_irqs(pdev);
	if (info->lradc_clk) {
		clk_disable_unprepare(info->lradc_clk);
		clk_put(info->lradc_clk);
		info->lradc_clk = NULL;
	}
	//ddi_bc_ShutDown();
	power_supply_unregister(&info->ac);
	power_supply_unregister(&info->bat);
	printk("done\n");
	msleep(50);
	return 0;
}

static void mxs_bat_shutdown(struct platform_device *pdev)
{
	ddi_bc_ShutDown();
}


#ifdef CONFIG_PM

suspend_state_t mxs_pm_get_target(void);
/*static u32 power_clk_regs[] = {
		HW_POWER_CTRL,
		HW_POWER_5VCTRL,
		HW_POWER_VDDDCTRL,
		HW_POWER_VDDACTRL,
		HW_POWER_VDDIOCTRL,
};*/

void backup_power_reg(struct mxs_info *info)
{
	/*int i;
	if (mxs_pm_get_target() == PM_SUSPEND_MEM)  {
		for (i = 0; i < ARRAY_SIZE(power_clk_regs); i++)
			info->clks[i] = __raw_readl(REGS_POWER_BASE +	power_clk_regs[i]);
  }*/
}

void resume_power_reg(struct mxs_info *info)
{
	/*int i;

	if (mxs_pm_get_target() == PM_SUSPEND_MEM) {
		for (i = 0; i < ARRAY_SIZE(power_clk_regs); i++)
				__raw_writel(info->clks[i], REGS_POWER_BASE +	power_clk_regs[i]);
	}*/
}

static int mxs_bat_suspend(struct platform_device *pdev, pm_message_t msg)
{
	struct mxs_info *info = platform_get_drvdata(pdev);

	printk("%s...\n", __FUNCTION__);

	mutex_lock(&info->sm_lock);

	/* enable USB 5v wake up so don't disable irq here*/
	if (info->powersource == NORMAL_SOURCE) {
		ddi_bc_SetDisable();
		/* cancel state machine timer */
		del_timer_sync(&info->sm_timer);
	}
	backup_power_reg(info);

	mutex_unlock(&info->sm_lock);
	return 0;
}

static int mxs_bat_resume(struct platform_device *pdev)
{
	struct mxs_info *info = platform_get_drvdata(pdev);
	ddi_bc_Cfg_t *cfg = info->sm_cfg;

	mutex_lock(&info->sm_lock);

	resume_power_reg(info);

	if (info->powersource == NORMAL_SOURCE) {
		if (is_ac_online()) {
			/* ac supply connected */
			dev_dbg(info->dev, "ac/5v present, enabling state machine\n");

			info->is_ac_online = 1;
			ddi_bc_RampSetLimit(NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA /*mA*/);
			ddi_bc_SetEnable();
		} else {
			/* not powered */
			dev_dbg(info->dev, "%s: 5v not present\n", __func__);

			info->is_ac_online = 0;
		}

		/* enable 5v irq */
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO, HW_POWER_CTRL_SET);

		/* reschedule calls to state machine */
		mod_timer(&info->sm_timer,
			jiffies + msecs_to_jiffies(cfg->u32StateMachinePeriod));
	}
	mutex_unlock(&info->sm_lock);
	return 0;
}

#else
#define mxs_bat_suspend NULL
#define mxs_bat_resume  NULL
#endif

static const struct of_device_id mxs_power_dt_ids[] = {
	{ .compatible = "fsl,imx28-power", },
	{ }
};

static struct platform_driver mxs_batdrv = {
	.probe		= mxs_bat_probe,
	.remove		= mxs_bat_remove,
	.shutdown       = mxs_bat_shutdown,
	.suspend	= mxs_bat_suspend,
	.resume		= mxs_bat_resume,
	.driver		= {
		.name	= "mxs-battery",
		.owner	= THIS_MODULE,
		.of_match_table = mxs_power_dt_ids,
	},
};

static int __init mxs_bat_init(void)
{
	printk("%s...\n", __FUNCTION__);
	return platform_driver_register(&mxs_batdrv);
}

static void __exit mxs_bat_exit(void)
{
	platform_driver_unregister(&mxs_batdrv);
}

#ifdef CONFIG_MXS_VBUS_CURRENT_DRAW
	fs_initcall(mxs_bat_init);
#else
	module_init(mxs_bat_init);
#endif
module_exit(mxs_bat_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steve Longerbeam <stevel@embeddedalley.com>");
MODULE_DESCRIPTION("Linux glue to MXS battery state machine");
