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
#include <linux/jiffies.h>
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
#include "mxs-battery.h"
#include "regs-lradc.h"
#include "regs-power.h"
#include "ddi_power_battery.h"



#define to_mxs_info(x) container_of((x), struct mxs_info, bat)

uint32_t batdbg = 0;
module_param_named(batdbg, batdbg, int, S_IRUGO | S_IWUSR | S_IWGRP);

//#define  POWER_FIQ

/* #define DEBUG_IRQS */

/* There is no direct way to detect wall power presence, so assume the AC
 * power source is valid if 5V presents and USB device is disconnected.
 * If USB device is connected then assume that AC is offline and USB power
 * is online.
 */


/*
 * ring buf of irq event.
 *
 * wr_idx: write index, write from here
 * rd_idx: read  index, read from here
 *
 * item is writable if it's -1, readable if it's not -1, so after read, have to set it as -1
 */
#define BAT_MAX_EVENTS 20
#define BAT_NEXT_INDEX(n) ((((n) + 1) == BAT_MAX_EVENTS) ? 0 : ((n) + 1))
static int event_log[BAT_MAX_EVENTS];
static int rd_idx = 0;
static int wr_idx = 0;

char* stat_5v_names[] = {
	"EVENT 5v /|\\",       /* STAT_5V_ON_NEW */
	"EVENT 5v on stable",  /* STAT_5V_ON_STABLE */
	"EVENT 5v \\|/",       /* STAT_5V_OFF_NEW */
	"EVENT 5v off half",   /* STAT_5V_OFF_HALF */
	"EVENT 5v off stable", /* STAT_5V_OFF_STABLE */
};

int mxs_pwr_irqs[NUMS_MXS_PWR_IRQS];
/*void __iomem *mxs_digctl_base;*/
void __iomem *mxs_rtc_base;
void __iomem *mxs_lradc_base;
void __iomem *mxs_pinctrl_base;

extern void (*pm_power_off)(void);

EXPORT_SYMBOL(mxs_lradc_base);

void mxs_pm_power_off(void)
{
	/* turn off vbat_gsm, gpio_3_28 output 0 */
	__raw_writel(1<<28, mxs_pinctrl_base + 0x738); /* OUT: clr bit */
	__raw_writel(1<<28, mxs_pinctrl_base + 0xb34); /* OE:  set bit */

	/* turn off fec_3v3, gpio_0_26, output 1 */
	__raw_writel(1<<26, mxs_pinctrl_base + 0x704); /* OUT: set bit */
	__raw_writel(1<<26, mxs_pinctrl_base + 0xb00); /* OE:  set bit */

	/* turn off usb0/1 vbus, gpio_4_11, gpio_4_12, output 0 */
	__raw_writel(3<<11, mxs_pinctrl_base + 0x748); /* OUT: clr bit */
	__raw_writel(3<<11, mxs_pinctrl_base + 0xb44); /* OE:  set bit */
	/* turn off vccio_3v3, gpio_4_14, output 0 */
	__raw_writel(1<<14, mxs_pinctrl_base + 0x748); /* OUT: clr bit */
	__raw_writel(1<<14, mxs_pinctrl_base + 0xb44); /* OE:  set bit */

	/* clear auto-restart bit */
	__raw_writel(0x20000, mxs_rtc_base+0x68);

	mxspwr_shutdown("usr request");
}

void save_irq_event(enum MXS_IRQ_EVENT evt)
{
	if ((EVNT_NONE) == event_log[wr_idx]) {
		event_log[wr_idx] = evt;
		wr_idx = BAT_NEXT_INDEX(wr_idx);
	}
}
bool has_irq_event(void)
{
	return (((EVNT_NONE) != event_log[rd_idx]) ? true : false);
}
void fetch_irq_event(struct mxs_info *info)
{
	enum MXS_IRQ_EVENT evt = EVNT_NONE;
	int event_num = 0;
	bool missed = false;
	uint32_t timeout_ms;
	uint32_t now;


	while (EVNT_NONE != event_log[rd_idx]) {
			evt = event_log[rd_idx];
			event_log[rd_idx] = EVNT_NONE;
			rd_idx = BAT_NEXT_INDEX(rd_idx);

		switch (evt) {
			case EVNT_5V_OFF:
				info->state_5v = STAT_5V_OFF_NEW;
				event_num++;
				break;
			case EVNT_5V_ON:
				info->state_5v = STAT_5V_ON_NEW;
				event_num++;
				break;
			default:
				BATT_LOG("[BAT] Error: unknown event %d\n", evt);
				break;
		}
	}

	/* double check for missed irq */
	if ((STAT_5V_ON_STABLE == info->state_5v) || (STAT_5V_ON_NEW == info->state_5v)) {
		if (!PWRREG_STS_IS_5VON()) { /* missed 5v off irq */
			PWRREG_ENABLE_VDDDAIO_INT(false);
			info->state_5v = STAT_5V_OFF_NEW;
			event_num++;
			missed = true;
		}
	} else {
		if (PWRREG_STS_IS_5VON()) {  /* missed 5v on irq */
			info->state_5v = STAT_5V_ON_NEW;
			event_num++;
			missed = true;
		}
	}

	if (!event_num) {
		switch (info->state_5v) {
			case STAT_5V_OFF_NEW:  /* expecting shift -> OFF_HALF */
				timeout_ms = _5V_OFF_HALF_DEBOUNCE;
				break;
			case STAT_5V_OFF_HALF: /* expecting shift -> OFF_STABLE */
				timeout_ms = _5V_OFF_STABLE_DEBOUNCE;
				break;
			case STAT_5V_ON_NEW:   /* expecting shift -> ON_STABLE */
				timeout_ms = _5V_ON_STABLE_DEBOUNCE;
				break;
			default:               /* timeout_ms(0) indicates not expecting state shift */
				timeout_ms = 0;
				break;
		}

		now = jiffies_to_msecs(jiffies);
		if (    timeout_ms                                 /* expecting state shift */
			 && (MXS_DELTA(info->stamp_5v, now) >= timeout_ms) /* and expired */
		   )
		{
			info->state_5v++;
			event_num++;
		}
	}

	if (event_num) {
		/* something did happend during this loop, mark the new stamp */
		info->stamp_5v = jiffies_to_msecs(jiffies);
		BATT_LOG("[BAT] %s %d%s\n", stat_5v_names[info->state_5v], event_num, missed?"(missed)":"");
	}
}

/* Power properties for 5v ac */
static enum power_supply_property mxs_5v_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
static int mxs_5v_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mxs_info *info;

	switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
				info = container_of(psy, struct mxs_info, ac);
				val->intval = PWRREG_STS_IS_5VON();
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

	switch (psp) {
		case POWER_SUPPLY_PROP_STATUS:                           /* "status" property */
			if (BAT_STAT_CHARGING == info->bat_state)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			else if (BAT_STAT_FULL_CHRGD == info->bat_state)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;

		case POWER_SUPPLY_PROP_PRESENT:                          /* "present" property */
			if (BAT_STAT_WAIT_BAT == info->bat_state)
				val->intval = 0;
			else
				val->intval = 1;
			break;

		case POWER_SUPPLY_PROP_HEALTH:                           /* "health" property */
			if (info->die_tmp_alarm)
				val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			else
				val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:                       /* "technology" property */
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:                      /* "voltage_now" property, uV */
			val->intval = PWRREG_GET_BATVOL() * 1000;
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:                      /* "current_now" property, uA */
			val->intval = mxspwr_get_charging_cur() * 1000;
			break;

		case POWER_SUPPLY_PROP_TEMP:                             /* "temp" property, die temp in celsius */
			val->intval = info->die_tmp;
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
	int ret;

	ret = schedule_work(&info->sm_work);
	if (!ret)
		dev_dbg(info->dev, "state machine failed to schedule\n");

	/* schedule next call to state machine */
	mod_timer(&info->sm_timer, jiffies + msecs_to_jiffies(BC_ROUTINE_INTV));

}
/*
 * [luheng] real state machine handler, schedule by timer per 100ms
 */
static void state_machine_work(struct work_struct *work)
{
	struct mxs_info *info = container_of(work, struct mxs_info, sm_work);

	batt_sm_routine(info);
}

static irqreturn_t mxs_irq_vddio_brnout(int irq, void *cookie) { mxspwr_shutdown("vddio \\|/"); return IRQ_HANDLED; }
static irqreturn_t mxs_irq_batt_brnout (int irq, void *cookie) { mxspwr_shutdown("batt \\|/");  return IRQ_HANDLED; }
static irqreturn_t mxs_irq_vddd_brnout (int irq, void *cookie) { mxspwr_shutdown("vddd \\|/");  return IRQ_HANDLED; }
static irqreturn_t mxs_irq_vdda_brnout (int irq, void *cookie) { mxspwr_shutdown("vdda \\|/");  return IRQ_HANDLED; }

/*static irqreturn_t mxs_irq_dcdc4p2_bo(int irq, void *cookie)
{
	ddi_power_handle_dcdc4p2_bo();
	return IRQ_HANDLED;
}*/

/*
 * [luheng]
 * 5v-droop irq handler.
 *		1. set 4p2 target = batt vol, 4p2<->batt cmptrip = (1.05 * batt)
 *		2. if batt-BO, shut down immediately
 *		3. disable vddio/4p2/vddd BO INT, disable 5v-droop INT, only enable BATT-BO INT
 */
static irqreturn_t mxs_irq_vdd5v_droop(int irq, void *cookie)
{
	struct mxs_info *info = (struct mxs_info *)cookie;

	/* if batt-BO occurs, shutdown asap */
	if (RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_BATT_BO)
		mxspwr_shutdown("5v \\|/ wo batt");

	/* disable 5v-off INT, enable 5v-on INT */
	save_irq_event(EVNT_5V_OFF);
	PWRREG_ENABLE_5VOFF_INT(false);
	PWRREG_ENABLE_5VON_INT(true);
	PWRREG_TURN_CHRGER(false);
	mxspwr_set_charging_cur(0);

	/* due to 5v connect vddio bo chip bug, we need to disable vddio interrupts until 5v is stable
	 * either on or off.  We want to allow some debounce time before enabling connect detection. */
	PWRREG_ENABLE_VDDDAIO_INT(false);

	/* 4p2 as DCDC src criteria: (4p2 >= 1.05 * batt); set 4p2 target vol as batt vol */
	SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BF_POWER_DCDC4P2_CMPTRIP(0x1F) | BM_POWER_DCDC4P2_TRG);
	mod_timer(&info->sm_timer, jiffies + 1);

	return IRQ_HANDLED;
}


/*
 * [luheng]
 * 5v attach irq handler
 *   1. disable 5v-on irq, enable 5v-off irq
 *   2. save event stamp.
 */
static irqreturn_t mxs_irq_vdd5v(int irq, void *cookie)
{
	struct mxs_info *info = (struct mxs_info *)cookie;

	PWRREG_ENABLE_5VON_INT(false);
	PWRREG_ENABLE_5VOFF_INT(true);
	save_irq_event(EVNT_5V_ON);
	mod_timer(&info->sm_timer, jiffies + 1);
	return IRQ_HANDLED;
}

static int mxs_bat_init_regs(struct platform_device *pdev)
{
	struct device_node *np;

	np = pdev->dev.of_node;                                       mxs_pwr_base    = of_iomap(np, 0);
	//np = of_find_compatible_node(NULL, NULL, "fsl,imx28-digctl"); mxs_digctl_base = of_iomap(np, 0);
	np = of_find_compatible_node(NULL, NULL, "fsl,imx28-pinctrl"); mxs_pinctrl_base = of_iomap(np, 0);
	np = of_find_compatible_node(NULL, NULL, "fsl,stmp3xxx-rtc");  mxs_rtc_base     = of_iomap(np, 0);

	if (0 == mxs_lradc_base) {
		printk("battery: unbeliveable, lradc driver hasn't init base addrs!\n");
		np = of_find_compatible_node(NULL, NULL, "fsl,imx28-lradc");   mxs_lradc_base   = of_iomap(np, 0);
	} else {
		printk("battery: yes it is, lradc driver has inited base addrs!\n");
	}

	IRQ_BATT_BRNOUT    = platform_get_irq_byname(pdev, "batt_bo");
	IRQ_VDDD_BRNOUT    = platform_get_irq_byname(pdev, "vddd_bo");
	IRQ_VDDIO_BRNOUT   = platform_get_irq_byname(pdev, "vddio_bo");
	IRQ_VDDA_BRNOUT    = platform_get_irq_byname(pdev, "vdda_bo");
	IRQ_VDD5V_DROOP    = platform_get_irq_byname(pdev, "vdd5v_droop");
	IRQ_VDD5V          = platform_get_irq_byname(pdev, "vdd5v");
	//IRQ_DCDC4P2_BRNOUT = platform_get_irq_byname(pdev, "dcdc4p2_bo");

	if (    (IRQ_BATT_BRNOUT  < 0) || (IRQ_VDDD_BRNOUT < 0)
	     || (IRQ_VDDIO_BRNOUT < 0) || (IRQ_VDDA_BRNOUT < 0)
	     || (IRQ_VDD5V_DROOP  < 0) || (IRQ_VDD5V < 0) /*|| (IRQ_DCDC4P2_BRNOUT < 0)*/
	   )
	{
		printk("Battery: one of the irqs not specified in dtb!\n");
		return -ENXIO;
	}

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
	int index;
	struct mxs_info *info = platform_get_drvdata(pdev);

	if (NULL == info)
		return -EINVAL;

	for (index = 0; index < BAT_MAX_EVENTS; index++)
		event_log[index] = EVNT_NONE;

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
	
	/*if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_DCDC4P2_BRNOUT, mxs_irq_dcdc4p2_bo,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_DCDC4P2_BRNOUT\n");
		goto out;
	} else
		info->irq_dcdc4p2_bo = IRQ_DCDC4P2_BRNOUT;*/

	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_BATT_BRNOUT, mxs_irq_batt_brnout,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_BATT_BRNOUT\n");
		goto out;
	} else
		info->irq_batt_brnout = IRQ_BATT_BRNOUT;

	if ((ret = devm_request_threaded_irq(&(pdev->dev), IRQ_VDDD_BRNOUT, mxs_irq_vddd_brnout,
			NULL, 0, dev_name(&(pdev->dev)), info)))
	{
		dev_err(info->dev, "failed to request IRQ_VDDD_BRNOUT\n");
		goto out;
	} else
		info->irq_vddd_brnout = IRQ_VDDD_BRNOUT;

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
		info->lradc_clk = NULL;
	} else if (clk_prepare_enable(info->lradc_clk)) {
		printk("%s: failed to enable lradc clock!\n", __FUNCTION__);
		clk_put(info->lradc_clk);
		kfree(info);
		return NULL;
	}

	info->irq_vdd5v = info->irq_vddio_brnout = info->irq_dcdc4p2_bo = info->irq_batt_brnout =
		info->irq_vddd_brnout = info->irq_vdda_brnout = info->irq_vdd5v_droop = -1;

	platform_set_drvdata(pdev, info);

	mutex_init(&info->sm_lock);

	info->dev    = &pdev->dev;
	/* initialize bat power_supply struct */
	info->bat.name           = "battery";
	info->bat.type           = POWER_SUPPLY_TYPE_BATTERY;
	info->bat.properties     = mxs_bat_props;
	info->bat.num_properties = ARRAY_SIZE(mxs_bat_props);
	info->bat.get_property   = mxs_bat_get_property;
	/* initialize ac power_supply struct */
	info->ac.name           = "ac";
	info->ac.type           = POWER_SUPPLY_TYPE_MAINS;
	info->ac.properties     = mxs_5v_props;
	info->ac.num_properties = ARRAY_SIZE(mxs_5v_props);
	info->ac.get_property   = mxs_5v_get_property;

	init_timer(&info->sm_timer);
	info->sm_timer.data = (unsigned long)info;
	info->sm_timer.function = state_machine_timer;

	/* init batt charger para */
	info->die_tmp_high = 70;
	info->die_tmp_low  = 60;
	info->prv_die_tmp_alarm = false;
	info->die_tmp_alarm = false;
	INIT_WORK(&(info->sm_work), state_machine_work);

	return info;
}

static int mxs_bat_probe(struct platform_device *pdev)
{
	struct mxs_info *info = NULL;
	int ret = 0;

	if (NULL == (info = mxs_bat_init_info(pdev))) /* alloc & init mem, request lradc clock */
		return -ENOMEM;

	if ((ret = mxs_bat_init_regs(pdev))) {        /* init hw-module base address from dt */
		goto free_info;
	}

	if ((ret = mxspwr_init_bat())) {       /* enable batt vol measrement, enable 5v detect */
		printk(KERN_ERR "Aborting power driver initialization\n");
		goto free_info;
	}

	if (mxs_init_irqs(pdev))                      /* request battery irqs */
		goto free_info;

	if ((ret = power_supply_register(&pdev->dev, &info->bat))) {
		dev_err(info->dev, "failed to register battery\n");
		goto free_info;
	}

	if ((ret = power_supply_register(&pdev->dev, &info->ac))) {
		dev_err(info->dev, "failed to register ac power supply\n");
		goto unregister_bat;
	}

	pm_power_off = mxs_pm_power_off;

	/* handoff protection handling from bootlets protection method
	 * to kernel protection method */
	init_batt_sm(info);

	return 0;

unregister_bat:
	power_supply_unregister(&info->bat);

free_info:
	mxs_free_irqs(pdev);
	if (info->lradc_clk) {
		clk_disable_unprepare(info->lradc_clk);
		clk_put(info->lradc_clk);
	}
	platform_set_drvdata(pdev, NULL);
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
	//ddi_bc_SetDisable();
	/* cancel state machine timer */
	del_timer_sync(&info->sm_timer);
	backup_power_reg(info);

	mutex_unlock(&info->sm_lock);
	return 0;
}

static int mxs_bat_resume(struct platform_device *pdev)
{
	struct mxs_info *info = platform_get_drvdata(pdev);

	mutex_lock(&info->sm_lock);

	resume_power_reg(info);

	if (PWRREG_STS_IS_5VON()) {
		/* ac supply connected */
		dev_dbg(info->dev, "ac/5v present, enabling state machine\n");

	} else {
		/* not powered */
		dev_dbg(info->dev, "%s: 5v not present\n", __func__);
	}

	/* enable 5v irq */
	WR_PWR_REG(HW_POWER_CTRL_SET, BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO);

	/* reschedule calls to state machine */
	init_batt_sm(info);
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
