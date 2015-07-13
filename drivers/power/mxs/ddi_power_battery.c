#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/processor.h> /* cpu_relax */
#include "ddi_bc.h"
#include "regs-power.h"
#include "regs-lradc.h"
#include "ddi_bc_internal.h"
#include "mxs-battery.h"

/* brief Base voltage to start battery calculations for LiIon */
#define BATT_BRWNOUT_LIION_BASE_MV 2800

/* brief Constant to help with determining whether to round up or not during calculation */
#define BATT_BRWNOUT_LIION_CEILING_OFFSET_MV 39

/* brief Number of mV to add if rounding up in LiIon mode */
#define BATT_BRWNOUT_LIION_LEVEL_STEP_MV 40

/* brief Constant value to be calculated by preprocessing */
#define BATT_BRWNOUT_LIION_EQN_CONST \
	(BATT_BRWNOUT_LIION_BASE_MV - BATT_BRWNOUT_LIION_CEILING_OFFSET_MV)

/* brief Base voltage to start battery calculations for Alkaline/NiMH */
#define BATT_BRWNOUT_ALKAL_BASE_MV 800

/* brief Constant to help with determining whether to round up or */
/*  not during calculation */
#define BATT_BRWNOUT_ALKAL_CEILING_OFFSET_MV 19

/* brief Number of mV to add if rounding up in Alkaline/NiMH mode */
#define BATT_BRWNOUT_ALKAL_LEVEL_STEP_MV 20

/* brief Constant value to be calculated by preprocessing */
#define BATT_BRWNOUT_ALKAL_EQN_CONST \
	(BATT_BRWNOUT_ALKAL_BASE_MV - BATT_BRWNOUT_ALKAL_CEILING_OFFSET_MV)

#define GAIN_CORRECTION 1012    /* 1.012 */

#define VBUSVALID_THRESH_2_90V		0x0
#define VBUSVALID_THRESH_4_00V		0x1
#define VBUSVALID_THRESH_4_10V		0x2
#define VBUSVALID_THRESH_4_20V		0x3
#define VBUSVALID_THRESH_4_30V		0x4
#define VBUSVALID_THRESH_4_40V		0x5
#define VBUSVALID_THRESH_4_50V		0x6
#define VBUSVALID_THRESH_4_60V		0x7

#define LINREG_OFFSET_STEP_BELOW	0x2
#define BP_POWER_BATTMONITOR_BATT_VAL	16
#define BP_POWER_CHARGE_BATTCHRG_I	0
#define BP_POWER_CHARGE_STOP_ILIMIT	8

#define VDD4P2_ENABLED

#define DDI_POWER_BATTERY_XFER_THRESHOLD_MV 3200


#ifndef BATTERY_VOLTAGE_CMPTRIP100_THRESHOLD_MV
#define BATTERY_VOLTAGE_CMPTRIP100_THRESHOLD_MV 4000
#endif

#ifndef BATTERY_VOLTAGE_CMPTRIP105_THRESHOLD_MV
#define BATTERY_VOLTAGE_CMPTRIP105_THRESHOLD_MV 3800
#endif

/* #define DEBUG_IRQS */

/* to be re-enabled once FIQ functionality is added */
#define DISABLE_VDDIO_BO_PROTECTION

/* Select your 5V Detection method */
static uint32_t bit_irqen_presence5v = BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO;
static ddi_power_5vDetection_t DetectionMethod = DDI_POWER_5V_VDD5V_GT_VDDIO;
/* static uint32_t bit_irqen_presence5v = BM_POWER_CTRL_ENIRQ_VBUS_VALID */
/* static ddi_power_5vDetection_t DetectionMethod = DDI_POWER_5V_VBUSVALID; */


/* current represented by bitfields HW_POWER_CHARGE.STOP_ILIMIT and HW_POWER_CHARGE.BATTCHRG_I.
 *
 *                                       bit0 bit1 bit2 bit3 bit4 bit5  */
static const uint16_t currentPerBit[] = {  10,  20,  50, 100, 200, 400 };

/* [luheng] convert a current value to bitfields, where
 *     bit0:10mA, bit1:20mA, bit2:50mA, bit3:100mA, bit4:200mA, bit5:400mA
 */
uint16_t ddi_power_convert_current_to_setting(uint16_t u16Current)
{
	int       i;
	uint16_t  u16Mask = (0x1 << 5); /* start from hightest bit, highest current */
	uint16_t  u16Setting = 0;

	for (i = 5; (i >= 0) && (u16Current > 0); i--, u16Mask >>= 1) {
		if (u16Current >= currentPerBit[i]) {
			u16Current -= currentPerBit[i];
			u16Setting |= u16Mask;
		}
	}
	return u16Setting;
}


/* [luheng] convert a bitfields to current value where
 *     bit0:10mA, bit1:20mA, bit2:50mA, bit3:100mA, bit4:200mA, bit5:400mA
 */
uint16_t ddi_power_convert_setting_to_current(uint16_t u16Setting)
{
	int       i;
	uint16_t  u16Mask = (0x1 << 5); /* start from hightest bit, highest current */
	uint16_t  u16Current = 0;

	for (i = 5; i >= 0; i--, u16Mask >>= 1) {
		if (u16Setting & u16Mask)
			u16Current += currentPerBit[i];
	}
	return u16Current;
}

/* [luheng]
 *		1. Don't power down device for 5v-BO
 *		2. Turn on VBUSVALID comparator(>4.3v)
 *		3. Set vddio/vdda/vddd linreg output 25mv lower than DCDC counterpart
 *		4. clear 5v-presence irq stats and enable the irq(VBUSVALID or VDD5V-GT-VDDIO).
 */
void ddi_power_Enable5vDetection(void)
{
	u32 setbits;

	/* Don't power down device when 5V-BO happens */
	WR_PWR_REG(BM_POWER_5VCTRL_PWDN_5VBRNOUT, HW_POWER_5VCTRL_CLR);

	/* Turn on VBUSVALID comparator(>4.3v) even if VDD5V_GT_VDDIO is used for 5V presence
	 * detection, in case other drivers (e.g. USB) might also monitor VBUSVALID status */
	WR_PWR_REG(BM_POWER_5VCTRL_VBUSVALID_5VDETECT, HW_POWER_5VCTRL_SET);
	WR_PWR_REG(BF_POWER_5VCTRL_VBUSVALID_TRSH(VBUSVALID_THRESH_4_30V), HW_POWER_5VCTRL_SET);

	/* set vddio/vdda/vddd lingreg output 1-step-below(25mv) DCDC counterparts,
	 * standard practive when lingreg and DCDC are all on */
	setbits = BF_POWER_VDDIOCTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW);
	CLR_SET_PWR_REG_BITS(HW_POWER_VDDIOCTRL, BM_POWER_VDDIOCTRL_LINREG_OFFSET, setbits);

	setbits = BF_POWER_VDDACTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW);
	CLR_SET_PWR_REG_BITS(HW_POWER_VDDACTRL, BM_POWER_VDDACTRL_LINREG_OFFSET, setbits);

	setbits = BF_POWER_VDDDCTRL_LINREG_OFFSET(LINREG_OFFSET_STEP_BELOW);
	CLR_SET_PWR_REG_BITS(HW_POWER_VDDDCTRL, BM_POWER_VDDDCTRL_LINREG_OFFSET, setbits);

	/* Clear vbusvalid & vdd5v-gt-vddio interrupt flag */
	WR_PWR_REG(BM_POWER_CTRL_VBUSVALID_IRQ,     HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ, HW_POWER_CTRL_CLR);

	/* enable either vbusvalid or vdd5v-gt-vddio irq, according to global */
	WR_PWR_REG(bit_irqen_presence5v, HW_POWER_CTRL_SET);
}

/*
 * [luheng]
 * switch 5v-detection: detect on -> detect off
 *
 * This function prepares the hardware for a 5V-to-battery handoff. It assumes
 * the current configuration is using 5V as the power source.  The 5V
 * interrupt will be set up for a 5V removal.
 */
void ddi_power_enable_5v_to_battery_handoff(void)
{
	/* Clear vbusvalid interrupt flag */
	WR_PWR_REG(BM_POWER_CTRL_VBUSVALID_IRQ, HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ, HW_POWER_CTRL_CLR);

	/* TO detect 5v unplug */
	WR_PWR_REG(BM_POWER_CTRL_POLARITY_VBUSVALID, HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO, HW_POWER_CTRL_CLR);

#ifndef VDD4P2_ENABLED
	/* Enable automatic transition to DCDC */
	WR_PWR_REG(BM_POWER_5VCTRL_DCDC_XFER, HW_POWER_5VCTRL_SET);
#endif
}

/*
 * [luheng]
 * switch source from 5v to battery, this is called when 5v-droop detected.
 *
 * turn off 5v->charger_&_4p2 output, turn off 5v->4p2 linreg, turn off 4p2->DCDC input
 * set VBUSVALID to 4.4v for 5v_presence detection for future 5v plug again.
 */
void ddi_power_execute_5v_to_battery_handoff(void)
{
	/* disable 5v->4p2 linreg, disable 4p2 as DCDC input */
	CLR_PWR_REG_BITS(HW_POWER_DCDC4P2, (BM_POWER_DCDC4P2_ENABLE_DCDC | BM_POWER_DCDC4P2_ENABLE_4P2));

	/* turn off 5v outputs to 4p2&charger */
	WR_PWR_REG(BM_POWER_5VCTRL_PWD_CHARGE_4P2, HW_POWER_5VCTRL_SET);

	/* set VBUSVALID_TRSH 4400mV */
	WR_PWR_REG(BM_POWER_5VCTRL_VBUSVALID_TRSH, HW_POWER_5VCTRL_CLR);
	WR_PWR_REG(BF_POWER_5VCTRL_VBUSVALID_TRSH(VBUSVALID_THRESH_4_40V), HW_POWER_5VCTRL_SET);
}

/*
 * [luheng]
 * switch 5v-detection: detect off -> detect on, assure DCDC active after 5v is on
 *
 * This function sets up battery-to-5V handoff. The power switch from
 * battery to 5V is automatic. This funtion enables the 5V present detection
 * such that the 5V interrupt can be generated if it is enabled. (The interrupt
 * handler can inform software the 5V present event.) To deal with noise or
 * a high current, this function enables DCDC1/2 based on the battery mode.
 */
void ddi_power_enable_battery_to_5v_handoff(void)
{
	/* Clear 5v-presence irq flag */
	WR_PWR_REG(BM_POWER_CTRL_VBUSVALID_IRQ, HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ, HW_POWER_CTRL_CLR);

	/* prepare to detect 5v plug-in */
	WR_PWR_REG(BM_POWER_CTRL_POLARITY_VBUSVALID, HW_POWER_CTRL_SET);
	WR_PWR_REG(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO, HW_POWER_CTRL_SET);

	/* Allow DCDC be to active when 5V is present. */
	WR_PWR_REG(BM_POWER_5VCTRL_ENABLE_DCDC, HW_POWER_5VCTRL_SET);
}

/*
 * [luheng] transition DCDC source from batt to 4p2, 5v->charger_&_4p2 limit is 450mA.
 *
 * This function handles the transitions on each of theVDD5V_GT_VDDIO power
 * rails necessary to power the chip from the 5V power supply when it was
 * previously powered from the battery power supply.
 */
void ddi_power_execute_battery_to_5v_handoff(void)
{
	ddi_power_Enable4p2(450);
}

/*
 * [luheng] turn on 4p2->DCDC input route elegantly.(avoiding false BO, droop, plug/unplug detection).
 *		parameter: battery_ready, unused!
 */
void ddi_power_Start4p2Dcdc(bool battery_ready)
{
	uint32_t temp_reg, old_values;
	bool vdda_pwdn = false, vddd_pwdn = false, vddio_pwdn = false;

	SET_PWR_REG_BITS(HW_POWER_BATTMONITOR, BM_POWER_BATTMONITOR_PWDN_BATTBRNOUT); /* enable batt-BO shutdown */
	WR_PWR_REG(BM_POWER_5VCTRL_VBUSDROOP_TRSH, HW_POWER_5VCTRL_CLR)               /* VBUS DROOP thresh = 4.3V */;
	WR_PWR_REG(BM_POWER_5VCTRL_PWRUP_VBUS_CMPS, HW_POWER_5VCTRL_SET);             /* enable all 5v-comparators, erratte workaround */
	WR_PWR_REG(BM_POWER_5VCTRL_VBUSVALID_5VDETECT, HW_POWER_5VCTRL_CLR);          /* turn on VBUSVALID comparator */

	/* store vddio/vddd/vdda BO-shutdown triggers, disable triggers, clear BO-irq stats */
	if (RD_PWR_REG(HW_POWER_VDDIOCTRL) & BM_POWER_VDDIOCTRL_PWDN_BRNOUT) vddio_pwdn = true;
	if (RD_PWR_REG(HW_POWER_VDDDCTRL ) & BM_POWER_VDDDCTRL_PWDN_BRNOUT ) vddd_pwdn  = true;
	if (RD_PWR_REG(HW_POWER_VDDACTRL ) & BM_POWER_VDDACTRL_PWDN_BRNOUT ) vdda_pwdn  = true;
	CLR_PWR_REG_BITS(HW_POWER_VDDACTRL,  BM_POWER_VDDACTRL_PWDN_BRNOUT);
	CLR_PWR_REG_BITS(HW_POWER_VDDDCTRL,  BM_POWER_VDDDCTRL_PWDN_BRNOUT);
	CLR_PWR_REG_BITS(HW_POWER_VDDIOCTRL, BM_POWER_VDDIOCTRL_PWDN_BRNOUT);
	if ((RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_VDDIO_BO) == 0) WR_PWR_REG(BM_POWER_CTRL_VDDIO_BO_IRQ, HW_POWER_CTRL_CLR);
	if ((RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_VDDD_BO ) == 0) WR_PWR_REG(BM_POWER_CTRL_VDDD_BO_IRQ,  HW_POWER_CTRL_CLR);
	if ((RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_VDDA_BO ) == 0) WR_PWR_REG(BM_POWER_CTRL_VDDA_BO_IRQ,  HW_POWER_CTRL_CLR);

	/* save and disable possible false irq en-bit */
	temp_reg = (BM_POWER_CTRL_ENIRQ_VDDD_BO  | BM_POWER_CTRL_ENIRQ_VDDA_BO | BM_POWER_CTRL_ENIRQ_VDDIO_BO
		      | BM_POWER_CTRL_ENIRQ_VDD5V_DROOP | BM_POWER_CTRL_ENIRQ_VBUS_VALID);
	old_values = RD_PWR_REG(HW_POWER_CTRL) & temp_reg;
	WR_PWR_REG(temp_reg, HW_POWER_CTRL_CLR);

	/* !!!!!!!!!!enable 4p2->DCDC input. This is the purpose of this function!!!!!!!!!! */
	SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_ENABLE_DCDC);
	mdelay(1);

	/* clear possible false irq stat-bit */
	temp_reg = (BM_POWER_CTRL_VDDD_BO_IRQ  | BM_POWER_CTRL_VDDA_BO_IRQ | BM_POWER_CTRL_VDDIO_BO_IRQ
		      | BM_POWER_CTRL_VDD5V_DROOP_IRQ | BM_POWER_CTRL_VBUSVALID_IRQ);
	WR_PWR_REG(temp_reg, HW_POWER_CTRL_CLR);

	/* loop until the false BO disappear or 5V actually off */
	while ((RD_PWR_REG(HW_POWER_CTRL) & temp_reg) && (!(RD_PWR_REG(HW_POWER_CTRL) & BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ))) {
		WR_PWR_REG(temp_reg, HW_POWER_CTRL_CLR);
		mdelay(1);
	}
	/* restore interested irq-en bits, restore BO-shutdown triggers */
	WR_PWR_REG(old_values, HW_POWER_CTRL_SET);
	if (vdda_pwdn) SET_PWR_REG_BITS(HW_POWER_VDDACTRL,  BM_POWER_VDDACTRL_PWDN_BRNOUT);
	if (vddd_pwdn) SET_PWR_REG_BITS(HW_POWER_VDDDCTRL,  BM_POWER_VDDDCTRL_PWDN_BRNOUT);
	if (vddio_pwdn)SET_PWR_REG_BITS(HW_POWER_VDDIOCTRL, BM_POWER_VDDIOCTRL_PWDN_BRNOUT);

	if (DetectionMethod == DDI_POWER_5V_VBUSVALID)
		WR_PWR_REG(BM_POWER_5VCTRL_VBUSVALID_5VDETECT, HW_POWER_5VCTRL_SET);
}


/* [luheng]
 * Set 4p2 VS batt comparator trip value.
 * Ideal: use 4p2 as much as possible. the only condition is 4p2 can fulfill the use
 *
 * set the optimal CMPTRIP for the best possible 5V
 * disconnection handling but without drawing power
 * from the power on a stable 4p2 rails (at 4.2V).
 */
void ddi_power_handle_cmptrip(void)
{
	enum ddi_power_5v_status pmu_5v_status;
	uint32_t setbits = 0;
	uint16_t batt_vol;

	pmu_5v_status = ddi_power_GetPmu5vStatus();
	batt_vol = ddi_power_GetBattery();

	if (pmu_5v_status != existing_5v_connection)                    /* 5v changed. or remains off */
		setbits = (31 << BP_POWER_DCDC4P2_CMPTRIP);                 /*     4p2 >= 1.05 * batt     */
	else if (batt_vol > BATTERY_VOLTAGE_CMPTRIP100_THRESHOLD_MV)    /* bat > 4.0v                 */
		setbits = (1 << BP_POWER_DCDC4P2_CMPTRIP);                  /*     4p2 >= 0.86 * batt     */
	else if (batt_vol > BATTERY_VOLTAGE_CMPTRIP105_THRESHOLD_MV)    /* bat > 3.8v                 */
		setbits = (24 << BP_POWER_DCDC4P2_CMPTRIP);                 /*     4p2 >= 1.00 * batt     */
	else                                                            /* else                       */
		setbits = (31 << BP_POWER_DCDC4P2_CMPTRIP);                 /*     4p2 >= 1.05 * batt     */

	CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_CMPTRIP, setbits);
}

/*
 * [luheng]
 * Set DCDC source to max(4p2,batt), set 4p2-batt-cmptrip to use 4p2 source as much as possible.
 */
void ddi_power_Init4p2Params(void)
{
	uint32_t temp;

	/* adjust 4p2-batt-cmptrip to use 4p2 as much as possible. */
	ddi_power_handle_cmptrip();

	/* 4p2 out: 4.2v, DROPOUT_CTRL(b1010): DCDC source = max(4p2,batt), 4p2BO:100mv below TRG */
	CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2,
		(BM_POWER_DCDC4P2_TRG | BM_POWER_DCDC4P2_DROPOUT_CTRL),
		(0xa << BP_POWER_DCDC4P2_DROPOUT_CTRL));

	/* useless: read but not written back. HEADROOM_ADJ to 4, CHARGE_4P2_ILIMIT to 0 */
	temp = RD_PWR_REG(HW_POWER_5VCTRL);
	temp &= ~(BM_POWER_5VCTRL_HEADROOM_ADJ | BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT);
	temp |= (4 << BP_POWER_5VCTRL_HEADROOM_ADJ);
}

/* [luheng] check if (batt-vol > 3.2v) */
bool ddi_power_IsBattRdyForXfer(void)
{
	uint16_t u16BatteryVoltage = ddi_power_GetBattery();

	if (u16BatteryVoltage > DDI_POWER_BATTERY_XFER_THRESHOLD_MV)
		return true;
	else
		return false;
}

/* [luheng] enable 5v-droop irq after clear associated irq stat */
void ddi_power_EnableVbusDroopIrq(void)
{
	WR_PWR_REG(BM_POWER_CTRL_VDD5V_DROOP_IRQ,   HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDD5V_DROOP, HW_POWER_CTRL_SET);
}

/* [luheng] turn on 5v -> charger_&_4p2 -> DCDC, if battery not damaged; or just enable batt-BO irq if batt damaged */
void ddi_power_Enable4p2(uint16_t target_current_limit_ma)
{
	uint32_t temp_reg;

	ddi_power_Init4p2Params(); /* configure DCDC source as max(4p2,batt), 4p2 as much as possible */

	WR_PWR_REG(BM_POWER_CTRL_ENIRQ_DCDC4P2_BO, HW_POWER_CTRL_CLR);               /* disable 4p2-BO-shutdown trigger */

	if (ddi_power_GetBattery() > DDI_POWER_BATTERY_XFER_THRESHOLD_MV) {          /* batt usable(> 3.2v) */
		WR_PWR_REG(BM_POWER_5VCTRL_PWD_CHARGE_4P2, HW_POWER_5VCTRL_SET);         /* turn off 5v->4p2 linreg */

		CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_CMPTRIP, (31 << BP_POWER_DCDC4P2_CMPTRIP)); /* cmptrip:4p2>batt */

		ddi_power_Start4p2Dcdc(true);                                            /* turn on 4p2->DCDC route */
		WR_PWR_REG(BM_POWER_5VCTRL_VBUSDROOP_TRSH, HW_POWER_5VCTRL_CLR);         /* enable 5v-droop(VBUSDROOP), thresh=4.3V */
		ddi_power_EnableVbusDroopIrq();
		temp_reg = ddi_power_BringUp4p2Regulator(target_current_limit_ma, true); /* turn on 5v->4p2 linreg */

		/* if we still 5V state unchanged, disable batt-BO irq.  This is because the
		 * VDD5VDROOP IRQ handler will also shutdown if batt-BO occurs and it will enable the batt-BO
		 * and bring VBUSVALID_TRSH level back to a normal level which caused the hardware batt-BO shutdown
		 * to be enabled.  The benefit of this is that device that have detachable batteries (or devices going through
		 * the assembly line and running this firmware to test with) can avoid shutting down if 5V is present and
		 * battery voltage goes away. */
		if (!(RD_PWR_REG(HW_POWER_CTRL) & (BM_POWER_CTRL_VBUSVALID_IRQ | BM_POWER_CTRL_VDD5V_DROOP_IRQ))) {
			ddi_power_EnableBatteryBoInterrupt(false);
		}

		printk(KERN_DEBUG "4P2 rail started.  5V current limit set to %dmA\n",	temp_reg);

	} else {                                                                     /* batt not usable */
		printk(KERN_ERR "4P2 rail was attempted to be started from a system\
			                   with a very low battery voltage.  This is\
			                   not yet handled by the kernel driver, only\
			                   by the bootlets. Remaining on battery power.\n");
		if (RD_PWR_REG(HW_POWER_5VCTRL) & BM_POWER_5VCTRL_ENABLE_DCDC)           /* enable batt-BO irq if DCDC stays active on 5v? */
			ddi_power_EnableBatteryBoInterrupt(true);

	}

}

/* [luheng] turn on 5v->charger_&_4p2 route and ramp up current limit of (charger + 4p2)
 *     1. enable 5v->4p2 linreg, output target: 4.2v, 4p2-BO thresh: 3.6v
 *     2. turn on 5v->charger_&_4p2 route
 *     3. ramp up current limit from 0 to 'target_current_limit_ma'
 *
 * para:
 *		target_current_limit_ma    current limit of (charger + 4p2)
 *		b4p2_dcdc_enabled          whether 4p2->DCDC route turned on? to guide how to ramp up
 */
uint16_t ddi_power_BringUp4p2Regulator(uint16_t target_current_limit_ma, bool b4p2_dcdc_enabled)
{
	uint16_t charge_4p2_ilimit = 0;

	WR_PWR_REG(BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT, HW_POWER_5VCTRL_CLR); /* 5v->charger_&_4p2 cur limit: 0 */
	SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_ENABLE_4P2);    /* enable 5v->4p2 linreg */
	CLR_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_TRG);           /* 4p2 linreg output target: 4.2v */
	WR_PWR_REG(BM_POWER_5VCTRL_PWD_CHARGE_4P2, HW_POWER_5VCTRL_CLR);    /* turn on 5v->charger_&_4p2 route */

	if (target_current_limit_ma > 780)
		target_current_limit_ma = 780;

	ddi_power_Set4p2BoLevel(4150);                                      /* 4p2-BO thresh: 4.15v, temp for ramp up current limit */
	WR_PWR_REG(BM_POWER_CHARGE_ENABLE_LOAD, HW_POWER_CHARGE_SET);       /* enable 100ohm load on 4p2 output temporarily for ramp up */

	while (charge_4p2_ilimit < target_current_limit_ma) {
		if (RD_PWR_REG(HW_POWER_CTRL) & (BM_POWER_CTRL_VBUSVALID_IRQ | BM_POWER_CTRL_VDD5V_DROOP_IRQ))
			break;                                                      /* break if 5v disappears */

		charge_4p2_ilimit += 100;
		if (charge_4p2_ilimit > target_current_limit_ma)
			charge_4p2_ilimit = target_current_limit_ma;

		ddi_power_set_4p2_ilimit(charge_4p2_ilimit);

		/* 4p2-BO is detected by DCDC, so if 4p2->DCDC route is off, we cannot detect 4p2-BO. */
		if (!(b4p2_dcdc_enabled)) /* 4p2-BO not able to detect, ramp-up by step carefully */
			msleep(1);
		else {                    /* 4p2-BO able to detect, detect it */
			if	(RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_DCDC_4P2_BO)
				msleep(1);        /* 4p2-BO occurs, ramp-up by step */
			else {                /* no 4p2-BO, set ultimate limit directly */
				charge_4p2_ilimit = target_current_limit_ma;
				ddi_power_set_4p2_ilimit(charge_4p2_ilimit);
			}
		}
	}

	ddi_power_Set4p2BoLevel(3600);                                      /* eventual 4p2-BO thresh for normal use */
	WR_PWR_REG(BM_POWER_CTRL_DCDC4P2_BO_IRQ, HW_POWER_CTRL_CLR);        /* clear 4p2-BO irq stat */
	WR_PWR_REG(BM_POWER_CHARGE_ENABLE_LOAD, HW_POWER_CHARGE_CLR);       /* disable 100ohm for normal use */

	return charge_4p2_ilimit;

}

/* [luheng] set 4p2-BO thresh, unit:mv.
 *     Note: 4p2-BO can only be detected if 4p2->DCDC input route is turned on */
void ddi_power_Set4p2BoLevel(uint16_t bo_voltage_mv)
{
	uint16_t bo_reg_value;

	if (bo_voltage_mv < 3600)
		bo_voltage_mv = 3600;
	else if (bo_voltage_mv > 4375)
		bo_voltage_mv = 4375;

	bo_reg_value = (bo_voltage_mv - 3600) / 25;

	CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_BO, (bo_reg_value << BP_POWER_DCDC4P2_BO));
}

/* [luheng] enable 5v detection. detect 'on' if now 'off', detect 'off' if now 'on' */
void ddi_power_init_handoff(void)
{
	/* enable 5v presence detection, to detect on or off is not specified here */
	ddi_power_Enable5vDetection();

	if (ddi_power_Get5vPresentFlag())
		ddi_power_enable_5v_to_battery_handoff(); /* 5v is on, detecting off */
	else
		ddi_power_enable_battery_to_5v_handoff(); /* 5v is off, detecting on, keep DCDC alive when 5v on */

	/* Finally enable the battery adjust */
	SET_PWR_REG_BITS(HW_POWER_BATTMONITOR, BM_POWER_BATTMONITOR_EN_BATADJ);
}

/* [luheng] enable batt-BO irq, para 'enable' unused!!!! */
void ddi_power_EnableBatteryInterrupt(bool enable)
{
	WR_PWR_REG(BM_POWER_CTRL_BATT_BO_IRQ, HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_ENIRQBATT_BO, HW_POWER_CTRL_SET);
}

/*
 * [luheng] init lradc to measure batt vol, and report to power module per 100ms
 */
static void ddi_pwr_init_batt_mon(void)
{
		uint16_t wait_time = 0;

		/* disable div2 */
		WR_LRADC_REG(BF_LRADC_CTRL2_DIVIDE_BY_TWO(1 << BATTERY_VOLTAGE_CH), HW_LRADC_CTRL2_CLR);
		/* Clear the accumulator & NUM_SAMPLES */
		WR_LRADC_REG(0xFFFFFFFF, HW_LRADC_CHn_CLR(BATTERY_VOLTAGE_CH));
		/* forever trigger lradc-ch-7 at 100ms interval on delay-ch3 */
		hw_lradc_set_delay_trigger(
			LRADC_DELAY_TRIGGER_BATTERY,        /* set delay-ch-3 */
			1 << BATTERY_VOLTAGE_CH,            /* trigger lradc-ch-7 */
			1 << LRADC_DELAY_TRIGGER_BATTERY,   /* trigger myself: delay-ch-3 also */
			0,                                  /* loop once, but since it trigger itself, forever */
			200);                               /* timer, 200 * (1 / 2k Hz), i.e. 100ms */

		/* set to LiIon scale factor */
		WR_LRADC_REG(BM_LRADC_CONVERSION_SCALE_FACTOR, HW_LRADC_CONVERSION_CLR);
		WR_LRADC_REG(BF_LRADC_CONVERSION_SCALE_FACTOR(BV_LRADC_CONVERSION_SCALE_FACTOR__LI_ION),
			HW_LRADC_CONVERSION_SET);
		/* auto update to vol_val in batt_mon */
		WR_LRADC_REG(BM_LRADC_CONVERSION_AUTOMATIC, HW_LRADC_CONVERSION_SET);
		/* clear previous "measured" footprint */
		WR_LRADC_REG(1 << BATTERY_VOLTAGE_CH, HW_LRADC_CTRL1_CLR);
		/* kick off the trigger */
		WR_LRADC_REG(BM_LRADC_DELAYn_KICK, HW_LRADC_DELAYn_SET(LRADC_DELAY_TRIGGER_BATTERY));

		/* wait for 1st measurement before enabling auto volt update */
		while (!(RD_LRADC_REG(HW_LRADC_CTRL1) & (1 << BATTERY_VOLTAGE_CH))
				 &&(wait_time < 10)) {
			wait_time++;
			mdelay(1);
		}
}

/* [luheng] enable lradc to measure batt vol, enable 5v detection */
int ddi_power_init_battery(void)
{
	if (!(RD_PWR_REG(HW_POWER_5VCTRL) & BM_POWER_5VCTRL_ENABLE_DCDC)) {
		printk(KERN_ERR "WARNING: Power Supply not initialized correctly\n");
		WR_PWR_REG(BM_POWER_5VCTRL_ENABLE_DCDC,HW_POWER_5VCTRL);
	}
	if ((RD_PWR_REG(HW_POWER_BATTMONITOR) & BM_POWER_BATTMONITOR_BATT_VAL) == 0) {
		printk(KERN_INFO "WARNING : No battery connected !\r\n");
	}

	if (!hw_lradc_present(BATTERY_VOLTAGE_CH)) {
		printk(KERN_ERR "%s: hw_lradc_present failed\n", __func__);
		return -ENODEV;
	} else {
		ddi_pwr_init_batt_mon(); /* lradc-ch-7 to measure batt-vol and cp to power module per 100ms */
	}
	printk("Battery voltage measured: %d\n", ddi_bc_hwGetBatteryVoltage());

	ddi_power_init_handoff(); /* enable 5v detection */
	return 0;
}

/*
 * Use the the lradc channel
 * get the die temperature from on-chip sensor.
 */
uint16_t MeasureInternalDieTemperature(void)
{
	uint32_t  ch8Value, ch9Value, lradc_irq_mask, channel;

	channel = g_ddi_bc_Configuration.u8DieTempChannel;
	lradc_irq_mask = 1 << channel;

	/* power up internal tep sensor block */
	WR_LRADC_REG(BM_LRADC_CTRL2_TEMPSENSE_PWD, HW_LRADC_CTRL2_CLR);

	/* mux to the lradc 8th temp channel */
	WR_LRADC_REG((0xF << (4 * channel)), HW_LRADC_CTRL4_CLR);
	WR_LRADC_REG((8 << (4 * channel)), HW_LRADC_CTRL4_SET);

	/* Clear the interrupt flag */
	WR_LRADC_REG(lradc_irq_mask, HW_LRADC_CTRL1_CLR);
	WR_LRADC_REG(BF_LRADC_CTRL0_SCHEDULE(1 << channel), HW_LRADC_CTRL0_SET);

	/* Wait for conversion complete*/
	while (!(RD_LRADC_REG(HW_LRADC_CTRL1) & lradc_irq_mask))
		cpu_relax();

	/* Clear the interrupt flag again */
	WR_LRADC_REG(lradc_irq_mask, HW_LRADC_CTRL1_CLR);

	/* read temperature value and clr lradc */
	ch8Value = RD_LRADC_REG(HW_LRADC_CHn(channel)) & BM_LRADC_CHn_VALUE;


	WR_LRADC_REG(BM_LRADC_CHn_VALUE, HW_LRADC_CHn_CLR(channel));

	/* mux to the lradc 9th temp channel */
	WR_LRADC_REG((0xF << (4 * channel)), HW_LRADC_CTRL4_CLR);
	WR_LRADC_REG((9 << (4 * channel)), HW_LRADC_CTRL4_SET);

	/* Clear the interrupt flag */
	WR_LRADC_REG(lradc_irq_mask, HW_LRADC_CTRL1_CLR);
	WR_LRADC_REG(BF_LRADC_CTRL0_SCHEDULE(1 << channel), HW_LRADC_CTRL0_SET);
	/* Wait for conversion complete */
	while (!(RD_LRADC_REG(HW_LRADC_CTRL1) & lradc_irq_mask))
		cpu_relax();

	/* Clear the interrupt flag */
	WR_LRADC_REG(lradc_irq_mask, HW_LRADC_CTRL1_CLR);
	/* read temperature value */
	ch9Value = RD_LRADC_REG(HW_LRADC_CHn(channel)) & BM_LRADC_CHn_VALUE;


	WR_LRADC_REG(BM_LRADC_CHn_VALUE, HW_LRADC_CHn_CLR(channel));

	/* power down temp sensor block */
	WR_LRADC_REG(BM_LRADC_CTRL2_TEMPSENSE_PWD, HW_LRADC_CTRL2_SET);


	return (uint16_t)((ch9Value - ch8Value) * GAIN_CORRECTION / 4000);
}


/*
 * Use the the lradc channel to get the battery temperature.
 * A thermistor is used for external temperature sensing
 * which attached to LRADC0. This function returns the thermister
 * resistance value in ohm. Please check the specifiction of the
 * thermister to convert the resistance value to temperature.
 */

#define NUM_TEMP_READINGS_TO_AVG 3
uint16_t MeasureInternalBatteryTemperature(void)
{
	uint32_t  value, lradc_irq_mask, channel, sum = 0;
  uint16_t  out_value;
  int i;

	channel = g_ddi_bc_Configuration.u8BatteryTempChannel;
	lradc_irq_mask = 1 << channel;

  /* Enable the temperature sensor. */
	WR_LRADC_REG(BM_LRADC_CTRL2_TEMPSENSE_PWD, HW_LRADC_CTRL2_CLR);


  /*Setup the temperature sensor for current measurement.
   100uA is used for thermistor  */

	WR_LRADC_REG(BF_LRADC_CTRL2_TEMP_ISRC0(BV_LRADC_CTRL2_TEMP_ISRC0__100), HW_LRADC_CTRL2_SET);
	WR_LRADC_REG(BM_LRADC_CTRL2_TEMP_SENSOR_IENABLE0, HW_LRADC_CTRL2_SET);

  /* Wait while the current ramps up.  */
  msleep(1);


  /* mux analog input lradc 0 for conversion on LRADC channel . */

  WR_LRADC_REG((0xF << (4 * channel)), HW_LRADC_CTRL4_CLR);
  WR_LRADC_REG((0 << (4 * channel)), HW_LRADC_CTRL4_SET);


	for (i = 0; i < NUM_TEMP_READINGS_TO_AVG; i++) {
		/* Clear the interrupt flag */
    WR_LRADC_REG(lradc_irq_mask, HW_LRADC_CTRL1_CLR);
    WR_LRADC_REG(BF_LRADC_CTRL0_SCHEDULE(1 << channel), HW_LRADC_CTRL0_SET);

    /* Wait for conversion complete*/
    while (!(RD_LRADC_REG(HW_LRADC_CTRL1) & lradc_irq_mask))
				cpu_relax();

    /* Clear the interrupt flag again */
    WR_LRADC_REG(lradc_irq_mask, HW_LRADC_CTRL1_CLR);

    /* read temperature value and clr lradc */
    value = RD_LRADC_REG(HW_LRADC_CHn(channel)) & BM_LRADC_CHn_VALUE;

    WR_LRADC_REG(BM_LRADC_CHn_VALUE, HW_LRADC_CHn_CLR(channel));

    sum += value;
  }

  /* Turn off the current to the temperature sensor to save power */

	WR_LRADC_REG(BF_LRADC_CTRL2_TEMP_ISRC0(BV_LRADC_CTRL2_TEMP_ISRC0__ZERO), HW_LRADC_CTRL2_SET);

	WR_LRADC_REG(BM_LRADC_CTRL2_TEMP_SENSOR_IENABLE0, HW_LRADC_CTRL2_CLR);

	/* power down temp sensor block */
	WR_LRADC_REG(BM_LRADC_CTRL2_TEMPSENSE_PWD, HW_LRADC_CTRL2_SET);


  /* Take the voltage average.  */
  value = sum/NUM_TEMP_READINGS_TO_AVG;

  /* convert the voltage to thermister resistance value in 10ohm.  */
  /* ohm = (ADC value) * 1.85/(2^12) / current 100uA  */
  value = value * 18500/4096;

  out_value = value/10;

	return out_value;
}

/* [luheng] always return MODE_LIION */
ddi_power_BatteryMode_t ddi_power_GetBatteryMode(void)
{
	return DDI_POWER_BATT_MODE_LIION;
}

/* [luheng] always return enabled! */
bool ddi_power_GetBatteryChargerEnabled(void)
{
	return 1;
}

/* [luheng] whether batt-charger is turned on, return true(on), false(off) */
bool ddi_power_GetChargerPowered(void)
{
	return (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_PWD_BATTCHRG) ? 0 : 1;
}

/* [luheng] turn on/off batt-charger. To turn on, also turn on 5v->charger_&_4p2 route */
void ddi_power_SetChargerPowered(bool bPowerOn)
{
	if (bPowerOn) {
		WR_PWR_REG(BM_POWER_CHARGE_PWD_BATTCHRG, HW_POWER_CHARGE_CLR);
		WR_PWR_REG(BM_POWER_5VCTRL_PWD_CHARGE_4P2, HW_POWER_5VCTRL_CLR);
	} else {
		WR_PWR_REG(BM_POWER_CHARGE_PWD_BATTCHRG, HW_POWER_CHARGE_SET);
	}
}

/* [luheng] return if charger is working, true(working), false(idle) */
int ddi_power_GetChargeStatus(void)
{
	return (RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_CHRGSTS) ? 1 : 0;
}

/* [luheng] read battery voltage, unit:mv */
#define BATT_VOLTAGE_8_MV 8
uint16_t ddi_power_GetBattery(void)
{
	uint32_t    u16BattVolt;

	/* Get the raw result of battery measurement */
	u16BattVolt = RD_PWR_REG(HW_POWER_BATTMONITOR);
	u16BattVolt &= BM_POWER_BATTMONITOR_BATT_VAL;
	u16BattVolt >>= BP_POWER_BATTMONITOR_BATT_VAL;
	u16BattVolt *= BATT_VOLTAGE_8_MV /* Adjust for 8-mV LSB resolution and return */;

	return u16BattVolt;
}

/* [luheng] set max hw charging current */
uint16_t ddi_power_SetMaxBatteryChargeCurrent(uint16_t u16MaxCur)
{
	uint32_t   u16OldSetting;
	uint32_t   u16NewSetting;
	uint32_t   u16ToggleMask;

	u16OldSetting = (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >> BP_POWER_CHARGE_BATTCHRG_I;
	u16NewSetting = ddi_power_convert_current_to_setting(u16MaxCur);
#if 1
	u16ToggleMask = u16OldSetting ^ u16NewSetting;
	WR_PWR_REG(u16ToggleMask << BP_POWER_CHARGE_BATTCHRG_I, HW_POWER_CHARGE_TOG);
#else
	WR_PWR_REG(BM_POWER_CHARGE_BATTCHRG_I,HW_POWER_CHARGE_CLR);
	WR_PWR_REG(u16NewSetting << BP_POWER_CHARGE_BATTCHRG_I,HW_POWER_CHARGE_SET);
#endif

	return ddi_power_convert_setting_to_current(u16NewSetting);
}

/* [luheng] read from reg current max charging current setting */
uint16_t ddi_power_GetMaxBatteryChargeCurrent(void)
{
	uint32_t u8Bits;
	u8Bits = (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >> BP_POWER_CHARGE_BATTCHRG_I;
	return ddi_power_convert_setting_to_current(u8Bits);
}

/* [luheng] set stop-charging lower thresh, unit:mA */
uint16_t ddi_power_SetBatteryChargeCurrentThreshold(uint16_t u16Thresh)
{
	uint32_t   u16OldSetting;
	uint32_t   u16NewSetting;
	uint32_t   u16ToggleMask;

	if (u16Thresh > 180) /* only 4 bits, all add up to 180 */
		u16Thresh = 180;

	u16OldSetting = (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_STOP_ILIMIT) >> BP_POWER_CHARGE_STOP_ILIMIT;
	u16NewSetting = ddi_power_convert_current_to_setting(u16Thresh);
	u16ToggleMask = u16OldSetting ^ u16NewSetting;
	WR_PWR_REG(BF_POWER_CHARGE_STOP_ILIMIT(u16ToggleMask), HW_POWER_CHARGE_TOG);

	return ddi_power_convert_setting_to_current(u16NewSetting);
}

/* [luheng] read from reg the stop-charging lower thresh, unit:mA */
uint16_t ddi_power_GetBatteryChargeCurrentThreshold(void)
{
	uint32_t u16Threshold;
	u16Threshold = (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_STOP_ILIMIT) >> BP_POWER_CHARGE_STOP_ILIMIT;
	return ddi_power_convert_setting_to_current(u16Threshold);
}

/* [luheng] convert a current value (mA) to mxs-recognizable current value (mA) */
uint16_t ddi_power_ExpressibleCurrent(uint16_t u16Current)
{
	return ddi_power_convert_setting_to_current(ddi_power_convert_current_to_setting(u16Current));
}

/* [luheng] Check if 5v is present from POWER_STS reg, true:presence, false:absence */
bool ddi_power_Get5vPresentFlag(void)
{
	switch (DetectionMethod) {
		case DDI_POWER_5V_VBUSVALID:/* Check VBUSVALID for 5V present */
			return ((RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_VBUSVALID0) != 0);
		case DDI_POWER_5V_VDD5V_GT_VDDIO:/* Check VDD5V_GT_VDDIO for 5V present */
			return ((RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_VDD5V_GT_VDDIO) != 0);
		default:
			break;
	}

	return 0;
}

/* brief Report on the die temperature. */
/*  */
/*  This function reports on the die temperature. */
/*  */
/* param[out]  pLow   The low  end of the temperature range. */
/* param[out]  pHigh  The high end of the temperature range. */
/*  */
/* Temperature constant */
#define TEMP_READING_ERROR_MARGIN 5
#define KELVIN_TO_CELSIUS_CONST 273
void ddi_power_GetDieTemp(int16_t *pLow, int16_t *pHigh)
{
	int16_t i16High, i16Low;
	uint16_t u16Reading;

	/* Get the reading in Kelvins */
	u16Reading = MeasureInternalDieTemperature();

	/* Adjust for error margin */
	i16High = u16Reading + TEMP_READING_ERROR_MARGIN;
	i16Low  = u16Reading - TEMP_READING_ERROR_MARGIN;

	/* Convert to Celsius */
	i16High -= KELVIN_TO_CELSIUS_CONST;
	i16Low  -= KELVIN_TO_CELSIUS_CONST;

	/* Return the results */
	*pHigh = i16High;
	*pLow  = i16Low;
}


void ddi_power_GetBatteryTemp(uint16_t *pReading)
{
	*pReading = MeasureInternalBatteryTemperature();
}

/* [luheng] check if ENABLE_DCDC is set in 5VCTRL reg, if set means DCDC keep active when 5v is present */
bool ddi_power_IsDcdcOn(void)
{
	return (RD_PWR_REG(HW_POWER_5VCTRL) & BM_POWER_5VCTRL_ENABLE_DCDC) ? 1 : 0;
}

void ddi_power_SetPowerClkGate(bool bGate)
{
}
bool ddi_power_GetPowerClkGate(void)
{
	return 0;
}

/*
 * [luheng] check 5v state:
 *      new_5v_connection           new on
 *      existing_5v_disconnection   old on
 *      new_5v_disconnection        new off
 *      existing_5v_connection      old off
 */
enum ddi_power_5v_status ddi_power_GetPmu5vStatus(void)
{
	uint32_t pwr_ctrl_val = RD_PWR_REG(HW_POWER_CTRL);
	
	if (DetectionMethod == DDI_POWER_5V_VDD5V_GT_VDDIO) {
		if (pwr_ctrl_val & BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO) { /* 5v, previously off, expecting on */
			if (  (pwr_ctrl_val & BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ)
				|| ddi_power_Get5vPresentFlag())
				return new_5v_connection;                           /* off -> on */
			else
				return existing_5v_disconnection;                   /* still off */
		} else {                                                    /* 5v, previously on, expecting off */ 
			if (   (pwr_ctrl_val & BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ)
				|| !ddi_power_Get5vPresentFlag()
				||	ddi_power_Get5vDroopFlag())
				return new_5v_disconnection;                        /* on -> off */
			else
				return existing_5v_connection;                      /* still on */
		}
	} else {
		if (pwr_ctrl_val & BM_POWER_CTRL_POLARITY_VBUSVALID) {
			if ((pwr_ctrl_val & BM_POWER_CTRL_VBUSVALID_IRQ) ||
				ddi_power_Get5vPresentFlag())
				return new_5v_connection;
			else
				return existing_5v_disconnection;
		} else {
			if ((pwr_ctrl_val & BM_POWER_CTRL_VBUSVALID_IRQ) ||
				!ddi_power_Get5vPresentFlag() ||
				ddi_power_Get5vDroopFlag())
				return new_5v_disconnection;
			else
				return existing_5v_connection;
		}
	}
}

/* [luheng] disable 5v plug/unplug irq */
void ddi_power_disable_5v_connection_irq(void)
{
	WR_PWR_REG((BM_POWER_CTRL_ENIRQ_VBUS_VALID | BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO), HW_POWER_CTRL_CLR);
}

/* [luheng] starting detect 5v detache, will trigger an irq on detection */
void ddi_power_enable_5v_disconnect_detection(void)
{
	WR_PWR_REG(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO | BM_POWER_CTRL_POLARITY_VBUSVALID, HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ | BM_POWER_CTRL_VBUSVALID_IRQ, HW_POWER_CTRL_CLR);

	if (DetectionMethod == DDI_POWER_5V_VDD5V_GT_VDDIO) {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO, HW_POWER_CTRL_SET);
	} else {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VBUS_VALID, HW_POWER_CTRL_SET);
	}
}

/* [luheng] starting detect 5v attach, will trigger an irq on attach */
void ddi_power_enable_5v_connect_detection(void)
{
	WR_PWR_REG(BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO | BM_POWER_CTRL_POLARITY_VBUSVALID, HW_POWER_CTRL_SET);
	WR_PWR_REG(BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ | BM_POWER_CTRL_VBUSVALID_IRQ, HW_POWER_CTRL_CLR);

	if (DetectionMethod == DDI_POWER_5V_VDD5V_GT_VDDIO) {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO, HW_POWER_CTRL_SET);
	} else {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VBUS_VALID, HW_POWER_CTRL_SET);
	}
}

/* [luheng] turn on/off BATT-BO INT */
void ddi_power_EnableBatteryBoInterrupt(bool bEnable)
{
	if (bEnable) {
		WR_PWR_REG(BM_POWER_CTRL_BATT_BO_IRQ, HW_POWER_CTRL_CLR);
		WR_PWR_REG(BM_POWER_CTRL_ENIRQBATT_BO, HW_POWER_CTRL_SET);
		// TODO: make sure the battery brownout comparator is enabled in HW_POWER_BATTMONITOR
	} else {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQBATT_BO, HW_POWER_CTRL_CLR);
	}
}

/* [luheng] turn on/off 4p2-BO INT */
void ddi_power_EnableDcdc4p2BoInterrupt(bool bEnable)
{
	if (bEnable) {
		WR_PWR_REG(BM_POWER_CTRL_DCDC4P2_BO_IRQ, HW_POWER_CTRL_CLR);
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_DCDC4P2_BO, HW_POWER_CTRL_SET);
	} else {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_DCDC4P2_BO, HW_POWER_CTRL_CLR);
	}
}

/* [luheng] turn on/off 4p2-BO INT */
void ddi_power_EnableVdd5vDroopInterrupt(bool bEnable)
{
	if (bEnable) {
		WR_PWR_REG(BM_POWER_CTRL_VDD5V_DROOP_IRQ, HW_POWER_CTRL_CLR);
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDD5V_DROOP, HW_POWER_CTRL_SET);
	} else {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDD5V_DROOP, HW_POWER_CTRL_CLR);
	}
}

/* [luheng] enable/disable 5v-BO shutdown trigger */
void ddi_power_Enable5vDisconnectShutdown(bool bEnable)
{
	if (bEnable)
		WR_PWR_REG(BM_POWER_5VCTRL_PWDN_5VBRNOUT, HW_POWER_5VCTRL_SET);
	else
		WR_PWR_REG(BM_POWER_5VCTRL_PWDN_5VBRNOUT, HW_POWER_5VCTRL_CLR);
}

/* [luheng] enable/disable 5v->batt auto xfer. when disabled, shutdown upon 5v-BO */
void ddi_power_enable_5v_to_battery_xfer(bool bEnable)
{
	if (bEnable) {
		ddi_power_Enable5vDisconnectShutdown(false);
	} else {
		/* order matters */
		ddi_power_Enable5vDisconnectShutdown(true);
		ddi_power_EnableBatteryBoInterrupt(false);
	}
}

/* [luheng] enable vbusdroop irq, on (<4.3v) */
void ddi_power_init_4p2_protection(void)
{
	WR_PWR_REG(BM_POWER_5VCTRL_VBUSDROOP_TRSH, HW_POWER_5VCTRL_CLR); /* set vbus droop comparator <4.3V */
	ddi_power_EnableVbusDroopIrq();
}

/* [luheng] check if all switches on route  "5v -> 4p2_linreg -> DCDC"  are turned on */
bool ddi_power_check_4p2_bits(void)
{
	if (0 != (RD_PWR_REG(HW_POWER_5VCTRL) & BM_POWER_5VCTRL_PWD_CHARGE_4P2))
		return false;

	if (0 == (RD_PWR_REG(HW_POWER_DCDC4P2) & BM_POWER_DCDC4P2_ENABLE_DCDC))
		return false;

	if (0 != (RD_PWR_REG(HW_POWER_DCDC4P2) & BM_POWER_DCDC4P2_ENABLE_4P2))
		return true;
	else
		return false;
}

/* [luheng] set 5v->charger_&_4p2 current limit, unit:mA */
uint16_t ddi_power_set_4p2_ilimit(uint16_t ilimit)
{
	uint32_t bits;

	if (ilimit > 780)
		ilimit = 780;
	bits = BF_POWER_5VCTRL_CHARGE_4P2_ILIMIT(ddi_power_convert_current_to_setting(ilimit));
	CLR_SET_PWR_REG_BITS(HW_POWER_5VCTRL, BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT, bits);

	return ilimit;
}

/* [luheng] power down the device */
void ddi_power_shutdown(void)
{
	WR_PWR_REG(0x3e770001, HW_POWER_RESET);
}

/* [luheng] turn on batt-BO INT, turn off 4p2-BO INT, this is called when 5v detached and use batt only. */
void ddi_power_handle_dcdc4p2_bo(void)
{
	ddi_power_EnableBatteryBoInterrupt(true);
	ddi_power_EnableDcdc4p2BoInterrupt(false);
}

/* [luheng]: enable/disable vddio-BO INT */
void ddi_power_enable_vddio_interrupt(bool enable)
{
	if (enable) {
		WR_PWR_REG(BM_POWER_CTRL_VDDIO_BO_IRQ, HW_POWER_CTRL_CLR);
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDDIO_BO, HW_POWER_CTRL_SET);

	} else {
		WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDDIO_BO, HW_POWER_CTRL_CLR);
	}

}

/*
 * [luheng]
 *		vddio-BO irq handler
 *			if caused by false detection due to 5v plug/unplug, just disable vddio-BO INT
 *			else: shut down.
 */
void ddi_power_handle_vddio_brnout(void)
{
	if (    (ddi_power_GetPmu5vStatus() == new_5v_connection)
		 || (ddi_power_GetPmu5vStatus() == new_5v_disconnection)) {
		ddi_power_enable_vddio_interrupt(false);
	} else {
		ddi_power_shutdown();
	}
}

/*
 * [luheng]
 * 5v-droop irq handler.
 *		1. set 4p2 target = batt vol, 4p2<->batt cmptrip = (1.05 * batt)
 *		2. if batt-BO, shut down immediately
 *		3. disable vddio/4p2/vddd BO INT, disable 5v-droop INT, only enable BATT-BO INT
 */
void ddi_power_handle_vdd5v_droop(void)
{
	/* when (4p2 >= 1.05 * batt), 4p2 is regarded as higher than batt; set 4p2 target vol as batt vol */
	SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BF_POWER_DCDC4P2_CMPTRIP(0x1F) | BM_POWER_DCDC4P2_TRG);

	/* if batt-BO occurs, shutdown asap */
	if (RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_BATT_BO)
		ddi_power_shutdown();

	/* due to 5v connect vddio bo chip bug, we need to
	 * disable vddio interrupts until we reset the 5v
	 * detection for 5v connect detect.  We want to allow
	 * some debounce time before enabling connect detection.
	 */
	ddi_power_enable_vddio_interrupt(false); /* disable VDDIO-BO INT */
	ddi_power_EnableBatteryBoInterrupt(true); /* enable BATT-BO INT */
	ddi_power_EnableDcdc4p2BoInterrupt(false);
	ddi_power_EnableVdd5vDroopInterrupt(false);
	WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDDD_BO, HW_POWER_CTRL_CLR);

}

/* [luheng]
 * for each of vddd / vdda / vddio BO
 *		1. clear INT stats
 *		2. enable irqs
 *		3. disable PWDN-BO trgger
 */
void ddi_power_InitOutputBrownouts(void)
{
	WR_PWR_REG(BM_POWER_CTRL_VDDD_BO_IRQ   | BM_POWER_CTRL_VDDA_BO_IRQ   | BM_POWER_CTRL_VDDIO_BO_IRQ,   HW_POWER_CTRL_CLR);
	WR_PWR_REG(BM_POWER_CTRL_ENIRQ_VDDD_BO | BM_POWER_CTRL_ENIRQ_VDDA_BO | BM_POWER_CTRL_ENIRQ_VDDIO_BO, HW_POWER_CTRL_SET);

	CLR_PWR_REG_BITS(HW_POWER_VDDDCTRL,  BM_POWER_VDDDCTRL_PWDN_BRNOUT);
	CLR_PWR_REG_BITS(HW_POWER_VDDACTRL,  BM_POWER_VDDACTRL_PWDN_BRNOUT);
	CLR_PWR_REG_BITS(HW_POWER_VDDIOCTRL, BM_POWER_VDDIOCTRL_PWDN_BRNOUT);
}

/* [luheng] used for debugging purposes only, disable all intterupts of pwr-reg */
void ddi_power_disable_power_interrupts(void)
{
	WR_PWR_REG( BM_POWER_CTRL_ENIRQ_DCDC4P2_BO | BM_POWER_CTRL_ENIRQ_VDD5V_DROOP
		      | BM_POWER_CTRL_ENIRQ_PSWITCH    | BM_POWER_CTRL_ENIRQ_DC_OK
		      | BM_POWER_CTRL_ENIRQBATT_BO     | BM_POWER_CTRL_ENIRQ_VDDIO_BO
		      | BM_POWER_CTRL_ENIRQ_VDDA_BO    | BM_POWER_CTRL_ENIRQ_VDDD_BO
		      | BM_POWER_CTRL_ENIRQ_VBUS_VALID | BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO,
		HW_POWER_CTRL_CLR);

}

/* [luheng] read from pwr-sts reg if vdd5v_droop happens */
bool ddi_power_Get5vDroopFlag(void)
{
	return ((RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_VDD5V_DROOP) != 0);
}


