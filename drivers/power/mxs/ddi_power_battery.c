#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/processor.h> /* cpu_relax */
#include "regs-power.h"
#include "regs-lradc.h"
#include "mxs-battery.h"
#include "ddi_power_battery.h"




/* current represented by bitfields HW_POWER_CHARGE.STOP_ILIMIT and HW_POWER_CHARGE.BATTCHRG_I.
 *
 *                                       bit0 bit1 bit2 bit3 bit4 bit5  */
static const uint16_t currentPerBit[] = {  10,  20,  50, 100, 200, 400 };

/* [luheng] convert a current value to bitfields, where
 *     bit0:10mA, bit1:20mA, bit2:50mA, bit3:100mA, bit4:200mA, bit5:400mA
 */
uint16_t mxspwr_ma2bits(uint16_t u16ma)
{
	int       i;
	uint16_t  u16Mask = (0x1 << 5); /* start from hightest bit, highest current */
	uint16_t  u16Setting = 0;

	for (i = 5; (i >= 0) && (u16ma > 0); i--, u16Mask >>= 1) {
		if (u16ma >= currentPerBit[i]) {
			u16ma -= currentPerBit[i];
			u16Setting |= u16Mask;
		}
	}
	return u16Setting;
}


/* [luheng] convert a bitfields to current value where
 *     bit0:10mA, bit1:20mA, bit2:50mA, bit3:100mA, bit4:200mA, bit5:400mA
 */
uint16_t mxspwr_bits2ma(uint16_t u16bits)
{
	int       i;
	uint16_t  u16Mask = (0x1 << 5); /* start from hightest bit, highest current */
	uint16_t  u16ma = 0;

	for (i = 5; i >= 0; i--, u16Mask >>= 1) {
		if (u16bits & u16Mask)
			u16ma += currentPerBit[i];
	}
	return u16ma;
}

/* [luheng]
 *		1. Turn on VBUSVALID comparator(>4.3v) even VDD5V_GT_VDDIO is used to detect 5v presence
 *		3. Set vddio/vdda/vddd linreg output 25mv lower than DCDC counterpart
 */
void ddi_set_linreg_offset(void)
{
	/* set vddio/vdda/vddd lingreg output 1-step-below(25mv) DCDC counterparts,
	 * standard practive when lingreg and DCDC are all on */
	CLR_SET_PWR_REG_BITS(HW_POWER_VDDIOCTRL,
		BM_POWER_VDDIOCTRL_LINREG_OFFSET,
		BF_POWER_VDDIOCTRL_LINREG_OFFSET(LINREG_OFFSET_1_STEP_BL));

	CLR_SET_PWR_REG_BITS(HW_POWER_VDDACTRL,
		BM_POWER_VDDACTRL_LINREG_OFFSET,
		BF_POWER_VDDACTRL_LINREG_OFFSET(LINREG_OFFSET_1_STEP_BL));

	CLR_SET_PWR_REG_BITS(HW_POWER_VDDDCTRL,
		BM_POWER_VDDDCTRL_LINREG_OFFSET,
		BF_POWER_VDDDCTRL_LINREG_OFFSET(LINREG_OFFSET_1_STEP_BL));
}

/* [luheng]
 * Set 4p2 VS batt comparator trip value.
 * Ideal: use 4p2 as much as possible. the only condition is 4p2 can fulfill the use
 *
 * set the optimal CMPTRIP for the best possible 5V
 * disconnection handling but without drawing power
 * from the power on a stable 4p2 rails (at 4.2V).
 */
void mxspwr_adjust_cmptrip(void)
{
	uint16_t batt_vol;

	batt_vol = PWRREG_GET_BATVOL();

	if (batt_vol >= 4000)
		PWRREG_SET_CMPTRIP(CMPTRIP_4P2_GE_0_86_BAT); /* if batvol [4000,....), 4p2 >= (0.86 * batt) */
	else if (batt_vol >= 3800)
		PWRREG_SET_CMPTRIP(CMPTRIP_4P2_GE_1_00_BAT); /* if batvol [3800,4000), 4p2 >= (1.00 * batt) */
	else
		PWRREG_SET_CMPTRIP(CMPTRIP_4P2_GE_1_05_BAT); /* if batvol (....,3800), 4p2 >= (1.05 * batt) */
}

/* [luheng] turn on 5v -> charger_&_4p2 -> DCDC, if battery not damaged; or just enable batt-BO irq if batt damaged
 * 5v -> charger_&_4p2 current limit: 'target_current_limit_ma'
 */
/* 5v->4p2->DCDC is already open, just ramp-up 5v limit */
uint16_t mxspwr_rampup_chrgr_4p2_limit(uint16_t limit_ma)
{
	uint32_t temp_reg;
	uint16_t charge_4p2_ilimit = 0;

	limit_ma = mxspwr_expressible_cur(limit_ma);

	charge_4p2_ilimit = mxspwr_get_chrgr_4p2_limit();

	if (charge_4p2_ilimit == limit_ma) {       /* equal trg, no need to ramp */
		/*BATT_LOG("[BAT] 5v limit already %d, no ramp\n", charge_4p2_ilimit);*/
		return charge_4p2_ilimit;
	} else if (charge_4p2_ilimit > limit_ma) { /* ramp down, go directly */
		/*BATT_LOG("[BAT] 5v ramp down from %d to %d\n", charge_4p2_ilimit, limit_ma);*/
		mxspwr_set_chrgr_4p2_limit(limit_ma);
		return limit_ma;
	}

	/* we need to ramp up, steply */

	/* 4p2 steal charger's cur if (4p2 < (TRG - 100mV)), use cmptrip-max(4p2,batt) as DCDC src */
	PWRREG_SET_DCDC4P2_DROPCTRL(DROPCTRL_DCSRC_CMPTRP | DROPCTRL_STLTHR_100_BL);
	PWRREG_SET_CMPTRIP(CMPTRIP_4P2_GE_1_05_BAT);

	WR_PWR_REG(HW_POWER_5VCTRL_SET, BM_POWER_5VCTRL_PWRUP_VBUS_CMPS);
	WR_PWR_REG(HW_POWER_5VCTRL_CLR, BM_POWER_5VCTRL_VBUSVALID_5VDETECT); /* turn off vbusvalid detect. Errate */

	temp_reg = (BM_POWER_CTRL_VDDD_BO_IRQ | BM_POWER_CTRL_VDDA_BO_IRQ | BM_POWER_CTRL_VDDIO_BO_IRQ |
				 BM_POWER_CTRL_VDD5V_DROOP_IRQ | BM_POWER_CTRL_VBUSVALID_IRQ);

	WR_PWR_REG(HW_POWER_CTRL_CLR, temp_reg);

	/* loop until the false BO goes away or until 5V actually goes away */
	while ((RD_PWR_REG(HW_POWER_CTRL) & temp_reg) &&
		  !(RD_PWR_REG(HW_POWER_CTRL) & BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ))
	{
		WR_PWR_REG(HW_POWER_CTRL_CLR, temp_reg);
		mdelay(1);
	}

	mxspwr_set4p2bo_thsh(4150);                                       /* set 4p2 BO thresh at 4.15V */

	/* possibly not necessary but recommended, unloaded 4p2 rail */
	WR_PWR_REG(HW_POWER_CHARGE_SET, BM_POWER_CHARGE_ENABLE_LOAD);

	while (charge_4p2_ilimit < limit_ma) {                /* ramp up charger_&_4p2 cur limit */
		if (RD_PWR_REG(HW_POWER_CTRL) & (BM_POWER_CTRL_VBUSVALID_IRQ | BM_POWER_CTRL_VDD5V_DROOP_IRQ))
			break;

		charge_4p2_ilimit += 100;
		if (charge_4p2_ilimit > limit_ma)
			charge_4p2_ilimit = limit_ma;

		mxspwr_set_chrgr_4p2_limit(charge_4p2_ilimit);

		if (RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_DCDC_4P2_BO)
			mdelay(10);
		else {
			charge_4p2_ilimit = limit_ma;
			mxspwr_set_chrgr_4p2_limit(charge_4p2_ilimit);
		}
	}

	WR_PWR_REG(HW_POWER_CHARGE_CLR, BM_POWER_CHARGE_ENABLE_LOAD);
	mxspwr_set4p2bo_thsh(3600);
	WR_PWR_REG(HW_POWER_CTRL_CLR, BM_POWER_CTRL_DCDC4P2_BO_IRQ);
	WR_PWR_REG(HW_POWER_5VCTRL_SET, BM_POWER_5VCTRL_VBUSVALID_5VDETECT);

	return charge_4p2_ilimit;
}

uint16_t mxspwr_enable4p2(uint16_t limit_ma)
{
	uint32_t temp_reg;
	uint16_t charge_4p2_ilimit = 0;

	PWRREG_TURN_CHARGER_AND_4P2(false);                                  /* turn off charger_&_4p2 */

	/* 4p2 steal charger's cur if (4p2 < (TRG - 100mV)), use cmptrip-max(4p2,batt) as DCDC src */
	PWRREG_SET_DCDC4P2_DROPCTRL(DROPCTRL_DCSRC_CMPTRP | DROPCTRL_STLTHR_100_BL);

	PWRREG_SET_4P2LDO_TRG(TRG4P2_4_2);                                   /* set 4p2 LDO target: 4.2v */
	PWRREG_SET_CMPTRIP(CMPTRIP_4P2_GE_1_05_BAT);

	WR_PWR_REG(HW_POWER_5VCTRL_SET, BM_POWER_5VCTRL_PWRUP_VBUS_CMPS);
	WR_PWR_REG(HW_POWER_5VCTRL_CLR, BM_POWER_5VCTRL_VBUSVALID_5VDETECT); /* turn off vbusvalid detect. Errate */

	SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_ENABLE_DCDC);    /* turn on 4p2->DCDC */
	mdelay(1);

	temp_reg = (BM_POWER_CTRL_VDDD_BO_IRQ | BM_POWER_CTRL_VDDA_BO_IRQ | BM_POWER_CTRL_VDDIO_BO_IRQ |
				 BM_POWER_CTRL_VDD5V_DROOP_IRQ | BM_POWER_CTRL_VBUSVALID_IRQ);

	WR_PWR_REG(HW_POWER_CTRL_CLR, temp_reg);

	/* loop until the false BO goes away or until 5V actually goes away */
	while ((RD_PWR_REG(HW_POWER_CTRL) & temp_reg) &&
		  !(RD_PWR_REG(HW_POWER_CTRL) & BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ))
	{
		WR_PWR_REG(HW_POWER_CTRL_CLR, temp_reg);
		mdelay(1);
	}

	WR_PWR_REG(HW_POWER_5VCTRL_CLR, BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT);  /* clear charger_&_4p2 max current : 0 */
	SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_ENABLE_4P2);     /* turn on 4p2 linereg */
	PWRREG_TURN_CHARGER_AND_4P2(true);                                   /* turn on charger_&_4p2 gate */

	if (limit_ma > 780)
		limit_ma = 780;

	mxspwr_set4p2bo_thsh(4150);                                       /* set 4p2 BO thresh at 4.15V */

	/* possibly not necessary but recommended, unloaded 4p2 rail */
	WR_PWR_REG(HW_POWER_CHARGE_SET, BM_POWER_CHARGE_ENABLE_LOAD);

	while (charge_4p2_ilimit < limit_ma) {                /* ramp up charger_&_4p2 cur limit */
		if (RD_PWR_REG(HW_POWER_CTRL) & (BM_POWER_CTRL_VBUSVALID_IRQ | BM_POWER_CTRL_VDD5V_DROOP_IRQ))
			break;

		charge_4p2_ilimit += 100;
		if (charge_4p2_ilimit > limit_ma)
			charge_4p2_ilimit = limit_ma;

		mxspwr_set_chrgr_4p2_limit(charge_4p2_ilimit);

		if (RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_DCDC_4P2_BO)
			mdelay(10);
		else {
			charge_4p2_ilimit = limit_ma;
			mxspwr_set_chrgr_4p2_limit(charge_4p2_ilimit);
		}
	}

	mxspwr_set4p2bo_thsh(3600);

	WR_PWR_REG(HW_POWER_CTRL_CLR, BM_POWER_CTRL_DCDC4P2_BO_IRQ);

	WR_PWR_REG(HW_POWER_CHARGE_CLR, BM_POWER_CHARGE_ENABLE_LOAD);
	WR_PWR_REG(HW_POWER_5VCTRL_SET, BM_POWER_5VCTRL_VBUSVALID_5VDETECT);

	return charge_4p2_ilimit;
}


/* [luheng] set 4p2-BO thresh, unit:mv.
 *     Note: 4p2-BO can only be detected if 4p2->DCDC input route is turned on */
void mxspwr_set4p2bo_thsh(uint16_t bo_mv)
{
	uint16_t bo_reg_value;

	if (bo_mv < 3600)
		bo_mv = 3600;
	else if (bo_mv > 4375)
		bo_mv = 4375;

	bo_reg_value = (bo_mv - 3600) / 25;

	CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_BO, (bo_reg_value << BP_POWER_DCDC4P2_BO));
}

void hw_lradc_set_delay_trigger(int trgr, u32 trgr_lradc, u32 delay_trgr, u32 loops, u32 delays)
{
	WR_LRADC_REG(HW_LRADC_DELAYn_SET(trgr), BF_LRADC_DELAYn_TRIGGER_LRADCS(trgr_lradc));
	WR_LRADC_REG(HW_LRADC_DELAYn_SET(trgr), BF_LRADC_DELAYn_TRIGGER_DELAYS(delay_trgr));
	WR_LRADC_REG(HW_LRADC_DELAYn_CLR(trgr), (BM_LRADC_DELAYn_LOOP_COUNT | BM_LRADC_DELAYn_DELAY));
	WR_LRADC_REG(HW_LRADC_DELAYn_SET(trgr), BF_LRADC_DELAYn_LOOP_COUNT(loops));
	WR_LRADC_REG(HW_LRADC_DELAYn_SET(trgr), BF_LRADC_DELAYn_DELAY(delays));
}
/*
 * [luheng] init lradc to measure batt vol, and report to power module per 100ms
 */
static void ddi_pwr_init_batt_mon(void)
{
	uint16_t wait_time = 0;

	/* disable div2 */
	WR_LRADC_REG(HW_LRADC_CTRL2_CLR, BF_LRADC_CTRL2_DIVIDE_BY_TWO(1 << BATTERY_VOLTAGE_CH));
	/* Clear the accumulator & NUM_SAMPLES */
	WR_LRADC_REG(HW_LRADC_CHn_CLR(BATTERY_VOLTAGE_CH), 0xFFFFFFFF);
	/* forever trigger lradc-ch-7 at 100ms interval on delay-ch3 */
	hw_lradc_set_delay_trigger(
		LRADC_DELAY_TRIGGER_BATTERY,        /* set delay-ch-3 */
		1 << BATTERY_VOLTAGE_CH,            /* trigger lradc-ch-7 */
		1 << LRADC_DELAY_TRIGGER_BATTERY,   /* trigger myself: delay-ch-3 also */
		0,                                  /* loop once, but since it trigger itself, forever */
		200);                               /* timer, 200 * (1 / 2k Hz), i.e. 100ms */

	/* clear previous "measured" footprint */
	WR_LRADC_REG(HW_LRADC_CTRL1_CLR, 1 << BATTERY_VOLTAGE_CH);

	/* set to LiIon scale factor */
	WR_LRADC_REG(HW_LRADC_CONVERSION_CLR, BM_LRADC_CONVERSION_SCALE_FACTOR);
	WR_LRADC_REG(HW_LRADC_CONVERSION_SET,
		BF_LRADC_CONVERSION_SCALE_FACTOR(BV_LRADC_CONVERSION_SCALE_FACTOR__LI_ION));

	/* kick off the trigger */
	WR_LRADC_REG(HW_LRADC_DELAYn_SET(LRADC_DELAY_TRIGGER_BATTERY), BM_LRADC_DELAYn_KICK);

	/* wait for 1st measurement before enabling auto volt update */
	while (!(RD_LRADC_REG(HW_LRADC_CTRL1) & (1 << BATTERY_VOLTAGE_CH))
			 &&(wait_time < 10)) {
		wait_time++;
		mdelay(1);
	}

	/* auto update to vol_val in batt_mon */
	WR_LRADC_REG(HW_LRADC_CONVERSION_SET, BM_LRADC_CONVERSION_AUTOMATIC);
}

/* [luheng] enable lradc to measure batt vol, start detecting 5v plug/unplug */
int mxspwr_init_bat(void)
{

	if (!hw_lradc_present(BATTERY_VOLTAGE_CH)) {
		printk(KERN_ERR "%s: hw_lradc_present failed\n", __func__);
		return -ENODEV;
	} else {
		ddi_pwr_init_batt_mon(); /* lradc-ch-7 to measure batt-vol and cp to power module per 100ms */
	}
	printk("[BAT]: Bat-vol: %d\n", PWRREG_GET_BATVOL());

	ddi_set_linreg_offset();

	WR_PWR_REG(HW_POWER_5VCTRL_CLR,
		  BM_POWER_5VCTRL_ENABLE_LINREG_ILIMIT
		| BM_POWER_5VCTRL_VBUSVALID_TRSH);

	WR_PWR_REG(HW_POWER_5VCTRL_SET,
		  BM_POWER_5VCTRL_VBUSVALID_5VDETECT
		/*| BM_POWER_5VCTRL_DCDC_XFER*/
		| BM_POWER_5VCTRL_ENABLE_DCDC
		| BF_POWER_5VCTRL_VBUSVALID_TRSH(VBUSVALID_THRESH_4_30V));

	/* Finally enable the battery adjust */
	SET_PWR_REG_BITS(HW_POWER_BATTMONITOR, BM_POWER_BATTMONITOR_EN_BATADJ);

	/* 4p2 out: 4.2v, DROPOUT_CTRL(b1010): DCDC source = max(4p2,batt), 4p2 steal current form batt when 100mv below TRG */
	CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2,
		(BM_POWER_DCDC4P2_TRG | BM_POWER_DCDC4P2_DROPOUT_CTRL),
		(0xa << BP_POWER_DCDC4P2_DROPOUT_CTRL));

	/* HEADROOM_ADJ to 4 */
	CLR_SET_PWR_REG_BITS(HW_POWER_5VCTRL,BM_POWER_5VCTRL_HEADROOM_ADJ,(4 << BP_POWER_5VCTRL_HEADROOM_ADJ));

	/* disable shutdown on 5v/vddd/a/io BO */
	PWRREG_DISABLE_VDDDAIO5V_BO_SHUTDOWN();

	/* turn on bat-bo detecting circuit, don't shut down on batt-BO */
	CLR_PWR_REG_BITS(HW_POWER_BATTMONITOR,
		(BM_POWER_BATTMONITOR_PWDN_BATTBRNOUT | BM_POWER_BATTMONITOR_BRWNOUT_PWD));

	/* set 5v-droop thresh at < 4.3V */
	PWRREG_SET_VBUSDROOP_TRSH(VBUSDROOP_TRSH_4_3);

	/* always detect attaching */
	WR_PWR_REG(HW_POWER_CTRL_SET, BM_POWER_CTRL_POLARITY_VDD5V_GT_VDDIO | BM_POWER_CTRL_POLARITY_VBUSVALID);

	/* disable all INT at first, will be set properly in state machine */
	PWRREG_ENABLE_ALL_INT(false);

	return 0;
}

/* [luheng] set max hw charging current */
uint16_t mxspwr_set_charging_cur(uint16_t u16ma)
{
	uint32_t   u16old_bits;
	uint32_t   u16new_bits;
	uint32_t   u16ToggleMask;

	u16old_bits = (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >> BP_POWER_CHARGE_BATTCHRG_I;
	u16new_bits = mxspwr_ma2bits(u16ma);
#if 1
	u16ToggleMask = u16old_bits ^ u16new_bits;
	WR_PWR_REG(HW_POWER_CHARGE_TOG, u16ToggleMask << BP_POWER_CHARGE_BATTCHRG_I);
#else
	WR_PWR_REG(HW_POWER_CHARGE_CLR,BM_POWER_CHARGE_BATTCHRG_I);
	WR_PWR_REG(HW_POWER_CHARGE_SET,u16NewSetting << BP_POWER_CHARGE_BATTCHRG_I);
#endif

	return mxspwr_bits2ma(u16new_bits);
}

/* [luheng] read from reg current max charging current setting */
uint16_t mxspwr_get_charging_cur(void)
{
	uint32_t u8Bits;
	u8Bits = (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_BATTCHRG_I) >> BP_POWER_CHARGE_BATTCHRG_I;
	return mxspwr_bits2ma(u8Bits);
}

/* [luheng] set stop-charging lower thresh, unit:mA */
uint16_t mxspwr_set_charg_stop_thsh(uint16_t u16Thresh)
{
	uint32_t   u16old_bits;
	uint32_t   u16new_bits;
	uint32_t   u16ToggleMask;

	if (u16Thresh > 180) /* only 4 bits, all add up to 180 */
		u16Thresh = 180;

	u16old_bits = (RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_STOP_ILIMIT) >> BP_POWER_CHARGE_STOP_ILIMIT;
	u16new_bits = mxspwr_ma2bits(u16Thresh);
	u16ToggleMask = u16old_bits ^ u16new_bits;
	WR_PWR_REG(HW_POWER_CHARGE_TOG, BF_POWER_CHARGE_STOP_ILIMIT(u16ToggleMask));

	return mxspwr_bits2ma(u16new_bits);
}

/* [luheng] convert a current value (mA) to mxs-recognizable current value (mA) */
uint16_t mxspwr_expressible_cur(uint16_t u16ma)
{
	return mxspwr_bits2ma(mxspwr_ma2bits(u16ma));
}

/* [luheng] get hw 5v->charger_&_4p2 current limit, unit:mA */
uint16_t mxspwr_get_chrgr_4p2_limit(void)
{
	uint32_t bits;

	bits = (RD_PWR_REG(HW_POWER_5VCTRL) | BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT) >> BP_POWER_5VCTRL_CHARGE_4P2_ILIMIT;
	return mxspwr_bits2ma((uint16_t)bits);
}

/* [luheng] set 5v->charger_&_4p2 current limit, unit:mA */
uint16_t mxspwr_set_chrgr_4p2_limit(uint16_t ilimit)
{
	uint32_t bits;

	if (ilimit > 780)
		ilimit = 780;
	bits = BF_POWER_5VCTRL_CHARGE_4P2_ILIMIT(mxspwr_ma2bits(ilimit));
	CLR_SET_PWR_REG_BITS(HW_POWER_5VCTRL, BM_POWER_5VCTRL_CHARGE_4P2_ILIMIT, bits);

	return ilimit;
}

/* [luheng] power down the device */
void mxspwr_shutdown(char *reason)
{
	printk("power down: %s\n", reason);
	mdelay(500);
	WR_PWR_REG(HW_POWER_RESET, 0x3e770001);
}


