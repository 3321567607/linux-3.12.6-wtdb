
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include "mxs-battery.h"
#include "ddi_power_battery.h"
#include "regs-power.h"
#include "regs-lradc.h"

#define KELVIN_TO_CELSIUS_CONST 273
#define GAIN_CORRECTION 1012

#define ROUTINE_GAP   MXS_DELTA(info->prv_routine_stamp,   info->cur_routine_stamp)
#define STATE_AGE     MXS_DELTA(info->state_begin_stamp,   info->cur_routine_stamp)
#define SUBSTATE_AGE  MXS_DELTA(info->substate_begin_stamp,info->cur_routine_stamp)
#define BATDETECT_AGE MXS_DELTA(info->stamp_batvol,        info->cur_routine_stamp)

static void state_uninited(struct mxs_info *info);
static void state_wait5v(struct mxs_info *info);
static void state_waitbat(struct mxs_info *info);
static void state_charging(struct mxs_info *info);
static void state_full(struct mxs_info *info);

static char* state_name[] = {
	"[UNINITED]",
	"[WAIT-5V ]",
	"[WAIT-BAT]",
	"[CHARGING]",
	"[FULCHRGD]",
};

static void (*const (state_fun[]))(struct mxs_info *) = {
	state_uninited,
	state_wait5v,
	state_waitbat,
	state_charging,
	state_full,
};

static char g_reason[50];

void enter_state(struct mxs_info *info, enum BATT_STATE newstate, char *reason)
{
	if (newstate != info->bat_state) {
		BATT_LOG("[BAT] %s ==> %s by %s!\n",
			state_name[info->bat_state], state_name[newstate], reason ? reason : "None");
		info->bat_state = newstate;
		info->state_begin_stamp = info->cur_routine_stamp;
		info->state_entered = false;
	}
}

/* this func doesn't check old substate is the same with new one, caller to avoid such thing */
void enter_substate(struct mxs_info *info, enum BATT_SUB_STATE newstate)
{
	info->substate_begin_stamp = info->cur_routine_stamp;
	info->substate_entered = false;
	info->bat_substate = newstate;
}

void init_batt_sm(struct mxs_info *info)
{
	info->prv_routine_stamp = jiffies_to_msecs(jiffies);
	info->cur_routine_stamp = info->prv_routine_stamp;
	enter_state(info, BAT_STAT_UNINITED, "init");
	save_irq_event(EVNT_5V_ON);
	mod_timer(&info->sm_timer, jiffies + 1);

}

static uint16_t measure_die_tmp(void)
{
	uint32_t  ch8Value, ch9Value, lradc_irq_mask, channel;

	channel = BATTERY_DIETEMP_CH;
	lradc_irq_mask = 1 << channel;

	/* power up internal tep sensor block */
	WR_LRADC_REG(HW_LRADC_CTRL2_CLR, BM_LRADC_CTRL2_TEMPSENSE_PWD);

	/* mux to the lradc 8th temp channel */
	WR_LRADC_REG(HW_LRADC_CTRL4_CLR, (0xF << (4 * channel)));
	WR_LRADC_REG(HW_LRADC_CTRL4_SET, (8 << (4 * channel)));

	/* Clear the interrupt flag */
	WR_LRADC_REG(HW_LRADC_CTRL1_CLR, lradc_irq_mask);
	WR_LRADC_REG(HW_LRADC_CTRL0_SET, BF_LRADC_CTRL0_SCHEDULE(1 << channel));

	/* Wait for conversion complete*/
	while (!(RD_LRADC_REG(HW_LRADC_CTRL1) & lradc_irq_mask))
		cpu_relax();

	/* Clear the interrupt flag again */
	WR_LRADC_REG(HW_LRADC_CTRL1_CLR, lradc_irq_mask);

	/* read temperature value and clr lradc */
	ch8Value = RD_LRADC_REG(HW_LRADC_CHn(channel)) & BM_LRADC_CHn_VALUE;


	WR_LRADC_REG(HW_LRADC_CHn_CLR(channel), BM_LRADC_CHn_VALUE);

	/* mux to the lradc 9th temp channel */
	WR_LRADC_REG(HW_LRADC_CTRL4_CLR, (0xF << (4 * channel)));
	WR_LRADC_REG(HW_LRADC_CTRL4_SET, (9 << (4 * channel)));

	/* Clear the interrupt flag */
	WR_LRADC_REG(HW_LRADC_CTRL1_CLR, lradc_irq_mask);
	WR_LRADC_REG(HW_LRADC_CTRL0_SET, BF_LRADC_CTRL0_SCHEDULE(1 << channel));
	/* Wait for conversion complete */
	while (!(RD_LRADC_REG(HW_LRADC_CTRL1) & lradc_irq_mask))
		cpu_relax();

	/* Clear the interrupt flag */
	WR_LRADC_REG(HW_LRADC_CTRL1_CLR, lradc_irq_mask);
	/* read temperature value */
	ch9Value = RD_LRADC_REG(HW_LRADC_CHn(channel)) & BM_LRADC_CHn_VALUE;


	WR_LRADC_REG(HW_LRADC_CHn_CLR(channel), BM_LRADC_CHn_VALUE);

	/* power down temp sensor block */
	WR_LRADC_REG(HW_LRADC_CTRL2_SET, BM_LRADC_CTRL2_TEMPSENSE_PWD);


	return (uint16_t)((ch9Value-ch8Value)*GAIN_CORRECTION/4000) - KELVIN_TO_CELSIUS_CONST;
}
static void update_charging_limit(struct mxs_info *info)
{
	static int check_count;
	uint16_t limit;
	uint16_t batvol;
	bool in_condition;

	/* average temperature during past 1 sec */
	info->die_tmp = ((info->die_tmp * 9) + measure_die_tmp()) / 10;
	if (info->die_tmp_alarm) {
		if (info->die_tmp < info->die_tmp_low)
			info->die_tmp_alarm = false;
	} else {
		if (info->die_tmp > info->die_tmp_high)
			info->die_tmp_alarm = true;
	}

	/* check per 5 sec */
	check_count++;
	if (check_count >= CHRG_CUR_ADJUST_INTV) {
		check_count = 0;

		if (info->die_tmp > 70)
			limit = CHRG_CUR_TMP_GT_70;
		else if (info->die_tmp > 65)
			limit = CHRG_CUR_TMP_GT_65;
		else if (info->die_tmp > 60)
			limit = CHRG_CUR_TMP_GT_60;
		else if (info->die_tmp > 55)
			limit = CHRG_CUR_TMP_GT_55;
		else if (info->die_tmp > 50)
			limit = CHRG_CUR_TMP_GT_50;
		else
			limit = CHRG_CUR_TMP_LT_50;

		batvol = PWRREG_GET_BATVOL();
		in_condition = (info->charging_ms > TO_CHARGING) || (batvol < CONDITION_VOL);
		
		if (    in_condition
		     && (limit > CHRG_CUR_CONDITION)
		   )
		{
			limit = CHRG_CUR_CONDITION;
		}

		if (limit != info->chrg_trg) {
			info->chrg_trg = limit;
			if (BAT_STAT_CHARGING == info->bat_state) {
				BATT_LOG("[BAT] set charge=%d(mA), tmp:%d bat:%d(mV) charged:%d(min)\n",
					limit, info->die_tmp, batvol, info->charging_ms / 60000);
			}
		}
	}
}

void batt_sm_routine(struct mxs_info *info)
{
	enum BATT_STATE oldstate;

	update_charging_limit(info);
	mxspwr_adjust_cmptrip();

	info->cur_routine_stamp = jiffies_to_msecs(jiffies);
	fetch_irq_event(info);
	if ((STAT_5V_ON_NEW == info->state_5v) || (STAT_5V_OFF_NEW == info->state_5v)) {
		enter_state(info, BAT_STAT_WAIT_5V, stat_5v_names[info->state_5v]);
	}
	oldstate = info->bat_state;
	state_fun[info->bat_state](info);
	while (oldstate != info->bat_state) {
		oldstate = info->bat_state;
		state_fun[info->bat_state](info);
	}

	info->prv_routine_stamp = info->cur_routine_stamp;
}

static void substate_detecting_bat(struct mxs_info *info)
{
	static uint16_t initvol;
	static bool init_charger_on;
	uint16_t nowvol;
	uint32_t age;

	if (!(info->substate_entered)) {
		info->substate_entered = true;
		info->bat_vol = BATVOL_DETECTING;
		init_charger_on = PWRREG_IS_CHRGERON();
		initvol = PWRREG_GET_BATVOL();

		if (init_charger_on) {
			/* initial on, now turn off and set fastfalling */
			mxspwr_set_charging_cur(0);
			PWRREG_TURN_CHRGER(false);
			//PWRREG_SET_FASTFALLING(true);
		} else {
			/* initial off, now turn on with minimum charging cur */
			mxspwr_set_charging_cur(30);
			PWRREG_TURN_CHRGER(true);
		}
	}

	nowvol = PWRREG_GET_BATVOL();
	age = SUBSTATE_AGE;
	if (ABS_DIFF(nowvol, initvol) > NO_BAT_VOL_DIFF) {
		info->bat_vol = BATVOL_NOBATT;
	} else if (age > MS_BAT_DETECT_COST) {
		/* record batt voltage with charger on */
		info->bat_vol = init_charger_on ? initvol : nowvol;
	}

	if (BATVOL_DETECTING != info->bat_vol) {
		/* on exiting this sub-state, charger should be off */
		if (init_charger_on) {
			/* initial on, later off, keep off and clr fastfalling */
			//PWRREG_SET_FASTFALLING(false);
		} else {
			/* initial off, later on, turn off */
			PWRREG_TURN_CHRGER(false);
			mxspwr_set_charging_cur(0);
		}
		info->stamp_batvol = info->cur_routine_stamp;
		/*BATT_LOG("[BAT] %s %s %d/%d takes %d ms\n",
			state_name[info->bat_state],
			info->bat_vol ? "batt detected" : "No batt",
			initvol, nowvol, age);*/
	}
}

static bool substate_ramping(struct mxs_info *info)
{
	static uint16_t tmp_curlimit;
	uint16_t delta;
	uint16_t thsh;
	bool done = false;

	if (!(info->substate_entered)) {
		info->substate_entered = true;
		if (PWRREG_IS_CHRGERON()) {
			tmp_curlimit = mxspwr_get_charging_cur();
		} else {
			tmp_curlimit = 0;
			mxspwr_set_charging_cur(0);
			PWRREG_TURN_CHRGER(true);
		}
	}

	if (tmp_curlimit >= info->chrg_trg) {
		tmp_curlimit = info->chrg_trg;
	} else {
		delta = (CHRG_CUR_RAMP_SLOPE * ROUTINE_GAP) / 1000;
		if (delta > 20)
			delta = 20;
		tmp_curlimit += delta;
		if (tmp_curlimit > info->chrg_trg)
			tmp_curlimit = info->chrg_trg;
	}

	mxspwr_set_charging_cur(tmp_curlimit);
	done = (tmp_curlimit >= info->chrg_trg);

	if (done) {
		thsh = tmp_curlimit / 10;
		if (thsh < 30)
			thsh = 30;
		mxspwr_set_charg_stop_thsh(thsh);
		//BATT_LOG("[BAT] %s ramping to %d mV takes %d ms\n", state_name[info->bat_state], tmp_curlimit, SUBSTATE_AGE);
	}
	
	return done;
}

static void state_uninited(struct mxs_info *info)
{
	/* do nothing, wait 5v event to enter wait5v */
}

static void state_wait5v(struct mxs_info *info)
{
	static enum MXS_5V_STATE old_5vstate;
	if (!(info->state_entered)) {
		info->state_entered = true;
		old_5vstate = info->state_5v;
		return;
	}

	switch (old_5vstate) {
		case STAT_5V_OFF_NEW:
			if (STAT_5V_OFF_HALF == info->state_5v) {
				/* turn off 5v->4p2->DCDC, clear charger_4p2 limit */
				if (PWRREG_IS_5V4P2DCDC_ON()) {
					CLR_PWR_REG_BITS(HW_POWER_DCDC4P2, BM_POWER_DCDC4P2_ENABLE_DCDC | BM_POWER_DCDC4P2_ENABLE_4P2);
					WR_PWR_REG(HW_POWER_5VCTRL_SET, BM_POWER_5VCTRL_PWD_CHARGE_4P2);
					PWRREG_SET_VBSVALID_THRSH(VBUSVALID_THRESH_4_40V);
				}
				//BATT_LOG("[BAT] %s 5v really disappears...\n", state_name[info->bat_state]);
			}
			break;

		case STAT_5V_OFF_HALF:
			if (STAT_5V_OFF_STABLE == info->state_5v) {
				if (!has_irq_event()) {
					disable_irq(IRQ_VDD5V_DROOP);
					disable_irq(IRQ_VDD5V);
					if (!has_irq_event()) {
						PWRREG_ENABLE_5VON_INT(true);
						PWRREG_ENABLE_VDDDAIO_INT(true);
						PWRREG_ENABLE_BATBO_INT(true); /* enable BATT-BO INT */
					}
					enable_irq(IRQ_VDD5V_DROOP);
					enable_irq(IRQ_VDD5V);
				}
			}
			break;

		case STAT_5V_ON_NEW:
			if (STAT_5V_ON_STABLE == info->state_5v) {
				if (!has_irq_event()) {
					disable_irq(IRQ_VDD5V_DROOP);
					disable_irq(IRQ_VDD5V);
					if (!has_irq_event()) {
						PWRREG_ENABLE_5VON_INT(false);
						PWRREG_ENABLE_5VOFF_INT(false);
						PWRREG_ENABLE_VDDDAIO_INT(false);

						if (PWRREG_IS_5V4P2DCDC_ON())
							mxspwr_rampup_chrgr_4p2_limit(780);
						else
							mxspwr_enable4p2(780);

						PWRREG_ENABLE_5VOFF_INT(true);
						PWRREG_ENABLE_VDDDAIO_INT(true);
						PWRREG_ENABLE_BATBO_INT(false);
						
						enable_irq(IRQ_VDD5V_DROOP);
						enable_irq(IRQ_VDD5V);

						/*BATT_LOG("[BAT] %s ramping 4p2 takes %d ms\n", state_name[info->bat_state],
							MXS_DELTA((info->cur_routine_stamp), jiffies_to_msecs(jiffies)));*/
						enter_state(info, BAT_STAT_WAIT_BAT, stat_5v_names[info->state_5v]);
					} else {
						enable_irq(IRQ_VDD5V_DROOP);
						enable_irq(IRQ_VDD5V);
					}
				}
			}
			break;

		case STAT_5V_OFF_STABLE:
		case STAT_5V_ON_STABLE:
		default:
			break;
	}
	old_5vstate = info->state_5v;
}

static void state_waitbat(struct mxs_info *info)
{
	static uint16_t oldvol;
	uint16_t newvol;
	unsigned int substate_age;

	if (!(info->state_entered)) {
		info->state_entered = true;
		enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
	}

	switch(info->bat_substate) {
		case BAT_SUB_STATE_DETECTING_BAT:
			substate_detecting_bat(info);
			switch (info->bat_vol) {
				case BATVOL_DETECTING:
					break;

				case BATVOL_NOBATT:
					enter_substate(info, BAT_SUB_STATE_NORMAL);
					break;

				default:
					sprintf(g_reason, "batt detected at %d mV", info->bat_vol);
					enter_state(info, BAT_STAT_CHARGING, g_reason);
					break;
			}
			break;

		case BAT_SUB_STATE_NORMAL:
			substate_age = SUBSTATE_AGE;
			//printk("wangluheng: 0x%x - 0x%x = 0x%x\n", info->cur_routine_stamp, info->substate_begin_stamp, substate_age);
			if (substate_age < 1000)
				break;

			if (!(info->substate_entered)) {
				/* 1st time */
				info->substate_entered = true;
				oldvol = PWRREG_GET_BATVOL();
			} else {
				/* not 1st time */
				newvol = PWRREG_GET_BATVOL();
				if (ABS_DIFF(oldvol,newvol) > DUBIOUS_VOL_DIFF) {
					BATT_LOG("[BAT] %s dubious batvol, %d/%d\n", state_name[info->bat_state], oldvol, newvol);
					enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
				} else if (BATDETECT_AGE > BAT_DETECT_INTV_NOBAT) {
					//BATT_LOG("[BAT] %s batdetect timeout %d\n", state_name[info->bat_state], BATDETECT_AGE);
					enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
				} else {
					oldvol = newvol;
				}
			}
			break;

		default:
			enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
			break;
	}
}

static void state_charging(struct mxs_info *info)
{
	static uint16_t oldvol;
	static uint32_t chrgr_halted_count;
	static uint32_t stagingoff_stamp;
	uint16_t newvol;
	uint32_t batdetect_age;

	if (!(info->state_entered)) {
		info->state_entered = true;
		chrgr_halted_count = 0;
		info->charging_ms = 0;
		enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
	}

	switch (info->bat_substate) {
		case BAT_SUB_STATE_DETECTING_BAT:
			substate_detecting_bat(info);
			switch (info->bat_vol) {
				case BATVOL_DETECTING:
					break;

				case BATVOL_NOBATT:
					info->charging_ms = 0;
					enter_state(info, BAT_STAT_WAIT_BAT, "batt disappears");
					break;

				default:
					enter_substate(info, BAT_SUB_STATE_RAMPING);
					break;
			}
			break;

		case BAT_SUB_STATE_RAMPING:
			if (substate_ramping(info)) {
				/* ramping done */
				enter_substate(info, BAT_SUB_STATE_NORMAL);
			}
			break;

		case BAT_SUB_STATE_NORMAL:
			newvol = PWRREG_GET_BATVOL();
			info->charging_ms += ROUTINE_GAP;

			if (info->charging_ms > TO_CHARGING) {
				if (newvol > 4100) {
					sprintf(g_reason, "charging expires at %d mV", newvol);
					info->charging_ms = 0;
					enter_state(info, BAT_STAT_FULL_CHRGD, g_reason);
					break;
				}
			}

			if (mxspwr_expressible_cur(info->chrg_trg) != mxspwr_get_charging_cur()) {
				enter_substate(info, BAT_SUB_STATE_RAMPING);
				break;
			}

			if (SUBSTATE_AGE < 1000) {
				/* charging state is entered from ramping, wait a debounce for batt-vol stable and possible halt flag stable */
				break;
			}

			if (!(info->substate_entered)) {
				/* 1st time entering this sub-state */
				info->substate_entered = true;
			} else {
				/* not 1st time */
				if (ABS_DIFF(oldvol,newvol) > DUBIOUS_VOL_DIFF) {
					BATT_LOG("[BAT] %s dubious batvol, %d/%d\n", state_name[info->bat_state], oldvol, newvol);
					enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
					break;
				}

				if (PWRREG_CHRG_HALTED()) {
					chrgr_halted_count++;
				} else {
					if (chrgr_halted_count >= 11) {
						BATT_LOG("[BAT] %s exiting staging-off", state_name[info->bat_state]);
					}
					chrgr_halted_count = 0;
				}

				batdetect_age = BATDETECT_AGE;
				if (0 == chrgr_halted_count) {
					/* hw is charging normally */
					if (batdetect_age > BAT_DETECT_INTV_CHARGING) {
						enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
					}
				} else if (10 > chrgr_halted_count) {
					/* charging halt flag detected not enough times, do nothing and wait */
				} else if (10 == chrgr_halted_count) {
					/* prepare to enter staging-off, but before that, detect batt for sure */
					enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
				} else if (11 == chrgr_halted_count) {
					/* charging halt flag detected enough times, batt detected also, enter staging-off, save stamp */
					BATT_LOG("[BAT] %s entering staging-off...\n", state_name[info->bat_state]);
					stagingoff_stamp = info->cur_routine_stamp;
				} else {
					/* in staging-off, wait timeout to enter full state */
					if (batdetect_age > BAT_DETECT_INTV_STAGINGOFF) {
						enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
					} else if (MXS_DELTA(stagingoff_stamp,info->cur_routine_stamp) > TO_STAGINGOFF) {
						sprintf(g_reason, "stagingoff enough at %d mV", newvol);
						info->charging_ms = 0;
						enter_state(info, BAT_STAT_FULL_CHRGD, "stagingoff timeout");
					}
				}
			}
			oldvol = newvol;
			break;

		default:
			enter_substate(info, BAT_SUB_STATE_DETECTING_BAT);
			break;
	}
}

static void state_full(struct mxs_info *info)
{
	uint16_t batvol;

	batvol = PWRREG_GET_BATVOL();
	if (batvol < 4000) {
		sprintf(g_reason, "batvol at %d mV", batvol);
		enter_state(info, BAT_STAT_CHARGING, g_reason);
	}
}



