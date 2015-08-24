#ifndef __POWER__MXS_BATTERY_H

#define __POWER__MXS_BATTERY_H

#include <linux/power_supply.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/io.h>
#include "regs-power.h"


//#define DBG_BATT 1
//#if DBG_BATT
//#define BATT_LOG printk
//#else
//#define BATT_LOG(x,...) do {} while(0)
//#endif
extern uint32_t batdbg;
#define BATT_LOG(fmt, arg...)	do { if (batdbg) printk(KERN_INFO fmt , ## arg); } while (0)


/* DIG_CTL REGS DEFINITION */
//#define HW_DIGCTL_MICROSECONDS	(0x000000c0)

extern int mxs_pwr_irqs[];
extern void __iomem *mxs_pwr_base;
extern void __iomem *mxs_rtc_base;
extern void __iomem *mxs_lradc_base;
extern char* stat_5v_names[];
//extern void __iomem *mxs_digctl_base;

enum MXS_PWR_IRQ_INDEX_type {
	INDEX_IRQ_BATT_BO,
	INDEX_IRQ_VDDD_BO,
	INDEX_IRQ_VDDIO_BO,
	INDEX_IRQ_VDDA_BO,
	INDEX_IRQ_VDD5V_DROOP,
	INDEX_IRQ_DCDC4P2_BO,
	INDEX_IRQ_VDD5V,
	NUMS_MXS_PWR_IRQS,
};

enum MXS_5V_STATE {
	STAT_5V_ON_NEW,
	STAT_5V_ON_STABLE,
	STAT_5V_OFF_NEW,
	STAT_5V_OFF_HALF,
	STAT_5V_OFF_STABLE,
};

enum MXS_IRQ_EVENT {
	EVNT_NONE,
	EVNT_5V_ON,
	EVNT_5V_OFF,
};

#define MXS_DELTA(prev,now) (((now) >= (prev)) ? ((now) - (prev)) : ((now) + (0xffffffff - (prev))))
#define ABS_DIFF(a,b) (((a) >= (b)) ? ((a) - (b)) : ((b) - (a)))

#define IRQ_BATT_BRNOUT		(mxs_pwr_irqs[INDEX_IRQ_BATT_BO])
#define IRQ_VDDD_BRNOUT		(mxs_pwr_irqs[INDEX_IRQ_VDDD_BO])
#define IRQ_VDDIO_BRNOUT	(mxs_pwr_irqs[INDEX_IRQ_VDDIO_BO])
#define IRQ_VDDA_BRNOUT		(mxs_pwr_irqs[INDEX_IRQ_VDDA_BO])
#define IRQ_VDD5V_DROOP		(mxs_pwr_irqs[INDEX_IRQ_VDD5V_DROOP])
#define IRQ_DCDC4P2_BRNOUT	(mxs_pwr_irqs[INDEX_IRQ_DCDC4P2_BO])
#define IRQ_VDD5V			(mxs_pwr_irqs[INDEX_IRQ_VDD5V])

#define REGS_POWER_BASE mxs_pwr_base
#define REGS_LRADC_BASE mxs_lradc_base

#define LINREG_OFFSET_1_STEP_BL	0x2
#define BATVOL_UNIT_8MV 8

#define BATTERY_VOLTAGE_CH 7
#define BATTERY_DIETEMP_CH 0

#define LRADC_DELAY_TRIGGER_BATTERY	3

/* when consecutive batt vol diff > this, batt detaching/attaching might happened */
#define DUBIOUS_VOL_DIFF 180

/* bat-vol difference between charger on and off, when diff > 'this', no batt exists */
#define NO_BAT_VOL_DIFF 400

/* bat detecing spent time, milli sec */
#define MS_BAT_DETECT_COST 1000

#define CONDITION_VOL 3000


#define CHRG_CUR_TMP_GT_70 0
#define CHRG_CUR_TMP_GT_65 120
#define CHRG_CUR_TMP_GT_60 200
#define CHRG_CUR_TMP_GT_55 300
#define CHRG_CUR_TMP_GT_50 400
#define CHRG_CUR_TMP_LT_50 500

#define CHRG_CUR_CONDITION 160

#define CHRG_CUR_RAMP_SLOPE 150

#define CHRG_CUR_ADJUST_INTV 50

/* interval for detecting batt
 *     1.5 min in charging state
 *     5   min in staging-off state
 *     20  sec in waiting bat state */
#define BAT_DETECT_INTV_CHARGING   90000
#define BAT_DETECT_INTV_STAGINGOFF 300000
#define BAT_DETECT_INTV_NOBAT      20000

/* timeout in staging-off, when expires, enter full state, 30 min (30 * 60 * 1000) */
#define TO_STAGINGOFF 1800000
/* timeout in charging, when expires, lower charging cur if bat < 4.1 else enter full, 4 hour (4*60*60*1000) */
#define TO_CHARGING   14400000

#define WR_PWR_REG(reg,val)   __raw_writel((val), mxs_pwr_base +   (reg))
#define WR_LRADC_REG(reg,val) __raw_writel((val), mxs_lradc_base + (reg))
#define RD_PWR_REG(reg)       __raw_readl(mxs_pwr_base +   (reg))
#define RD_LRADC_REG(reg)     __raw_readl(mxs_lradc_base + (reg))

#define CLR_PWR_REG_BITS(reg,clr) WR_PWR_REG((reg),(RD_PWR_REG(reg) & (~(clr))))
#define SET_PWR_REG_BITS(reg,set) WR_PWR_REG((reg),(RD_PWR_REG(reg) | (set)))
#define CLR_SET_PWR_REG_BITS(reg,clr,set) WR_PWR_REG((reg),((RD_PWR_REG(reg) & (~(clr))) | (set)))

#define hw_lradc_present(ch) (((ch) < 0 || (ch) > 7) ? 0 : \
	(0 != (RD_LRADC_REG(HW_LRADC_STATUS) & (1 << (16 + (ch))))))

#define BATVOL_DETECTING -1
#define BATVOL_NOBATT   0

/* HW_POWER_5VCTRL : VBUSVALID_TRSH bits, bit[8,10] */
#define VBUSVALID_THRESH_2_90V		0x0
#define VBUSVALID_THRESH_4_00V		0x1
#define VBUSVALID_THRESH_4_10V		0x2
#define VBUSVALID_THRESH_4_20V		0x3
#define VBUSVALID_THRESH_4_30V		0x4
#define VBUSVALID_THRESH_4_40V		0x5
#define VBUSVALID_THRESH_4_50V		0x6
#define VBUSVALID_THRESH_4_60V		0x7

/* HW_POWER_5VCTRL : VBUSDROOP_TRSH bits, bit[28,29] */
#define VBUSDROOP_TRSH_4_3          0x0
#define VBUSDROOP_TRSH_4_4          0x1
#define VBUSDROOP_TRSH_4_5          0x2
#define VBUSDROOP_TRSH_4_7          0x3


/* HW_POWER_DCDC4P2 : CMPTRIP bits, bit [0,4]
 * 4p2 >= 0.85 * batt
 * 4p2 >= 0.86 * batt
 * 4p2 >= 1.00 * batt
 * 4p2 >= 1.05 * batt
 */
#define CMPTRIP_4P2_GE_0_85_BAT     0x00
#define CMPTRIP_4P2_GE_0_86_BAT     0x01
#define CMPTRIP_4P2_GE_1_00_BAT     0x18
#define CMPTRIP_4P2_GE_1_05_BAT     0x1f

/* HW_POWER_DCDC4P2 : TRG bits, bit [16,18] */
#define TRG4P2_4_2                  0x0
#define TRG4P2_4_1                  0x1
#define TRG4P2_4_0                  0x2
#define TRG4P2_3_9                  0x3
#define TRG4P2_BAT                  0x4

/* HW_POWER_DCDC4P2 : DROPOUT_CTRL : STEAL_THRESH bits, [30,31], DCDC_SRC_SEL, [28,29]
 * use DCDC source: always 4p2, max(4p2,batt), cmptrip_max(4p2,batt)
 * 4p2 steal from charger when (4p2 < TRG - 25/50/100/200 mV), 
 */
#define DROPCTRL_DCSRC_4P2          0x0000
#define DROPCTRL_DCSRC_HIGHER       0x0001
#define DROPCTRL_DCSRC_CMPTRP       0x0010
#define DROPCTRL_STLTHR__25_BL      0x0000
#define DROPCTRL_STLTHR__50_BL      0x0100
#define DROPCTRL_STLTHR_100_BL      0x1000
#define DROPCTRL_STLTHR_200_BL      0x1100

#define PWRREG_SET_VBUSDROOP_TRSH(trsh)                                            \
	do {                                                                           \
		WR_PWR_REG(HW_POWER_5VCTRL_CLR, BM_POWER_5VCTRL_VBUSDROOP_TRSH);           \
		if (trsh)                                                                  \
			WR_PWR_REG(HW_POWER_5VCTRL_SET, BF_POWER_5VCTRL_VBUSDROOP_TRSH(trsh)); \
	} while (0)

#define PWRREG_SET_4P2LDO_TRG(trg) CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2,      \
	                                                 BM_POWER_DCDC4P2_TRG,     \
	                                                 BF_POWER_DCDC4P2_TRG(trg))

#define PWRREG_SET_DCDC4P2_DROPCTRL(val) CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2,                  \
	                                                          BM_POWER_DCDC4P2_DROPOUT_CTRL,     \
	                                                          BF_POWER_DCDC4P2_DROPOUT_CTRL(val))

#define PWRREG_SET_CMPTRIP(val) CLR_SET_PWR_REG_BITS(HW_POWER_DCDC4P2,             \
	                                                 BM_POWER_DCDC4P2_CMPTRIP,     \
	                                                 BF_POWER_DCDC4P2_CMPTRIP(val))

#define PWRREG_TURN_CHARGER_AND_4P2(on)                                      \
    do {                                                                     \
        if (on)                                                              \
            WR_PWR_REG(HW_POWER_5VCTRL_CLR, BM_POWER_5VCTRL_PWD_CHARGE_4P2); \
        else                                                                 \
            WR_PWR_REG(HW_POWER_5VCTRL_SET, BM_POWER_5VCTRL_PWD_CHARGE_4P2); \
    } while (0)

#define PWRREG_TURN_CHRGER(on)                                             \
    do {                                                                   \
        if (on)                                                            \
            WR_PWR_REG(HW_POWER_CHARGE_CLR, BM_POWER_CHARGE_PWD_BATTCHRG); \
        else                                                               \
            WR_PWR_REG(HW_POWER_CHARGE_SET, BM_POWER_CHARGE_PWD_BATTCHRG); \
    } while (0)

#define PWRREG_IS_CHRGERON() ((RD_PWR_REG(HW_POWER_CHARGE) & BM_POWER_CHARGE_PWD_BATTCHRG) == 0)

#define PWRREG_IS_5V4P2DCDC_ON() (    (0 == (RD_PWR_REG(HW_POWER_5VCTRL) & BM_POWER_5VCTRL_PWD_CHARGE_4P2)) \
                                   && (0 != (RD_PWR_REG(HW_POWER_DCDC4P2) & BM_POWER_DCDC4P2_ENABLE_4P2))   \
                                   && (0 != (RD_PWR_REG(HW_POWER_DCDC4P2) & BM_POWER_DCDC4P2_ENABLE_DCDC))  \
                                  )

#define PWRREG_CHRG_HALTED() ((RD_PWR_REG(HW_POWER_STS) & BM_POWER_STS_CHRGSTS) == 0)

#define PWRREG_GET_BATVOL() (uint16_t)(((RD_PWR_REG(HW_POWER_BATTMONITOR) & BM_POWER_BATTMONITOR_BATT_VAL) \
                                        >> BP_POWER_BATTMONITOR_BATT_VAL) * 8)

#define DETECT_5V_METHOD_VDD5V_GT_VDDIO
#ifdef DETECT_5V_METHOD_VDD5V_GT_VDDIO
#define BM_POWER_STS_5VON        BM_POWER_STS_VDD5V_GT_VDDIO
#define BM_POWER_CTRL_ENIRQ_5VON BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO
#define BM_POWER_CTRL_5VON_IRQ   BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ
#else
#define BM_POWER_STS_5VON        BM_POWER_STS_VBUSVALID0
#define BM_POWER_CTRL_ENIRQ_5VON BM_POWER_CTRL_ENIRQ_VBUS_VALID
#define BM_POWER_CTRL_5VON_IRQ   BM_POWER_CTRL_VBUSVALID_IRQ
#endif

#define PWRREG_STS_IS_5VON() ((BM_POWER_STS_5VON & RD_PWR_REG(HW_POWER_STS)) != 0)

#define PWRREG_SET_FASTFALLING(on)                                          \
    do {                                                                    \
        if (on)                                                             \
            WR_PWR_REG(HW_POWER_REFCTRL_SET,BM_POWER_REFCTRL_FASTSETTLING); \
        else                                                                \
            WR_PWR_REG(HW_POWER_REFCTRL_CLR,BM_POWER_REFCTRL_FASTSETTLING); \
    } while (0)

#define PWRREG_SET_VBSVALID_THRSH(val)                                           \
	do {                                                                         \
		WR_PWR_REG(HW_POWER_5VCTRL_CLR,BM_POWER_5VCTRL_VBUSVALID_TRSH);          \
		if (val)                                                                 \
			WR_PWR_REG(HW_POWER_5VCTRL_SET,BF_POWER_5VCTRL_VBUSVALID_TRSH(val)); \
	} while (0)

#define PWRREG_ENABLE_ALL_INT(enable)                                                      \
    do {                                                                                   \
        if (enable) {                                                                      \
            WR_PWR_REG(HW_POWER_CTRL_CLR,                  BM_POWER_CTRL_VBUSVALID_IRQ     \
                    | BM_POWER_CTRL_VDD5V_GT_VDDIO_IRQ   | BM_POWER_CTRL_VDDD_BO_IRQ       \
                    | BM_POWER_CTRL_VDDA_BO_IRQ          | BM_POWER_CTRL_VDDIO_BO_IRQ      \
                    | BM_POWER_CTRL_BATT_BO_IRQ          | BM_POWER_CTRL_DC_OK_IRQ         \
                    | BM_POWER_CTRL_PSWITCH_IRQ          | BM_POWER_CTRL_VDD5V_DROOP_IRQ   \
                    | BM_POWER_CTRL_DCDC4P2_BO_IRQ);                                       \
            WR_PWR_REG(HW_POWER_CTRL_SET,                  BM_POWER_CTRL_ENIRQ_VBUS_VALID  \
                    | BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO | BM_POWER_CTRL_ENIRQ_VDDD_BO     \
                    | BM_POWER_CTRL_ENIRQ_VDDA_BO        | BM_POWER_CTRL_ENIRQ_VDDIO_BO    \
                    | BM_POWER_CTRL_ENIRQBATT_BO         | BM_POWER_CTRL_ENIRQ_DC_OK       \
                    | BM_POWER_CTRL_ENIRQ_PSWITCH        | BM_POWER_CTRL_ENIRQ_VDD5V_DROOP \
                    | BM_POWER_CTRL_ENIRQ_DCDC4P2_BO);                                     \
       	} else {                                                                           \
            WR_PWR_REG(HW_POWER_CTRL_CLR,                  BM_POWER_CTRL_ENIRQ_VBUS_VALID  \
                    | BM_POWER_CTRL_ENIRQ_VDD5V_GT_VDDIO | BM_POWER_CTRL_ENIRQ_VDDD_BO     \
                    | BM_POWER_CTRL_ENIRQ_VDDA_BO        | BM_POWER_CTRL_ENIRQ_VDDIO_BO    \
                    | BM_POWER_CTRL_ENIRQBATT_BO         | BM_POWER_CTRL_ENIRQ_DC_OK       \
                    | BM_POWER_CTRL_ENIRQ_PSWITCH        | BM_POWER_CTRL_ENIRQ_VDD5V_DROOP \
                    | BM_POWER_CTRL_ENIRQ_DCDC4P2_BO);                                     \
        }                                                                                  \
    } while (0)

#define PWRREG_ENABLE_5VON_INT(enable)                               \
    do {                                                             \
        if (enable) {                                                \
            WR_PWR_REG(HW_POWER_CTRL_CLR, BM_POWER_CTRL_5VON_IRQ);   \
            WR_PWR_REG(HW_POWER_CTRL_SET, BM_POWER_CTRL_ENIRQ_5VON); \
    	} else {                                                     \
            WR_PWR_REG(HW_POWER_CTRL_CLR, BM_POWER_CTRL_ENIRQ_5VON); \
        }                                                            \
    } while (0)

#define PWRREG_ENABLE_5VOFF_INT(enable)                                     \
    do {                                                                    \
        if (enable) {                                                       \
            WR_PWR_REG(HW_POWER_CTRL_CLR, BM_POWER_CTRL_VDD5V_DROOP_IRQ);   \
            WR_PWR_REG(HW_POWER_CTRL_SET, BM_POWER_CTRL_ENIRQ_VDD5V_DROOP); \
        } else {                                                            \
            WR_PWR_REG(HW_POWER_CTRL_CLR,BM_POWER_CTRL_ENIRQ_VDD5V_DROOP);  \
        }                                                                   \
    } while (0)

#define PWRREG_ENABLE_VDDDAIO_INT(enable)                                           \
    do {                                                                            \
        if (enable) {                                                               \
            WR_PWR_REG(HW_POWER_CTRL_CLR,            BM_POWER_CTRL_VDDD_BO_IRQ   |  \
				       BM_POWER_CTRL_VDDA_BO_IRQ   | BM_POWER_CTRL_VDDIO_BO_IRQ);   \
				                                                                    \
            WR_PWR_REG(HW_POWER_CTRL_SET,            BM_POWER_CTRL_ENIRQ_VDDD_BO |  \
                       BM_POWER_CTRL_ENIRQ_VDDA_BO | BM_POWER_CTRL_ENIRQ_VDDIO_BO); \
        } else {                                                                    \
            WR_PWR_REG( HW_POWER_CTRL_CLR,           BM_POWER_CTRL_ENIRQ_VDDD_BO |  \
				       BM_POWER_CTRL_ENIRQ_VDDA_BO | BM_POWER_CTRL_ENIRQ_VDDIO_BO); \
        }                                                                           \
    }while (0)

#define PWRREG_ENABLE_BATBO_INT(enable)                                \
    do {                                                               \
        if (enable) {                                                  \
            WR_PWR_REG(HW_POWER_CTRL_CLR, BM_POWER_CTRL_BATT_BO_IRQ);  \
            WR_PWR_REG(HW_POWER_CTRL_SET, BM_POWER_CTRL_ENIRQBATT_BO); \
        } else {                                                       \
            WR_PWR_REG(HW_POWER_CTRL_CLR, BM_POWER_CTRL_ENIRQBATT_BO); \
        }                                                              \
    } while (0)

#define PWRREG_DISABLE_VDDDAIO5V_BO_SHUTDOWN() \
  do { \
    CLR_PWR_REG_BITS(HW_POWER_VDDDCTRL,  BM_POWER_VDDDCTRL_PWDN_BRNOUT); \
    CLR_PWR_REG_BITS(HW_POWER_VDDACTRL,  BM_POWER_VDDACTRL_PWDN_BRNOUT); \
    CLR_PWR_REG_BITS(HW_POWER_VDDIOCTRL, BM_POWER_VDDIOCTRL_PWDN_BRNOUT); \
    WR_PWR_REG(HW_POWER_5VCTRL_CLR, BM_POWER_5VCTRL_PWDN_5VBRNOUT); \
  } while (0)

enum BATT_STATE {
	BAT_STAT_UNINITED,
	BAT_STAT_WAIT_5V,
	BAT_STAT_WAIT_BAT,
	BAT_STAT_CHARGING,
	BAT_STAT_FULL_CHRGD,
};

enum BATT_SUB_STATE {
	BAT_SUB_STATE_DETECTING_BAT,
	BAT_SUB_STATE_RAMPING,
	BAT_SUB_STATE_NORMAL,
	
};

#define to_mxs_info(x) container_of((x), struct mxs_info, bat)

#define BC_ROUTINE_INTV 100

#define _5V_ON_STABLE_DEBOUNCE 500
#define _5V_OFF_HALF_DEBOUNCE 400
#define _5V_OFF_STABLE_DEBOUNCE 200

struct mxs_info {
	struct device *dev;

	struct power_supply bat;
	struct power_supply ac;

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

	enum MXS_5V_STATE state_5v;
	uint32_t stamp_5v;          /* jiffies when 'state_5v' changed to current state */
	uint16_t chrg_trg;
	int      bat_vol; 			/* last detected batt voltage */
	uint32_t stamp_batvol;      /* jiffies when 'bat_vol' is set */
	int      charging_ms;       /* time elapsed in charging */

	bool     die_tmp_alarm;
	bool     prv_die_tmp_alarm;
	uint8_t  die_tmp_high;
	uint8_t  die_tmp_low;
	uint16_t bat_tmp_high;
	uint16_t bat_tmp_low;
	uint16_t die_tmp;

	enum BATT_STATE bat_state;
	enum BATT_SUB_STATE bat_substate;
	uint32_t prv_routine_stamp;
	uint32_t cur_routine_stamp;
	uint32_t state_begin_stamp;
	uint32_t substate_begin_stamp;
	bool     state_entered;
	bool     substate_entered;
	
	u32	clks[10];
	struct clk *lradc_clk;
};


void fetch_irq_event(struct mxs_info *info);
void save_irq_event(enum MXS_IRQ_EVENT evt);
bool has_irq_event(void);

void init_batt_sm(struct mxs_info *info);
void batt_sm_routine(struct mxs_info *info);

#endif


