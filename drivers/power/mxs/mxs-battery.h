#ifndef __POWER__MXS_BATTERY_H

#define __POWER__MXS_BATTERY_H


/* DIG_CTL REGS DEFINITION */
#define HW_DIGCTL_MICROSECONDS	(0x000000c0)

extern int mxs_pwr_irqs[];
extern void __iomem *mxs_pwr_base;
extern void __iomem *mxs_digctl_base;
extern void __iomem *mxs_rtc_base;
extern void __iomem *mxs_lradc_base;

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

#define IRQ_BATT_BRNOUT		(mxs_pwr_irqs[INDEX_IRQ_BATT_BO])
#define IRQ_VDDD_BRNOUT		(mxs_pwr_irqs[INDEX_IRQ_VDDD_BO])
#define IRQ_VDDIO_BRNOUT	(mxs_pwr_irqs[INDEX_IRQ_VDDIO_BO])
#define IRQ_VDDA_BRNOUT		(mxs_pwr_irqs[INDEX_IRQ_VDDA_BO])
#define IRQ_VDD5V_DROOP		(mxs_pwr_irqs[INDEX_IRQ_VDD5V_DROOP])
#define IRQ_DCDC4P2_BRNOUT	(mxs_pwr_irqs[INDEX_IRQ_DCDC4P2_BO])
#define IRQ_VDD5V			(mxs_pwr_irqs[INDEX_IRQ_VDD5V])

#define REGS_POWER_BASE mxs_pwr_base
#define REGS_LRADC_BASE mxs_lradc_base

#define USB_ONLINE      0x01
#define USB_REG_SET     0x02
#define USB_SM_RESTART  0x04
#define USB_SHUTDOWN    0x08
#define USB_N_SEND      0x10
#define NORMAL_SOURCE  0x00
#define NO_DCDC_BATT_SOURCE 0x01
#define NO_VDD5V_SOURCE 0x02

#define LINREG_OFFSET_1_STEP_BL	0x2

#define BATTERY_VOLTAGE_CH 7
#define LRADC_DELAY_TRIGGER_BUTTON	0
#define LRADC_DELAY_TRIGGER_TOUCHSCREEN	1
#define LRADC_DELAY_TRIGGER_DIE		2
#define LRADC_DELAY_TRIGGER_BATTERY	3

#define WR_PWR_REG(val,addr) (__raw_writel((val), mxs_pwr_base + (addr)))
#define RD_PWR_REG(addr) (__raw_readl(mxs_pwr_base + (addr)))

/* clear bits of a pwr-reg */
#define CLR_PWR_REG_BITS(reg,clr) WR_PWR_REG((RD_PWR_REG(reg) & (~(clr))),(reg))

/* set bits of a pwr-reg */
#define SET_PWR_REG_BITS(reg,set) WR_PWR_REG((RD_PWR_REG(reg) | (set)),(reg))

/* clr first, set then */
#define CLR_SET_PWR_REG_BITS(reg,clr,set) WR_PWR_REG(((RD_PWR_REG(reg) & (~(clr))) | (set)),(reg))

#define WR_LRADC_REG(val,addr) __raw_writel((val), mxs_lradc_base + (addr))
#define RD_LRADC_REG(addr) __raw_readl(mxs_lradc_base + (addr))

#define hw_lradc_present(ch) ((ch) < 0 || (ch) > 7) ? 0 \
	: 0 != (RD_LRADC_REG(HW_LRADC_STATUS) & (1 << (16 + (ch))))


enum application_5v_status{
	_5v_connected_verified,
	_5v_connected_unverified,
	_5v_disconnected_unverified,
	_5v_disconnected_verified,
};


#define to_mxs_info(x) container_of((x), struct mxs_info, bat)

#define NON_USB_5V_SUPPLY_CURRENT_LIMIT_MA 780
#define POWERED_USB_5V_CURRENT_LIMIT_MA 450
#define UNPOWERED_USB_5V_CURRENT_LIMIT_MA 80
#define _5V_DEBOUNCE_TIME_MS 500
#define OS_SHUTDOWN_BATTERY_VOLTAGE_THRESHOLD_MV 3350

void hw_lradc_set_delay_trigger(int trgr, u32 trgr_lradc, u32 delay_trgr, u32 loops, u32 delays);

#endif


