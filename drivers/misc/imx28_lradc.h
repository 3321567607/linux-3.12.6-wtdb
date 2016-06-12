
#ifndef _MXS_LRADC_H__
#define _MXS_LRADC_H__

#include <linux/io.h>
#include <linux/clk.h>


extern void __iomem *mxs_lradc_base;

/* 13 IRQs for lradc module */
#define IMX28_LRADC_IRQ_COUNT 13
/* total 8 virtual channels */
#define IMX28_LRADC_VIRCHAN_NUM 8
/* total lradc pins */
#define IMX28_LRADC_PIN_NUM 7
/* default sample interval, ms */
#define IMX28_DEF_INTV 500
/* default number of samples */
#define IMX28_DEF_LRADC_NUM_SAMPLES 5
/* delay channel for lradc pins, delay channel 0, delay channel 3 is used for batt volt measurement */
#define IMX28_LRADC_COM_DELAY_CHAN 0
/* default divby2 bit */
#define IMX28_DEF_DIVBY2_BIT 1

#define BIT_I(i) (1<<(i))

#define REG_LRADC_CTRL0        (mxs_lradc_base + 0x0000)
#define REG_LRADC_CTRL1        (mxs_lradc_base + 0x0010)
#define REG_LRADC_CTRL2        (mxs_lradc_base + 0x0020)
#define REG_LRADC_CTRL3        (mxs_lradc_base + 0x0030)
#define REG_LRADC_STAT         (mxs_lradc_base + 0x0040)
#define REG_LRADC_CTRL4        (mxs_lradc_base + 0x0140)
#define REG_LRADC_CHAN(n)      (mxs_lradc_base + 0x0050 + (n) * 0x10)
#define REG_LRADC_DELAY(n)     (mxs_lradc_base + 0x00d0 + (n) * 0x10)

#define WRT_REG_VAL(val,reg)   writel((val),(reg))
#define SET_REG_BITS(bits,reg) writel((bits),((reg) + 0x4))
#define CLR_REG_BITS(bits,reg) writel((bits),((reg) + 0x8))
#define TOG_REG_BITS(bits,reg) writel((bits),((reg) + 0xC))

#define OFFSET_LRADC_DELAY_TRIIGER_LRADCS   24
#define OFSSET_LRADC_DELAY_KICK             20
#define	OFFSET_LRADC_DELAY_TRIGGER_DELAYS	16
#define OFFSET_LRADC_DELAY_LOOP_COUNT       11
#define	OFFSET_LRADC_DELAY_DELAY		    0
#define MAXVAL_LRADC_DELAY_DELAY            0x3FF
#define MASK_LRADC_DELAY_DELAY              (MAXVAL_LRADC_DELAY_DELAY << OFFSET_LRADC_DELAY_DELAY)
#define MAXVAL_LRADC_DELAY_LOOP_COUNT       0x1F
#define MASK_LRADC_DELAY_LOOP_COUNT         (MAXVAL_LRADC_DELAY_LOOP_COUNT << OFFSET_LRADC_DELAY_LOOP_COUNT)



#define OFFSET_LRADC_CHN_VALUE              0
#define OFFSET_LRADC_CHN_NUM_SAMPLES        24
#define OFFSET_LRADC_CHN_ACCUMULATE         29
#define OFFSET_LRADC_CHN_TOGGLE             31
#define MASK_LRADC_CHN_VALUE                0x3FFFF
#define BIT_LRADC_CHN_ACCUMU                (1 << OFFSET_LRADC_CHN_ACCUMULATE)
#define MAX_NUM_SAMPLES                     0x1F

#if (IMX28_DEF_LRADC_NUM_SAMPLES)
#define IMX28_DEF_CHAN_SETTING ((1 << OFFSET_LRADC_CHN_ACCUMULATE) | (IMX28_DEF_LRADC_NUM_SAMPLES << OFFSET_LRADC_CHN_NUM_SAMPLES))
#else
#define IMX28_DEF_CHAN_SETTING 0x00000000
#endif


#define OFFSET_LRADC_CTRL1_IRQ    0
#define OFFSET_LRADC_CTRL1_IRQ_EN 16

#define LRADC_BIND_CHAN(vidx,pidx) do { CLR_REG_BITS((0x0F << (vidx << 2)),REG_LRADC_CTRL4); \
	                                    SET_REG_BITS(((pidx) << (vidx << 2)),REG_LRADC_CTRL4); \
                                      } while (0)

#define LRADC_KICK_DELAY_CHAN(n)   SET_REG_BITS((1 << OFSSET_LRADC_DELAY_KICK), REG_LRADC_DELAY(n))
#define LRADC_ENABLE_IRQ_CHAN(n)   SET_REG_BITS(((1 << (n)) << OFFSET_LRADC_CTRL1_IRQ_EN),REG_LRADC_CTRL1)
#define LRADC_DISABLE_IRQ_CHAN(n)  CLR_REG_BITS(((1 << (n)) << OFFSET_LRADC_CTRL1_IRQ_EN),REG_LRADC_CTRL1)
#define INVALID_CHAN_INDEX 0xFF
#define VALID_CHANN(idx) ((INVALID_CHAN_INDEX == idx) ? 0 : 1)


struct imx28_lradc_phychan {
	unsigned char virchan_idx;
	unsigned int  volt;
	bool          div_by2;
	bool          new_div_by2;
	struct device *dev;
};

struct imx28_lradc_info {
	unsigned char            bind_table[IMX28_LRADC_VIRCHAN_NUM];     /* for each virtual channel, which hw pin is wired? */
	unsigned char            numsample;     /* number of samples */
	unsigned char            new_numsample; /* new value */
	unsigned char            usedvirt;      /* 1:allocated, 0:free */
	unsigned char            groupbits;     /* each bit represents a virtual channel, all of them group into a delayed channel */
	unsigned char            max_virt;
	unsigned char            hwpins;        /* lradc0 ~ lradc6, which pin is used on this board */
	unsigned char            debug;
	bool                     has_miscdev;
	bool                     has_attr_delay;
	bool                     has_attr_numsam;
	bool                     has_attr_debug;
	unsigned int             chan_cfg;
	int                      interval;      /* delay count for delay channel */
	int                      new_interval;  /*  */
	struct device            *dev;
	void __iomem             *base;
	struct class             *lradc_class;
	struct clk               *lradc_clk;
	int                      irq[IMX28_LRADC_IRQ_COUNT];
	struct imx28_lradc_phychan phychan[IMX28_LRADC_PIN_NUM];          /* physical channel, 0 ~ 7, each for a lradc pin */
};


#endif
