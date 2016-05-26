#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include "imx28_lradc.h"

extern int stmp_reset_block(void __iomem *);

extern void __iomem *mxs_lradc_base;
struct imx28_lradc_info *g_lradc_info;
static const char * const imx28_lradc_irq_names[IMX28_LRADC_IRQ_COUNT] = {
	"mxs-lradc-touchscreen",
	"mxs-lradc-thresh0",
	"mxs-lradc-thresh1",
	"mxs-lradc-channel0",
	"mxs-lradc-channel1",
	"mxs-lradc-channel2",
	"mxs-lradc-channel3",
	"mxs-lradc-channel4",
	"mxs-lradc-channel5",
	"mxs-lradc-channel6",
	"mxs-lradc-channel7",
	"mxs-lradc-button0",
	"mxs-lradc-button1",
};

static int lradc_open   (struct inode *inode, struct file *file) { return 0; }
static int lradc_release(struct inode *inode, struct file *file) { return 0; }
static long lradc_ioctl(struct file *file, unsigned int cmd, unsigned long arg) { return 0; }
static struct file_operations lradc_fops = {
	.owner = THIS_MODULE,
	.open = lradc_open,
	.release = lradc_release,
	.unlocked_ioctl = lradc_ioctl
};
static struct miscdevice lradc_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "imx28-lradc",
	.fops = &lradc_fops
};

static ssize_t interval_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", (g_lradc_info->interval >> 1));
}

static ssize_t interval_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_ms = simple_strtoul(_buf, NULL, 10);
	int max_ms = MAXVAL_LRADC_DELAY_DELAY << 1;

	if ((new_ms >= 0) && (new_ms <= max_ms)) {
		printk("lradc: set interval to %d ms!\n", new_ms);
		g_lradc_info->new_interval = new_ms << 1;
		return _count; 
	} else {
		printk("lradc: invalid interval %d ms. Must be between 0 ~ %d!\n", new_ms, max_ms);
		return -EINVAL;
	}
}
static ssize_t num_samples_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", g_lradc_info->numsample);
}

static ssize_t num_samples_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 10);

	if ((new_val >= 0) && (new_val <= MAX_NUM_SAMPLES)) {
		printk("lradc: set number of samples to %d!\n", new_val);
		g_lradc_info->new_numsample = new_val;
		return _count; 
	} else {
		printk("lradc: invalid interval %d ms. Must be between 0 ~ %d!\n", new_val, MAX_NUM_SAMPLES);
		return 0;
	}
}
static ssize_t debug_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", g_lradc_info->debug);
}

static ssize_t debug_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 10);

	if ((new_val == 0) || (new_val == 1)) {
		printk("lradc: set debug to %d!\n", new_val);
		g_lradc_info->debug = new_val;
		return _count; 
	} else {
		printk("lradc: invalid value %d. Must be 1 or 0!\n", new_val);
		return -EINVAL;
	}
}

static CLASS_ATTR(interval, 0666, interval_read, interval_write);
static CLASS_ATTR(num_samples, 0666, num_samples_read, num_samples_write);
static CLASS_ATTR(debug, 0666, debug_read, debug_write);

static ssize_t lradc_voltage_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imx28_lradc_phychan *p_phychan = dev_get_drvdata(dev);

	return (sprintf(buf, "%d\n", p_phychan->volt));
}
static ssize_t lradc_divby_2_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imx28_lradc_phychan *p_phychan = dev_get_drvdata(dev);

	return(sprintf(buf, "%d\n", p_phychan->div_by2));
}
static ssize_t lradc_divby_2_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	struct imx28_lradc_phychan *p_phychan = dev_get_drvdata(dev);
	ssize_t				status;
	unsigned long		value;

	value = simple_strtoul(buf, NULL, 10);
	if ((0 != value) && (1 != value)) {
		printk("lradc: error value %d. Must be 1 or 0\n", (int)value);
		status = -EINVAL;
	} else {
		status = size;
		p_phychan->new_div_by2 = value;
	}

	return status;
}

static const DEVICE_ATTR(voltage, 0666, lradc_voltage_show, NULL);
static const DEVICE_ATTR(divby_2, 0666, lradc_divby_2_show, lradc_divby_2_store);

static const struct attribute *lradc_attrs[] = {
	&dev_attr_voltage.attr,
	&dev_attr_divby_2.attr,
	NULL,
};
static const struct attribute_group lradc_attr_group = {
	.attrs = (struct attribute **) lradc_attrs,
};

static int init_user_intf(struct imx28_lradc_info *lradc)
{
	int ret = -1;
	int i;
	int stat;

	if (0 == misc_register(&lradc_misc)) {
		lradc->has_miscdev = true;
	} else {
		printk("lradc: misc_register err, ignore...\n");
		goto out;
	}

	if (IS_ERR(lradc->lradc_class = class_create(THIS_MODULE, "imx28-lradc"))) {
		lradc->lradc_class = 0;
		printk("lradc: create class err, ignore...\n");
		goto out;
	}

	if (0 == (class_create_file(lradc->lradc_class, &class_attr_interval))) {
		lradc->has_attr_delay = true;
	} else {
		printk("lradc: create class attrib interval err, ignore...\n");
	}

	if (0 == (class_create_file(lradc->lradc_class, &class_attr_num_samples))) {
		lradc->has_attr_numsam = true;
	} else {
		printk("lradc: create class attrib num_samples err, ignore...\n");
	}

	if (0 == (class_create_file(lradc->lradc_class, &class_attr_debug))) {
		lradc->has_attr_debug = true;
	} else {
		printk("lradc: create class attrib debug err, ignore...\n");
	}

	for (i = 0; i < IMX28_LRADC_PIN_NUM; i++) {
		if (VALID_CHANN(lradc->phychan[i].virchan_idx)) {
			lradc->phychan[i].dev = device_create(lradc->lradc_class, lradc->dev, MKDEV(0, 0),
						&(lradc->phychan[i]), "lradc%u", i);
			if (IS_ERR(lradc->phychan[i].dev)) {
				printk("lradc: failed to create lradc class dev %d, ignore...\n", i);
				lradc->phychan[i].dev = 0;
				continue;
			}

			stat = sysfs_create_group(&(lradc->phychan[i].dev->kobj), &lradc_attr_group);
			if (stat) {
				printk("lradc: failed to create lradc class dev %d attr group, ignore...\n", i);
				device_unregister(lradc->phychan[i].dev);
				put_device(lradc->phychan[i].dev);
				lradc->phychan[i].dev = 0;
			}
		}
	}

	ret = 0;
out:
	return ret;
}

static void deinit_user_intf(struct imx28_lradc_info *lradc)
{
	int i;

	if (lradc) {
		if (lradc->lradc_class) {
			for (i = 0; i < IMX28_LRADC_PIN_NUM; i++) {
				if (lradc->phychan[i].dev) {
					device_unregister(lradc->phychan[i].dev);
					put_device(lradc->phychan[i].dev);
					lradc->phychan[i].dev = 0;
				}
			}

			if (lradc->has_attr_debug) {
				class_remove_file(lradc->lradc_class, &class_attr_debug);
				lradc->has_attr_debug = 0;
			}

			if (lradc->has_attr_numsam) {
				class_remove_file(lradc->lradc_class, &class_attr_num_samples);
				lradc->has_attr_numsam = 0;
			}

			if (lradc->has_attr_delay) {
				class_remove_file(lradc->lradc_class, &class_attr_interval);
				lradc->has_attr_delay = 0;
			}

			class_destroy(lradc->lradc_class);
			lradc->lradc_class = 0;
		}

		if (lradc->has_miscdev) {
			misc_deregister(&lradc_misc);
			lradc->has_miscdev = 0;
		}
	}
}

/* alloc lradc drv data buf, init it */
static struct imx28_lradc_info *alloc_lradc_buf(void)
{
	struct imx28_lradc_info *lradc = 0;
	int i;

	if (0 != (lradc = kzalloc(sizeof(struct imx28_lradc_info), GFP_KERNEL))) {
		for (i = 0; i < IMX28_LRADC_IRQ_COUNT; i++)
			lradc->irq[i] = -1;

		for (i = 0; i < IMX28_LRADC_PIN_NUM; i++) {
			lradc->phychan[i].virchan_idx = INVALID_CHAN_INDEX;
			lradc->phychan[i].new_div_by2 = IMX28_DEF_DIVBY2_BIT;
			lradc->phychan[i].div_by2     = !IMX28_DEF_DIVBY2_BIT;
		}
		lradc->max_virt = INVALID_CHAN_INDEX;

		for (i = 0; i < IMX28_LRADC_VIRCHAN_NUM; i++)
			lradc->bind_table[i] = INVALID_CHAN_INDEX;

		lradc->new_interval = (IMX28_DEF_INTV << 1);
		lradc->interval     = IMX28_DEF_INTV;

		lradc->new_numsample = IMX28_DEF_LRADC_NUM_SAMPLES;
		lradc->numsample     = IMX28_DEF_LRADC_NUM_SAMPLES ? (IMX28_DEF_LRADC_NUM_SAMPLES - 1) : 1;

		/* battery driver used virtual channel 0 and 7, reserve them */
		lradc->usedvirt = (1 << 0) | (1 << 7);
	}

	return lradc;
}

static inline void unkick_delaychan(void)
{
	CLR_REG_BITS(0xFFFFFFFF,REG_LRADC_DELAY(IMX28_LRADC_COM_DELAY_CHAN));
}

static void destroy_lradc_buf(struct imx28_lradc_info *lradc)
{
	int i;

	unkick_delaychan();

	if (lradc) {
		for (i = 0; i < IMX28_LRADC_IRQ_COUNT; i++) {
			if (lradc->irq[i] > 0) {
				devm_free_irq(lradc->dev, lradc->irq[i], lradc);
				lradc->irq[i] = -1;
			}
		}

		deinit_user_intf(lradc);
	
		if (VALID_CHANN(lradc->max_virt))
			LRADC_DISABLE_IRQ_CHAN(lradc->max_virt);
	
		kfree(lradc);
		g_lradc_info = 0;
	}
}

static inline void reconfig_delaychan(struct imx28_lradc_info *lradc)
{
	int i;
	struct imx28_lradc_phychan *p_phychan;
	unsigned int cfg_bits;
	unsigned char divby2_bits = 0;
	int intv_changed = 0;
	int nums_changed = 0;
	int div2_changed = 0;

	if (lradc->numsample != lradc->new_numsample) {
		lradc->numsample = lradc->new_numsample;
		nums_changed = 1;
	}

	if (lradc->interval != lradc->new_interval) {
		lradc->interval = lradc->new_interval;
		intv_changed = 1;
	}

	for (i = 0; i < IMX28_LRADC_PIN_NUM; i++) {
		p_phychan = &(lradc->phychan[i]);

		if (VALID_CHANN(p_phychan->virchan_idx)) {
			if (p_phychan->div_by2 != p_phychan->new_div_by2) {
				p_phychan->div_by2 = p_phychan->new_div_by2;
				div2_changed = 1;
			}
			divby2_bits |= p_phychan->div_by2 ? (1 << (p_phychan->virchan_idx)) : 0;
		}
	}

	if (nums_changed || intv_changed || div2_changed) {
		/* unkick delay channel */
		CLR_REG_BITS(0xFFFFFFFF,REG_LRADC_DELAY(IMX28_LRADC_COM_DELAY_CHAN));

		if (div2_changed) {
			/* change divby_2 for each virtual channel */
			CLR_REG_BITS((lradc->groupbits << 24),REG_LRADC_CTRL2); /* clr all pin's divby_2 bits */
			SET_REG_BITS(divby2_bits << 24,REG_LRADC_CTRL2);        /* set configured divby_2 bits */
		}

		if (nums_changed) {
			/* change num_samples for each virtual channel */
			lradc->chan_cfg = (lradc->numsample ? BIT_LRADC_CHN_ACCUMU : 0) | (lradc->numsample << 24);
			for (i = 0; i < IMX28_LRADC_VIRCHAN_NUM; i++) {
				if (BIT_I(i) & lradc->groupbits)
					WRT_REG_VAL(lradc->chan_cfg,REG_LRADC_CHAN(i));
			}
		}

		cfg_bits =   ((1 << IMX28_LRADC_COM_DELAY_CHAN) << OFFSET_LRADC_DELAY_TRIGGER_DELAYS) /* trigger delay channel 0 */
		           | ((lradc->interval) << OFFSET_LRADC_DELAY_DELAY)                          /* delay count */
		           | ((lradc->groupbits) << OFFSET_LRADC_DELAY_TRIIGER_LRADCS);               /* trigger lradc channel */
		SET_REG_BITS(cfg_bits,REG_LRADC_DELAY(IMX28_LRADC_COM_DELAY_CHAN));

		/* kick delay channel */
		LRADC_KICK_DELAY_CHAN(IMX28_LRADC_COM_DELAY_CHAN);
	}
}

static irqreturn_t imx28_lradc_handle_irq(int irq, void *data)
{
	struct imx28_lradc_info *lradc = data;

	unsigned int intstat;
	int i;
	unsigned int voltage;
	unsigned int toggle;
	unsigned char pin_idx;

	intstat = readl(REG_LRADC_CTRL1);

	for (i = 0; i < 8; i++) {
		if ((BIT_I(i) & intstat) & (lradc->groupbits)) {
			pin_idx = lradc->bind_table[i];
			voltage = readl(REG_LRADC_CHAN(i));

			toggle = voltage & 0x80000000;
			WRT_REG_VAL((toggle|lradc->chan_cfg),REG_LRADC_CHAN(i));

			voltage = ((voltage & MASK_LRADC_CHN_VALUE) * 370) / 819;
			if (lradc->phychan[pin_idx].div_by2)
				voltage <<= 1;
			if (lradc->numsample)
				voltage /= lradc->numsample + 1;

			lradc->phychan[pin_idx].volt = voltage;
			if (lradc->debug)
				printk("lradc: lradc%d voltage=%d mv\n", pin_idx, voltage);
		}
	}

	CLR_REG_BITS((0x1FFF & (lradc->groupbits)),REG_LRADC_CTRL1);
	reconfig_delaychan(lradc);

	return IRQ_HANDLED;
}

static unsigned char imx28_lradc_bind_chann(struct imx28_lradc_info *lradc, unsigned char pin_idx)
{
	int vir_idx;
	struct imx28_lradc_phychan *p_phychan;
	unsigned char ret = 0xFF;

	if (0xFF == lradc->usedvirt) /* no free virtual channel to alloc */
		goto out;

	p_phychan = &((lradc->phychan)[pin_idx]);
	for (vir_idx = 0; (vir_idx < IMX28_LRADC_VIRCHAN_NUM) && (!VALID_CHANN(p_phychan->virchan_idx)); vir_idx++) {
		if (0 == (BIT_I(vir_idx) & (lradc->usedvirt))) {
			/* set physical channel's virtual index */
			p_phychan->virchan_idx = vir_idx;
			/* alloc virtual channel 'i' */
			lradc->usedvirt  |= BIT_I(vir_idx);
			lradc->groupbits |= BIT_I(vir_idx);
			lradc->bind_table[vir_idx] = pin_idx;

			if (!VALID_CHANN(lradc->max_virt)) {
				lradc->max_virt = vir_idx;
			} else if (vir_idx > lradc->max_virt) {
				lradc->max_virt = vir_idx;
			} /* else, do nothing */

			/* hw bind virtual channel 'i' to physical channel 'phychan_idx' */
			LRADC_BIND_CHAN(vir_idx,pin_idx);
			
			ret = vir_idx;
		}
	}

out:
	if (VALID_CHANN(ret)) {
		printk("lradc: bind lradc pin %d to virtual channel %d\n", pin_idx, ret);
	} else {
		printk("lradc: bind lradc pin %d failed\n", pin_idx);
	}

	return ret;
}

static int imx28_lradc_reset_hw(struct imx28_lradc_info *lradc)
{
	int ret = 0;
	int i;
	
	ret = stmp_reset_block(lradc->base);
	if (0 != (ret = stmp_reset_block(lradc->base))) {
		printk("lradc: failed to reset block!\n");
		ret = -EBUSY;
		goto out;
	}

	if (IS_ERR(lradc->lradc_clk =  clk_get(lradc->dev, NULL))) {
		printk("%s: failed to get lradc clock!\n", __FUNCTION__);
		ret = -EPERM;
		goto out;
	}

	if (clk_prepare_enable(lradc->lradc_clk)) {
		printk("%s: failed to enable lradc clock!\n", __FUNCTION__);
		clk_put(lradc->lradc_clk);
		ret = -EPERM;
		goto out;
	}

	CLR_REG_BITS(0x07FF0000,REG_LRADC_CTRL0); /* init ctrl_0, disable button-detect, ts-detect */
	SET_REG_BITS(0x04000000,REG_LRADC_CTRL0); /* turn on on-chip ground ref, or not */

	CLR_REG_BITS(0x1FFF0000,REG_LRADC_CTRL1); /* init ctrl_1, disable all irq */
	CLR_REG_BITS(0x00001FFF,REG_LRADC_CTRL1); /* init ctrl_1, clr all irq stat */

	CLR_REG_BITS(0xFF0073FF,REG_LRADC_CTRL2); /* init ctrl_2, disable divby2/tmpsense */
	SET_REG_BITS(0x00008000,REG_LRADC_CTRL2); /* init ctrl_2, power down tmpsense */

	CLR_REG_BITS(0x00000300,REG_LRADC_CTRL3); /* init ctrl_3, set lradc clk freq as 6M Hz */

	/* for each channel, clear accumulative and num_samples */
	for (i = 0; i < 8; i++) {
		WRT_REG_VAL(0x00000000,REG_LRADC_CHAN(i));
	}

	printk("lradc: ctrl_0 = 0x%08x, ctrl_1 = 0x%08x, ctrl_2 = 0x%08x\n",
		readl(REG_LRADC_CTRL0), readl(REG_LRADC_CTRL1), readl(REG_LRADC_CTRL2));

out:
	return ret;
}

static int init_lradc(struct imx28_lradc_info *lradc)
{
	struct device_node *node = lradc->dev->of_node;
	uint32_t val;
	unsigned char pin_idx;
	int ret = 0;

	/* init hw registers */
	if ((ret = imx28_lradc_reset_hw(lradc)))
		goto out;

	/* get wired pin info from dtb */
	if (of_property_read_u32(node, "hwpins", &val)) {
		printk("%s read hwpins failed!\n", __FUNCTION__);
	} else {
		lradc->hwpins = (unsigned char)val;
	}

	/* bind hw pin to virtual channel */
	for (pin_idx = 0; pin_idx < IMX28_LRADC_PIN_NUM; pin_idx++) {      /* for each physical pin */
		if (lradc->hwpins & BIT_I(pin_idx))                            /* if wired */
			imx28_lradc_bind_chann(lradc, pin_idx);
	}

	/* enable only one irq of virtual channels, with the largest virtual index */
	if (VALID_CHANN(lradc->max_virt))
		LRADC_ENABLE_IRQ_CHAN(lradc->max_virt);

	printk("lradc: ctrl_4=0x%08x, used_virt=0x%08x, groupbits=0x%08x, max_virt=%d\n", readl(REG_LRADC_CTRL4), lradc->usedvirt, lradc->groupbits, lradc->max_virt);

out:
	return ret;
}


static const struct of_device_id imx28_lradc_dt_ids[] = {
	{ .compatible = "fsl,imx28-lradc", },
	{ }
};

static int imx28_lradc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx28_lradc_info *lradc;
	int ret = 0;
	int i;
	int irq;

	platform_set_drvdata(pdev,0);

	/* Grab the memory area */
	mxs_lradc_base = devm_ioremap_resource(dev, platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(mxs_lradc_base)) {
		ret = PTR_ERR(mxs_lradc_base);
		mxs_lradc_base = 0;
		goto out;
	}

	/* alloc lradc buf and init it */
	if (!(lradc = alloc_lradc_buf())) {
		ret = -ENOMEM;
		goto out;
	}

	lradc->base = mxs_lradc_base;
	lradc->dev = dev;
	platform_set_drvdata(pdev, lradc);

	/* Configure the hardware and data. */
	if ((ret = init_lradc(lradc)))
		goto out;

	init_user_intf(lradc);

	/* register all lradc irqs */
	for (i = 0; i < IMX28_LRADC_IRQ_COUNT; i++) {
		if ((irq = platform_get_irq(pdev, i)) < 0) {
			ret = -EINVAL;
			goto out;
		}
		if ((ret = devm_request_irq(dev, irq, imx28_lradc_handle_irq, 0, imx28_lradc_irq_names[i], lradc))) {
			goto out;
		}
		lradc->irq[i] = irq;
	}

	reconfig_delaychan(lradc);

	g_lradc_info = lradc;
	printk("lradc: %s OK!\n", __FUNCTION__);

out:
	if (ret) {
		destroy_lradc_buf(lradc);
	}

	return ret;
}

static int imx28_lradc_remove(struct platform_device *pdev)
{
	struct imx28_lradc_info *lradc = platform_get_drvdata(pdev);

	destroy_lradc_buf(lradc);
	platform_set_drvdata(pdev,NULL);

	return 0;
}



static __refdata struct platform_driver imx28_lradc_drv = {
	.probe = imx28_lradc_probe,
	.remove = imx28_lradc_remove,
	.driver = {
		.name = "imx28-lradc",
		.owner = THIS_MODULE,
		.of_match_table = imx28_lradc_dt_ids,
	}
};

static int __init imx28_lradc_init(void)
{
	printk("%s...\n", __FUNCTION__);
	return platform_driver_register(&imx28_lradc_drv);
}

static void __exit imx28_lradc_exit(void)
{
	platform_driver_unregister(&imx28_lradc_drv);
}

subsys_initcall(imx28_lradc_init);
module_exit(imx28_lradc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luheng Wang <wangluheng@intelgine.com>");
MODULE_DESCRIPTION("Stand-alone driver for mxs lradc module.");

