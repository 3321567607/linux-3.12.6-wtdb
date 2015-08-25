#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/circ_buf.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/slab.h>

MODULE_LICENSE("GPL");

struct ug95_gpio {
	int gpio;
	enum of_gpio_flags flags;
};

struct ug95_info {
	struct ug95_gpio gpio_pwrkey;
	struct ug95_gpio gpio_reset;
	struct ug95_gpio gpio_pwrdown;
	struct ug95_gpio gpio_sim2;
	int              cur_sim;
	bool             is_power_on;
	bool             has_attr_pwron;
	bool             has_attr_sim;
	bool             has_miscdev;
	struct class    *modem_class;
};

static struct ug95_info *g_devinfo;

static int  ug95_open   (struct inode *inode, struct file *file) { return 0; }
static int  ug95_release(struct inode *inode, struct file *file) { return 0; }
static long ug95_ioctl  (struct file *file, unsigned int cmd, unsigned long arg) { return 0; }

static struct file_operations ug95_fops = {
	.owner = THIS_MODULE,
	.open = ug95_open,
	.release = ug95_release,
	.unlocked_ioctl = ug95_ioctl
};
static struct miscdevice ug95_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ug95",
	.fops = &ug95_fops
};

/* request a gpio specified by dt, initial disabled */
static int ug95_get_gpio(struct platform_device *pdev, struct ug95_gpio *pgpio, char *gpioname)
{
	struct device_node *np = pdev->dev.of_node;
	unsigned long init_flags;
	int ret;

	pgpio->gpio  = of_get_named_gpio_flags(np, gpioname, 0, &(pgpio->flags));
	if (pgpio->gpio < 0) {
		printk("%s cannot get %s!\n", __FUNCTION__, gpioname);
		ret = -EFAULT;
	} else {
		/* init as inactive */
		if (OF_GPIO_ACTIVE_LOW == pgpio->flags) {
			init_flags = GPIOF_OUT_INIT_HIGH;
		} else {
			init_flags = GPIOF_OUT_INIT_LOW;
		}

		ret = devm_gpio_request_one(&(pdev->dev), pgpio->gpio, init_flags, gpioname);
		if (ret) {
			pgpio->gpio = -1;
			dev_err(&pdev->dev, "failed to request %s: %d\n", gpioname, ret);
		}
	}

	return ret;
}

static void ug95_enable_gpio(struct ug95_gpio *pgpio, bool enable)
{
	int value;

	if (OF_GPIO_ACTIVE_LOW == pgpio->flags) {
		if (enable)
			value = 0;
		else
			value = 1;
	} else {
		if (enable)
			value = 1;
		else
			value = 0;
	}

	gpio_set_value_cansleep(pgpio->gpio, value);
}

/* pull gpio as active for pulse_ms milisec */
static void ug95_pulse_gpio(struct ug95_gpio *pgpio, unsigned int pulse_ms)
{
	ug95_enable_gpio(pgpio, true);
	msleep(pulse_ms);
	ug95_enable_gpio(pgpio, false);
}

static ssize_t sim_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", g_devinfo->cur_sim);
}

static ssize_t sim_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val;
	bool changed = false;
	bool enable_sim2;

	new_val = simple_strtoul(_buf, NULL, 16);

	if (new_val != g_devinfo->cur_sim) {
		if ((1 == new_val) || (2 == new_val)) {
			changed = true;
		} else {
			printk("%s, invalid parameter %d\n", __FUNCTION__, new_val);
		}
	}

	if (changed) {
		g_devinfo->cur_sim = new_val;
		enable_sim2 = (new_val == 2);
		ug95_enable_gpio(&(g_devinfo->gpio_sim2), enable_sim2);
		if (g_devinfo->is_power_on) {
			ug95_pulse_gpio(&(g_devinfo->gpio_pwrdown), 100);
			msleep(200);
			ug95_pulse_gpio(&(g_devinfo->gpio_pwrkey), 1200);
		}
	}

	return _count;
}

static ssize_t pwron_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", (g_devinfo->is_power_on) ? 1 : 0);
}

static ssize_t pwron_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 16);
	bool changed = false;
	struct ug95_gpio *pgpio;
	int pulsewidth;

	if (new_val != g_devinfo->is_power_on) {
		if ((0 == new_val) || (1 == new_val)) {
			changed = true;
		} else {
			printk("%s, invalid parameter %d\n", __FUNCTION__, new_val);
		}
	}

	if (changed) {
		pgpio = (0 == new_val) ? (&(g_devinfo->gpio_pwrdown)) : (&(g_devinfo->gpio_pwrkey));
		pulsewidth = (0 == new_val) ? 100 : 1200;
		ug95_pulse_gpio(pgpio, pulsewidth);
		g_devinfo->is_power_on = new_val;
		printk("%s, turn %s ug95 via %d %d ms!\n",
			__FUNCTION__, (0 == new_val) ? "off" : "on", pgpio->gpio, pulsewidth);
	}

	return _count; 
}

static CLASS_ATTR(pwron, 0664, pwron_read, pwron_write);
static CLASS_ATTR(sim, 0664, sim_read, sim_write);

static void ug95_destroy_buf(struct platform_device *pdev, struct ug95_info *devinfo)
{
	if (devinfo) {
		if (devinfo->has_miscdev)
			misc_deregister(&ug95_misc);
		
		if (!IS_ERR(devinfo->modem_class)) {
			if (devinfo->has_attr_sim) {
				class_remove_file(devinfo->modem_class, &class_attr_sim);
				devinfo->has_attr_sim = false;
			}
			if (devinfo->has_attr_pwron) {
				class_remove_file(devinfo->modem_class, &class_attr_pwron);
				devinfo->has_attr_pwron = false;
			}
			class_destroy(devinfo->modem_class);
		}

		if (devinfo->gpio_pwrkey.gpio >= 0) {
			devm_gpio_free(&(pdev->dev), devinfo->gpio_pwrkey.gpio);
		}
		if (devinfo->gpio_reset.gpio >= 0) {
			devm_gpio_free(&(pdev->dev), devinfo->gpio_reset.gpio);
		}
		if (devinfo->gpio_pwrdown.gpio >= 0) {
			devm_gpio_free(&(pdev->dev), devinfo->gpio_pwrdown.gpio);
		}
		if (devinfo->gpio_sim2.gpio >= 0) {
			devm_gpio_free(&(pdev->dev), devinfo->gpio_sim2.gpio);
		}

		kfree(devinfo);
	}
}

static int ug95_probe(struct platform_device *pdev)
{
	int ret ;
	struct ug95_info *devinfo;

	devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
	if (!devinfo) {
		printk("%s alloc mem failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

	/* request pwrkey-gpio, initial low */
	if (0 != (ret = ug95_get_gpio(pdev, &(devinfo->gpio_pwrkey ), "gsm_pwrkey_gpio" ))) goto end;
	if (0 != (ret = ug95_get_gpio(pdev, &(devinfo->gpio_pwrdown), "gsm_pwrdown_gpio"))) goto end;
	if (0 != (ret = ug95_get_gpio(pdev, &(devinfo->gpio_reset  ), "gsm_reset_gpio"  ))) goto end;

	/* we can tolerate without switch gpio, so don't return if failed with switch */
	ug95_get_gpio(pdev, &(devinfo->gpio_sim2), "gsm_sim2_gpio");

	devinfo->cur_sim = 1;
	devinfo->is_power_on = false;

	devinfo->modem_class = class_create(THIS_MODULE, "modem");
	if (IS_ERR(devinfo->modem_class)) {
		ret = (NULL == devinfo->modem_class) ? (-ENOMEM) : PTR_ERR(devinfo->modem_class);
		goto end;
	}

	if (0 == (ret =  class_create_file(devinfo->modem_class, &class_attr_pwron))) {
		devinfo->has_attr_pwron = true;
	} else {
		printk("Fail to class modem-ug95!\n");
		goto end;
	}

	if (devinfo->gpio_sim2.gpio >= 0) {
		if (0 == (ret = class_create_file(devinfo->modem_class, &class_attr_sim))) {
			devinfo->has_attr_sim = true;
		} else {
			/* don't care if sim file created successfully or not */
			ret = 0;
		}
	}

	/* don't care wether we succeeded or not */
	if (0 == misc_register(&ug95_misc))
		devinfo->has_miscdev = true;
	else
		devinfo->has_miscdev = false;

	platform_set_drvdata(pdev, devinfo);
	g_devinfo = devinfo;

end:
	if (ret) {
		ug95_destroy_buf(pdev, devinfo);
	}

	return ret;
}

static int ug95_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int ug95_resume(struct platform_device *pdev)
{
	return 0;
}

static void ug95_shutdown(struct platform_device *pdev)
{
	struct ug95_info *devinfo;

	devinfo = platform_get_drvdata(pdev);
	ug95_pulse_gpio(&(devinfo->gpio_pwrdown), 100);
	ug95_destroy_buf(pdev, devinfo);
	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id ug95_dt_ids[] = {
	{ .compatible = "quectel,gsm-ug95-m95", },
	{ }
};

static struct platform_driver ug95_driver = {
	.probe		= ug95_probe,
	.shutdown	= ug95_shutdown,
	.suspend	= ug95_suspend,
	.resume		= ug95_resume,
	.driver	= {
		.name	= "UG95",
		.owner	= THIS_MODULE,
		.of_match_table = ug95_dt_ids,
	},
};

static int __init ug95_init(void)
{
	return platform_driver_register(&ug95_driver);
}

static void __exit ug95_exit(void)
{
	platform_driver_unregister(&ug95_driver);
}

module_init(ug95_init);
module_exit(ug95_exit);
