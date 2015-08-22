#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/slab.h>


MODULE_LICENSE("GPL");

struct gps_gpio {
	int gpio;
	enum of_gpio_flags flags;
};

struct gps_info {
	struct gps_gpio gpio_standby;
	struct gps_gpio gpio_reset;
	bool            is_standby;
	bool            has_attr_standby;
	bool            has_attr_reset;
	bool            has_miscdev;
	struct class   *gps_class;
};

static struct gps_info *g_devinfo;

static int gps_open   (struct inode *inode, struct file *file) { return 0; }
static int gps_release(struct inode *inode, struct file *file) { return 0; }
static long gps_ioctl(struct file *file, unsigned int cmd, unsigned long arg) { return 0; }
static struct file_operations gps_fops = {
	.owner = THIS_MODULE,
	.open = gps_open,
	.release = gps_release,
	.unlocked_ioctl = gps_ioctl
};
static struct miscdevice gps_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ublox",
	.fops = &gps_fops
};


static void gps_enable_gpio(struct gps_gpio *pgpio, bool enable)
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

static ssize_t standby_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", g_devinfo->is_standby ? 1 : 0);
}

static ssize_t standby_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 16);
	bool changed = false;

	if (new_val != g_devinfo->is_standby) {
		if ((0 == new_val) || (1 == new_val)) {
			changed = true;
		} else {
			printk("%s, invalid parameter %d\n", __FUNCTION__, new_val);
		}
	}

	if (changed) {
		g_devinfo->is_standby = (1 == new_val);
		gps_enable_gpio(&(g_devinfo->gpio_standby), g_devinfo->is_standby);
		printk("%s, %s gps standby\n", __FUNCTION__, new_val ? "enter" : "exit");
	}

	return _count; 
}

static void gps_reset(void)
{
	gps_enable_gpio(&(g_devinfo->gpio_reset), true);
	msleep(10);
	gps_enable_gpio(&(g_devinfo->gpio_reset), false);
	g_devinfo->is_standby = false;
}

static ssize_t reset_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "0\n");
}

static ssize_t reset_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 16);

	if (new_val == 1) {
		gps_reset();
	} else {
		printk("%s, invalid parameter %d\n", __FUNCTION__, new_val);
	}
	return _count; 
}

static int gps_get_gpio(struct platform_device *pdev, struct gps_gpio *pgpio, char *gpioname)
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

static CLASS_ATTR(standby, 0666, standby_read, standby_write);
static CLASS_ATTR(reset, 0666, reset_read, reset_write);

static void gps_destroy_buf(struct platform_device *pdev, struct gps_info *devinfo)
{
	if (devinfo) {
		if (devinfo->has_miscdev)
			misc_deregister(&gps_misc);
		
		if (!IS_ERR(devinfo->gps_class)) {
			if (devinfo->has_attr_standby) {
				class_remove_file(devinfo->gps_class, &class_attr_standby);
			}
			if (devinfo->has_attr_reset) {
				class_remove_file(devinfo->gps_class, &class_attr_reset);
			}
			class_destroy(devinfo->gps_class);
		}

		if (devinfo->gpio_standby.gpio >= 0) {
			devm_gpio_free(&(pdev->dev), devinfo->gpio_standby.gpio);
		}
		if (devinfo->gpio_reset.gpio >= 0) {
			devm_gpio_free(&(pdev->dev), devinfo->gpio_reset.gpio);
		}

		kfree(devinfo);
	}
}
static int gps_probe(struct platform_device *pdev)
{
	int ret;
	struct gps_info *devinfo;

	printk("%s\n", __FUNCTION__);

	devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
	if (!devinfo) {
		printk("%s alloc mem failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

	if (0 != (ret = gps_get_gpio(pdev, &(devinfo->gpio_standby), "gps_standby_gpio" ))) goto end;
	if (0 != (ret = gps_get_gpio(pdev, &(devinfo->gpio_reset  ), "gps_reset_gpio"  ))) goto end;

	devinfo->gps_class = class_create(THIS_MODULE, "gps");
	if (IS_ERR(devinfo->gps_class)) {
		ret = (NULL == devinfo->gps_class) ? (-ENOMEM) : PTR_ERR(devinfo->gps_class);
		goto end;
	}

	;
	if (0 == (ret = class_create_file(devinfo->gps_class, &class_attr_standby))) {
		devinfo->has_attr_standby = true;
	} else {
		printk("Fail to class gps.\n");
		goto end;
	}

	if (0 == (ret = class_create_file(devinfo->gps_class, &class_attr_reset))) {
		devinfo->has_attr_reset = true;
	} else {
		printk("Fail to class gps.\n");
		goto end;
	}

	if (0 == misc_register(&gps_misc)) {
		devinfo->has_miscdev = true;
	} else {
		printk("%s: misc_register err\n", __FUNCTION__);
	}
	
	platform_set_drvdata(pdev, devinfo);
	g_devinfo = devinfo;
	gps_reset();
end:
	if (ret) {
		gps_destroy_buf(pdev, devinfo);
	}
	return ret;
}

int gps_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int gps_resume(struct platform_device *pdev)
{
	return 0;
}

void gps_shutdown(struct platform_device *pdev)
{
	struct gps_info *devinfo;

	devinfo = platform_get_drvdata(pdev);
	gps_destroy_buf(pdev, devinfo);
	platform_set_drvdata(pdev, NULL);}

static const struct of_device_id gps_dt_ids[] = {
	{ .compatible = "quectel,gps-l70", },
	{ }
};

static struct platform_driver gps_driver = {
	.probe		= gps_probe,
	.shutdown	= gps_shutdown,
	.suspend	= gps_suspend,
	.resume		= gps_resume,
	.driver	= {
		.name	= "gps_ublox",
		.owner	= THIS_MODULE,
		.of_match_table = gps_dt_ids,
	},
};

static int __init gps_init(void)
{
	return platform_driver_register(&gps_driver);
}

static void __exit gps_exit(void)
{
	platform_driver_unregister(&gps_driver);
}

module_init(gps_init);

module_exit(gps_exit);
