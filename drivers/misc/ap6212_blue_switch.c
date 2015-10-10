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

struct blue_gpio {
	int gpio;
	enum of_gpio_flags flags;
};

struct blue_info {
	struct blue_gpio gpio_pwrkey;
	struct class    *blue_class;
	bool             is_power_on;
	bool             has_miscdev;
	bool             has_attr_pwr;
};

static struct blue_info *g_devinfo;

static int  blue_open   (struct inode *inode, struct file *file) { return 0; }
static int  blue_release(struct inode *inode, struct file *file) { return 0; }
static long blue_ioctl  (struct file *file, unsigned int cmd, unsigned long arg) { return 0; }

static struct file_operations blue_fops = {
	.owner = THIS_MODULE,
	.open = blue_open,
	.release = blue_release,
	.unlocked_ioctl = blue_ioctl
};
static struct miscdevice blue_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ap6212_bluetooth",
	.fops = &blue_fops
};

/* request a gpio specified by dt, initial disabled */
static int blue_get_gpio(struct platform_device *pdev, struct blue_gpio *pgpio, char *gpioname)
{
	struct device_node *np = pdev->dev.of_node;
	int ret;

	pgpio->gpio  = of_get_named_gpio_flags(np, gpioname, 0, &(pgpio->flags));
	if (pgpio->gpio < 0) {
		printk("%s cannot get %s!\n", __FUNCTION__, gpioname);
		ret = -EFAULT;
	}
	ret = 0;

	return ret;
}

static void blue_enable_gpio(struct blue_gpio *pgpio, bool enable)
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

	gpio_request(pgpio->gpio, "ap6212-bluetooth");
	gpio_direction_output(pgpio->gpio, value);
	gpio_free(pgpio->gpio);
}
static void blue_pwron(bool on)
{
	if (g_devinfo->is_power_on != on) {
		if (on) {
			blue_enable_gpio(&(g_devinfo->gpio_pwrkey), false);
			mdelay(1);
			printk("ap6216-blue power on\n");
		} else
			printk("ap6216-blue power off\n");
		blue_enable_gpio(&(g_devinfo->gpio_pwrkey), on);
		g_devinfo->is_power_on = on;
	}
}

static ssize_t pwr_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", (g_devinfo->is_power_on) ? 1 : 0);
}

static ssize_t pwr_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 16);
	bool changed = false;

	if (new_val != g_devinfo->is_power_on) {
		if ((0 == new_val) || (1 == new_val)) {
			changed = true;
		} else {
			printk("%s, invalid parameter %d\n", __FUNCTION__, new_val);
		}
	}

	if (changed) {
		blue_pwron(new_val);
	}

	return _count; 
}

static CLASS_ATTR(pwr, 0664, pwr_read, pwr_write);
static void blue_destroy_buf(struct platform_device *pdev, struct blue_info *devinfo)
{
	if (devinfo) {
		if (devinfo->has_miscdev)
			misc_deregister(&blue_misc);
		
		if (!IS_ERR(devinfo->blue_class)) {
			if (devinfo->has_attr_pwr) {
				class_remove_file(devinfo->blue_class, &class_attr_pwr);
				devinfo->has_attr_pwr = false;
			}
			class_destroy(devinfo->blue_class);
		}

		kfree(devinfo);
	}
}

static int blue_probe(struct platform_device *pdev)
{
	int ret ;
	struct blue_info *devinfo;

	devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
	if (!devinfo) {
		printk("%s alloc mem failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

	/* request pwrkey-gpio, initial low */
	if (0 != (ret = blue_get_gpio(pdev, &(devinfo->gpio_pwrkey ), "blue_pwron_gpio" ))) goto end;

	devinfo->blue_class = class_create(THIS_MODULE, "ap6212_bluetooth");
	if (IS_ERR(devinfo->blue_class)) {
		printk("%s: create class failed: ap6212_bluetooth\n", __FUNCTION__);
		ret = (NULL == devinfo->blue_class) ? (-ENOMEM) : PTR_ERR(devinfo->blue_class);
		goto end;
	}

	if (0 == (ret =  class_create_file(devinfo->blue_class, &class_attr_pwr))) {
		devinfo->has_attr_pwr = true;
	} else {
		printk("Fail to class ap6212_bluetooth file pwr!\n");
		goto end;
	}

	/* don't care wether we succeeded or not */
	if (0 == misc_register(&blue_misc))
		devinfo->has_miscdev = true;

	platform_set_drvdata(pdev, devinfo);
	g_devinfo = devinfo;
	//blue_pwron(true);

end:
	if (ret) {
		blue_destroy_buf(pdev, devinfo);
	}

	return ret;
}

static int blue_suspend(struct platform_device *pdev, pm_message_t state) {	return 0; }
static int blue_resume(struct platform_device *pdev) { return 0; }

static void blue_shutdown(struct platform_device *pdev)
{
	struct blue_info *devinfo;

	blue_pwron(false);
	devinfo = platform_get_drvdata(pdev);
	blue_destroy_buf(pdev, devinfo);
	platform_set_drvdata(pdev, NULL);
}

static const struct of_device_id blue_dt_ids[] = {
	{ .compatible = "ampak,ap6212-blue", },
	{ }
};

static struct platform_driver blue_driver = {
	.probe		= blue_probe,
	.shutdown	= blue_shutdown,
	.suspend	= blue_suspend,
	.resume		= blue_resume,
	.driver	= {
		.name	= "AP6212-blue",
		.owner	= THIS_MODULE,
		.of_match_table = blue_dt_ids,
	},
};

static int __init blue_init(void)
{
	return platform_driver_register(&blue_driver);
}

static void __exit blue_exit(void)
{
	platform_driver_unregister(&blue_driver);
}

//late_initcall(blue_init);
module_init(blue_init);
module_exit(blue_exit);
