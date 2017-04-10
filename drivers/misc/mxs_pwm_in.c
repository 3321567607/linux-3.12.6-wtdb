#include <linux/module.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/stmp_device.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/slab.h>


MODULE_LICENSE("GPL");

#define HW_TIMROT_TIMCTRLn(n)		(0x20 + (n) * 0x40)
#define HW_TIMROT_RUNNING_COUNTn(n)	(0x30 + (n) * 0x40)
#define BV_TIMROTv2_TIMCTRLn_SELECT__PWM2           0x3
#define BV_TIMROTv2_TIMCTRLn_SELECT__PWM3           0x4
#define BV_TIMROTv2_TIMCTRLn_SELECT__PWM4           0x5
#define BM_TIMROT_TIMCTRLn_PRESCALE_DIV2 (1 << 4)
#define BM_TIMROT_TIMCTRLn_PRESCALE_DIV4 (2 << 4)
#define BM_TIMROT_TIMCTRLn_PRESCALE_DIV8 (3 << 4)
#define BM_TIMROT_TIMCTRLn_MATCHMODE (1 << 11)

/* unit: usec */
#define PWMIN_MAX_PERIOD 2000
#define PWMIN_MIN_PERIOD 2

struct pwmin_info {
	bool            has_attr_standby;
	bool            has_attr_reset;
	bool            has_miscdev;
    int             pwm_in_gpio;
    int             pwm_in_gpioval;
	struct class   *pwmin_class;
    void __iomem   *timrot_base;
};

extern u64 local_clock(void);

static struct pwmin_info *g_pwmin_devinfo;

static int pwmin_open   (struct inode *inode, struct file *file) { return 0; }
static int pwmin_release(struct inode *inode, struct file *file) { return 0; }
static long pwmin_ioctl(struct file *file, unsigned int cmd, unsigned long arg) { return 0; }
static struct file_operations pwmin_fops = {
	.owner = THIS_MODULE,
	.open = pwmin_open,
	.release = pwmin_release,
	.unlocked_ioctl = pwmin_ioctl
};
static struct miscdevice pwmin_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pwmin",
	.fops = &pwmin_fops
};


static ssize_t standby_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", 1);
}

static ssize_t standby_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 16);

	printk("%s, invalid parameter %d\n", __FUNCTION__, new_val);

	return _count; 
}

static ssize_t reset_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "0\n");
}

static ssize_t reset_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int new_val = simple_strtoul(_buf, NULL, 16);

	if (new_val == 1) {
		printk("%s, invalid parameter %d\n", __FUNCTION__, new_val);
	}
	return _count; 
}


static CLASS_ATTR(standby, 0666, standby_read, standby_write);
static CLASS_ATTR(reset, 0666, reset_read, reset_write);

static void pwmin_destroy_buf(struct platform_device *pdev, struct pwmin_info *devinfo)
{
	if (devinfo) {
		if (devinfo->has_miscdev)
			misc_deregister(&pwmin_misc);
		
		if (!IS_ERR(devinfo->pwmin_class)) {
			if (devinfo->has_attr_standby) {
				class_remove_file(devinfo->pwmin_class, &class_attr_standby);
			}
			if (devinfo->has_attr_reset) {
				class_remove_file(devinfo->pwmin_class, &class_attr_reset);
			}
			class_destroy(devinfo->pwmin_class);
		}

		kfree(devinfo);
	}
}

static inline u64 get_cur_usec(void)
{
    u64 ret;

    ret = local_clock();
    do_div(ret, 1000);

    return ret;
}

#if 1
static int mxs_pwm_thread_fn(void *arg)
{
    u64 prev_begin_usec;
    u64 begin_usec;
    u64 end_usec;
    unsigned long sleepusec;

    g_pwmin_devinfo->pwm_in_gpioval = gpio_get_value_cansleep(g_pwmin_devinfo->pwm_in_gpio);
    while (1) {
        begin_usec = get_cur_usec();
        



        end_usec = get_cur_usec();
        if (end_usec > begin_usec + PWMIN_MAX_PERIOD - PWMIN_MIN_PERIOD) {
            sleepusec = PWMIN_MIN_PERIOD;
        } else {
            sleepusec = (unsigned long)(begin_usec + PWMIN_MAX_PERIOD - end_usec);
        }
        prev_begin_usec = begin_usec;

        usleep_range(sleepusec - 1, sleepusec);
    }

    return 0;
}
#else
static int mxs_pwm_thread_fn(void *arg)
{
    u64 prev_begin_usec;
    u64 begin_usec;
    u64 end_usec;
    unsigned long sleepusec;
    int gpioval = 0;
    
    prev_begin_usec = get_cur_usec();
    g_pwmin_devinfo->pwm_in_gpioval = gpio_get_value_cansleep(g_pwmin_devinfo->pwm_in_gpio);

    while (1) {
        begin_usec = get_cur_usec();
        gpioval = gpio_get_value_cansleep(g_pwmin_devinfo->pwm_in_gpio);

        if (gpioval != g_pwmin_devinfo->pwm_in_gpioval) {
            printk("wlh: %lu\n", (unsigned long)(begin_usec - prev_begin_usec));
            prev_begin_usec = begin_usec;
            g_pwmin_devinfo->pwm_in_gpioval = gpioval;
        }
        


        usleep_range(15, 20);
    }

    return 0;
}
#endif

struct task_struct *pwm_in_thread;

static int pwmin_probe(struct platform_device *pdev)
{
	int ret;
	struct pwmin_info *devinfo;
    struct device_node *np;
    enum of_gpio_flags flags;

	printk("wangluheng: %s\n", __FUNCTION__);

	devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
	if (!devinfo) {
		printk("%s alloc mem failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

    np = of_find_compatible_node(NULL, NULL, "fsl,imx28-timrot");
    devinfo->timrot_base = of_iomap(np, 0);
    devinfo->pwm_in_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "pwm_in_gpio", 0, &flags);
    if (devinfo->pwm_in_gpio < 0) {
        printk("wangluheng: cannot get pwm-in gpio\n");
    } else {
        if (devm_gpio_request_one(&(pdev->dev), devinfo->pwm_in_gpio, GPIOF_IN, "pwm_in_gpio")) {
            printk("wangluheng: cannot request gpio %d\n", devinfo->pwm_in_gpio);
        }
    }
    

	devinfo->pwmin_class = class_create(THIS_MODULE, "pwmin");
	if (IS_ERR(devinfo->pwmin_class)) {
		ret = (NULL == devinfo->pwmin_class) ? (-ENOMEM) : PTR_ERR(devinfo->pwmin_class);
		goto end;
	}

	;
	if (0 == (ret = class_create_file(devinfo->pwmin_class, &class_attr_standby))) {
		devinfo->has_attr_standby = true;
	} else {
		printk("Fail to class gps.\n");
		goto end;
	}

	if (0 == (ret = class_create_file(devinfo->pwmin_class, &class_attr_reset))) {
		devinfo->has_attr_reset = true;
	} else {
		printk("Fail to class gps.\n");
		goto end;
	}

	if (0 == misc_register(&pwmin_misc)) {
		devinfo->has_miscdev = true;
	} else {
		printk("%s: misc_register err\n", __FUNCTION__);
	}
	
	platform_set_drvdata(pdev, devinfo);
	g_pwmin_devinfo = devinfo;

	pwm_in_thread = kthread_run(mxs_pwm_thread_fn, NULL, "pwm_in");
	if (WARN_ON(!pwm_in_thread)) {
		pr_cont("FAILED\n");
	}
    
end:
	if (ret) {
		pwmin_destroy_buf(pdev, devinfo);
	}
	return ret;
}

int pwmin_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

int pwmin_resume(struct platform_device *pdev)
{
	return 0;
}

void pwmin_shutdown(struct platform_device *pdev)
{
	struct pwmin_info *devinfo;

	devinfo = platform_get_drvdata(pdev);
	pwmin_destroy_buf(pdev, devinfo);
	platform_set_drvdata(pdev, NULL);}

static const struct of_device_id pwmin_dt_ids[] = {
	{ .compatible = "fsl,pwm-in", },
	{ }
};

static struct platform_driver pwmin_driver = {
	.probe		= pwmin_probe,
	.shutdown	= pwmin_shutdown,
	.suspend	= pwmin_suspend,
	.resume		= pwmin_resume,
	.driver	= {
		.name	= "mxs_pwmin",
		.owner	= THIS_MODULE,
		.of_match_table = pwmin_dt_ids,
	},
};

static int __init pwmin_init(void)
{
	return platform_driver_register(&pwmin_driver);
}

static void __exit pwmin_exit(void)
{
	platform_driver_unregister(&pwmin_driver);
}

module_init(pwmin_init);

module_exit(pwmin_exit);
