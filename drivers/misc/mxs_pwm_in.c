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

extern u64 local_clock(void);

#define PWMIN_OF_NAME "pwmin-gpios"

/* period of polling gpios, usec */
#define PWMIN_POLL_INTV 990

#define PWMIN_TOTAL_PINS     11
#define PWMIN_MAX_EVENTS    10

#define PWMIN_THRD_UNINITED 0
#define PWMIN_THRD_STARTED  1
#define PWMIN_THRD_STOPPING 2
#define PWMIN_THRD_STOPPED  3

/* ioctl cmd */
#define PWMIN_IOCTL_GETINPUT 1

typedef struct iot_gio {
    int                gpio;
    enum of_gpio_flags flags;
} T_IOT_GPIO;

typedef struct pwmin_pin_val {
    unsigned int elapsed_us; /* usecs elapsed since last time pins changed */
    unsigned int iobits;
} T_PWMIN_PINVAL;

typedef struct pwmin_input_q {
    T_PWMIN_PINVAL v_list[PWMIN_MAX_EVENTS];
    unsigned char  ridx;
    unsigned char  widx;
} T_PWMIN_INPUT_Q;

#define PWMIN_DIFF_US(now,pre) (((now) > ((pre) + 0xFFFFFFFF)) ? 0xFFFFFFFF : ((now) - (pre)))

#define PWMIN_Q_PREIDX(idx) ((idx) ? ((idx) - 1) : PWMIN_MAX_EVENTS)
#define PWMIN_Q_NEXIDX(idx) (((idx) >= (PWMIN_MAX_EVENTS - 1)) ? 0 : ((idx) + 1))
#define PWMIN_Q_IS_FULL(q) ((PWMIN_Q_PREIDX(q->ridx) == q->widx) ? 1 : 0)
#define PWMIN_Q_IS_EMPTY(q) ((q->ridx == q->widx) ? 1 : 0)
#define PWMIN_Q_CLEAR(q) do { q->ridx = q->widx; } while (0)
#define PWMIN_Q_RSV_1(q) do { q->ridx = PWMIN_Q_PREIDX(q->widx); } while (0)
#define PWMIN_Q_GET_CUR(q) (q->v_list[PWMIN_Q_PREIDX(q->widx)].iobits)
#define PWMIN_Q_LOG_NEW(q,bits,elapsed) do {                                         \
                                            if (PWMIN_Q_IS_FULL(q)) {                \
                                                q->ridx = PWMIN_Q_NEXIDX(q->ridx);   \
                                            }                                        \
                                            q->v_list[q->widx].iobits = bits;        \
                                            q->v_list[q->widx].elapsed_us = elapsed; \
                                            q->widx = PWMIN_Q_NEXIDX(q->widx);       \
                                        } while (0)

#define PWMIN_USLEEP(us) do { usleep_range((us), (us) + 1); } while (0)

struct pwmin_info {
    u64                 ev_usec;         /* stamp that gpios changed previously */
    T_IOT_GPIO          pins[PWMIN_TOTAL_PINS];

    struct mutex        inputs_mutex;
    int                 mutex_inited;
    wait_queue_head_t   wait_q_hd;       /* user process queue, waiting for new input coming */
    T_PWMIN_INPUT_Q     q_in;

    int                 thread_stat;
    struct task_struct *p_thread;

	bool                misc_reged;         /* misc dev registered or not */
};

struct pwmin_info *g_p_devinfo;

static int pwmin_release(struct inode *inode, struct file *file)
{
    printk("%s\n", __FUNCTION__);
    return 0;
}

/*
 * every time the dev is opened, we reserve only the last 1 event to allow APP to read current gpio states.
 */
static int pwmin_open(struct inode *inode, struct file *file)
{
    T_PWMIN_INPUT_Q *p_q     = &(g_p_devinfo->q_in);
    struct mutex    *p_mutex = &(g_p_devinfo->inputs_mutex);

    printk("%s\n", __FUNCTION__);

    mutex_lock(p_mutex);
    PWMIN_Q_RSV_1(p_q);
    mutex_unlock(p_mutex);

    return 0;
}

static long pwmin_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long ret;
    T_PWMIN_INPUT_Q tmpbuf;
    T_PWMIN_INPUT_Q *p_srcq;
    struct mutex    *p_mutex;

    switch (cmd) {
        case PWMIN_IOCTL_GETINPUT:
            if (g_p_devinfo) {
                p_srcq  = &(g_p_devinfo->q_in);
                p_mutex = &(g_p_devinfo->inputs_mutex);

                ret = wait_event_interruptible((g_p_devinfo->wait_q_hd), (!(PWMIN_Q_IS_EMPTY(p_srcq))));
                if (ret) {
                    if (-ERESTARTSYS == ret) {
                        printk("%s interrupted\n", __FUNCTION__);
                    } else {
                        printk("Err: %s, wait_event return %ld\n", __FUNCTION__, ret);
                    }
                } else {
                    /* has user inputs, copy it */
                    mutex_lock(p_mutex);
                    memcpy(&tmpbuf, p_srcq, sizeof(tmpbuf));
                    PWMIN_Q_CLEAR(p_srcq);
                    mutex_unlock(p_mutex); 

                    //printk("has user input\n");
                    
                    ret = copy_to_user((void __user *)arg, &(tmpbuf), sizeof(tmpbuf));
                }
            } else {
                printk("Err: %s, no devinfo\n", __FUNCTION__);
                ret = -ENODEV;
            }
            break;

        default:
            printk("Err: %s, unknown cmd 0x%x\n", __FUNCTION__, cmd);
            ret = -EPERM;
            break;
    }
    return ret;
}

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

static inline u64 pwmin_get_cur_usec(void)
{
    u64 ret;

    ret = local_clock();
    do_div(ret, 1000);

    return ret;
}

static void pwmin_init_gpio(T_IOT_GPIO *p_gpio)
{
    p_gpio->gpio = -1;
}

struct pwmin_info *pwmin_alloc_devinfo(void)
{
    struct pwmin_info *p_info;
    int i;

    if ((p_info = kzalloc(sizeof(struct pwmin_info), GFP_KERNEL))) {
        mutex_init(&(p_info->inputs_mutex));
        p_info->mutex_inited = 1;
        init_waitqueue_head(&(p_info->wait_q_hd));

        for (i = 0; i < PWMIN_TOTAL_PINS; i++) {
            pwmin_init_gpio(&(p_info->pins[i]));
        }

        p_info->thread_stat = PWMIN_THRD_UNINITED;
    } else {
        printk("Err: cannot alloc devinfo for pwmin.\n");
    }

    return p_info;
}

static int pwmin_cfg_gpio(struct device *p_dev, const char *of_name, int idx, T_IOT_GPIO *p_gpio)
{
    struct device_node *p_node;
    int ret = -1;
    char label[50];

    if (!(p_node = p_dev->of_node)) {
        printk("%s: no of_node found\n", __FUNCTION__);
        goto out;
    }

    p_gpio->gpio = of_get_named_gpio_flags(p_node, of_name, idx, &(p_gpio->flags));
    if (p_gpio->gpio < 0) {
        printk("%s: cannot find gpio %s:%d\n", __FUNCTION__, of_name, idx);
        goto out;
    }

    sprintf(label, "%s_%d", of_name, idx);
    if (devm_gpio_request_one(p_dev, p_gpio->gpio, GPIOF_IN, label)) {
        printk("%s: cannot request gpio: %s:%d\n", __FUNCTION__, of_name, idx);
        goto out;
    }

    ret = 0;
out:
    if (ret) {
        p_gpio->gpio = -1;
    }
    return ret;
}

static void pwmin_update_gpios(struct pwmin_info *p_info)
{
    u64 now;
    unsigned int elapsed_us;
    int i;
    T_IOT_GPIO *p_gpio = p_info->pins;
    unsigned int value = 0;
    int wake = 0;
    T_PWMIN_INPUT_Q *p_q = &(p_info->q_in);

    for (i = 0; i < PWMIN_TOTAL_PINS; i++) {
        if (gpio_get_value_cansleep(p_gpio->gpio)) {
            value |= (1 << i);
        }
        p_gpio++;
    }

    mutex_lock(&(p_info->inputs_mutex));
    now = pwmin_get_cur_usec();
    elapsed_us = PWMIN_DIFF_US(now,p_info->ev_usec);
    if ((value != PWMIN_Q_GET_CUR(p_q)) || (elapsed_us >= 1000000)) {            /* needs update */
        p_info->ev_usec = now;
        PWMIN_Q_LOG_NEW(p_q, value, elapsed_us);
        wake = 1;
    }
    mutex_unlock(&(p_info->inputs_mutex));

    if (wake) {
        wake_up_interruptible(&(p_info->wait_q_hd));
    }
}

static int pwmin_cfg_dtb(struct device *p_dev, struct pwmin_info *p_info)
{
    int ret = -1;
    int i;

    for (i = 0; i < PWMIN_TOTAL_PINS; i++) {
        if (pwmin_cfg_gpio(p_dev, PWMIN_OF_NAME, i, &(p_info->pins[i]))) {
            goto out;
        }
    }
    pwmin_update_gpios(p_info);
    /* force to set stamp in case unchanged by above func due to default values unchanged */
    p_info->ev_usec = pwmin_get_cur_usec();

    ret = 0;
out:
    return ret;
}

static void pwmin_decfg_gpio(struct device *p_dev, T_IOT_GPIO *p_gpio)
{
    if (p_gpio->gpio >= 0) {
        devm_gpio_free(p_dev, p_gpio->gpio);
    }
}


static void pwmin_decfg_dtb(struct device *p_dev, struct pwmin_info *p_info)
{
    int i;

    for (i = 0; i < PWMIN_TOTAL_PINS; i++) {
        pwmin_decfg_gpio(p_dev, &(p_info->pins[i]));
    }
}

static void pwmin_destroy_buf(struct platform_device *p_pltdev)
{
    struct pwmin_info *p_info;

    p_info = (struct pwmin_info *)platform_get_drvdata(p_pltdev);

	if (p_info) {
        if (PWMIN_THRD_STARTED == p_info->thread_stat) {
            p_info->thread_stat = PWMIN_THRD_STOPPING;
            while (PWMIN_THRD_STOPPING == p_info->thread_stat) {
                PWMIN_USLEEP(1);
            }
            PWMIN_USLEEP(1);
            printk("%s: thread stopped\n", __FUNCTION__);
        }

		if (p_info->misc_reged) {
			misc_deregister(&pwmin_misc);
        }

        pwmin_decfg_dtb(&(p_pltdev->dev), p_info);

        if (p_info->mutex_inited) {
            mutex_destroy(&(p_info->inputs_mutex));
        }

		kfree(p_info);
        platform_set_drvdata(p_pltdev, NULL);
	}
}

/*
 * regard interval <= 3ms as fastest screw level
 *    and interval >= 1.5s as slowest screw level
 */
static int mxs_pwm_thread_fn(void *arg)
{
    u64 begin_usec;
    u64 end_usec;
    unsigned long sleepusec;
    unsigned long waste;
    struct pwmin_info *p_info;

    p_info = (struct pwmin_info *)arg;

    while (PWMIN_THRD_STOPPING != p_info->thread_stat) {
        begin_usec = pwmin_get_cur_usec();
        pwmin_update_gpios(p_info);
        end_usec = pwmin_get_cur_usec();

        waste = end_usec - begin_usec;
        if (waste < PWMIN_POLL_INTV) {
            sleepusec = PWMIN_POLL_INTV - waste;
        } else {
            sleepusec = 1;
        }
        // sleepusec = PWMIN_POLL_INTV;

        PWMIN_USLEEP(sleepusec);
    }

    printk("%s: thread stopped\n", __FUNCTION__);    
    p_info->thread_stat = PWMIN_THRD_STOPPED;

    return 0;
}


static int pwmin_probe(struct platform_device *p_pltdev)
{
	int ret;
	struct pwmin_info *p_info;
    struct device *p_dev;

    p_dev = &(p_pltdev->dev);
    /* alloc mem for dev-info */
	if (!(p_info = pwmin_alloc_devinfo())) {
		goto out;
	}
	platform_set_drvdata(p_pltdev, p_info);
    /* config gpios from dtb */
    if (pwmin_cfg_dtb(p_dev, p_info)) {
        goto out;
    }
    /* device node */
	if (misc_register(&pwmin_misc)) {
		printk("%s: misc_register err\n", __FUNCTION__);
        goto out;
	}
    p_info->misc_reged = true;
	/* init thread to read gpios */
	if (!(p_info->p_thread = kthread_run(mxs_pwm_thread_fn, p_info, "pwm_in"))) {
        p_info->thread_stat = PWMIN_THRD_UNINITED;
        ret = -ECHILD;
		goto out;
	}
	p_info->thread_stat = PWMIN_THRD_STARTED;

    g_p_devinfo = p_info;

    ret = 0;
    printk("pwmin device probed!\n");
out:
	if (ret) {
		pwmin_destroy_buf(p_pltdev);
	}
	return ret;
}

int pwmin_suspend(struct platform_device *p_pltdev, pm_message_t state)
{
	return 0;
}

int pwmin_resume(struct platform_device *p_pltdev)
{
	return 0;
}

int pwmin_remove(struct platform_device *p_pltdev)
{
    printk("%s\n", __FUNCTION__);
	pwmin_destroy_buf(p_pltdev);

    return 0;
}

static const struct of_device_id pwmin_dt_ids[] = {
	{ .compatible = "fsl,pwm-in", },
	{ }
};

static struct platform_driver pwmin_driver = {
	.probe		= pwmin_probe,
	.remove	    = pwmin_remove,
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
