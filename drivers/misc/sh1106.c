
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>

#define SH1106_IIC_SPEED (100 * 1000)
#define OP_CMD 0x00
#define OP_DATA 0x40
#define SH1106_COL_START 2
#define SH1106_COL_NUM 128
#define SH1106_PAGE_NUM 8
#define SH1106_FBSIZE (SH1106_COL_NUM * SH1106_PAGE_NUM)

#define SHIO_PANDISPLAY 0x3721

struct sh1106_gpio {
	int gpio;
	enum of_gpio_flags flags;
};
struct sh1106_info {
    struct sh1106_gpio gpio_blen;
    struct sh1106_gpio gpio_lcda0;
    struct sh1106_gpio gpio_reset;
    struct sh1106_gpio gpio_lcden;
    struct device_node *p_node;
    struct platform_device *p_pltdev;
    struct i2c_client *p_client;
    int i2cdvr_added;

    struct miscdevice miscdev;
    int opened;

    void           *virt_addr;
    unsigned long   phys_addr;
    unsigned char  *p_prevscrn;
};

static struct platform_device *p_dummy_sh1106_dev;
static struct sh1106_info g_sh1106_info;
static uint8_t sh1106_init_cmds[] = {
    0xAE,   /* display off */

    0xD5,   /* divide ratio/osf, cont */
    0x80,   /* divide-ratio = 1(0b0000), osf +15%(0b1000) */

    0xA8,   /* multiplex ration mode set, cont */
    0x3F,   /* 64, full value, POR */

    0xD3,   /* Display offset mode set, cont */
    0x00,   /* start line 0, POR */
    //0x20,   /* start line 32, POR */

    0x40,   /* Set display start line - 0 */
    //0x60,   /* Set display start line - 32 */

    0xAD,   /* DC-DC control mode set, cont  */
    //0x8B,   /* turn on DCDC */
    0x8A,   /* turn off DCDC, use external vpp */

    //0x32,   /* set pump voltage to 8.4 */

#if 0
    0xA1,   /* set segment re-map, reverse direction, left rotates */
    0xC8,   /* set common output scan direction: from COM[N-1] to COM0 */
#else
    /* rotate 180 degree */
    0xA0,   /* set segment re-map, normal direction, right rotates */
    0xC0,   /* set common output scan direction: from COM0 to COM[N-1] */
#endif

    0xDA,   /* common pads hardware configuration mode set */
    0x12,   /* alternative */

    0x81,   /* contrast control mode set, cont */
    0x40,   /* contrast value 64, 0~255 */

    0xD9,   /* set discharge/precharge period */
    0x1F,   /* discharge: 1 dclks, precharge: 15 dclks */

    0xDB,   /* set VCOM deselect level, cont */
    0x40,

    0xA4,   /* normal display status is provided */
    //0xA5,   /* entire display on is provided */

#if 1
    /* blue font, black background */
    0xA6,   /* set normal display */
#else
    /* black font, blue background */
    0xA7,   /* reverse display */
#endif

    0xAF,   /* display on OLED */
};

static inline u64 sh1106_get_cur_usec(void)
{
    u64 ret;

    ret = local_clock();
    do_div(ret, 1000);

    return ret;
}
static void sh1106_wr_cmd(struct sh1106_info *p_info, uint8_t cmd)
{
    i2c_master_reg8_send(p_info->p_client, OP_CMD, &cmd, 1, SH1106_IIC_SPEED);
}
/*
static void sh1106_wr_data(struct sh1106_info *p_info, uint8_t data)
{
    i2c_master_reg8_send(p_info->p_client, OP_DATA, &data, 1, SH1106_IIC_SPEED);
}
*/
static void sh1106_wr_cmds(struct sh1106_info *p_info, uint8_t *p_cmd, int len)
{
    i2c_master_reg8_send(p_info->p_client, OP_CMD, p_cmd, len, SH1106_IIC_SPEED);
}
static void sh1106_wr_multidata(struct sh1106_info *p_info, uint8_t *p_buf, int len)
{
    i2c_master_reg8_send(p_info->p_client, OP_DATA, p_buf, len, SH1106_IIC_SPEED);
}

static int sh1106_open(struct inode *inode, struct file *fl)
{
    struct sh1106_info *p_info = &g_sh1106_info;

    if (p_info->opened) {
        return -EMFILE;
    } else {
        p_info->opened++;
        printk("dev sh1106 opened!\n");
        return 0;
    }
}

static int sh1106_release(struct inode *inode, struct file *fl)
{
    struct sh1106_info *p_info = &g_sh1106_info;

    if (p_info->opened) {
        p_info->opened--;
        printk("dev sh1106 released!\n");
        return 0;
    } else {
        return -ENODEV;
    }
}

#if 1
static void sh1106_update_1page(struct sh1106_info *p_info, int page_index)
{
    int updated_column = 0;
    unsigned char cmds[3];
    unsigned char col, colhi, collo;
    unsigned char prev_colhi = 0xFF;
    unsigned char prev_collo = 0xFF;
    unsigned char start = 0;
    unsigned char end;
    unsigned char *p_new;
    unsigned char *p_old;
    int i;
    int page_sent = 0;
    int offset;

    offset = page_index << 7;
    p_new = ((char *)p_info->virt_addr) + offset;
    p_old = p_info->p_prevscrn + offset;
    while (updated_column < SH1106_COL_NUM) {
        if (p_new[updated_column] == p_old[updated_column]) {
            /* values are same, no update */
            updated_column++;
        } else {
            i = 0; /* cmd index */
            /* old value is different from new value, update */
            start = updated_column;
            end = updated_column;

            while (end < (SH1106_COL_NUM - 1)) {
                if (    (end <= (SH1106_COL_NUM - 4))
                     && (p_new[end + 1] == p_old[end + 1])
                     && (p_new[end + 2] == p_old[end + 2])
                     && (p_new[end + 3] == p_old[end + 3])
                   )
                {
                    break;
                    
                } else {
                    end++;
                }
            }

            /* set page number */
            if (!page_sent) {
                //printk("set page number %d\n", page_index);
                cmds[i++] = 0xB0 + page_index;
                page_sent = 1;
            }

            /* set column number */
            col = SH1106_COL_START + start;
            collo =  (col & 0x0F);
            colhi = ((col & 0xF0) >> 4) | 0x10;
            if (1/*colhi != prev_colhi*/) {
                //printk("set col_hi %d\n", colhi);
                cmds[i++] = colhi;
            }
            if (1/*collo != prev_collo*/) {
                //printk("set col_lo %d\n", collo);
                cmds[i++] = collo;
            }
            if (i > 0) {
                /* jump to 'start' */
                sh1106_wr_cmds(p_info, cmds, i);
            } /* else: impossible */

            /* xfer dots data */
            //printk("send data %d - %d\n", start, end);
            sh1106_wr_multidata(p_info, &(p_new[start]), end - start + 1);
            prev_colhi = colhi;
            prev_collo = collo;
            updated_column = end + 1;
        }
    } /* end of while */

}
#else
static void sh1106_update_1page(struct sh1106_info *p_info, int i)
{
    unsigned char *p_data;
    unsigned char cmds[3];

    cmds[0] = 0xB0 +i;
    cmds[1] = 0x10;
    cmds[2] = 0x02;
    sh1106_wr_cmds(p_info, cmds, 3);

    p_data = (unsigned char *)(p_info->virt_addr);
    sh1106_wr_multidata(p_info, p_data + (i << 7), 128);
}
#endif

static long sh1106_ioctl(struct file *fl, unsigned int cmd, unsigned long arg)
{
    int i;
    u64 old, now;
    struct sh1106_info *p_info = &g_sh1106_info;

    switch(cmd) {
        case SHIO_PANDISPLAY:
            old = sh1106_get_cur_usec();
            for (i = 0; i < 8; i++) {
                sh1106_update_1page(p_info, i);
            }
            now = sh1106_get_cur_usec();
            printk("%d usec to update\n", (int)(now - old));
            memcpy(p_info->p_prevscrn, p_info->virt_addr, SH1106_FBSIZE);
            break;

        default:
            printk("Err: unknown cmd %d for sh1106!\n", cmd);
            break;
    }

    return 0;
}
static int sh1106_mmap(struct file *fl, struct vm_area_struct *vma)
{
    struct sh1106_info *p_info = &g_sh1106_info;

    printk("sh1106 mmap!\n");

    vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
    vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
    return vm_iomap_memory(vma, p_info->phys_addr, SH1106_FBSIZE);;
}
static struct file_operations sh1106_fops = {
	.owner = THIS_MODULE,
	.open = sh1106_open,
	.release = sh1106_release,
	.unlocked_ioctl = sh1106_ioctl,
	.mmap = sh1106_mmap,
};

static int sh1106_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
    struct sh1106_info *p_info = &g_sh1106_info;
    int i, k;
    u64 t1, t2;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk("Err: check sh1106 i2c func!\n");
		rc = -ENODEV;
        goto out;
	}

    p_info->p_client = client;

    sh1106_wr_cmds(p_info, sh1106_init_cmds, sizeof(sh1106_init_cmds));

    p_info->miscdev.fops = &sh1106_fops;
    p_info->miscdev.minor = MISC_DYNAMIC_MINOR;
    p_info->miscdev.name = "sh1106";
    
	if ((rc = misc_register(&(p_info->miscdev)))) {
		printk("%s: misc_register err\n", __FUNCTION__);
        goto out;
	}

    if (!(p_info->virt_addr = alloc_pages_exact(SH1106_FBSIZE, 0))) {
        printk("Err: alloc fb!\n");
        goto out;
    }
    p_info->phys_addr = virt_to_phys(p_info->virt_addr);
    p_info->p_prevscrn = kzalloc(SH1106_FBSIZE, GFP_KERNEL);
    memset(p_info->p_prevscrn, 0x55, SH1106_FBSIZE);

    t1 = sh1106_get_cur_usec();
    k = 0;
    for (i = 0; i < 8; i++) {
        sh1106_wr_cmd(p_info, 0xB0 + i);
        sh1106_wr_cmd(p_info, 0x10);
        sh1106_wr_cmd(p_info, 0x02);
        sh1106_wr_multidata(p_info, p_info->p_prevscrn, 128);
    }
    t2 = sh1106_get_cur_usec();

    rc = 0;
    printk("sh1106 inited %d!\n", (int)(t2 - t1));

out:
	return rc;
}

static int sh1106_remove(struct i2c_client *client)
{
    struct sh1106_info *p_info = &g_sh1106_info;

    misc_deregister(&(p_info->miscdev));

    free_pages_exact(p_info->virt_addr, SH1106_FBSIZE);
    p_info->virt_addr = NULL;

    kfree(p_info->p_prevscrn);
    p_info->p_prevscrn = NULL;

	return 0;
}

void sh1106_shutdown(struct i2c_client * client)
{
    /* don't disable clockout here */
    printk("wangluheng: entered %s\n", __FUNCTION__);
}

static const struct i2c_device_id sh1106_id[] = {
	{ "lcd_sh1106", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c,sh1106_id);
static const struct of_device_id sh1106_dt_ids[] = {
	{ .compatible = "sino,sh1106", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of,sh1106_dt_ids);


static struct i2c_driver sh1106_driver = {
	.driver		= {
		.name	= "lcd_sh1106",
		.owner	= THIS_MODULE,
		.of_match_table = sh1106_dt_ids,
	},
	.probe		= sh1106_probe,
	.remove		= sh1106_remove,
	.shutdown   = sh1106_shutdown,
	.id_table	= sh1106_id,
};

static void sh1106_enable_gpio(struct sh1106_gpio *pgpio, bool enable)
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
static int sh1106_get_gpio(struct sh1106_info *p_info, struct sh1106_gpio *pgpio, char *gpioname)
{
    struct platform_device *pdev = p_info->p_pltdev;
    struct device_node *np = p_info->p_node;
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

		ret = devm_gpio_request_one(&(pdev->dev), (unsigned int)(pgpio->gpio), init_flags, gpioname);
		if (ret) {
			pgpio->gpio = -1;
			dev_err(&pdev->dev, "failed to request %s: %d\n", gpioname, ret);
		}
	}

	return ret;
}
static int dummy_sh1106_probe(struct platform_device *pdev)
{
    int ret = -1;
    struct sh1106_info *p_info = &g_sh1106_info;

    p_info->p_pltdev = pdev;

    printk("powering sh1106...\n");

    if (!(p_info->p_node = of_find_compatible_node(NULL, NULL, "sino,sh1106"))) {
        printk("Err: %s, failed to get device-tree-node\n", __FUNCTION__);
        goto out;
    }

    sh1106_get_gpio(p_info, &(p_info->gpio_blen),  "blen-gpios");
    sh1106_get_gpio(p_info, &(p_info->gpio_lcda0), "lcda0-gpios");
    sh1106_get_gpio(p_info, &(p_info->gpio_reset), "lcdrst-gpios");
    sh1106_get_gpio(p_info, &(p_info->gpio_lcden), "lcden-gpios");

    sh1106_enable_gpio(&(p_info->gpio_lcda0), 1);
    msleep(10);
    sh1106_enable_gpio(&(p_info->gpio_lcden), 1);
    msleep(10);
    sh1106_enable_gpio(&(p_info->gpio_blen), 1);
    msleep(10);

    sh1106_enable_gpio(&(p_info->gpio_reset), 0);
    msleep(10);
    sh1106_enable_gpio(&(p_info->gpio_reset), 1);
    msleep(50);
    sh1106_enable_gpio(&(p_info->gpio_reset), 0);
    msleep(100);

    if (!i2c_add_driver(&sh1106_driver)) {
        p_info->i2cdvr_added = 1;
    }
    printk("sh1106 powered!\n");
    ret = 0;

out:
    if (ret) {
        p_info->p_pltdev = NULL;
    }
	return 0;
}

int dummy_sh1106_remove(struct platform_device *pdev)
{
    struct sh1106_info *p_info = &g_sh1106_info;

    sh1106_enable_gpio(&(p_info->gpio_reset), 1);
    sh1106_enable_gpio(&(p_info->gpio_blen), 0);
    sh1106_enable_gpio(&(p_info->gpio_lcden), 0);
    devm_gpio_free(&(pdev->dev), p_info->gpio_blen.gpio);
    devm_gpio_free(&(pdev->dev), p_info->gpio_lcda0.gpio);
    devm_gpio_free(&(pdev->dev), p_info->gpio_reset.gpio);
    devm_gpio_free(&(pdev->dev), p_info->gpio_lcden.gpio);

    return 0;
}
static struct platform_driver dummy_sh1106_driver = {
	.probe		= dummy_sh1106_probe,
    .remove     = dummy_sh1106_remove,
	.driver	= {
		.name	= "dummy_sh1106",
		.owner	= THIS_MODULE,
	},
};
static int __init sh1106_init(void)
{
    printk("wlh: %s\n", __FUNCTION__);
    p_dummy_sh1106_dev = platform_device_alloc("dummy_sh1106", 0);
    
    platform_device_add(p_dummy_sh1106_dev);
	return platform_driver_register(&dummy_sh1106_driver);
}

static void __exit sh1106_exit(void)
{
    if (g_sh1106_info.i2cdvr_added) {
        i2c_del_driver(&sh1106_driver);
    }

	platform_driver_unregister(&dummy_sh1106_driver);
    platform_device_unregister(p_dummy_sh1106_dev);
    printk("wangluheng: free dummy platform dev\n");
    //kfree(p_dummy_sh1106_dev);
    p_dummy_sh1106_dev = NULL;
}

MODULE_AUTHOR("wlh wlh@intelligen.com");
MODULE_DESCRIPTION("LCD sh1106 driver");
MODULE_LICENSE("GPL");

module_init(sh1106_init);
module_exit(sh1106_exit);

