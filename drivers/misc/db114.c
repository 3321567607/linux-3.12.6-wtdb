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

#define WTACK 1
#define WOACK 0

#define CMD_MSTMP 0x03
#define CMD_MSHUM 0x05
#define CMD_WSTAT 0x06
#define CMD_RSTAT 0x07
#define CMD_RESET 0x1e

struct db114_info {
	int          sck_gpio;
	int          sda_gpio;
	unsigned int value;
	struct class *humclass;
	bool         has_attr_value;
	bool         has_miscdev;
	bool         alive;
};

static struct db114_info *g_devinfo;

static int  db114_open   (struct inode *inode, struct file *file) { return 0; }
static int  db114_release(struct inode *inode, struct file *file) { return 0; }
static long db114_ioctl  (struct file *file, unsigned int cmd, unsigned long arg) { return 0; }

static struct file_operations db114_fops = {
	.owner = THIS_MODULE,
	.open = db114_open,
	.release = db114_release,
	.unlocked_ioctl = db114_ioctl
};
static struct miscdevice db114_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "db114",
	.fops = &db114_fops
};

static ssize_t value_show(struct class *cls, struct class_attribute *attr, char *_buf)
{
	return sprintf(_buf, "%d\n", (int)(g_devinfo->value));
}
static CLASS_ATTR_RO(value);

/* request a gpio specified by dt, initial disabled */
static int db114_get_gpio(struct platform_device *pdev, int *pgpio, char *gpioname)
{
	struct device_node *np = pdev->dev.of_node;
	int ret = 0;
	enum of_gpio_flags flags;

	*pgpio  = of_get_named_gpio_flags(np, gpioname, 0, &flags);
	if (*pgpio < 0) {
		printk("%s cannot get %s!\n", __FUNCTION__, gpioname);
		ret = -EFAULT;
	}

	ret = devm_gpio_request_one(&(pdev->dev), *pgpio, GPIOF_OUT_INIT_LOW, gpioname);
	if (ret) {
		*pgpio = -1;
		dev_err(&pdev->dev, "failed to request %s: %d\n", gpioname, ret);
	}
	return ret;
}

static void db114_destroy_buf(struct platform_device *pdev, struct db114_info *devinfo)
{
	if (devinfo) {
		if (devinfo->has_miscdev) {
			misc_deregister(&db114_misc);
			devinfo->has_miscdev = false;
		}
		
		if (!IS_ERR(devinfo->humclass)) {
			if (devinfo->has_attr_value) {
				class_remove_file(devinfo->humclass, &class_attr_value);
				devinfo->has_attr_value = false;
			}
			class_destroy(devinfo->humclass);
			devinfo->humclass = NULL;
		}

		if (devinfo->sck_gpio > 0) {
			devm_gpio_free(&(pdev->dev), devinfo->sck_gpio);
			devinfo->sck_gpio = -1;
		}

		if (devinfo->sda_gpio > 0) {
			devm_gpio_free(&(pdev->dev), devinfo->sda_gpio);
			devinfo->sda_gpio = -1;
		}

		kfree(devinfo);
	}
	g_devinfo = NULL;
}

/******************** start ********************/
static int s_write_byte(struct db114_info *pinfo, unsigned char value)
{
	int error = 0;
	unsigned char val;
	unsigned char i;

	gpio_set_value_cansleep(pinfo->sck_gpio, 0);
	for (i = 0x80; i > 0; i >>= 1) {
		udelay(1);
		if (i & value)
			gpio_set_value_cansleep(pinfo->sda_gpio, 1);
		else
			gpio_set_value_cansleep(pinfo->sda_gpio, 0);

		udelay(1);
		gpio_set_value_cansleep(pinfo->sck_gpio, 1);
		udelay(5);
		gpio_set_value_cansleep(pinfo->sck_gpio, 0);
	}

	gpio_direction_input(pinfo->sda_gpio);
	udelay(1);

	gpio_set_value_cansleep(pinfo->sck_gpio, 1);
	udelay(1);

	val = gpio_get_value_cansleep(pinfo->sda_gpio);
	if (val != 0) {
		error = 1;
	}

	gpio_set_value_cansleep(pinfo->sck_gpio, 0);
	gpio_direction_output(pinfo->sda_gpio, 1);
	udelay(1);

	return error;
}

static char s_read_byte(struct db114_info *pinfo, unsigned char ack)
{ 
	unsigned char i,val=0;

	gpio_direction_input(pinfo->sda_gpio);
	udelay(1);

	for (i = 0x80; i > 0; i >>= 1) {
		gpio_set_value_cansleep(pinfo->sck_gpio, 1);
		if (gpio_get_value_cansleep(pinfo->sda_gpio))
			val = (val | i);        //read bit  
		gpio_set_value_cansleep(pinfo->sck_gpio, 0);
		udelay(1);
	}

	gpio_direction_output(pinfo->sda_gpio, !ack);
	udelay(1);
	gpio_set_value_cansleep(pinfo->sck_gpio, 1);
	udelay(5);
	gpio_set_value_cansleep(pinfo->sck_gpio, 0);
	gpio_direction_output(pinfo->sda_gpio, 1);
	udelay(1);

	return val;
}

static void s_transstart(struct db114_info *pinfo)
// generates a transmission start 
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
{  
	gpio_set_value_cansleep(pinfo->sck_gpio, 0);
   gpio_set_value_cansleep(pinfo->sda_gpio, 1);
   udelay(2);

   gpio_set_value_cansleep(pinfo->sck_gpio, 1);
   udelay(2);
   gpio_set_value_cansleep(pinfo->sda_gpio, 0);
   udelay(2);
   gpio_set_value_cansleep(pinfo->sck_gpio, 0); 

   udelay(5);

   gpio_set_value_cansleep(pinfo->sck_gpio, 1);
   udelay(2);
   gpio_set_value_cansleep(pinfo->sda_gpio, 1);
   udelay(2);
   gpio_set_value_cansleep(pinfo->sck_gpio, 0); 
   udelay(1);
}

// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______
static void s_connectionreset(struct db114_info *pinfo)
{  
	unsigned char i; 

	gpio_set_value_cansleep(pinfo->sda_gpio, 1);
	gpio_set_value_cansleep(pinfo->sck_gpio, 0); 
	udelay(1);

	for (i = 0; i < 9; i++) {
		gpio_set_value_cansleep(pinfo->sck_gpio, 1);
		udelay(1);
		gpio_set_value_cansleep(pinfo->sck_gpio, 0); 
		udelay(1);
	}
	s_transstart(pinfo);                   //transmission start
}

// resets the sensor by a softreset 
static char s_softreset(struct db114_info *pinfo)
{ 
	unsigned char error=0;
	
	printk("db114: soft reset!\n");
	s_connectionreset(pinfo);
	error += s_write_byte(pinfo, CMD_RESET);
	msleep(50);
	return error;
}

// reads the status register with checksum (8-bit)
static char s_read_statusreg(struct db114_info *pinfo, unsigned char *p_value, unsigned char *p_checksum)
{ 
	unsigned char error=0;

	s_transstart(pinfo);                   //transmission start
	error=s_write_byte(pinfo, CMD_RSTAT); //send command to sensor
	*p_value=s_read_byte(pinfo, WTACK);        //read status register (8-bit)
	*p_checksum=s_read_byte(pinfo, WOACK);   //read checksum (8-bit)  

	return error;                     //error=1 in case of no response form the sensor
}

// writes the status register with checksum (8-bit)
static char s_write_statusreg(struct db114_info *pinfo, unsigned char *p_value)
{ 
	unsigned char error=0;
	s_transstart(pinfo);
	error+=s_write_byte(pinfo, CMD_WSTAT);
	error+=s_write_byte(pinfo, *p_value);    //send value of status register
	return error;                     //error>=1 in case of no response form the sensor
}
 							   
							   // makes a measurement (humidity/temperature) with checksum
static int s_measure(struct db114_info *pinfo, unsigned short *p_value, unsigned char *p_checksum, unsigned char measure_cmd)
{ 
	int error = 0;
	unsigned int i;
	bool available = false;

	s_transstart(pinfo);                   //transmission start

	error = s_write_byte(pinfo, measure_cmd);
	if (error)
		goto out;

	gpio_direction_input(pinfo->sda_gpio);

	for (i = 0; i < 70; i++) {
		if (0 == gpio_get_value_cansleep(pinfo->sda_gpio)) {
			available = true;
			break;
		}
		msleep(30);
	}
	if (70 == i) {
		error = 1;
		goto out;
	}

	*p_value = s_read_byte(pinfo, WTACK);
	*p_value <<= 8;
	*p_value |= s_read_byte(pinfo, WTACK);
	*p_checksum = s_read_byte(pinfo, WOACK);

out:
	return error;
}
/********************* end *********************/

static int db114_kthread(void *par)
{
	unsigned char checksum;
	unsigned short hum_val, tmp_val;
	int error;
	struct db114_info *pinfo = (struct db114_info *)par;


	s_softreset(pinfo);

	while (pinfo->alive) {
		error = 0;
		error = s_measure(pinfo, &hum_val, &checksum, CMD_MSHUM);  //measure humidity
		hum_val &= 0x0FFF; /* 12-bit humidity */
		error = s_measure(pinfo, &tmp_val, &checksum, CMD_MSTMP);  //measure temperature
		tmp_val &= 0x3FFF; /* 14-bit temperature */

		if (error != 0) {
			printk("failed to read humidity!\n");
			s_connectionreset(pinfo);                 //in case of an error: connection reset
		} else {
			pinfo->value = (tmp_val << 16) | hum_val;
			//printk("temp-value: 0x%04x\n", tmp_val);
			//printk("humi-value: 0x%04x\n", hum_val);
		}

		msleep(800);
	}

	return 0;
}

static int db114_probe(struct platform_device *pdev)
{
	int ret ;
	struct db114_info *devinfo;

	devinfo = kzalloc(sizeof(*devinfo), GFP_KERNEL);
	if (!devinfo) {
		printk("%s alloc mem failed!\n", __FUNCTION__);
		return -ENOMEM;
	}

	/* request pwrkey-gpio, initial low */
	if (0 != (ret = db114_get_gpio(pdev, &(devinfo->sck_gpio), "sck_gpio"))) goto end;
	if (0 != (ret = db114_get_gpio(pdev, &(devinfo->sda_gpio), "sda_gpio"))) goto end;


	devinfo->humclass = class_create(THIS_MODULE, "humidity");
	if (IS_ERR(devinfo->humclass)) {
		printk("%s: Fail to create class humility!\n", __FUNCTION__);
		ret = (NULL == devinfo->humclass) ? (-ENOMEM) : PTR_ERR(devinfo->humclass);
		goto end;
	}

	if (0 == (ret =  class_create_file(devinfo->humclass, &class_attr_value))) {
		devinfo->has_attr_value = true;
	} else {
		printk("%s: Fail to create class attrib file!\n", __FUNCTION__);
		goto end;
	}


	/* don't care wether we succeeded or not */
	if (0 == misc_register(&db114_misc))
		devinfo->has_miscdev = true;
	else
		devinfo->has_miscdev = false;

	platform_set_drvdata(pdev, devinfo);
	g_devinfo = devinfo;
	devinfo->alive = true;

	if (kernel_thread(db114_kthread, devinfo, 0) < 0) {
		printk("%s: kthread not created!\n", __FUNCTION__);
		ret= -EPIPE;
	}

end:
	if (ret) {
		printk("db114 probe failed!\n");
		db114_destroy_buf(pdev, devinfo);
	}

	return ret;
}

static int db114_suspend(struct platform_device *pdev, pm_message_t state) { return 0; }
static int db114_resume(struct platform_device *pdev) { return 0; }
static void db114_shutdown(struct platform_device *pdev){ }

static const struct of_device_id db114_dt_ids[] = {
	{ .compatible = "dabeco,db114", },
	{ }
};

static struct platform_driver db114_driver = {
	.probe		= db114_probe,
	.shutdown	= db114_shutdown,
	.suspend	= db114_suspend,
	.resume		= db114_resume,
	.driver	= {
		.name	= "DB114",
		.owner	= THIS_MODULE,
		.of_match_table = db114_dt_ids,
	},
};

static int __init db114_init(void)
{
	return platform_driver_register(&db114_driver);
}

static void __exit db114_exit(void)
{
	platform_driver_unregister(&db114_driver);
}

module_init(db114_init);
module_exit(db114_exit);
