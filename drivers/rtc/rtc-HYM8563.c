/* drivers/rtc/rtc-HYM8563.c - driver for HYM8563
 *
 * Copyright (C) 2010 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//#define DEBUG
#define pr_fmt(fmt) "rtc: %s: " fmt, __func__

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/bcd.h>
#include <linux/rtc.h>
#include <linux/delay.h>
//#include <linux/wakelock.h>
#include <linux/slab.h>
//#include <mach/iomux.h>
#include "linux/i2c/rtc-HYM8563.h"
//#include <mach/board.h>

#ifdef pr_debug
#undef pr_debug
#define pr_debug printk
#endif

#define RTC_SPEED 	200 * 1000

struct hym8563 {
	int irq;
	struct i2c_client *client;
	struct work_struct work;
	struct mutex mutex;
	struct rtc_device *rtc;
	int exiting;
	struct rtc_wkalrm alarm;
	//struct wake_lock wake_lock;
};

static int hym8563_i2c_read_regs(struct i2c_client *client, u8 reg, u8 buf[], unsigned len)
{
	int ret; 
	ret = i2c_master_reg8_recv(client, reg, buf, len, RTC_SPEED);
	return ret; 
}

static int hym8563_i2c_set_regs(struct i2c_client *client, u8 reg, u8 const buf[], __u16 len)
{
	int ret; 
	ret = i2c_master_reg8_send(client, reg, buf, (int)len, RTC_SPEED);
	return ret;
}


int hym8563_enable_count(struct i2c_client *client, int en)
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);	
	u8 regs[2];

	if (!hym8563)
		return -1;

	if (en) {
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
		regs[0] |= TIE;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		regs[0] = 0;
		regs[0] |= (TE | TD1);
		hym8563_i2c_set_regs(client, RTC_T_CTL, regs, 1);
	}
	else {
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
		regs[0] &= ~TIE;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		regs[0] = 0;
		regs[0] |= (TD0 | TD1);
		hym8563_i2c_set_regs(client, RTC_T_CTL, regs, 1);
	}
	return 0;
}

//0 < sec <=255
int hym8563_set_count(struct i2c_client *client, int sec)
{	
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[2];

	if (!hym8563)
		return -1;
		
	if (sec >= 255)
		regs[0] = 255;
	else if (sec <= 1)
		regs[0] = 1;
	else
		regs[0] = sec;
	
	hym8563_i2c_set_regs(client, RTC_T_COUNT, regs, 1);
	
	return 0;
}


/*the init of the hym8563 at first time */
static int hym8563_init_device(struct i2c_client *client)	
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[2];
	int sr;

	mutex_lock(&hym8563->mutex);
	regs[0]=0;
	sr = hym8563_i2c_set_regs(client, RTC_CTL1, regs, 1);		
	if (sr < 0)
		goto exit;
	
	//disable clkout
	/* wangluheng: don't enable clock out here */
	/*regs[0] = 0x80;
	sr = hym8563_i2c_set_regs(client, RTC_CLKOUT, regs, 1);
	if (sr < 0)
		goto exit; */

	/*enable alarm && count interrupt*/
	sr = hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0)
		goto exit;
	regs[0] = 0x0;
	regs[0] |= (AIE | TIE);
	sr = hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0)
		goto exit;
	sr = hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0)
		goto exit;

	sr = hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
	if (sr < 0) {
		pr_err("read CTL2 err\n");
		goto exit;
	}
	
	if(regs[0] & (AF|TF))
	{
		regs[0] &= ~(AF|TF);
		sr = hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	}
	
exit:
	mutex_unlock(&hym8563->mutex);
	
	return sr;
}

static int hym8563_read_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[HYM8563_RTC_SECTION_LEN] = { 0, };
	mutex_lock(&hym8563->mutex);
//	for (i = 0; i < HYM8563_RTC_SECTION_LEN; i++) {
//		hym8563_i2c_read_regs(client, RTC_SEC+i, &regs[i], 1);
//	}
	hym8563_i2c_read_regs(client, RTC_SEC, regs, HYM8563_RTC_SECTION_LEN);

	mutex_unlock(&hym8563->mutex);
	
	tm->tm_sec = bcd2bin(regs[0x00] & 0x7F);
	tm->tm_min = bcd2bin(regs[0x01] & 0x7F);
	tm->tm_hour = bcd2bin(regs[0x02] & 0x3F);
	tm->tm_mday = bcd2bin(regs[0x03] & 0x3F);
	tm->tm_wday = bcd2bin(regs[0x04] & 0x07);	
	
	tm->tm_mon = bcd2bin(regs[0x05] & 0x1F) ; 
	tm->tm_mon -= 1;			//inorder to cooperate the systerm time
	
	tm->tm_year = bcd2bin(regs[0x06] & 0xFF);
	if(regs[5] & 0x80)
		tm->tm_year += 1900;
	else
		tm->tm_year += 2000;
		
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);	
	tm->tm_year -= 1900;			//inorder to cooperate the systerm time	
	if(tm->tm_year < 0)
		tm->tm_year = 0;	
	tm->tm_isdst = 0;	

	pr_debug("rtc-%d read %4d-%02d-%02d(%d) %02d:%02d:%02d\n", client->adapter->nr,
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

static int hym8563_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return hym8563_read_datetime(to_i2c_client(dev), tm);
}

static int hym8563_set_time(struct i2c_client *client, struct rtc_time *tm)	
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[HYM8563_RTC_SECTION_LEN] = { 0, };
	u8 mon_day;

	pr_debug("rtc-%d write %4d-%02d-%02d(%d) %02d:%02d:%02d\n", client->adapter->nr,
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	mon_day = rtc_month_days((tm->tm_mon), tm->tm_year + 1900);
	
	if(tm->tm_sec >= 60 || tm->tm_sec < 0 )		//set  sec
		regs[0x00] = bin2bcd(0x00);
	else
		regs[0x00] = bin2bcd(tm->tm_sec);
	
	if(tm->tm_min >= 60 || tm->tm_min < 0 )		//set  min	
		regs[0x01] = bin2bcd(0x00);
	else
		regs[0x01] = bin2bcd(tm->tm_min);

	if(tm->tm_hour >= 24 || tm->tm_hour < 0 )		//set  hour
		regs[0x02] = bin2bcd(0x00);
	else
		regs[0x02] = bin2bcd(tm->tm_hour);
	
	if((tm->tm_mday) > mon_day)				//if the input month day is bigger than the biggest day of this month, set the biggest day 
		regs[0x03] = bin2bcd(mon_day);
	else if((tm->tm_mday) > 0)
		regs[0x03] = bin2bcd(tm->tm_mday);
	else if((tm->tm_mday) <= 0)
		regs[0x03] = bin2bcd(0x01);

	if( tm->tm_year >= 200)		// year >= 2100
		regs[0x06] = bin2bcd(99);	//year = 2099
	else if(tm->tm_year >= 100)			// 2000 <= year < 2100
		regs[0x06] = bin2bcd(tm->tm_year - 100);
	else if(tm->tm_year >= 0){				// 1900 <= year < 2000
		regs[0x06] = bin2bcd(tm->tm_year);	
		regs[0x05] |= 0x80;	
	}else{									// year < 1900
		regs[0x06] = bin2bcd(0);	//year = 1900	
		regs[0x05] |= 0x80;	
	}	
	regs[0x04] = bin2bcd(tm->tm_wday);		//set  the  weekday
	regs[0x05] = (regs[0x05] & 0x80)| (bin2bcd(tm->tm_mon + 1) & 0x7F);		//set  the  month
	
	mutex_lock(&hym8563->mutex);
//	for(i=0;i<HYM8563_RTC_SECTION_LEN;i++){
//		ret = hym8563_i2c_set_regs(client, RTC_SEC+i, &regs[i], 1);
//	}
	hym8563_i2c_set_regs(client, RTC_SEC, regs, HYM8563_RTC_SECTION_LEN);

	mutex_unlock(&hym8563->mutex);

	return 0;
}

static int hym8563_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	return hym8563_set_time(to_i2c_client(dev), tm);
}

static int hym8563_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *tm)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	u8 regs[4] = { 0, };
	
	pr_debug("enter\n");
	mutex_lock(&hym8563->mutex);
	hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);
	regs[0] = 0x0;
	regs[0] |= TIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
	mutex_unlock(&hym8563->mutex);
	return 0;
}

static int hym8563_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{	
	struct i2c_client *client = to_i2c_client(dev);
	struct hym8563 *hym8563 = i2c_get_clientdata(client);
	struct rtc_time now, *tm = &alarm->time;
	u8 regs[4] = { 0, };
	u8 mon_day;	
	unsigned long	alarm_sec, now_sec;
	int diff_sec = 0;
	
	pr_debug("%4d-%02d-%02d(%d) %02d:%02d:%02d enabled %d\n",
		1900 + tm->tm_year, tm->tm_mon + 1, tm->tm_mday, tm->tm_wday,
		tm->tm_hour, tm->tm_min, tm->tm_sec, alarm->enabled);
	
	
	hym8563_read_datetime(client, &now);

	
	mutex_lock(&hym8563->mutex);
	rtc_tm_to_time(tm, &alarm_sec);
	rtc_tm_to_time(&now, &now_sec);
	
	diff_sec = alarm_sec - now_sec;
	
	if((diff_sec > 0) && (diff_sec < 256))
	{	
		printk("%s:diff_sec= %ds , use time\n",__func__, diff_sec);
		hym8563_enable_count(client, 1);				
		hym8563_set_count(client, diff_sec);
	}
	else
	{				
		printk("%s:diff_sec= %ds , use alarm\n",__func__, diff_sec);
		hym8563_enable_count(client, 0);
		
		if(tm->tm_sec > 0)
		{
			rtc_tm_to_time(tm, &alarm_sec);
			rtc_time_to_tm(alarm_sec, tm);
		}

		hym8563->alarm = *alarm;

		regs[0] = 0x0;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		mon_day = rtc_month_days(tm->tm_mon, tm->tm_year + 1900);
		hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);

		if (tm->tm_min >= 60 || tm->tm_min < 0)		//set  min
		regs[0x00] = bin2bcd(0x00) & 0x7f;
		else
		regs[0x00] = bin2bcd(tm->tm_min) & 0x7f;
		if (tm->tm_hour >= 24 || tm->tm_hour < 0)	//set  hour
		regs[0x01] = bin2bcd(0x00) & 0x7f;
		else
		regs[0x01] = bin2bcd(tm->tm_hour) & 0x7f;
		regs[0x03] = bin2bcd (tm->tm_wday) & 0x7f;

		/* if the input month day is bigger than the biggest day of this month, set the biggest day */
		if (tm->tm_mday > mon_day)
		regs[0x02] = bin2bcd(mon_day) & 0x7f;
		else if (tm->tm_mday > 0)
		regs[0x02] = bin2bcd(tm->tm_mday) & 0x7f;
		else if (tm->tm_mday <= 0)
		regs[0x02] = bin2bcd(0x01) & 0x7f;

		hym8563_i2c_set_regs(client, RTC_A_MIN, regs, 4);	
		hym8563_i2c_read_regs(client, RTC_A_MIN, regs, 4);	
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);
		if (alarm->enabled == 1)
		regs[0] |= AIE;
		else
		regs[0] &= 0x0;
		regs[0] |= TIE;
		hym8563_i2c_set_regs(client, RTC_CTL2, regs, 1);
		hym8563_i2c_read_regs(client, RTC_CTL2, regs, 1);

		if(diff_sec <= 0)
		{		
			pr_info("alarm sec  <= now sec\n");
		}			

	}
	
	mutex_unlock(&hym8563->mutex);

	return 0;
}
#if defined(CONFIG_RTC_INTF_DEV) || defined(CONFIG_RTC_INTF_DEV_MODULE)
static int hym8563_i2c_open_alarm(struct i2c_client *client)
{
	u8 data;	
	hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data |= AIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);

	return 0;
}

static int hym8563_i2c_close_alarm(struct i2c_client *client)
{
	u8 data;	
	hym8563_i2c_read_regs(client, RTC_CTL2, &data, 1);
	data &= ~AIE;
	hym8563_i2c_set_regs(client, RTC_CTL2, &data, 1);

	return 0;
}

static int hym8563_rtc_ioctl(struct device *dev, unsigned int cmd, unsigned long arg)
{
	struct i2c_client *client = to_i2c_client(dev);
	
	switch (cmd) {
	case RTC_AIE_OFF:
		if(hym8563_i2c_close_alarm(client) < 0)
			goto err;
		break;
	case RTC_AIE_ON:
		if(hym8563_i2c_open_alarm(client))
			goto err;
		break;
	default:
		return -ENOIOCTLCMD;
	}	
	return 0;
err:
	return -EIO;
}
#else
#define hym8563_rtc_ioctl NULL
#endif

#if defined(CONFIG_RTC_INTF_PROC) || defined(CONFIG_RTC_INTF_PROC_MODULE)
static int hym8563_rtc_proc(struct device *dev, struct seq_file *seq)
{
	return 0;
}
#else
#define hym8563_rtc_proc NULL
#endif

#if 0
static irqreturn_t hym8563_wakeup_irq(int irq, void *data)
{
	struct hym8563 *hym8563 = data;	
	struct i2c_client *client = hym8563->client;	
	u8 value;
	
	mutex_lock(&hym8563->mutex);
	hym8563_i2c_read_regs(client, RTC_CTL2, &value, 1);
	value &= ~(AF|TF);
	hym8563_i2c_set_regs(client, RTC_CTL2, &value, 1);	
	mutex_unlock(&hym8563->mutex);
	
	rtc_update_irq(hym8563->rtc, 1, RTC_IRQF | RTC_AF | RTC_UF);

	//printk("%s:irq=%d\n",__func__,irq);
	return IRQ_HANDLED;
}
#endif

static const struct rtc_class_ops hym8563_rtc_ops = {
	.read_time	= hym8563_rtc_read_time,
	.set_time	= hym8563_rtc_set_time,
	.read_alarm	= hym8563_rtc_read_alarm,
	.set_alarm	= hym8563_rtc_set_alarm,
	.ioctl 		= hym8563_rtc_ioctl,
	.proc		= hym8563_rtc_proc
};

static int hym8563_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc = 0;
	u8 reg = 0;
	int enable_clkout = 0;
	struct hym8563_platform_data *p_data;
	struct hym8563 *hym8563;
	struct rtc_device *rtc = NULL;
	struct rtc_time tm_read, tm = {
		.tm_wday = 5,
		.tm_year = 115,
		.tm_mon = 3,
		.tm_mday =13,
		.tm_hour = 12,
		.tm_min = 30,
		.tm_sec = 44,
	};	
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		return -ENODEV;
	}
		
	hym8563 = kzalloc(sizeof(struct hym8563), GFP_KERNEL);
	if (!hym8563) {
		return -ENOMEM;
	}
	p_data = (struct hym8563_platform_data*)(client->dev.platform_data);

	hym8563->client = client;
	hym8563->alarm.enabled = 0;
	mutex_init(&hym8563->mutex);
	//wake_lock_init(&hym8563->wake_lock, WAKE_LOCK_SUSPEND, "rtc_hym8563");
	i2c_set_clientdata(client, hym8563);

	if (hym8563_init_device(client) < 0) {
		printk("hym8563: on i2c-%d nonexistent\n", client->adapter->nr);
		goto exit;
	}
	hym8563_enable_count(client, 0);	
	
	// check power down 
	hym8563_i2c_read_regs(client,RTC_SEC,&reg,1);
	hym8563_read_datetime(client, &tm_read);	//read time from hym8563
	
	if (((reg&0x80) | (tm_read.tm_year < 70) | (tm_read.tm_year > 137 )) | (tm_read.tm_mon == -1) | (rtc_valid_tm(&tm_read) != 0)) //if the hym8563 haven't initialized
	{
		dev_info(&client->dev, "clock/calendar information is no longer guaranteed\n");
		hym8563_set_time(client, &tm);	//initialize the hym8563 
	}	
	
	rtc = rtc_device_register(client->name, &client->dev,
				  &hym8563_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		rc = PTR_ERR(rtc);
		rtc = NULL;
		goto exit;
	}
	hym8563->rtc = rtc;

	if (of_property_read_u32(client->dev.of_node, "clkout", &enable_clkout))
		enable_clkout = 0;

	if (enable_clkout) {
		/* enable clock-out for rtc on i2c-0 */
		reg = 0x80;
	} else {
		/* disable clock-out for rtc on i2c-0 */
		reg = 0x00;
	}
	printk("%s rtc clock-out on I2C-%d\n", reg ? "enable" : "disable", client->adapter->nr);
	hym8563_i2c_set_regs(client, RTC_CLKOUT, &reg, 1);	

	return 0;

exit:
	if (rtc)
		rtc_device_unregister(rtc);
	if (hym8563) {
		//wake_lock_destroy(&hym8563->wake_lock);
		kfree(hym8563);
	}
	return rc;
}

static int hym8563_remove(struct i2c_client *client)
{
	struct hym8563 *hym8563 = i2c_get_clientdata(client);

	if (hym8563->irq > 0) {
		mutex_lock(&hym8563->mutex);
		hym8563->exiting = 1;
		mutex_unlock(&hym8563->mutex);

		free_irq(hym8563->irq, hym8563);
		cancel_work_sync(&hym8563->work);
	}

	rtc_device_unregister(hym8563->rtc);
	//wake_lock_destroy(&hym8563->wake_lock);
	kfree(hym8563);
	hym8563 = NULL;

	return 0;
}


void hym8563_shutdown(struct i2c_client * client)
{
    /* don't disable clockout here */
#if 0
	u8 regs[2];	
    int ret; 	
    //disable clkout	
    /* regs[0] = 0x00;	
    ret=hym8563_i2c_set_regs(client, RTC_CLKOUT, regs, 1);	
    if(ret<0)	
        printk("rtc shutdown is error\n"); */
#endif
}



static const struct i2c_device_id hym8563_id[] = {
	{ "rtc_hym8563", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hym8563_id);
static const struct of_device_id hym8563_dt_ids[] = {
	{ .compatible = "haoyu,hym8653", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, hym8563_dt_ids);


static struct i2c_driver hym8563_driver = {
	.driver		= {
		.name	= "rtc_hym8563",
		.owner	= THIS_MODULE,
		.of_match_table = hym8563_dt_ids,
	},
	.probe		= hym8563_probe,
	.remove		= hym8563_remove,
	.shutdown=hym8563_shutdown,
	.id_table	= hym8563_id,
};

static int __init hym8563_init(void)
{
	return i2c_add_driver(&hym8563_driver);
}

static void __exit hym8563_exit(void)
{
	i2c_del_driver(&hym8563_driver);
}

MODULE_AUTHOR("lhh lhh@rock-chips.com");
MODULE_DESCRIPTION("HYM8563 RTC driver");
MODULE_LICENSE("GPL");

module_init(hym8563_init);
module_exit(hym8563_exit);

