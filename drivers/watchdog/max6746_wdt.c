/*
 * Watchdog driver for the RTC based watchdog in STMP3xxx and i.MX23/28
 *
 * Author: Wolfram Sang <w.sang@pengutronix.de>
 *
 * Copyright (C) 2011-12 Wolfram Sang, Pengutronix
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/watchdog.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>

#define MAX6746_MAX_TIMEOUT (6000)

struct max6746_wdt_info {
	struct platform_device *p_platformdev;
	int                     feed_gpio;
	int                     enable_gpio;
	unsigned int            feedms;
	int                     feedcnt;
	int                     userstarted;
	enum of_gpio_flags      enable_flag;
	struct timer_list       wd_timer;
	struct work_struct      wd_work;
};

static void wdt_enable(struct max6746_wdt_info *pinfo, bool enable)
{
	struct platform_device *pdev = pinfo->p_platformdev;
	unsigned long init_flags;

	if (pinfo->enable_gpio > 0) {
		if (OF_GPIO_ACTIVE_LOW == pinfo->enable_flag) {
			if (enable)
				init_flags = GPIOF_OUT_INIT_LOW;
			else
				init_flags = GPIOF_OUT_INIT_HIGH;
		} else {
			if (enable)
				init_flags = GPIOF_OUT_INIT_HIGH;
			else
				init_flags = GPIOF_OUT_INIT_LOW;
		}

		if (0 == devm_gpio_request_one(&(pdev->dev), pinfo->enable_gpio, init_flags, "max6746_enable"))
			devm_gpio_free(&(pdev->dev), pinfo->enable_gpio);
	}
}

static int feed_max6746(struct max6746_wdt_info *pinfo)
{
	struct device *pdev = &(pinfo->p_platformdev->dev);
	unsigned int gpio = (unsigned int)(pinfo->feed_gpio);

	devm_gpio_request_one(pdev, gpio, GPIOF_OUT_INIT_HIGH, "max6746_feed");
	msleep(1);
	gpio_set_value_cansleep(gpio, 0);
	msleep(1);
	gpio_set_value_cansleep(gpio, 1);
	devm_gpio_free(pdev, gpio);
	//dev_info(pdev, "feed dog!\n");
	
	return 0;
}

static int wdt_ping(struct watchdog_device *wdd)
{
	return feed_max6746((struct max6746_wdt_info *)watchdog_get_drvdata(wdd));
}

static int wdt_start(struct watchdog_device *wdd)
{
	struct max6746_wdt_info *pinfo = watchdog_get_drvdata(wdd);
	struct device *pdev = &(pinfo->p_platformdev->dev);

	pinfo->userstarted = 1;
	wdt_enable((struct max6746_wdt_info *)watchdog_get_drvdata(wdd), true);
	dev_info(pdev, "started!\n");
	return 0;
}

static int wdt_stop(struct watchdog_device *wdd)
{
	struct max6746_wdt_info *pinfo = watchdog_get_drvdata(wdd);
	struct device *pdev = &(pinfo->p_platformdev->dev);
	
	wdt_enable((struct max6746_wdt_info *)watchdog_get_drvdata(wdd), false);
	dev_info(pdev, "stopped!\n");
	wdt_ping(wdd);
	return 0;
}

static const struct watchdog_info max6746_wdt_ident = {
	.options = WDIOF_MAGICCLOSE | WDIOF_KEEPALIVEPING,
	.identity = "MAX6746 RTC Watchdog",
};

static const struct watchdog_ops max6746_wdt_ops = {
	.owner = THIS_MODULE,
	.start = wdt_start,
	.stop = wdt_stop,
	.ping = wdt_ping,
};

static struct watchdog_device max6746_wdd = {
	.info = &max6746_wdt_ident,
	.ops = &max6746_wdt_ops,
	.min_timeout = 1,
	.max_timeout = MAX6746_MAX_TIMEOUT,
	.status = WATCHDOG_NOWAYOUT_INIT_STATUS,
};

static void feed_work(struct work_struct *work)
{
	struct max6746_wdt_info *pinfo = container_of(work, struct max6746_wdt_info, wd_work);

	feed_max6746(pinfo);
}

static void max6746_timer(unsigned long data)
{
	struct max6746_wdt_info *pinfo = (struct max6746_wdt_info *)data;

	if ((pinfo->feedcnt > 0) && (0 == pinfo->userstarted)) {
		
		schedule_work(&pinfo->wd_work);
		mod_timer(&pinfo->wd_timer, jiffies + msecs_to_jiffies(pinfo->feedms));
		pinfo->feedcnt--;
	} else {
		pinfo->feedcnt = 0;
		del_timer_sync(&pinfo->wd_timer);
	}

}

static int max6746_wdt_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = pdev->dev.of_node;
	struct max6746_wdt_info *pinfo = NULL;

	pinfo = (struct max6746_wdt_info *)kzalloc(sizeof(*pinfo), GFP_KERNEL);
	if (IS_ERR_OR_NULL(pinfo)) {
		ret = -ENOMEM;
		goto out;
	}

	pinfo->enable_gpio = of_get_named_gpio_flags(np, "enable-gpios", 0, &(pinfo->enable_flag));
	pinfo->feed_gpio   = of_get_named_gpio_flags(np, "feed-gpios", 0, NULL);
	if (0 != of_property_read_u32(np, "timeout", &(max6746_wdd.timeout))) {
		max6746_wdd.timeout = 1;
	}

	if (pinfo->feed_gpio < 0) {
		ret = -EIO;
		goto out;
	}

	pinfo->p_platformdev = pdev;

	watchdog_set_drvdata(&max6746_wdd, pinfo);

	ret = watchdog_register_device(&max6746_wdd);
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot register watchdog device\n");
		goto out;
	}

	if (0 != of_property_read_u32(np, "feedcount", &(pinfo->feedcnt))) {
		pinfo->feedcnt = -1;
	} else {
		init_timer(&pinfo->wd_timer);
		pinfo->wd_timer.data = (unsigned long)pinfo;
		pinfo->wd_timer.function = max6746_timer;
		pinfo->feedms = max6746_wdd.timeout * 1000;
		pinfo->wd_timer.expires = jiffies + msecs_to_jiffies(pinfo->feedms);
		add_timer(&(pinfo->wd_timer));
		INIT_WORK(&(pinfo->wd_work), feed_work);
	}
	pinfo->userstarted = 0;

	feed_max6746(pinfo);
	wdt_enable(pinfo, false);

	dev_info(&pdev->dev, "initialized watchdog with heartbeat %ds\n",
			max6746_wdd.timeout);


out:
	if (0 != ret) {
		
		if (!IS_ERR_OR_NULL(pinfo)) {
			kfree(pinfo);
		}
	}
	return ret;
}

static int max6746_wdt_remove(struct platform_device *pdev)
{
	struct max6746_wdt_info *pinfo = watchdog_get_drvdata(&max6746_wdd);

	if (pinfo->feedcnt > 0) {
		del_timer_sync(&pinfo->wd_timer);
		pinfo->feedcnt = 0;
	}

	watchdog_unregister_device(&max6746_wdd);
	kfree(watchdog_get_drvdata(&max6746_wdd));
	return 0;
}

static const struct of_device_id max6746_dt_ids[] = {
	{ .compatible = "maxsim,max6746", },
	{ }
};

static struct platform_driver max6746_wdt_driver = {
	.driver = {
		.name = "max6746_wdt",
		.of_match_table = max6746_dt_ids,
	},
	.probe = max6746_wdt_probe,
	.remove = max6746_wdt_remove,
};

static int __init mxs6746_init(void)
{
	return platform_driver_register(&max6746_wdt_driver);
}

static void __exit mxs6746_exit(void)
{
	platform_driver_unregister(&max6746_wdt_driver);
}

module_init(mxs6746_init);

module_exit(mxs6746_exit);

