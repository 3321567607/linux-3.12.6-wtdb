
#ifdef CUSTOMER_HW
#include <osl.h>
#include <dngl_stats.h>
#include <hnd_pktq.h>
#include <dhd.h>

#ifdef CONFIG_MACH_ODROID_4210
#include <mach/gpio.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <plat/sdhci.h>
#include <plat/devs.h>	// modifed plat-samsung/dev-hsmmcX.c EXPORT_SYMBOL(s3c_device_hsmmcx) added
#define	sdmmc_channel	s3c_device_hsmmc0
#endif

#ifdef CONFIG_SOC_IMX28
#include <linux/gpio.h>
#define WL_REG_ON   243
#endif


struct resource dhd_wlan_resources = {0};
struct wifi_platform_data dhd_wlan_control = {0};

#ifdef CUSTOMER_OOB
uint bcm_wlan_get_oob_irq(void)
{
	uint host_oob_irq = 0;

#ifdef CONFIG_MACH_ODROID_4210
	printk("GPIO(WL_HOST_WAKE) = EXYNOS4_GPX0(7) = %d\n", EXYNOS4_GPX0(7));
	host_oob_irq = gpio_to_irq(EXYNOS4_GPX0(7));
	gpio_direction_input(EXYNOS4_GPX0(7));
#endif
	printk("host_oob_irq: %d \r\n", host_oob_irq);

	return host_oob_irq;
}

uint bcm_wlan_get_oob_irq_flags(void)
{
	uint host_oob_irq_flags = 0;

#ifdef CONFIG_MACH_ODROID_4210
#ifdef HW_OOB
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE;
#else
	host_oob_irq_flags = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE | IORESOURCE_IRQ_SHAREABLE;
#endif
#endif
	printk("host_oob_irq_flags=%d\n", host_oob_irq_flags);

	return host_oob_irq_flags;
}
#endif

int bcm_wlan_set_power(bool on)
{
	int err = 0;

	if (on) {
#ifdef CONFIG_SOC_IMX28
			if (0 == (err = gpio_request(WL_REG_ON, "bcm dongle"))) {
				gpio_direction_output(WL_REG_ON, 0);
				mdelay(1);
				gpio_direction_output(WL_REG_ON, 1);	/* inform wifi-IC to power on */
				gpio_free(WL_REG_ON);
				printk("======== PULL WL_REG_ON HIGH! ========\n");
				mdelay(300);
			} else {
				printk("======== Cannot request WL_REG_ON ========\n");
			}

#elif defined(CONFIG_MACH_ODROID_4210)
		err = gpio_set_value(EXYNOS4_GPK1(0), 1);
#endif
		/* Lets customer power to get stable */
		//mdelay(100);
	} else {
		printk("======== PULL WL_REG_ON LOW! ========\n");
#ifdef CONFIG_SOC_IMX28
		if (0 == (err = gpio_request(WL_REG_ON, "bcm dongle"))) {
			gpio_set_value_cansleep(WL_REG_ON, 0);	/* inform wifi-IC to power off */
			gpio_free(WL_REG_ON);
		}
#elif defined(CONFIG_MACH_ODROID_4210)
		err = gpio_set_value(EXYNOS4_GPK1(0), 0);
#endif
	}

	return err;
}

extern void mxs_mmc_force_present(int present);
int bcm_wlan_set_carddetect(bool present)
{
	int err = 0;

	if (present) {
		printk("======== Card detection to detect SDIO card! ========\n");
#ifdef CONFIG_SOC_IMX28
		mxs_mmc_force_present(1);
#elif defined(CONFIG_MACH_ODROID_4210)
		err = sdhci_s3c_force_presence_change(&sdmmc_channel, 1);
#endif
	} else {
		printk("======== Card detection to remove SDIO card! ========\n");
#ifdef CONFIG_SOC_IMX28
		mxs_mmc_force_present(0);
#elif defined(CONFIG_MACH_ODROID_4210)
		err = sdhci_s3c_force_presence_change(&sdmmc_channel, 0);
#endif
	}

	return err;
}

#ifdef CONFIG_DHD_USE_STATIC_BUF
extern void *bcmdhd_mem_prealloc(int section, unsigned long size);
void* bcm_wlan_prealloc(int section, unsigned long size)
{
	void *alloc_ptr = NULL;
	alloc_ptr = bcmdhd_mem_prealloc(section, size);
	if (alloc_ptr) {
		printk("success alloc section %d, size %ld\n", section, size);
		if (size != 0L)
			bzero(alloc_ptr, size);
		return alloc_ptr;
	}
	printk("can't alloc section %d\n", section);
	return NULL;
}
#endif

int bcm_wlan_set_plat_data(void) {
	printk("======== %s ========\n", __FUNCTION__);
	dhd_wlan_control.set_power = bcm_wlan_set_power;
	dhd_wlan_control.set_carddetect = bcm_wlan_set_carddetect;
#ifdef CONFIG_DHD_USE_STATIC_BUF
	dhd_wlan_control.mem_prealloc = bcm_wlan_prealloc;
#endif
	return 0;
}

#endif /* CUSTOMER_HW */
