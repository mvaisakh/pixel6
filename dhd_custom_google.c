/*
 * Platform Dependent file
 *
 * Copyright (C) 1999-2018, Broadcom.
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id$
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/of_gpio.h>
#ifdef DHD_COREDUMP
#include <linux/platform_data/sscoredump.h>
#endif /* DHD_COREDUMP */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
extern int dhd_init_wlan_mem(void);
extern void *dhd_wlan_mem_prealloc(int section, unsigned long size);
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#define WIFI_TURNON_DELAY		200
#define WLAN_REG_ON_GPIO		491
#define WLAN_HOST_WAKE_GPIO		493

static int wlan_reg_on = -1;
#define DHD_DT_COMPAT_ENTRY		"android,bcmdhd_wlan"
#define WIFI_WL_REG_ON_PROPNAME		"wl_reg_on"

static int wlan_host_wake_up = -1;
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
static int wlan_host_wake_irq = 0;
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */
#define WIFI_WLAN_HOST_WAKE_PROPNAME    "wl_host_wake"

#if defined(CONFIG_SOC_EXYNOS9810) || defined(CONFIG_SOC_EXYNOS9820) || \
	defined(CONFIG_SOC_GS101)
#define EXYNOS_PCIE_RC_ONOFF
int pcie_ch_num = -1;
extern void exynos_pcie_pm_resume(int);
extern void exynos_pcie_pm_suspend(int);
#endif /* CONFIG_SOC_EXYNOS9810 || CONFIG_SOC_EXYNOS9820 || CONFIG_SOC_GS101 */

#ifdef DHD_COREDUMP
#define DEVICE_NAME "wlan"

static void sscd_release(struct device *dev);
static struct sscd_platform_data sscd_pdata;
static struct platform_device sscd_dev = {
	.name            = DEVICE_NAME,
	.driver_override = SSCD_NAME,
	.id              = -1,
	.dev             = {
		.platform_data = &sscd_pdata,
		.release       = sscd_release,
	},
};

static void sscd_release(struct device *dev)
{
	printk(KERN_INFO "%s: enter", __FUNCTION__);
}

/* trigger coredump */
static int
dhd_set_coredump(const char *buf, int buf_len)
{
	struct sscd_platform_data *pdata = dev_get_platdata(&sscd_dev.dev);
	struct sscd_segment seg;

	if (pdata->sscd_report) {
		memset(&seg, 0, sizeof(seg));
		seg.addr = (void *) buf;
		seg.size = buf_len;
		pdata->sscd_report(&sscd_dev, &seg, 1, 0, "wlan_crash");
	}
	return 0;
}
#endif /* DHD_COREDUMP */

int
dhd_wifi_init_gpio(void)
{
	int gpio_reg_on_val;
	/* ========== WLAN_PWR_EN ============ */
	char *wlan_node = DHD_DT_COMPAT_ENTRY;
	struct device_node *root_node = NULL;

	root_node = of_find_compatible_node(NULL, NULL, wlan_node);
	if (!root_node) {
		printk(KERN_ERR "failed to get device node of BRCM WLAN");
		return -ENODEV;
	}

	wlan_reg_on = of_get_named_gpio(root_node, WIFI_WL_REG_ON_PROPNAME, 0);
	if (!gpio_is_valid(wlan_reg_on)) {
		printk(KERN_ERR "Invalid gpio pin : %d\n", wlan_reg_on);
		return -ENODEV;
	}

	/* ========== WLAN_PWR_EN ============ */
	printk(KERN_INFO "%s: gpio_wlan_power : %d\n", __FUNCTION__, wlan_reg_on);

#ifdef EXYNOS_PCIE_RC_ONOFF
	if (of_property_read_u32(root_node, "ch-num", &pcie_ch_num)) {
		printk(KERN_INFO "%s: Failed to parse the channel number\n", __FUNCTION__);
		return -EINVAL;
	}
	/* ========== WLAN_PCIE_NUM ============ */
	printk(KERN_INFO "%s: pcie_ch_num : %d\n", __FUNCTION__, pcie_ch_num);
#endif /* EXYNOS_PCIE_RC_ONOFF */

	/*
	 * For reg_on, gpio_request will fail if the gpio is configured to output-high
	 * in the dts using gpio-hog, so do not return error for failure.
	 */
	if (gpio_request_one(wlan_reg_on, GPIOF_OUT_INIT_HIGH, "WL_REG_ON")) {
		printk(KERN_ERR "%s: Failed to request gpio %d for WL_REG_ON, "
			"might have configured in the dts\n",
			__FUNCTION__, wlan_reg_on);
	} else {
		printk(KERN_ERR "%s: gpio_request WL_REG_ON done - WLAN_EN: GPIO %d\n",
			__FUNCTION__, wlan_reg_on);
	}

	gpio_reg_on_val = gpio_get_value(wlan_reg_on);
	printk(KERN_INFO "%s: Initial WL_REG_ON: [%d]\n",
		__FUNCTION__, gpio_get_value(wlan_reg_on));

	if (gpio_reg_on_val == 0) {
		printk(KERN_INFO "%s: WL_REG_ON is LOW, drive it HIGH\n", __FUNCTION__);
		if (gpio_direction_output(wlan_reg_on, 1)) {
			printk(KERN_ERR "%s: WL_REG_ON is failed to pull up\n", __FUNCTION__);
			return -EIO;
		}
	}

	printk(KERN_ERR "%s: WL_REG_ON is pulled up\n", __FUNCTION__);

	/* Wait for WIFI_TURNON_DELAY due to power stability */
	msleep(WIFI_TURNON_DELAY);
#ifdef EXYNOS_PCIE_RC_ONOFF
        exynos_pcie_pm_resume(pcie_ch_num);
#endif /* EXYNOS_PCIE_RC_ONOFF */
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
	/* ========== WLAN_HOST_WAKE ============ */
	wlan_host_wake_up = of_get_named_gpio(root_node,
					      WIFI_WLAN_HOST_WAKE_PROPNAME, 0);
	printk(KERN_INFO "%s: gpio_wlan_host_wake : %d\n", __FUNCTION__, wlan_host_wake_up);

	if (gpio_request_one(wlan_host_wake_up, GPIOF_IN, "WLAN_HOST_WAKE")) {
		printk(KERN_ERR "%s: Failed to request gpio %d for WLAN_HOST_WAKE\n",
			__FUNCTION__, wlan_host_wake_up);
			return -ENODEV;
	} else {
		printk(KERN_ERR "%s: gpio_request WLAN_HOST_WAKE done"
			" - WLAN_HOST_WAKE: GPIO %d\n",
			__FUNCTION__, wlan_host_wake_up);
	}

	if (gpio_direction_input(wlan_host_wake_up)) {
		printk(KERN_ERR "%s: Failed to set WL_HOST_WAKE gpio direction\n", __FUNCTION__);
	}

	wlan_host_wake_irq = gpio_to_irq(wlan_host_wake_up);
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */
	return 0;
}

int
dhd_wlan_power(int onoff)
{
	printk(KERN_INFO"------------------------------------------------");
	printk(KERN_INFO"------------------------------------------------\n");
	printk(KERN_INFO"%s Enter: power %s\n", __func__, onoff ? "on" : "off");

	if (onoff) {
		if (gpio_direction_output(wlan_reg_on, 1)) {
			printk(KERN_ERR "%s: WL_REG_ON is failed to pull up\n", __FUNCTION__);
			return -EIO;
		}
		if (gpio_get_value(wlan_reg_on)) {
			printk(KERN_INFO"WL_REG_ON on-step-2 : [%d]\n",
				gpio_get_value(wlan_reg_on));
		} else {
			printk("[%s] gpio value is 0. We need reinit.\n", __func__);
			if (gpio_direction_output(wlan_reg_on, 1)) {
				printk(KERN_ERR "%s: WL_REG_ON is "
					"failed to pull up\n", __func__);
			}
		}
#ifdef EXYNOS_PCIE_RC_ONOFF
		exynos_pcie_pm_resume(pcie_ch_num);
#endif /* EXYNOS_PCIE_RC_ONOFF */
	} else {
#ifdef EXYNOS_PCIE_RC_ONOFF
		exynos_pcie_pm_suspend(pcie_ch_num);
#endif /* EXYNOS_PCIE_RC_ONOFF */
		if (gpio_direction_output(wlan_reg_on, 0)) {
			printk(KERN_ERR "%s: WL_REG_ON is failed to pull up\n", __FUNCTION__);
			return -EIO;
		}
		if (gpio_get_value(wlan_reg_on)) {
			printk(KERN_INFO"WL_REG_ON on-step-2 : [%d]\n",
				gpio_get_value(wlan_reg_on));
		}
	}
	return 0;
}
EXPORT_SYMBOL(dhd_wlan_power);

static int
dhd_wlan_reset(int onoff)
{
	return 0;
}

static int
dhd_wlan_set_carddetect(int val)
{
	return 0;
}

#ifdef BCMSDIO
static int dhd_wlan_get_wake_irq(void)
{
	return gpio_to_irq(wlan_host_wake_up);
}
#endif /* BCMSDIO */

#if defined(CONFIG_BCMDHD_OOB_HOST_WAKE) && defined(CONFIG_BCMDHD_GET_OOB_STATE)
int
dhd_get_wlan_oob_gpio(void)
{
        return gpio_is_valid(wlan_host_wake_up) ?
                gpio_get_value(wlan_host_wake_up) : -1;
}
EXPORT_SYMBOL(dhd_get_wlan_oob_gpio);
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE && CONFIG_BCMDHD_GET_OOB_STATE */

struct resource dhd_wlan_resources = {
	.name	= "bcmdhd_wlan_irq",
	.start	= 0, /* Dummy */
	.end	= 0, /* Dummy */
	.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE |
	IORESOURCE_IRQ_HIGHEDGE,
};
EXPORT_SYMBOL(dhd_wlan_resources);

struct wifi_platform_data dhd_wlan_control = {
	.set_power	= dhd_wlan_power,
	.set_reset	= dhd_wlan_reset,
	.set_carddetect	= dhd_wlan_set_carddetect,
#ifdef DHD_COREDUMP
	.set_coredump = dhd_set_coredump,
#endif /* DHD_COREDUMP */
#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */
#ifdef BCMSDIO
	.get_wake_irq	= dhd_wlan_get_wake_irq,
#endif // endif
};
EXPORT_SYMBOL(dhd_wlan_control);

int
dhd_wlan_init(void)
{
	int ret;

	printk(KERN_INFO"%s: START.......\n", __FUNCTION__);
	ret = dhd_wifi_init_gpio();
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to initiate GPIO, ret=%d\n",
			__FUNCTION__, ret);
		goto fail;
	}
#ifdef CONFIG_BCMDHD_OOB_HOST_WAKE
	dhd_wlan_resources.start = wlan_host_wake_irq;
	dhd_wlan_resources.end = wlan_host_wake_irq;
#endif /* CONFIG_BCMDHD_OOB_HOST_WAKE */

#ifdef CONFIG_BROADCOM_WIFI_RESERVED_MEM
	ret = dhd_init_wlan_mem();
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to alloc reserved memory,"
			" ret=%d\n", __FUNCTION__, ret);
		goto fail;
	}
#endif /* CONFIG_BROADCOM_WIFI_RESERVED_MEM */

#ifdef DHD_COREDUMP
	platform_device_register(&sscd_dev);
#endif /* DHD_COREDUMP */

fail:
	printk(KERN_INFO "%s: FINISH.......\n", __FUNCTION__);
	return ret;
}

int
dhd_wlan_deinit(void)
{
	if (gpio_is_valid(wlan_host_wake_up)) {
		gpio_free(wlan_host_wake_up);
	}
	if (gpio_is_valid(wlan_reg_on)) {
		gpio_free(wlan_reg_on);
	}

#ifdef DHD_COREDUMP
	platform_device_unregister(&sscd_dev);
#endif /* DHD_COREDUMP */

	return 0;
}
#ifndef BCMDHD_MODULAR
/* Required only for Built-in DHD */
device_initcall(dhd_wlan_init);
#endif /* BCMDHD_MODULAR */
