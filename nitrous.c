// SPDX-License-Identifier: GPL-2.0
/*
 * Bluetooth low power control via GPIO
 *
 * Copyright 2015-2020 Google LLC.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/rfkill.h>

struct nitrous_bt_lpm {
	struct gpio_desc *gpio_dev_wake;     /* Host -> Dev WAKE GPIO */
	struct gpio_desc *gpio_host_wake;    /* Dev -> Host WAKE GPIO */
	struct gpio_desc *gpio_power;        /* GPIO to control power */
	int wake_polarity;           /* 0: active low; 1: active high */

	struct device *dev;
	struct rfkill *rfkill;
	bool rfkill_blocked;         /* blocked: OFF; not blocked: ON */
};

/*
 * Set BT power on/off (blocked is true: OFF; blocked is false: ON)
 */
static int nitrous_rfkill_set_power(void *data, bool blocked)
{
	struct nitrous_bt_lpm *lpm = data;

	if (!lpm) {
		pr_err("[BT] %s: missing lpm\n", __func__);
		return -EINVAL;
	}

	pr_info("[BT] %s: %s (blocked=%d)\n", __func__, blocked ? "off" : "on",
		blocked);

	if (blocked == lpm->rfkill_blocked) {
		pr_info("[BT] %s(%s) already in requested state\n", __func__,
			blocked ? "off" : "on");
		return 0;
	}

	if (!blocked) {
		/* Power up the BT chip. delay between consecutive toggles. */
		pr_debug("[BT] REG_ON: Low");
		gpiod_set_value_cansleep(lpm->gpio_power, false);
		msleep(30);
		pr_debug("[BT] REG_ON: High");
		gpiod_set_value_cansleep(lpm->gpio_power, true);

		pr_debug("[BT] DEV_WAKE: High");
		gpiod_set_value_cansleep(lpm->gpio_dev_wake, true);
	} else {
		pr_debug("[BT] DEV_WAKE: Low");
		gpiod_set_value_cansleep(lpm->gpio_dev_wake, false);

		/* Power down the BT chip */
		pr_debug("[BT] REG_ON: Low");
		gpiod_set_value_cansleep(lpm->gpio_power, false);
	}
	lpm->rfkill_blocked = blocked;

	return 0;
}

static const struct rfkill_ops nitrous_rfkill_ops = {
	.set_block = nitrous_rfkill_set_power,
};

static int nitrous_rfkill_init(struct nitrous_bt_lpm *lpm)
{
	int rc;

	lpm->gpio_power = devm_gpiod_get_optional(lpm->dev, "shutdown", GPIOD_OUT_LOW);
	if (IS_ERR(lpm->gpio_power))
		return PTR_ERR(lpm->gpio_power);

	lpm->rfkill = rfkill_alloc(
		"nitrous_bluetooth",
		lpm->dev,
		RFKILL_TYPE_BLUETOOTH,
		&nitrous_rfkill_ops,
		lpm
	);
	if (unlikely(!lpm->rfkill))
		return -ENOMEM;

	/* Make sure rfkill core is initialized to be blocked initially. */
	rfkill_init_sw_state(lpm->rfkill, true);
	rc = rfkill_register(lpm->rfkill);
	if (unlikely(rc))
		goto err_rfkill_register;

	/* Power off chip at startup. */
	nitrous_rfkill_set_power(lpm, true);
	return 0;

err_rfkill_register:
	rfkill_destroy(lpm->rfkill);
	lpm->rfkill = NULL;
	return rc;
}

static void nitrous_rfkill_cleanup(struct nitrous_bt_lpm *lpm)
{
	nitrous_rfkill_set_power(lpm, true);
	rfkill_unregister(lpm->rfkill);
	rfkill_destroy(lpm->rfkill);
	lpm->rfkill = NULL;
}

static int nitrous_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nitrous_bt_lpm *lpm;
	int rc = 0;

	pr_debug("[BT] %s\n", __func__);

	lpm = devm_kzalloc(dev, sizeof(struct nitrous_bt_lpm), GFP_KERNEL);
	if (!lpm)
		return -ENOMEM;

	lpm->dev = dev;

	if (device_property_read_u32(dev, "wake-polarity", &lpm->wake_polarity)) {
		pr_warn("[BT] Wake polarity not in dev tree\n");
		lpm->wake_polarity = 1;
	}

	lpm->gpio_dev_wake = devm_gpiod_get_optional(dev, "device-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(lpm->gpio_dev_wake))
		return PTR_ERR(lpm->gpio_dev_wake);

	lpm->gpio_host_wake = devm_gpiod_get_optional(dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(lpm->gpio_host_wake))
		return PTR_ERR(lpm->gpio_host_wake);

	rc = nitrous_rfkill_init(lpm);
	if (unlikely(rc))
		goto err_rfkill_init;

	platform_set_drvdata(pdev, lpm);

	return rc;

err_rfkill_init:
	nitrous_rfkill_cleanup(lpm);
	devm_kfree(dev, lpm);
	return rc;
}

static int nitrous_remove(struct platform_device *pdev)
{
	struct nitrous_bt_lpm *lpm = platform_get_drvdata(pdev);

	if (!lpm) {
		pr_err("[BT] %s: missing lpm\n", __func__);
		return -EINVAL;
	}

	nitrous_rfkill_cleanup(lpm);

	devm_kfree(&pdev->dev, lpm);

	return 0;
}

static struct of_device_id nitrous_match_table[] = {
	{ .compatible = "goog,nitrous" },
	{}
};

static struct platform_driver nitrous_platform_driver = {
	.probe = nitrous_probe,
	.remove =  nitrous_remove,
	.driver = {
		.name = "nitrous_bluetooth",
		.owner = THIS_MODULE,
		.of_match_table = nitrous_match_table,
		.pm = NULL,
	},
};

static int __init nitrous_init(void)
{
	return platform_driver_register(&nitrous_platform_driver);
}

static void __exit nitrous_exit(void)
{
	platform_driver_unregister(&nitrous_platform_driver);
}

module_init(nitrous_init);
module_exit(nitrous_exit);
MODULE_DESCRIPTION("Nitrous Oxide Driver for Bluetooth");
MODULE_AUTHOR("Google");
MODULE_LICENSE("GPL");
