// SPDX-License-Identifier: GPL-2.0
/*
 * Bluetooth low power control via GPIO
 *
 * Copyright 2015-2020 Google LLC.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/rfkill.h>

#define NITROUS_AUTOSUSPEND_DELAY   1000 /* autosleep delay 1000 ms */

struct nitrous_bt_lpm {
	struct pinctrl *pinctrls;
	struct pinctrl_state *pinctrl_default_state;
	struct gpio_desc *gpio_dev_wake;     /* Host -> Dev WAKE GPIO */
	struct gpio_desc *gpio_host_wake;    /* Dev -> Host WAKE GPIO */
	struct gpio_desc *gpio_power;        /* GPIO to control power */
	int irq_host_wake;           /* IRQ associated with HOST_WAKE GPIO */
	int wake_polarity;           /* 0: active low; 1: active high */

	bool is_suspended;           /* driver is in suspend state */
	bool pending_irq;            /* pending host wake IRQ during suspend */

	struct device *dev;
	struct rfkill *rfkill;
	bool rfkill_blocked;         /* blocked: OFF; not blocked: ON */
	bool lpm_enabled;
};

/*
 * Wake up or sleep BT device for Tx.
 */
static inline void nitrous_wake_controller(struct nitrous_bt_lpm *lpm, bool wake)
{
	int assert_level = (wake == lpm->wake_polarity);
	pr_debug("[BT] DEV_WAKE: %s", (assert_level ? "Assert" : "Dessert"));
	gpiod_set_value_cansleep(lpm->gpio_dev_wake, assert_level);
}

/*
 * ISR to handle host wake line from the BT chip.
 *
 * If an interrupt is received during system suspend, the handling of the
 * interrupt will be delayed until the driver is resumed.  This allows the use
 * of pm runtime framework to wake the serial driver.
 */
static irqreturn_t nitrous_host_wake_isr(int irq, void *data)
{
	struct nitrous_bt_lpm *lpm = data;

	pr_debug("[BT] Host wake IRQ: %u\n", gpiod_get_value(lpm->gpio_host_wake));
	if (lpm->rfkill_blocked) {
		pr_err("[BT] %s: Unexpected Host wake IRQ\n", __func__);
		return IRQ_HANDLED;
	}

	pm_runtime_get(lpm->dev);
	pm_runtime_mark_last_busy(lpm->dev);
	pm_runtime_put_autosuspend(lpm->dev);

	return IRQ_HANDLED;
}

static int nitrous_lpm_runtime_enable(struct nitrous_bt_lpm *lpm)
{
	int rc;

	if (lpm->irq_host_wake <= 0)
		return -EOPNOTSUPP;

	if (lpm->rfkill_blocked) {
		pr_err("[BT] Unexpected LPM request\n");
		return -EINVAL;
	}

	if (lpm->lpm_enabled) {
		pr_warn("[BT] Try to request LPM twice\n");
		return 0;
	}

	rc = devm_request_irq(lpm->dev, lpm->irq_host_wake, nitrous_host_wake_isr,
			lpm->wake_polarity ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING,
			"bt_host_wake", lpm);
	if (unlikely(rc)) {
		pr_err("[BT] Unable to request IRQ for bt_host_wake GPIO\n");
		lpm->irq_host_wake = rc;
		return rc;
	}

	device_init_wakeup(lpm->dev, true);
	pm_runtime_set_autosuspend_delay(lpm->dev, NITROUS_AUTOSUSPEND_DELAY);
	pm_runtime_use_autosuspend(lpm->dev);
	pm_runtime_set_active(lpm->dev);
	pm_runtime_enable(lpm->dev);

	bt_lpm->lpm_enabled = true;

	return rc;
}

static void nitrous_lpm_runtime_disable(struct nitrous_bt_lpm *lpm)
{
	if (lpm->irq_host_wake <= 0)
		return;

	if (!lpm->lpm_enabled)
		return;

	devm_free_irq(lpm->dev, lpm->irq_host_wake, lpm);
	device_init_wakeup(lpm->dev, false);
	pm_runtime_disable(lpm->dev);
	pm_runtime_set_suspended(lpm->dev);

	bt_lpm->lpm_enabled = false;
}

static int nitrous_lpm_init(struct nitrous_bt_lpm *lpm)
{
	lpm->irq_host_wake = gpiod_to_irq(lpm->gpio_host_wake);
	pr_info("[BT] IRQ: %d active: %s\n", lpm->irq_host_wake,
		(lpm->wake_polarity ? "High" : "Low"));
	return 0;
}

static void nitrous_lpm_cleanup(struct nitrous_bt_lpm *lpm)
{
	nitrous_lpm_runtime_disable(lpm);
	lpm->irq_host_wake = 0;
}

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

	/* Reset to make sure LPM is disabled */
	nitrous_lpm_runtime_disable(lpm);

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

	if (!lpm->rfkill_blocked)
		nitrous_lpm_runtime_enable(lpm);

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

	lpm->pinctrls = devm_pinctrl_get(lpm->dev);
	if (IS_ERR(lpm->pinctrls)) {
		pr_warn("[BT] Can't get pinctrl\n");
	} else {
		lpm->pinctrl_default_state =
			pinctrl_lookup_state(lpm->pinctrls, "default");
		if (IS_ERR(lpm->pinctrl_default_state))
			pr_warn("[BT] Can't get default pinctrl state\n");
	}

	lpm->gpio_dev_wake = devm_gpiod_get_optional(dev, "device-wakeup", GPIOD_OUT_LOW);
	if (IS_ERR(lpm->gpio_dev_wake))
		return PTR_ERR(lpm->gpio_dev_wake);

	lpm->gpio_host_wake = devm_gpiod_get_optional(dev, "host-wakeup", GPIOD_IN);
	if (IS_ERR(lpm->gpio_host_wake))
		return PTR_ERR(lpm->gpio_host_wake);

	rc = nitrous_lpm_init(lpm);
	if (unlikely(rc))
		goto err_lpm_init;

	rc = nitrous_rfkill_init(lpm);
	if (unlikely(rc))
		goto err_rfkill_init;

	if (!IS_ERR_OR_NULL(lpm->pinctrl_default_state)) {
		rc = pinctrl_select_state(lpm->pinctrls,
					  lpm->pinctrl_default_state);
		if (unlikely(rc))
			pr_warn("[BT] Can't set default pinctrl state\n");
	}

	platform_set_drvdata(pdev, lpm);

	return rc;

err_rfkill_init:
	nitrous_rfkill_cleanup(lpm);
err_lpm_init:
	nitrous_lpm_cleanup(lpm);
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
	nitrous_lpm_cleanup(lpm);

	devm_kfree(&pdev->dev, lpm);

	return 0;
}

static int nitrous_suspend_device(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);

	pr_debug("[BT] %s from %s\n", __func__,
		 (lpm->is_suspended ? "asleep" : "awake"));

	nitrous_wake_controller(lpm, false);
	lpm->is_suspended = true;

	return 0;
}

static int nitrous_resume_device(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);

	pr_debug("[BT] %s from %s\n", __func__,
		 (lpm->is_suspended ? "asleep" : "awake"));

	nitrous_wake_controller(lpm, true);
	lpm->is_suspended = false;

	return 0;
}

static int nitrous_suspend(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);

	pr_debug("[BT] %s\n", __func__);

	if (pm_runtime_active(dev))
		nitrous_suspend_device(dev);

	if (device_may_wakeup(dev) && lpm->lpm_enabled) {
		enable_irq_wake(lpm->irq_host_wake);
		pr_debug("[BT] Host wake IRQ enabled\n");
	}

	return 0;
}

static int nitrous_resume(struct device *dev)
{
	struct nitrous_bt_lpm *lpm = dev_get_drvdata(dev);

	pr_debug("[BT] %s\n", __func__);

	if (device_may_wakeup(dev) && lpm->lpm_enabled) {
		disable_irq_wake(lpm->irq_host_wake);
		pr_debug("[BT] Host wake IRQ disabled\n");
	}

	nitrous_resume_device(dev);

	return 0;
}

static struct of_device_id nitrous_match_table[] = {
	{ .compatible = "goog,nitrous" },
	{}
};

static const struct dev_pm_ops nitrous_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(nitrous_suspend, nitrous_resume)
	SET_RUNTIME_PM_OPS(nitrous_suspend_device, nitrous_resume_device, NULL)
};

static struct platform_driver nitrous_platform_driver = {
	.probe = nitrous_probe,
	.remove =  nitrous_remove,
	.driver = {
		.name = "nitrous_bluetooth",
		.owner = THIS_MODULE,
		.of_match_table = nitrous_match_table,
		.pm = &nitrous_pm_ops,
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
