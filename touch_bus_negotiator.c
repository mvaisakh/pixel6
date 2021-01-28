// SPDX-License-Identifier: GPL-2.0
/*
 * Touch Bus Negotiator for Google Pixel devices.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/module.h>
#include <linux/net.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include "touch_bus_negotiator.h"


enum tbn_operation {
	AP_RELEASE_BUS,
	AP_REQUEST_BUS,
};

static irqreturn_t tbn_aoc2ap_irq_thread(int irq, void *ptr)
{
	struct tbn_context *tbn = ptr;

	if (completion_done(&tbn->bus_released) && completion_done(&tbn->bus_requested))
		return IRQ_HANDLED;

	/*
	 * For bus release, there two possibilities:
	 * 1. aoc2ap gpio value already changed to AOC
	 * 2. tbn_release_bus() with TBN_RELEASE_BUS_TIMEOUT_MS timeout
	 *    for complete_all(&tbn->bus_released);
	 */
	while (!completion_done(&tbn->bus_released)) {
		if (gpio_get_value(tbn->aoc2ap_gpio) == TBN_BUS_OWNER_AOC)
			complete_all(&tbn->bus_released);
		else
			usleep_range(10000, 10000);	/* wait 10 ms for gpio stablized */
	}

	/*
	 * For bus request, there two possibilities:
	 * 1. aoc2ap gpio value already changed to AP
	 * 2. tbn_request_bus() with TBN_REQUEST_BUS_TIMEOUT_MS timeout
	 *    for complete_all(&tbn->bus_requested);
	 */
	while (!completion_done(&tbn->bus_requested)) {
		if (gpio_get_value(tbn->aoc2ap_gpio) == TBN_BUS_OWNER_AP)
			complete_all(&tbn->bus_requested);
		else
			usleep_range(10000, 10000);	/* wait 10 ms for gpio stablized */
	}

	return IRQ_HANDLED;
}

int tbn_handshaking(struct tbn_context *tbn, enum tbn_operation operation)
{
	struct completion *wait_for_completion;
	enum tbn_bus_owner bus_owner;
	unsigned int irq_type;
	unsigned int timeout;
	const char *msg;

	if (!tbn || !tbn->connected)
		return 0;

	if (operation == AP_REQUEST_BUS) {
		wait_for_completion = &tbn->bus_requested;
		bus_owner = TBN_BUS_OWNER_AP;
		irq_type = IRQF_TRIGGER_FALLING;
		timeout = TBN_REQUEST_BUS_TIMEOUT_MS;
		msg = "request";
	} else {
		wait_for_completion = &tbn->bus_released;
		bus_owner = TBN_BUS_OWNER_AOC;
		irq_type = IRQF_TRIGGER_RISING;
		timeout = TBN_RELEASE_BUS_TIMEOUT_MS;
		msg = "release";
	}

	reinit_completion(wait_for_completion);

	if (tbn->mode == TBN_MODE_GPIO) {
		irq_set_irq_type(tbn->aoc2ap_irq, irq_type);
		enable_irq(tbn->aoc2ap_irq);
		gpio_direction_output(tbn->ap2aoc_gpio, bus_owner);
		if (wait_for_completion_timeout(wait_for_completion,
						msecs_to_jiffies(timeout)) == 0) {
			dev_err(tbn->dev, "AP %s bus ... timeout!\n", msg);
			complete_all(wait_for_completion);
		} else
			dev_info(tbn->dev, "AP %s bus ... SUCCESS!\n", msg);
		disable_irq_nosync(tbn->aoc2ap_irq);
	}

	return 0;
}

int tbn_request_bus(struct tbn_context *tbn)
{
	return tbn_handshaking(tbn, AP_REQUEST_BUS);
}
EXPORT_SYMBOL_GPL(tbn_request_bus);

int tbn_release_bus(struct tbn_context *tbn)
{
	return tbn_handshaking(tbn, AP_RELEASE_BUS);
}
EXPORT_SYMBOL_GPL(tbn_release_bus);

struct tbn_context *tbn_init(struct device *dev)
{
	int err = 0;
	struct tbn_context *tbn = NULL;
	struct device_node *np = dev->of_node;

	tbn = devm_kzalloc(dev, sizeof(struct tbn_context), GFP_KERNEL);
	if (!tbn)
		goto failed;

	tbn->dev = dev;

	if (of_property_read_bool(np, "tbn,ap2aoc_gpio") &&
		of_property_read_bool(np, "tbn,aoc2ap_gpio")) {
		tbn->mode = TBN_MODE_GPIO;

		tbn->ap2aoc_gpio = of_get_named_gpio(np, "tbn,ap2aoc_gpio", 0);
		if (gpio_is_valid(tbn->ap2aoc_gpio)) {
			err = devm_gpio_request_one(tbn->dev, tbn->ap2aoc_gpio,
				GPIOF_OUT_INIT_LOW, "tbn,ap2aoc_gpio");
			if (err) {
				dev_err(tbn->dev, "%s: Unable to request ap2aoc_gpio %d, err %d!\n",
					__func__, tbn->ap2aoc_gpio, err);
				goto failed;
			}
		}

		tbn->aoc2ap_gpio = of_get_named_gpio(np, "tbn,aoc2ap_gpio", 0);
		if (gpio_is_valid(tbn->aoc2ap_gpio)) {
			err = devm_gpio_request_one(tbn->dev, tbn->aoc2ap_gpio,
				GPIOF_DIR_IN, "tbn,aoc2ap_gpio");
			if (err) {
				dev_err(tbn->dev, "%s: Unable to request aoc2ap_gpio %d, err %d!\n",
					__func__, tbn->aoc2ap_gpio, err);
				goto failed;
			}
			tbn->aoc2ap_irq = gpio_to_irq(tbn->aoc2ap_gpio);
			err = devm_request_threaded_irq(tbn->dev,
				tbn->aoc2ap_irq, NULL,
				tbn_aoc2ap_irq_thread,
				IRQF_TRIGGER_RISING |
				IRQF_ONESHOT, "tbn", tbn);
			if (err) {
				dev_err(tbn->dev,
					"%s: Unable to request_threaded_irq, err %d!\n",
					__func__, err);
				goto failed;
			}
			disable_irq_nosync(tbn->aoc2ap_irq);
		} else {
			dev_err(tbn->dev, "%s: invalid aoc2ap_gpio %d!\n",
				__func__, tbn->aoc2ap_gpio);
			goto failed;
		}

		tbn->connected = true;
	} else {
		tbn->mode = TBN_MODE_DISABLED;
		tbn->connected = false;
	}

	init_completion(&tbn->bus_requested);
	init_completion(&tbn->bus_released);
	complete_all(&tbn->bus_requested);
	complete_all(&tbn->bus_released);

	dev_info(tbn->dev,
		"%s: gpios(aoc2ap: %d ap2aoc: %d), mode %d\n",
		__func__, tbn->aoc2ap_gpio, tbn->ap2aoc_gpio, tbn->mode);

	dev_dbg(tbn->dev, "bus negotiator initialized: %pK\n", tbn);

	return tbn;

failed:
	return NULL;
}
EXPORT_SYMBOL_GPL(tbn_init);

void tbn_cleanup(struct tbn_context *tbn)
{
	if (!tbn)
		return;

	dev_dbg(tbn->dev, "destructing bus negotiator: %pK\n", tbn);
}
EXPORT_SYMBOL_GPL(tbn_cleanup);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Touch Bus Negotiator");
MODULE_AUTHOR("Google, Inc.");
