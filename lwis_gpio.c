/*
 * Google LWIS GPIO Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-gpio: " fmt

#include <linux/gpio.h>
#include <linux/kernel.h>

#include "lwis_gpio.h"

/* debug function */
void lwis_gpio_list_print(char *name, struct gpio_descs *gpios)
{
	int i;

	if (IS_ERR_OR_NULL(gpios)) {
		pr_info("name: %s error: %ld\n", name, PTR_ERR(gpios));
	} else {
		pr_info("name: %s, count: %d\n", name, gpios->ndescs);
		for (i = 0; i < gpios->ndescs; i++) {
			pr_info("gpio number: %d\n",
				desc_to_gpio(gpios->desc[i]));
		}
	}
}

struct gpio_descs *lwis_gpio_list_get(struct device *dev, const char *name)
{
	/* By default, the GPIO pins are acquired but uninitialized */
	return devm_gpiod_get_array(dev, name, GPIOD_ASIS);
}

void lwis_gpio_list_put(struct gpio_descs *gpios, struct device *dev)
{
	devm_gpiod_put_array(dev, gpios);
}

int lwis_gpio_list_set_output_value(struct gpio_descs *gpios, int value)
{
	int i;
	int ret;

	if (!gpios) {
		return -EINVAL;
	}

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = gpiod_direction_output(gpios->desc[i], value);
		if (ret) {
			pr_err("Failed to set value for GPIO %d\n", i);
			return ret;
		}
	}

	return 0;
}

int lwis_gpio_list_set_input(struct gpio_descs *gpios)
{
	int i;
	int ret;

	if (!gpios) {
		return -EINVAL;
	}

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = gpiod_direction_input(gpios->desc[i]);
		if (ret) {
			pr_err("Failed to set GPIO %d to input\n", i);
			return ret;
		}
	}

	return 0;
}
