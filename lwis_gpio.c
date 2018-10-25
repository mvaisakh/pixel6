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

struct gpio_descs *lwis_gpio_get_list(struct device *dev, const char *name)
{
	/* By default, the GPIO pins are acquired but uninitialized */
	return devm_gpiod_get_array(dev, name, GPIOD_ASIS);
}

void lwis_gpio_put_list(struct gpio_descs *gpios, struct device *dev)
{
	devm_gpiod_put_array(dev, gpios);
}

int lwis_gpio_set_direction(struct gpio_desc *gpio, enum lwis_gpio_dir dir,
			    int init_value)
{
	if (dir == LWIS_GPIO_INPUT) {
		return gpiod_direction_input(gpio);
	}

	return gpiod_direction_output(gpio, init_value);
}

int lwis_gpio_set_direction_all(struct gpio_descs *gpios,
				enum lwis_gpio_dir dir, int init_value)
{
	int i;
	int ret;

	if (!gpios) {
		return -EINVAL;
	}

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = lwis_gpio_set_direction(gpios->desc[i], dir, init_value);
		if (ret) {
			pr_err("Failed to set direction for GPIO %d\n", i);
			return ret;
		}
	}

	return 0;
}

int lwis_gpio_set_value(struct gpio_desc *gpio, int value)
{
	if (gpiod_get_direction(gpio) == GPIOF_DIR_IN) {
		return -EINVAL;
	}

	gpiod_set_value(gpio, value);

	return 0;
}

int lwis_gpio_set_value_all(struct gpio_descs *gpios, int value)
{
	int i;
	int ret;

	if (!gpios) {
		return -EINVAL;
	}

	for (i = 0; i < gpios->ndescs; ++i) {
		ret = lwis_gpio_set_value(gpios->desc[i], value);
		if (ret) {
			pr_err("Failed to set value for GPIO %d\n", i);
			return ret;
		}
	}

	return 0;
}
