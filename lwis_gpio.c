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

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_gpio.h"

struct lwis_gpio_list *lwis_gpio_list_alloc(int num_gpios)
{
	struct lwis_gpio_list *list;

	if (num_gpios <= 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kzalloc(sizeof(struct lwis_gpio_list), GFP_KERNEL);
	if (!list) {
		return ERR_PTR(-ENOMEM);
	}

	list->gpio = kzalloc(num_gpios * sizeof(struct lwis_gpio), GFP_KERNEL);
	if (!list->gpio) {
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = num_gpios;

	return list;
}

void lwis_gpio_list_free(struct lwis_gpio_list *list)
{
	if (!list) {
		return;
	}

	if (list->gpio) {
		kfree(list->gpio);
	}

	kfree(list);
}

int lwis_gpio_get(struct lwis_gpio_list *list, int index, int pin,
		  bool is_active_high)
{
	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	list->gpio[index].pin = pin;
	list->gpio[index].is_active_high = is_active_high;

	return 0;
}

int lwis_gpio_pin_set_level(struct lwis_gpio_list *list, int index,
			    enum lwis_gpio_pin_level level)
{
	unsigned long flag;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	if (!gpio_is_valid(list->gpio[index].pin)) {
		return -EINVAL;
	}

	/* Truth table between level (Lvl) and is_active_high (Act)
	 *
	 *                   (Lvl)
	 *            | inactive | active
	 *  ----------+----------+--------
	 *  (Act)  0  |   HIGH   |   LOW
	 *         1  |    LOW   |  HIGH
	 */
	if ((level == LWIS_GPIO_PIN_ACTIVE &&
	     list->gpio[index].is_active_high) ||
	    (level == LWIS_GPIO_PIN_INACTIVE &&
	     !list->gpio[index].is_active_high)) {
		flag = GPIOF_OUT_INIT_HIGH;
	} else {
		flag = GPIOF_OUT_INIT_LOW;
	}

	return gpio_request_one(list->gpio[index].pin, flag,
				(flag == GPIOF_OUT_INIT_HIGH) ? "OUTPUT_HIGH"
							      : "OUTPUT_LOW");
}

int lwis_gpio_pin_set_level_all(struct lwis_gpio_list *list,
				enum lwis_gpio_pin_level level)
{
	int i;
	int ret;

	if (!list) {
		return -EINVAL;
	}

	for (i = 0; i < list->count; ++i) {
		ret = lwis_gpio_pin_set_level(list, i, level);
		if (ret) {
			pr_err("Error setting GPIO pin %d\n", i);
			return ret;
		}
	}

	return 0;
}

void lwis_gpio_print(struct lwis_gpio_list *list)
{
	int i;

	for (i = 0; i < list->count; ++i) {
		pr_info("%s: gpio: %d active high: %d\n", __func__,
			list->gpio[i].pin, list->gpio[i].is_active_high);
	}
}
