/*
 * Google LWIS GPIO Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_GPIO_H_
#define LWIS_GPIO_H_

#include <linux/gpio.h>

/*
 *  LWIS GPIO Defines
 */

enum lwis_gpio_pin_level {
	LWIS_GPIO_PIN_INACTIVE = 0,
	LWIS_GPIO_PIN_ACTIVE
};

/*
 *  LWIS GPIO Structures
 */

struct lwis_gpio {
	int pin;
	bool is_active_high;
};

struct lwis_gpio_list {
	struct lwis_gpio *gpio;
	int count;
};

/*
 *  LWIS GPIO Interface Functions
 */

/*
 *  lwis_gpio_list_alloc: Allocate an instance of the lwis_gpio_list and
 *  initialize the data structures according to the number of GPIOs
 *  specified.
 *  NOTE: This does not register the GPIO pins.
 */
struct lwis_gpio_list *lwis_gpio_list_alloc(int num_gpios);

/*
 *  lwis_gpio_list_free: Deallocate the instance of lwis_gpio_list.
 */
void lwis_gpio_list_free(struct lwis_gpio_list *list);

/*
 *  lwis_gpio_set: Register the GPIO pin and polarity.
 */
int lwis_gpio_set(struct lwis_gpio_list *list, int index, int pin,
		  bool is_active_high);

/*
 *  lwis_gpio_pin_set_level: Set a particular GPIO pin to the specified
 *  active level.
 */
int lwis_gpio_pin_set_level(struct lwis_gpio_list *list, int index,
			    enum lwis_gpio_pin_level level);

/*
 *  lwis_gpio_pin_set_level_all: Set all GPIO pins to the specified
 *  active level.
 */
int lwis_gpio_pin_set_level_all(struct lwis_gpio_list *list,
				enum lwis_gpio_pin_level level);

/*  lwis_gpio_print: Debug function to print all the GPIO pins in the
 *  supplied list structure.
 */
void lwis_gpio_print(struct lwis_gpio_list *list);

#endif  /* LWIS_GPIO_H_ */
