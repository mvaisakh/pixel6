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

#include <linux/gpio/consumer.h>

enum lwis_gpio_dir { LWIS_GPIO_INPUT = 0, LWIS_GPIO_OUTPUT };

/*
 *  LWIS GPIO Interface Functions
 */

/*
 *  lwis_gpio_get_list: acquire GPIO descriptors.
 */
struct gpio_descs *lwis_gpio_get_list(struct device *dev, const char *name);

/*
 *  lwis_gpio_put_list: release GPIO descriptors.
 */
void lwis_gpio_put_list(struct gpio_descs *gpios, struct device *dev);

/*
 *  lwis_gpio_set_direction: Set input/output direction for a particular GPIO.
 */
int lwis_gpio_set_direction(struct gpio_desc *gpio, enum lwis_gpio_dir dir,
			    int init_value);

/*
 *  lwis_gpio_set_direction_all: Set input/output direction for all the GPIOs
 *  in the list.
 */
int lwis_gpio_set_direction_all(struct gpio_descs *gpios,
				enum lwis_gpio_dir dir, int init_value);

/*
 *  lwis_gpio_set_value: (Output GPIO only) Set output value for GPIO.  This
 *  function takes active high/low into consideration already, so 0 means
 *  deasserted, 1 means asserted.
 */
int lwis_gpio_set_value(struct gpio_desc *gpio, int value);

/*
 *  lwis_gpio_set_value_all: (Output GPIOs only) Set output value for all the
 *  GPIOs in the list.  This function takes active high/low into consideration
 *  already, so 0 means deasserted, 1 means asserted.
 */
int lwis_gpio_set_value_all(struct gpio_descs *gpios, int value);

#endif /* LWIS_GPIO_H_ */
