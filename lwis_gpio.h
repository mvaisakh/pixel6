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

/*
 *  LWIS GPIO Interface Functions
 */

/* debug function */
void lwis_gpio_list_print(char *name, struct gpio_descs *gpios);

/*
 *  Acquire GPIO descriptors.
 */
struct gpio_descs *lwis_gpio_list_get(struct device *dev, const char *name);

/*
 *  Release GPIO descriptors.
 */
void lwis_gpio_list_put(struct gpio_descs *gpios, struct device *dev);

/*
 *  Set output value for all the GPIOs in the list.  This function takes active
 *  high/low into consideration already, i.e. 0 = deasserted, 1 = asserted.
 */
int lwis_gpio_list_set_output_value(struct gpio_descs *gpios, int value);

/*
 *  Set output value for all the GPIOs in the list.  This function ignores the
 *  active-low or open drain property of a GPIO and work on the raw line value,
 *  i.e. 0 = physical line low, 1 = physical line high.
 */
int lwis_gpio_list_set_output_value_raw(struct gpio_descs *gpios, int value);

/*
 *  Set all the GPIO pins in the list to input.
 */
int lwis_gpio_list_set_input(struct gpio_descs *gpios);

#endif /* LWIS_GPIO_H_ */
