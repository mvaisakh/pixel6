/*
 * Google LWIS I2C Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DEVICE_I2C_H_
#define LWIS_DEVICE_I2C_H_

#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>

#include "lwis_device.h"

/*
 *  struct lwis_i2c_device
 *  "Derived" lwis_device struct, with added i2c related elements.
 */
struct lwis_i2c_device {
	struct lwis_device base_dev;
	int address;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct pinctrl *state_pinctrl;
};

int lwis_i2c_device_deinit(void);
#endif /* LWIS_DEVICE_I2C_H_ */
