/*
 * Google LWIS I2C Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_I2C_H_
#define LWIS_I2C_H_

#include <linux/i2c.h>
#include <linux/pinctrl/consumer.h>

#include "lwis_commands.h"
#include "lwis_device_i2c.h"

#define I2C_STATE_OFF_STRING "off_i2c"
#define I2C_STATE_ON_STRING "on_i2c"

/*
 *  lwis_i2c_set_state: Enable or disable the i2c device.
 *  NOTE: state_str must match the pinctrl-names defined in the i2c driver.
 *  Pinctrl states can be found in the device tree, look for the i2c entry and
 *  the state names are defined under "pinctrl-names".  Their corresponding
 *  functions are defined under "pinctrl-N".
 */
int lwis_i2c_set_state(struct lwis_i2c_device *i2c, const char *state_str);

/*
 *  lwis_i2c_read_batch: Read from i2c bus in a batch - register information
 *  is provided through lwis_io_entry.  The read back values will be stored in
 *  the entries also.
 */
int lwis_i2c_read_batch(struct lwis_i2c_device *i2c,
			struct lwis_io_entry *entries, int num_entries);

/*
 *  lwis_i2c_write_batch: Write to i2c bus in a batch - register and value
 *  information are provided through lwis_io_entry.
 */
int lwis_i2c_write_batch(struct lwis_i2c_device *i2c,
			 struct lwis_io_entry *entries, int num_entries);

/*
 *  lwis_i2c_read: Single read from i2c bus.
 */
int lwis_i2c_read(struct lwis_i2c_device *i2c, uint64_t offset,
		  uint64_t *value);

/*
 *  lwis_i2c_write: Single write to i2c bus.
 */
int lwis_i2c_write(struct lwis_i2c_device *i2c, uint64_t offset,
		   uint64_t value);

#endif /* LWIS_I2C_H_ */
