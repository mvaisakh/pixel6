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

#define I2C_STATE_OFF_STRING	"off_i2c"
#define I2C_STATE_ON_STRING	"on_i2c"

/*
 *  LWIS I2C Defines
 */


/*
 *  LWIS I2C Structures
 */


struct lwis_i2c {
	struct i2c_client *pclient;
	struct pinctrl *ppc;
};

/*
 *  lwis_i2c_initialize: Initialize the lwis_i2c structure.
 */
int lwis_i2c_initialize(struct lwis_i2c *pi2c);

/*
 *  lwis_i2c_set_state: Enable or disable the i2c device.
 *  NOTE: state_str must match the pinctrl-names defined in the i2c driver.
 */
int lwis_i2c_set_state(struct lwis_i2c *pi2c, const char *state_str);

int lwis_i2c_read(struct lwis_i2c *pi2c, u16 addr, u8 *rbuf, u32 bufsize);

int lwis_i2c_write(struct lwis_i2c *pi2c, u16 addr, u8 *wbuf, u32 bufsize);

#endif  /* LWIS_I2C_H_ */
