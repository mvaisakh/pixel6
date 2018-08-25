/*
 * Google LWIS Sensor Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_SENSOR_H_
#define LWIS_SENSOR_H_

#include <linux/device.h>
#include <linux/i2c.h>

/*
 *  Sensor Data Structures
 */

struct lwis_sensor_ops {
	int (*init)(void);
};

struct lwis_sensor {
	struct lwis_sensor_ops *ops;
};

/*
 *  Sensor Interface Functions
 */

int lwis_sensor_init(struct lwis_sensor *psensor);

/*
 *  Sensor Helper Functions
 */

/*
 *  lwis_sensor_get_ptr: Obtain sensor instance pointer via the
 *  i2c_client struct.
 */
struct lwis_sensor *lwis_sensor_get_ptr(struct i2c_client *pclient);

#endif /* LWIS_SENSOR_H_ */
