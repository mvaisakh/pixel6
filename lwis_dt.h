/*
 * Google LWIS Device Tree Parser
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DT_H_
#define LWIS_DT_H_

#include <linux/device.h>

#include "lwis_device.h"

/*
 *  lwis_device_parse_dt: Parse top level device configurations, and scan
 *  for peripheral components based on device tree definitions.
 */
int lwis_device_parse_dt(struct lwis_device *pldev);

/*
 *  lwis_sensor_parse_config_dt: Parse sensor specific configurations, and
 *  obtain lists of GPIOs, clocks, regulators, etc. for the sensor.
 */
int lwis_sensor_parse_config_dt(struct device *pdev,
				struct lwis_sensor *psensor);

#endif  /* LWIS_DT_H_ */
