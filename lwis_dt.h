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
 *  lwis_device_parse_dt: Parse device configurations based on device tree
 *  entries.
 */
int lwis_device_parse_dt(struct lwis_device *lwis_dev);

#endif /* LWIS_DT_H_ */
