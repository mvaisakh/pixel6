/*
 * Google LWIS Top Level Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DEVICE_TOP_H_
#define LWIS_DEVICE_TOP_H_

#include "lwis_device.h"

/*
 *  struct lwis_top_device
 *  "Derived" lwis_device struct, with added top device related elements.
 */
struct lwis_top_device {
	struct lwis_device base_dev;
};

#endif /* LWIS_DEVICE_TOP_H_ */
