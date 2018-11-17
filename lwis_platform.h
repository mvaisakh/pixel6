/*
 * Google LWIS Declarations of Platform-specific Functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_PLATFORM_H_
#define LWIS_PLATFORM_H_

#include "lwis_device.h"

/*
 *  lwis_platform_probe: handles platform-specific parts of
 *  device init
 */
int lwis_platform_probe(struct lwis_device *lwis_dev);

/*
 *  lwis_platform_device_enable: handles platform-specific parts of
 *  device enable
 */
int lwis_platform_device_enable(struct lwis_device *lwis_dev);

/*
 *  lwis_platform_device_disable: handles platform-specific parts of
 *  device disable
 */
int lwis_platform_device_disable(struct lwis_device *lwis_dev);

#endif /* LWIS_PLATFORM_H_ */
