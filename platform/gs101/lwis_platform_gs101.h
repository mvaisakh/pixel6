/*
 * Google LWIS GS101 Platform-Specific Functions
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_PLATFORM_GS101_H_
#define LWIS_PLATFORM_GS101_H_

#include <linux/pm_qos.h>

struct lwis_platform {
	/* TODO(b/157514330): Refactor this to be dynamically controlled
	 * from userspace */
	struct pm_qos_request pm_qos_int_cam;
	struct pm_qos_request pm_qos_int;
	struct pm_qos_request pm_qos_cam;
	struct pm_qos_request pm_qos_mem;
	struct pm_qos_request pm_qos_hpg;
	struct pm_qos_request pm_qos_tnr;
};

#endif /* LWIS_PLATFORM_GS101_H_ */