/*
 * Google LWIS Dynamic Power Managerment
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DPM_DEVICE_H_
#define LWIS_DPM_DEVICE_H_

#include "lwis_commands.h"
#include "lwis_device.h"

enum clock_family {
	CLOCK_FAMILY_INVALID = -1,
	CLOCK_FAMILY_CAM,
	CLOCK_FAMILY_INTCAM,
	CLOCK_FAMILY_TNR,
	CLOCK_FAMILY_MAX
};

/*
 *  struct lwis_dpm_device
 *  The device majorly control/handle requests from dpm clients.
 */
struct lwis_dpm_device {
	struct lwis_device base_dev;
};

/*
 *  lwis_dpm_update_clock: update clock setting to lwis device.
 *  clk_settings needs to be freed on the end of this function.
 */
int lwis_dpm_update_clock(struct lwis_device *lwis_dev,
			  struct lwis_clk_setting *clk_settings,
			  size_t num_settings);

int lwis_dpm_device_deinit(void);

#endif /* LWIS_DPM_DEVICE_H_ */
