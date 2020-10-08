/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU thermal driver header.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#ifndef __EDGETPU_THERMAL_H__
#define __EDGETPU_THERMAL_H__

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/thermal.h>

#define EDGETPU_COOLING_NAME "tpu_cooling"

struct edgetpu_thermal {
	struct device *dev;
	struct dentry *cooling_root;
	struct thermal_cooling_device *cdev;
	struct mutex lock;
	void *op_data;
	unsigned long cooling_state;
};

struct edgetpu_state_pwr {
	unsigned long state;
	u32 power;
};

/*
 * Creates a managed edgetpu_thermal object.
 *
 * Returns -errno on error.
 */
struct edgetpu_thermal *devm_tpu_thermal_create(struct device *dev);

#endif /* __EDGETPU_THERMAL_H__ */
