/*
 * Google LWIS Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DEVICE_H_
#define LWIS_DEVICE_H_

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#include "lwis_clock.h"
#include "lwis_gpio.h"
#include "lwis_i2c.h"
#include "lwis_regulator.h"

#define LWIS_TOP_DEVICE_COMPAT "google,lwis-top-device"
#define LWIS_I2C_DEVICE_COMPAT "google,lwis-i2c-device"
#define LWIS_MMAP_DEVICE_COMPAT "google,lwis-mmap-device"

/* Device tree strings have a maximum length of 31, according to specs.
   Adding 1 byte for the null character. */
#define MAX_DEVICE_NAME_STRING 32

/*
 * lwis_device_types
 * top : top level device that overlooks all the LWIS devices. Will be used to
 *       list the information of the other LWIS devices in the system.
 * i2c : for controlling i2c devices
 * mmap: for controlling memory mapped devices
 */
enum lwis_device_types {
	DEVICE_TYPE_UNKNOWN = -1,
	DEVICE_TYPE_TOP = 0,
	DEVICE_TYPE_I2C,
	DEVICE_TYPE_MMAP,
	NUM_DEVICE_TYPES
};

/*
 *  struct lwis_core
 *  This struct applies to all LWIS devices that are defined in the
 *  device tree.
 */
struct lwis_core {
	struct class *dev_class;
	struct idr *idr;
	struct cdev *chr_dev;
	struct mutex lock;
	int device_major;
	struct list_head lwis_dev_list;
};

/*
 *  struct lwis_device
 *  This struct applies to each of the LWIS devices, e.g. /dev/lwis*
 */
struct lwis_device {
	int id;
	enum lwis_device_types type;
	char name[MAX_DEVICE_NAME_STRING];
	struct device *dev;
	struct platform_device *plat_dev;
	struct lwis_gpio_list *reset_gpios;
	struct lwis_gpio_list *enable_gpios;
	struct lwis_regulator_list *regulators;
	struct lwis_clock_list *clocks;
	struct lwis_i2c *i2c;
	struct pinctrl *mclk_ctrl;
	struct list_head dev_list;
};

/*
 *  struct lwis_client
 *  This struct applies to each client that uses a LWIS device, i.e. each
 *  application that calls open() on a /dev/lwis* device.
 */
struct lwis_client {
	struct mutex lock;
	struct lwis_device *lwis_dev;
};

#endif /* LWIS_DEVICE_H_ */
