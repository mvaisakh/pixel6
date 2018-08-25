/*
 * Google LWIS IOCTL Commands and Data Structures
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_COMMANDS_H_
#define LWIS_COMMANDS_H_

#ifdef __KERNEL__
#include <linux/ioctl.h>
#include <linux/types.h>
#else
#include <stdint.h>
#include <sys/ioctl.h>
#endif  /* __KERNEL__ */

/*
 *  IOCTL Types and Data Structures
 */

struct lwis_device_info
{
	int dummy0;
	int dummy1;
	int dummy2;
};

struct lwis_sensor_info
{
	int width;
	int height;
};

struct lwis_buffer
{
	int64_t id;
	int64_t size;
	int64_t handle;
};

/*
 *  IOCTL Commands
 */

#define LWIS_IOC_TYPE		'L'

#define LWIS_GET_DEVICE_INFO	_IOWR(LWIS_IOC_TYPE, 1, struct lwis_device_info)
#define LWIS_ENROLL_BUFFER	_IOWR(LWIS_IOC_TYPE, 2, struct lwis_buffer)
#define LWIS_SENSOR_INIT	_IOWR(LWIS_IOC_TYPE, 10, struct lwis_sensor_info)

#endif  /* LWIS_COMMANDS_H_ */