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
#endif /* __KERNEL__ */

/*
 *  IOCTL Types and Data Structures
 */

struct lwis_device_info {
};

struct lwis_buffer {
	int64_t id;
	int64_t size;
	int64_t handle;
};

struct lwis_io_data {
	int64_t offset;
	uint64_t val;
	int access_size;
};

struct lwis_io_msg {
	int bid;
	struct lwis_io_data *buf;
	int num_entries;
	int offset_bitwidth;
};

/*
 *  IOCTL Commands
 */

#define LWIS_IOC_TYPE 'L'

#define LWIS_GET_DEVICE_INFO _IOWR(LWIS_IOC_TYPE, 1, struct lwis_device_info)
#define LWIS_ENROLL_BUFFER _IOWR(LWIS_IOC_TYPE, 2, struct lwis_buffer)
#define LWIS_REG_READ _IOWR(LWIS_IOC_TYPE, 3, struct lwis_io_msg)
#define LWIS_REG_WRITE _IOWR(LWIS_IOC_TYPE, 4, struct lwis_io_msg)
#define LWIS_DEVICE_ENABLE _IO(LWIS_IOC_TYPE, 5)
#define LWIS_DEVICE_DISABLE _IO(LWIS_IOC_TYPE, 6)

#endif /* LWIS_COMMANDS_H_ */