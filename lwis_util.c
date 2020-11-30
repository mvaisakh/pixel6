/*
 * Google LWIS Misc Utility Functions and Wrappers
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-util: " fmt

#include "lwis_util.h"
#include "lwis_device.h"

int lwis_device_single_register_write(struct lwis_device *lwis_dev, bool non_blocking, int bid,
				      uint64_t offset, uint64_t value, int access_size)
{
	struct lwis_io_entry entry = {};

	BUG_ON(!lwis_dev);
	if (lwis_dev->vops.register_io == NULL) {
		return -EINVAL;
	}

	entry.type = LWIS_IO_ENTRY_WRITE;
	entry.rw.offset = offset;
	entry.rw.val = value;
	entry.rw.bid = bid;

	return lwis_dev->vops.register_io(lwis_dev, &entry, non_blocking, access_size);
}

int lwis_device_single_register_read(struct lwis_device *lwis_dev, bool non_blocking, int bid,
				     uint64_t offset, uint64_t *value, int access_size)
{
	int ret = -EINVAL;
	struct lwis_io_entry entry = {};

	BUG_ON(!lwis_dev);
	if (lwis_dev->vops.register_io == NULL) {
		return -EINVAL;
	}

	entry.type = LWIS_IO_ENTRY_READ;
	entry.rw.offset = offset;
	entry.rw.bid = bid;

	ret = lwis_dev->vops.register_io(lwis_dev, &entry, non_blocking, access_size);
	if (!ret && value) {
		*value = entry.rw.val;
	}
	return ret;
}

const char *lwis_device_type_to_string(enum lwis_device_types type)
{
	switch (type) {
	case DEVICE_TYPE_TOP:
		return "TOP";
	case DEVICE_TYPE_I2C:
		return "I2C";
	case DEVICE_TYPE_IOREG:
		return "IOREG";
	case DEVICE_TYPE_SLC:
		return "SLC";
	case DEVICE_TYPE_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}
