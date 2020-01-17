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

int lwis_device_single_register_write(struct lwis_device *lwis_dev,
				      bool non_blocking, int bid,
				      uint64_t offset, uint64_t value)
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

	return lwis_dev->vops.register_io(lwis_dev, &entry, non_blocking);
}

int lwis_device_single_register_read(struct lwis_device *lwis_dev,
				     bool non_blocking, int bid,
				     uint64_t offset, uint64_t *value)
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

	ret = lwis_dev->vops.register_io(lwis_dev, &entry, non_blocking);
	if (!ret && value) {
		*value = entry.rw.val;
	}
	return ret;
}
