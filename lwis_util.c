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
				      int64_t offset, uint64_t value)
{
	BUG_ON(!lwis_dev);
	if (lwis_dev->vops.register_write) {
		struct lwis_io_data data = {};
		struct lwis_io_msg msg = {};

		data.offset = offset;
		data.val = value;
		data.access_size = 0; // use default

		msg.buf = &data;
		msg.num_entries = 1;
		msg.offset_bitwidth = 0; // use default
		msg.bid = bid;

		return lwis_dev->vops.register_write(lwis_dev, &msg,
						     non_blocking);
	}
	return -EINVAL;
}

int lwis_device_single_register_read(struct lwis_device *lwis_dev,
				     bool non_blocking, int bid, int64_t offset,
				     uint64_t *value)
{
	int ret = -EINVAL;
	BUG_ON(!lwis_dev);
	if (lwis_dev->vops.register_read) {
		struct lwis_io_data data = {};
		struct lwis_io_msg msg = {};

		data.offset = offset;
		data.access_size = 0; // use default

		msg.buf = &data;
		msg.num_entries = 1;
		msg.offset_bitwidth = 0; // use default
		msg.bid = bid;

		ret = lwis_dev->vops.register_read(lwis_dev, &msg,
						     non_blocking);
		if(!ret && value) {
			*value = data.val;
		}
	}
	return ret;
}
