/*
 * Google LWIS Register I/O Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_IOREG_H_
#define LWIS_IOREG_H_

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/types.h>

#include "lwis_commands.h"

struct lwis_ioreg {
	phys_addr_t start;
	int size;
	unsigned int __iomem *base;
};

struct lwis_ioreg_list {
	struct lwis_ioreg *block;
	int count;
};

/*
 *  lwis_ioreg_list_alloc: Allocate a lwis_ioreg_list struct with the specified
 *  number of entries.
 */
struct lwis_ioreg_list *lwis_ioreg_list_alloc(int num_blocks);

/*
 *  lwis_ioreg_list_free: Deallocate the lwis_ioreg list struct provided.
 */
void lwis_ioreg_list_free(struct lwis_ioreg_list *list);

/*
 *  lwis_ioreg_get: Setup the content of a lwis_ioreg entry.
 */
int lwis_ioreg_get(struct lwis_ioreg_list *list, int index,
		   struct platform_device *plat_dev);

/*
 *  lwis_ioreg_put: Deinitialize the content of a lwis_ioreg entry.
 */
int lwis_ioreg_put(struct lwis_ioreg_list *list, int index,
		   struct platform_device *plat_dev);

/*
 *  lwis_ioreg_read_batch: Read memory mapped registers in batch.
 */
int lwis_ioreg_read_batch(struct lwis_ioreg_list *list, int index,
			  struct lwis_io_data *data, int num_entries);

/*
 *  lwis_ioreg_write_batch: Write memory mapped registers in batch.
 */
int lwis_ioreg_write_batch(struct lwis_ioreg_list *list, int index,
			   struct lwis_io_data *data, int num_entries);

/*
 * lwis_ioreg_read: Read single memory mapped register
 */
int lwis_ioreg_read(struct lwis_ioreg_list *list, int index, int64_t offset,
		    int value_bits, uint64_t *value);

/*
 * lwis_ioreg_write: Write single memory mapped register
 */
int lwis_ioreg_write(struct lwis_ioreg_list *list, int index, int64_t offset,
		     int value_bits, uint64_t value);

#endif /* LWIS_IOREG_H_ */