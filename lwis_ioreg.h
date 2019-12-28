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
#include "lwis_device_ioreg.h"

/*
 *  lwis_ioreg_list_alloc: Allocate a lwis_ioreg_list struct with the specified
 *  number of entries.
 */
int lwis_ioreg_list_alloc(struct lwis_ioreg_device *ioreg_dev, int num_blocks);

/*
 *  lwis_ioreg_list_free: Deallocate the lwis_ioreg list struct provided.
 */
void lwis_ioreg_list_free(struct lwis_ioreg_device *ioreg_dev);

/*
 *  lwis_ioreg_get: Setup the content of a lwis_ioreg entry.
 */
int lwis_ioreg_get(struct lwis_ioreg_device *ioreg_dev, int index, char *name);

/*
 *  lwis_ioreg_put_by_idx: Deinitialize the content of a lwis_ioreg entry
 *  by index.
 */
int lwis_ioreg_put_by_idx(struct lwis_ioreg_device *ioreg_dev, int index);

/*
 *  lwis_ioreg_put_by_name: Deinitialize the content of a lwis_ioreg entry
 *  by name.
 */
int lwis_ioreg_put_by_name(struct lwis_ioreg_device *ioreg_dev, char *name);

/*
 *  lwis_ioreg_read_batch: Read memory mapped registers in batch.
 */
int lwis_ioreg_read_batch(struct lwis_ioreg_device *ioreg_dev,
			  struct lwis_io_entry *entries, int num_entries,
			  bool non_blocking);

/*
 *  lwis_ioreg_write_batch: Write memory mapped registers in batch.
 */
int lwis_ioreg_write_batch(struct lwis_ioreg_device *ioreg_dev,
			   struct lwis_io_entry *entries, int num_entries,
			   bool non_blocking);

/*
 *  lwis_ioreg_read_by_block_idx: Read single memory mapped register by
 *  block index.
 */
int lwis_ioreg_read_by_block_idx(struct lwis_ioreg_device *ioreg_dev, int index,
				 uint64_t offset, uint64_t *value,
				 bool non_blocking);

/*
 *  lwis_ioreg_read_by_block_name: Read single memory mapped register by
 *  block name.
 */
int lwis_ioreg_read_by_block_name(struct lwis_ioreg_device *ioreg_dev,
				  char *name, uint64_t offset, uint64_t *value,
				  bool non_blocking);

/*
 *  lwis_ioreg_write_by_block_idx: Write single memory mapped register by
 *  block index.
 */
int lwis_ioreg_write_by_block_idx(struct lwis_ioreg_device *ioreg_dev,
				  int index, uint64_t offset, uint64_t value,
				  bool non_blocking);

/*
 *  lwis_ioreg_write_by_block_name: Write single memory mapped register by
 *  block name.
 */
int lwis_ioreg_write_by_block_name(struct lwis_ioreg_device *ioreg_dev,
				   char *name, uint64_t offset, uint64_t value,
				   bool non_blocking);

#endif /* LWIS_IOREG_H_ */
