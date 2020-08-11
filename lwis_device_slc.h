/*
 * Google LWIS SLC Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef LWIS_DEVICE_SLC_H_
#define LWIS_DEVICE_SLC_H_

#define ENABLE_PT 0

#include "lwis_device.h"
#if ENABLE_PT
#include "pt/pt.h"
#endif

#define NUM_PT 5

struct slc_partition {
	int id;
	size_t size_kb;
#if ENABLE_PT
	ptid_t partition_id;
	struct pt_handle *partition_handle;
#endif
};

/*
 *  struct lwis_slc_device
 *  "Derived" lwis_device struct, with added slc related elements.
 */
struct lwis_slc_device {
	struct lwis_device base_dev;
	int num_pt;
	struct slc_partition pt[NUM_PT];
#if ENABLE_PT
	struct pt_handle *partition_handle;
#endif
};

int lwis_slc_device_deinit(void);

int lwis_slc_buffer_alloc(struct lwis_device *lwis_dev,
			  struct lwis_alloc_buffer_info *alloc_info);

int lwis_slc_buffer_free(struct lwis_device *lwis_dev, int fd);

#endif /* LWIS_DEVICE_SLC_H_ */
