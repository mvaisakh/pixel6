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

#include "lwis_device.h"

#include "pt/pt.h"

#define NUM_PT 5

struct slc_partition {
	int id;
	size_t size_kb;
	ptid_t ptid;
};

/*
 *  struct lwis_slc_device
 *  "Derived" lwis_device struct, with added slc related elements.
 */
struct lwis_slc_device {
	struct lwis_device base_dev;
	int num_pt;
	struct slc_partition pt[NUM_PT];
	struct pt_handle *pt_hnd;
};

int lwis_slc_device_deinit(void);

int lwis_slc_buffer_alloc(struct lwis_device *lwis_dev,
			  struct lwis_alloc_buffer_info *alloc_info);

int lwis_slc_buffer_free(struct lwis_device *lwis_dev, ptid_t ptid);

#endif /* LWIS_DEVICE_SLC_H_ */
