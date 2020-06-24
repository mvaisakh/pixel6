/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Provides functions for mapping buffers backed by dma-buf to EdgeTPU devices.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_MAP_DMABUF_H__
#define __EDGETPU_MAP_DMABUF_H__

#include "edgetpu-device-group.h"
#include "edgetpu-internal.h"
#include "edgetpu.h"

/*
 * Maps a dma-buf to a device group.
 *
 * @arg->device_address will be set as the mapped TPU VA on success.
 *
 * Returns zero on success or a negative errno on error.
 */
int edgetpu_map_dmabuf(struct edgetpu_device_group *group,
		       struct edgetpu_map_dmabuf_ioctl *arg);
/* unmap the dma-buf backed buffer from a device group */
int edgetpu_unmap_dmabuf(struct edgetpu_device_group *group, u32 die_index,
			 tpu_addr_t tpu_addr);

#endif /* __EDGETPU_MAP_DMABUF_H__ */
