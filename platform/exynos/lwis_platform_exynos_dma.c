/*
 * Google LWIS Exynos Platform-specific DMA Functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/ion_exynos.h>
#include <linux/exynos_iovmm.h>

#include "lwis_platform.h"
#include "lwis_platform_exynos.h"

struct dma_buf *lwis_platform_dma_buffer_alloc(const char *heap_name,
					       size_t len, unsigned int flags)
{
	return ion_alloc_dmabuf(heap_name, len, flags);
}

dma_addr_t lwis_platform_dma_buffer_map(struct lwis_device *lwis_dev,
					struct dma_buf_attachment *attachment,
					off_t offset, size_t size,
					enum dma_data_direction direction,
					int flags)
{
	return ion_iovmm_map(attachment, offset, size, direction, flags);
}

int lwis_platform_dma_buffer_unmap(struct lwis_device *lwis_dev,
				   struct dma_buf_attachment *attachment,
				   dma_addr_t address)
{
	ion_iovmm_unmap(attachment, address);
	return 0;
}
