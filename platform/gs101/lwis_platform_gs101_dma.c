/*
 * Google LWIS GS101 Platform-Specific DMA Functions
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>

#include <linux/iommu.h>
#include <linux/ion.h>
#include <linux/dma-buf.h>
#include "lwis_commands.h"
#include "lwis_init.h"
#include "lwis_platform.h"
#include "lwis_platform_dma.h"

struct dma_buf *lwis_platform_dma_buffer_alloc(size_t len, unsigned int flags)
{
	unsigned int ion_flags = 0;
	unsigned int heapmask;

	heapmask = ION_HEAP_SYSTEM;

	if (flags & LWIS_DMA_BUFFER_CACHED) {
		ion_flags |= ION_FLAG_CACHED;
	}

	return ion_alloc(len, heapmask, ion_flags);
}

dma_addr_t lwis_platform_dma_buffer_map(struct lwis_device *lwis_dev,
					struct lwis_enrolled_buffer *lwis_buffer, off_t offset,
					size_t size, int flags)
{
	return sg_dma_address(lwis_buffer->sg_table->sgl);
}

/*
 * We don't ever do dma_buf_vmap before. Instead, use the upstream dma-buf
 * interface to map ION buffers, so we don't need to do dma_buf_vunmap.
 * Keep this function by defult return 0
 */
int lwis_platform_dma_buffer_unmap(struct lwis_device *lwis_dev,
				   struct dma_buf_attachment *attachment, dma_addr_t address)
{
	return 0;
}
