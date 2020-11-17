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
#include <linux/dma-heap.h>

#define ION_SYSTEM_HEAP_NAME "ion_system_heap"
#define ION_SECURE_FACERAW_HEAP_NAME "farawimg_heap"
#define ION_EXYNOS_FLAG_PROTECTED (1 << 16)

static unsigned int lwis_platform_get_heapmask_by_name(const char *heap_name)
{
	struct ion_heap_data data[ION_NUM_MAX_HEAPS];
	int i, num_heaps = ion_query_heaps_kernel(NULL, 0);

	ion_query_heaps_kernel((struct ion_heap_data *)data, num_heaps);

	if (!heap_name) {
		pr_err("Heap name is NULL\n");
		return 0;
	}

	for (i = 0; i < num_heaps; ++i) {
		if (!strncmp(data[i].name, heap_name, MAX_HEAP_NAME)) {
			break;
		}
	}

	if (i == num_heaps) {
		pr_err("Heap %s is not found\n", heap_name);
		return 0;
	}

	return 1 << data[i].heap_id;
}

struct dma_buf *lwis_platform_dma_buffer_alloc(size_t len, unsigned int flags)
{
	unsigned int ion_flags = 0;
	unsigned int heapmask;
	const char *heapname = ION_SYSTEM_HEAP_NAME;

	if (flags & LWIS_DMA_BUFFER_CACHED) {
		ion_flags |= ION_FLAG_CACHED;
	}

#if IS_ENABLED(CONFIG_EXYNOS_CONTENT_PATH_PROTECTION)
	if ((flags & LWIS_DMA_BUFFER_SECURE) != 0) {
		heapname = ION_SECURE_FACERAW_HEAP_NAME;
		ion_flags |= ION_EXYNOS_FLAG_PROTECTED;
	}
#endif

	heapmask = lwis_platform_get_heapmask_by_name(heapname);

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
