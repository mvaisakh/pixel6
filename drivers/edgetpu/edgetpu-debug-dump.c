// SPDX-License-Identifier: GPL-2.0
/*
 * Module that defines structures and functions to retrieve debug dump segments
 * from edgetpu firmware.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#include "edgetpu-config.h"
#include "edgetpu-debug-dump.h"
#include "edgetpu-iremap-pool.h"
#include "edgetpu-kci.h"

static inline u64 word_align_offset(u64 offset)
{
	return offset/sizeof(u64) +
	       (((offset % sizeof(u64)) == 0) ? 0 : 1);
}

int edgetpu_debug_dump_init(struct edgetpu_dev *etdev)
{
#ifdef CONFIG_ABROLHOS
	size_t size;
	int ret;
	struct edgetpu_debug_dump_setup *dump_setup;

	size = EDGETPU_DEBUG_DUMP_MEM_SIZE;

	/*
	 * Allocate buffers for various dump segments and map them to FW
	 * accessible regions
	 */
	ret = edgetpu_iremap_alloc(etdev, size, &etdev->debug_dump_mem,
				   EDGETPU_CONTEXT_KCI);
	if (ret) {
		etdev_err(etdev, "Debug dump seg alloc failed");
		etdev->debug_dump_mem.vaddr = NULL;
		return ret;
	}
	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	dump_setup->dump_mem_size = size;
	memset(dump_setup, 0, dump_setup->dump_mem_size);
	return ret;
#else
	return 0;
#endif	/* CONFIG_ABROLHOS */
}

void edgetpu_debug_dump_exit(struct edgetpu_dev *etdev)
{
#ifdef CONFIG_ABROLHOS
	if (!etdev->debug_dump_mem.vaddr) {
		etdev_dbg(etdev, "Debug dump not allocated");
		return;
	}
	/*
	 * Free the memory assigned for debug dump
	 */
	edgetpu_iremap_free(etdev, &etdev->debug_dump_mem,
			    EDGETPU_CONTEXT_KCI);
#endif	/* CONFIG_ABROLHOS */
}

int edgetpu_get_debug_dump(struct edgetpu_dev *etdev, u64 type)
{
	int ret;
	struct edgetpu_debug_dump_setup *dump_setup;

	if (!etdev->debug_dump_mem.vaddr) {
		etdev_err(etdev, "Debug dump not allocated");
		return -EINVAL;
	}

	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	dump_setup->type = type;
	/* Signal the type of dump and buffer address to firmware */
	ret = edgetpu_kci_get_debug_dump(etdev->kci,
					 etdev->debug_dump_mem.tpu_addr,
					 etdev->debug_dump_mem.size);
	etdev_dbg(etdev, "Sent debug dump request, tpu addr: %llx",
		  (u64)etdev->debug_dump_mem.tpu_addr);
	if (ret)
		etdev_err(etdev, "KCI dump info req failed: %d", ret);

	return ret;
}

void edgetpu_debug_dump_resp_handler(struct edgetpu_dev *etdev)
{
	struct edgetpu_debug_dump_setup *dump_setup;
	struct edgetpu_debug_dump *debug_dump;
	u64 offset;

	if (!etdev->debug_dump_mem.vaddr) {
		etdev_err(etdev, "Debug dump memory not allocated");
		return;
	}
	dump_setup =
		(struct edgetpu_debug_dump_setup *)etdev->debug_dump_mem.vaddr;
	offset = sizeof(struct edgetpu_debug_dump_setup);
	debug_dump = (struct edgetpu_debug_dump *)((u64 *)dump_setup +
		     word_align_offset(offset));
	if (!debug_dump->host_dump_available_to_read)
		return;

	/*
	 * TODO (b/156049774): Dump segments may be collected here and exposed
	 * to SSCD.
	 */
	debug_dump->host_dump_available_to_read = false;
}
