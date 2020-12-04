/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform device driver for the Google Edge TPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_PLATFORM_H__
#define __EDGETPU_PLATFORM_H__

#include <linux/device.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/types.h>

#include "edgetpu-internal.h"
#include "abrolhos-pm.h"

#define to_abrolhos_dev(etdev)                                                 \
	container_of(etdev, struct edgetpu_platform_dev, edgetpu_dev)

struct edgetpu_platform_pwr {
	struct mutex policy_lock;
	enum tpu_pwr_state curr_policy;
};

struct edgetpu_platform_dev {
	struct edgetpu_dev edgetpu_dev;
	struct edgetpu_platform_pwr platform_pwr;
	int irq;
	phys_addr_t fw_region_paddr;
	size_t fw_region_size;
	void *shared_mem_vaddr;
	phys_addr_t shared_mem_paddr;
	size_t shared_mem_size;
	phys_addr_t csr_paddr;
	dma_addr_t csr_iova;
	size_t csr_size;
	struct device *gsa_dev;
	void __iomem *ssmt_base;
	struct edgetpu_coherent_mem log_mem;
	struct edgetpu_coherent_mem trace_mem;
};

#endif /* __EDGETPU_PLATFORM_H__ */
