/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform device driver for the Google Edge TPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_PLATFORM_H__
#define __EDGETPU_PLATFORM_H__

#include "edgetpu-internal.h"
#include "abrolhos-pm.h"

struct edgetpu_platform_pwr {
	struct mutex policy_lock;
	enum tpu_pwr_state curr_policy;
};

struct edgetpu_platform_dev {
	struct edgetpu_dev edgetpu_dev;
	struct edgetpu_platform_pwr platform_pwr;
	int irq;
	void *fw_region_vaddr;
	size_t fw_region_size;
	dma_addr_t csr_iova;
	size_t csr_size;
};

#endif /* __EDGETPU_PLATFORM_H__ */
