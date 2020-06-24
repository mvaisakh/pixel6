// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos Edge TPU ML accelerator device host support.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/irqreturn.h>

#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-telemetry.h"

#define HOST_NONSECURE_INTRSRCMASKREG	0x000f0004

irqreturn_t edgetpu_chip_irq_handler(int irq, void *arg)
{
	struct edgetpu_dev *etdev = arg;

	edgetpu_telemetry_irq_handler(etdev);

	/*
	 * TODO(b/137515582): trigger specific handler of mailbox(es) according
	 * to @irq
	 */
	return edgetpu_mailbox_handle_irq(etdev->mailbox_manager);
}

void edgetpu_chip_init(struct edgetpu_dev *etdev)
{
	/* Disable the CustomBlock Interrupt. */
	edgetpu_dev_write_32(etdev, HOST_NONSECURE_INTRSRCMASKREG, 0x1);
}

void edgetpu_chip_exit(struct edgetpu_dev *etdev)
{
}

bool edgetpu_chip_bypassed(struct edgetpu_dev *etdev)
{
	return false;
}

struct edgetpu_dumpregs_range edgetpu_chip_statusregs_ranges[] = {
};
int edgetpu_chip_statusregs_nranges =
	ARRAY_SIZE(edgetpu_chip_statusregs_ranges);

struct edgetpu_dumpregs_range edgetpu_chip_tile_statusregs_ranges[] = {
};
int edgetpu_chip_tile_statusregs_nranges =
	ARRAY_SIZE(edgetpu_chip_tile_statusregs_ranges);
