// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos Edge TPU ML accelerator device host support.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/irqreturn.h>

#include "edgetpu-config.h"
#include "edgetpu-debug-dump.h"
#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"
#include "abrolhos-platform.h"
#include "edgetpu-telemetry.h"

#define HOST_NONSECURE_INTRSRCMASKREG	0x000f0004

#define SSMT_NS_READ_STREAM_VID_OFFSET(n) (0x1000u + (0x4u * (n)))
#define SSMT_NS_WRITE_STREAM_VID_OFFSET(n) (0x1200u + (0x4u * (n)))

#define SSMT_NS_READ_STREAM_VID_REG(base, n)                                   \
	((base) + SSMT_NS_READ_STREAM_VID_OFFSET(n))
#define SSMT_NS_WRITE_STREAM_VID_REG(base, n)                                  \
	((base) + SSMT_NS_WRITE_STREAM_VID_OFFSET(n))

/*
 * The interrupt handler for mailboxes.
 *
 * This handler reads the IntSrcStatusReg available on Abrolhos to get the
 * response queue doorbell status of mailboxes 0-7 in one read. It loops
 * through the bits to check pending interrupts and invokes their IRQ
 * handlers. This allows for more efficient handling of interrupts than
 * edgetpu_mailbox_handle_irq handler.
 */
static irqreturn_t
abrolhos_mailbox_handle_irq(struct edgetpu_mailbox_manager *mgr)
{
	struct edgetpu_mailbox *mailbox;
	uint i;
	u32 val, clear_val;
	u8 mailbox_rsp_queue_status_bits;

	if (!mgr)
		return IRQ_NONE;

	read_lock(&mgr->mailboxes_lock);
	val = edgetpu_dev_read_32(mgr->etdev,
				  HOST_NONSECURE_INT_SRC_STATUS_REG);
	clear_val = val & (0xff << 1);
	edgetpu_dev_write_32(mgr->etdev,
			     HOST_NONSECURE_INT_SRC_CLEAR_REG, clear_val);

	mailbox_rsp_queue_status_bits = val >> 1;
	for (i = 0; i < mgr->num_mailbox; i++) {
		mailbox = mgr->mailboxes[i];
		if (mailbox && (mailbox_rsp_queue_status_bits & 0x1) &&
		    mailbox->handle_irq)
			mailbox->handle_irq(mailbox);
		mailbox_rsp_queue_status_bits >>= 1;
		if (!mailbox_rsp_queue_status_bits)
			break;
	}
	read_unlock(&mgr->mailboxes_lock);

	return IRQ_HANDLED;
}

irqreturn_t edgetpu_chip_irq_handler(int irq, void *arg)
{
	struct edgetpu_dev *etdev = arg;

	edgetpu_telemetry_irq_handler(etdev);
	edgetpu_debug_dump_resp_handler(etdev);

	return abrolhos_mailbox_handle_irq(etdev->mailbox_manager);
}

u64 edgetpu_chip_tpu_timestamp(struct edgetpu_dev *etdev)
{
	return edgetpu_dev_read_64(etdev, EDGETPU_REG_CPUNS_TIMESTAMP);
}

void edgetpu_chip_init(struct edgetpu_dev *etdev)
{
	int i;
	struct edgetpu_platform_dev *etpdev = container_of(
			etdev, struct edgetpu_platform_dev, edgetpu_dev);

	/* Disable the CustomBlock Interrupt. */
	edgetpu_dev_write_32(etdev, HOST_NONSECURE_INTRSRCMASKREG, 0x1);

	if (!etpdev->ssmt_base)
		return;

	/* Setup non-secure SCIDs, assume VID = SCID */
	for (i = 0; i < EDGETPU_NCONTEXTS; i++) {
		writel(i, SSMT_NS_READ_STREAM_VID_REG(etpdev->ssmt_base, i));
		writel(i, SSMT_NS_WRITE_STREAM_VID_REG(etpdev->ssmt_base, i));
	}
}

void edgetpu_chip_exit(struct edgetpu_dev *etdev)
{
}

void edgetpu_mark_probe_fail(struct edgetpu_dev *etdev)
{
}

struct edgetpu_dumpregs_range edgetpu_chip_statusregs_ranges[] = {
	{
		.firstreg = EDGETPU_REG_USER_HIB_FIRST_ERROR_STATUS,
		.lastreg = EDGETPU_REG_USER_HIB_FIRST_ERROR_STATUS,
	},
	{
		.firstreg = EDGETPU_REG_SC_RUNSTATUS,
		.lastreg = EDGETPU_REG_SC_RUNSTATUS,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_OUT_ACTVQ_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_OUT_ACTVQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_IN_ACTVQ_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_IN_ACTVQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_PARAMQ_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_PARAMQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_TOPLVL_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_TOPLVL_INT_STAT,
	},
};
int edgetpu_chip_statusregs_nranges =
	ARRAY_SIZE(edgetpu_chip_statusregs_ranges);

struct edgetpu_dumpregs_range edgetpu_chip_tile_statusregs_ranges[] = {
};
int edgetpu_chip_tile_statusregs_nranges =
	ARRAY_SIZE(edgetpu_chip_tile_statusregs_ranges);
