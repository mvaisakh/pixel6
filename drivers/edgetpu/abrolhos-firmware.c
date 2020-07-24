// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos Edge TPU ML accelerator firmware download support.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/dma-mapping.h>

#include "abrolhos-firmware.h"
#include "abrolhos-platform.h"
#include "edgetpu-config.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"

/*
 * Sets the reset state of the R52 core.
 * @val: 1 to put the core in reset state, 0 to release core from reset state.
 */
static void r52_reset(struct edgetpu_dev *etdev, u64 val)
{
	edgetpu_dev_write_64(etdev, EDGETPU_REG_RESET_CONTROL, val);
}

static void abrolhos_firmware_before_destroy(
		struct edgetpu_firmware *et_fw)
{
	r52_reset(et_fw->etdev, 1);
}

static int abrolhos_firmware_alloc_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct edgetpu_platform_dev *edgetpu_pdev = container_of(
			etdev, struct edgetpu_platform_dev, edgetpu_dev);

	fw_buf->vaddr = edgetpu_pdev->fw_region_vaddr;
	fw_buf->alloc_size = edgetpu_pdev->fw_region_size;
	fw_buf->used_size_align = 16;
	return 0;
}

static void abrolhos_firmware_free_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	fw_buf->vaddr = NULL;
	fw_buf->alloc_size = 0;
	fw_buf->used_size_align = 0;
}

static int abrolhos_firmware_setup_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
	return 0;
}

static void abrolhos_firmware_teardown_buffer(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_buffer *fw_buf)
{
}

static int abrolhos_firmware_prepare_run(struct edgetpu_firmware *et_fw,
					 struct edgetpu_firmware_buffer *fw_buf)
{
	struct edgetpu_dev *etdev = et_fw->etdev;

	/* Clear Substream ID (aka SCID) for instruction remapped addresses */
	u32 sec_reg = edgetpu_dev_read_32(
		etdev, EDGETPU_REG_INSTRUCTION_REMAP_SECURITY);
	sec_reg &= ~(0x0F << 10);
	edgetpu_dev_write_32(etdev, EDGETPU_REG_INSTRUCTION_REMAP_SECURITY,
			     sec_reg);

	/* Clear Substream ID (aka SCID) for all other addresses */
	sec_reg = edgetpu_dev_read_32(etdev, EDGETPU_REG_SECURITY);
	sec_reg &= ~(0x0F << 10);
	edgetpu_dev_write_32(etdev, EDGETPU_REG_SECURITY, sec_reg);

	r52_reset(etdev, 1);

	/* Reset KCI mailbox before start f/w, don't process anything old. */
	edgetpu_mailbox_reset(etdev->kci->mailbox);

	/* Remap TPU CPU instructions to the carveout IOVA. */
	edgetpu_dev_write_64(etdev, EDGETPU_REG_INSTRUCTION_REMAP_NEW_BASE,
			     FW_IOVA);
	edgetpu_dev_write_64(etdev, EDGETPU_REG_INSTRUCTION_REMAP_CONTROL, 1);

	r52_reset(etdev, 0);

	return 0;
}

static const struct edgetpu_firmware_handlers abrolhos_firmware_handlers = {
	.before_destroy = abrolhos_firmware_before_destroy,
	.alloc_buffer = abrolhos_firmware_alloc_buffer,
	.free_buffer = abrolhos_firmware_free_buffer,
	.setup_buffer = abrolhos_firmware_setup_buffer,
	.teardown_buffer = abrolhos_firmware_teardown_buffer,
	.prepare_run = abrolhos_firmware_prepare_run,
};

int abrolhos_edgetpu_firmware_create(struct edgetpu_dev *etdev)
{
	return edgetpu_firmware_create(etdev, &abrolhos_firmware_handlers);
}

void abrolhos_edgetpu_firmware_destroy(struct edgetpu_dev *etdev)
{
	edgetpu_firmware_destroy(etdev);
}

int edgetpu_chip_firmware_run(struct edgetpu_dev *etdev, const char *name,
			      enum edgetpu_firmware_flags flags)
{
	return edgetpu_firmware_run(etdev, name, flags);
}
