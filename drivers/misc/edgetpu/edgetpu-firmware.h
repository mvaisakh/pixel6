/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU firmware loader.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_FIRMWARE_H__
#define __EDGETPU_FIRMWARE_H__

#include <linux/seq_file.h>

#include "edgetpu-internal.h"

struct edgetpu_firmware_private;

struct edgetpu_firmware {
	struct edgetpu_dev *etdev;
	struct edgetpu_firmware_private *p;
};

struct edgetpu_firmware_buffer {
	/*
	 * fields set by alloc_buffer() handler for using custom allocated
	 * buffer
	 *
	 * edgetpu_firmware framework also updates these fields when using
	 * shared firmware buffer.
	 */
	/*
	 * kernel VA, leave NULL to indicate edgetpu_firmware using shared
	 * firmware buffer.
	 */
	void *vaddr;
	size_t alloc_size;	/* allocated size of @vaddr in bytes */
	size_t used_size_align;	/* firmware size alignment in bytes */

	/* fields set by setup_buffer() handler */
	dma_addr_t dma_addr;	/* DMA handle for downstream IOMMU, if any */

	/* fields set by prepare_run() handler */
	void __iomem *dram_kva;	/* kernel VA of device DRAM image or zero */
	phys_addr_t dram_tpa;	/* tpu phys addr of device DRAM image or zero */

	/*
	 * fields set by edgetpu_firmware, don't modify the following fields in
	 * the handlers
	 */
	size_t used_size;	/* actual size of firmware image */
	const char *name;	/* the name of this firmware */
};

/*
 * Each handler returns 0 to indicate success, non-zero value to
 * indicate error.
 */
struct edgetpu_firmware_handlers {
	int (*after_create)(struct edgetpu_firmware *et_fw);
	/*
	 * Release resource used in platform specific implementation,
	 * including stopping firmware. So that internal cleanup could
	 * invoke teardown_buffer() safely after then.
	 */
	void (*before_destroy)(struct edgetpu_firmware *et_fw);
	/*
	 * Allocate a buffer for loading firmware and filling out the
	 * information into @fw_buf before running. See comments of
	 * edgetpu_firmware_buffer for the details of each field.
	 *
	 * This is invoked for each run.
	 */
	int (*alloc_buffer)(struct edgetpu_firmware *et_fw,
			    struct edgetpu_firmware_buffer *fw_buf);
	/*
	 * Free the buffer allocated by alloc_buffer() handler after running.
	 * See comments of edgetpu_firmware_buffer for the details of each
	 * field.
	 *
	 * This is invoked for each run.
	 */
	void (*free_buffer)(struct edgetpu_firmware *et_fw,
			    struct edgetpu_firmware_buffer *fw_buf);
	/*
	 * Setup for an allocated host buffer, mainly for dma mapping,
	 * for loading firmware and filling out the information into
	 * @fw_buf before running. See comments of
	 * edgetpu_firmware_buffer for the details of each
	 * field.
	 *
	 * This is invoked for each run.
	 */
	int (*setup_buffer)(struct edgetpu_firmware *et_fw,
			    struct edgetpu_firmware_buffer *fw_buf);
	/* Release the resources previously allocated by setup_buffer(). */
	void (*teardown_buffer)(struct edgetpu_firmware *et_fw,
				struct edgetpu_firmware_buffer *fw_buf);
	/*
	 * Platform-specific handling after firmware loaded, before running
	 * the firmware, such as validating the firmware or resetting the R52
	 * processor.
	 */
	int (*prepare_run)(struct edgetpu_firmware *et_fw,
			   struct edgetpu_firmware_buffer *fw_buf);
};

/*
 * Top-level chip-specific run firmware routine.
 * Calls edgetpu_firmware_run() one or more times as appropriate for chip-
 * specific one- or two-stage bootloader processing.
 *
 * @name: the name passed into underlying request_firmware API
 */
int edgetpu_chip_firmware_run(struct edgetpu_dev *etdev, const char *name);

/*
 * Load and run firmware.  Called by edgetpu_chip_firmware_run().
 * @name: the name passed into underlying request_firmware API
 */
int edgetpu_firmware_run(struct edgetpu_dev *etdev, const char *name);

/*
 * Private data set and used by handlers. It is expected to
 * allocate and set the data on after_create() and release on
 * before_destroy().
 */
void edgetpu_firmware_set_data(struct edgetpu_firmware *et_fw, void *data);
void *edgetpu_firmware_get_data(struct edgetpu_firmware *et_fw);

int edgetpu_firmware_create(struct edgetpu_dev *etdev,
			    const struct edgetpu_firmware_handlers *handlers);
void edgetpu_firmware_destroy(struct edgetpu_dev *etdev);
void edgetpu_firmware_mappings_show(struct edgetpu_dev *etdev,
				    struct seq_file *s);

#endif /* __EDGETPU_FIRMWARE_H__ */
