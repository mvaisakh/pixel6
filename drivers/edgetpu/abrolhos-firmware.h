/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implements utilities for firmware management of Abrolhos.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __ABROLHOS_FIRMWARE_H__
#define __ABROLHOS_FIRMWARE_H__

#include <linux/sizes.h>

#include "edgetpu-internal.h"
#include "edgetpu.h"

/* abrolhos FW header size */
#define ABROLHOS_FW_HEADER_SIZE SZ_4K
/* The offset to the signed firmware header. */
#define ABROLHOS_HEADER_OFFSET 0x400
/* The offset to image configuration. */
#define ABROLHOS_IMAGE_CONFIG_OFFSET (ABROLHOS_HEADER_OFFSET + 0x160)

/*
 * The image configuration attached to the signed firmware.
 */
struct abrolhos_image_config {
	__u32 carveout_base;
	__u32 firmware_base;
	__u32 firmware_size;
	struct edgetpu_fw_version firmware_versions;
} __packed;

/*
 * Abrolhos firmware header.
 */
struct abrolhos_image_header {
	char sig[512];
	char pub[512];
	int Magic;
	int Generation;
	int RollbackInfo;
	int Length;
	char Flags[16];
	char BodyHash[32];
	char ChipId[32];
	char AuthConfig[256];
	struct abrolhos_image_config ImageConfig;
};

int abrolhos_edgetpu_firmware_create(struct edgetpu_dev *etdev);
void abrolhos_edgetpu_firmware_destroy(struct edgetpu_dev *etdev);

int abrolhos_edgetpu_firmware_run_default(struct edgetpu_dev *etdev);

#endif /* __ABROLHOS_FIRMWARE_H__ */
