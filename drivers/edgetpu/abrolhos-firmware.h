/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Implements utilities for firmware management of Abrolhos.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __ABROLHOS_FIRMWARE_H__
#define __ABROLHOS_FIRMWARE_H__

struct edgetpu_dev;

int abrolhos_edgetpu_firmware_create(struct edgetpu_dev *etdev);
void abrolhos_edgetpu_firmware_destroy(struct edgetpu_dev *etdev);

int abrolhos_edgetpu_firmware_run_default(struct edgetpu_dev *etdev);

#endif /* __ABROLHOS_FIRMWARE_H__ */
