/*
 * Google LWIS Debug Utilities
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "lwis_device.h"

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev,
			      struct dentry *dbg_root);
int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev);