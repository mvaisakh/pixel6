/*
 * Google LWIS Debug Utilities
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/fs.h>

#include "lwis_debug.h"
#include "lwis_util.h"

#ifdef CONFIG_DEBUG_FS

static ssize_t dev_info_read(struct file *fp, char __user *user_buf,
			     size_t count, loff_t *position)
{
	/* Main buffer for all information. */
	char k_buf[256] = {};
	struct lwis_device *lwis_dev = fp->f_inode->i_private;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	snprintf(k_buf, sizeof(k_buf), "%s Device Name: %s ID: %d State: %s\n",
		 lwis_device_type_to_string(lwis_dev->type), lwis_dev->name,
		 lwis_dev->id, lwis_dev->enabled ? "Enabled" : "Disabled");

	return simple_read_from_buffer(user_buf, count, position, k_buf,
				       strlen(k_buf));
}

static struct file_operations dev_info_fops = {
	.owner = THIS_MODULE,
	.read = dev_info_read,
};

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev,
			      struct dentry *dbg_root)
{
	struct dentry *dbg_dir;
	struct dentry *dbg_file;

	/* DebugFS not present, just return */
	if (dbg_root == NULL) {
		return 0;
	}

	dbg_dir = debugfs_create_dir(lwis_dev->name, dbg_root);
	if (IS_ERR_OR_NULL(dbg_dir)) {
		dev_err(lwis_dev->dev,
			"Failed to create DebugFS directory - %d",
			PTR_ERR(dbg_dir));
		return PTR_ERR(dbg_dir);
	}

	dbg_file = debugfs_create_file("dev_info", 0444, dbg_dir, lwis_dev,
				       &dev_info_fops);
	if (IS_ERR_OR_NULL(dbg_file)) {
		dev_warn(lwis_dev->dev, "Failed to create DebugFS file - %d",
			 PTR_ERR(dbg_file));
		dbg_file = NULL;
	}

	lwis_dev->dbg_dir = dbg_dir;
	lwis_dev->dbg_file = dbg_file;
	return 0;
}

int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev)
{
	/* DebugFS not present, just return */
	if (lwis_dev->dbg_dir == NULL) {
		return 0;
	}
	debugfs_remove_recursive(lwis_dev->dbg_dir);
	lwis_dev->dbg_dir = NULL;
	lwis_dev->dbg_file = NULL;
	return 0;
}

#else /* CONFIG_DEBUG_FS */

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev,
			      struct dentry *dbg_root)
{
	return 0;
}

int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev)
{
	return 0;
}

#endif