/*
 * Google LWIS Top Level Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-top-dev: " fmt

#include "lwis_device_top.h"

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-top"

static int lwis_top_register_read(struct lwis_device *lwis_dev,
				  struct lwis_io_entry *entry,
				  bool non_blocking);
static int lwis_top_register_write(struct lwis_device *lwis_dev,
				   struct lwis_io_entry *entry,
				   bool non_blocking);

static struct lwis_device_subclass_operations top_vops = {
	.register_read = lwis_top_register_read,
	.register_write = lwis_top_register_write,
	.device_enable = NULL,
	.device_disable = NULL,
	.event_enable = NULL,
	.event_flags_updated = NULL,
};

static int lwis_top_register_read(struct lwis_device *lwis_dev,
				  struct lwis_io_entry *entry,
				  bool non_blocking)
{
	struct lwis_top_device *top_dev = (struct lwis_top_device *)lwis_dev;

	if (entry->rw.offset >= SCRATCH_MEMORY_SIZE) {
		pr_err("Offset must be < %d\n", SCRATCH_MEMORY_SIZE);
		return -EINVAL;
	}
	entry->rw.val = top_dev->scratch_mem[entry->rw.offset];
	return 0;
}

static int lwis_top_register_write(struct lwis_device *lwis_dev,
				   struct lwis_io_entry *entry,
				   bool non_blocking)
{
	struct lwis_top_device *top_dev = (struct lwis_top_device *)lwis_dev;

	if (entry->rw.offset >= SCRATCH_MEMORY_SIZE) {
		pr_err("Offset must be < %d\n", SCRATCH_MEMORY_SIZE);
		return -EINVAL;
	}
	top_dev->scratch_mem[entry->rw.offset] = entry->rw.val;
	return 0;
}

static int lwis_top_device_setup(struct lwis_top_device *top_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_top_device_parse_dt(top_dev);
	if (ret) {
		pr_err("Failed to parse device tree\n");
	}
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -ENOSYS;
#endif

	return ret;
}

static int __init lwis_top_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_top_device *top_dev;

	/* Allocate top device specific data construct */
	top_dev = kzalloc(sizeof(struct lwis_top_device), GFP_KERNEL);
	if (!top_dev) {
		pr_err("Failed to allocate top device structure\n");
		return -ENOMEM;
	}

	top_dev->base_dev.type = DEVICE_TYPE_TOP;
	top_dev->base_dev.vops = top_vops;

	/* Call the base device probe function */
	ret = lwis_base_probe((struct lwis_device *)top_dev, plat_dev);
	if (ret) {
		pr_err("Error in lwis base probe\n");
		goto error_probe;
	}

	/* Call top device specific setup function */
	ret = lwis_top_device_setup(top_dev);
	if (ret) {
		pr_err("Error in top device initialization\n");
		goto error_probe;
	}

	return 0;

error_probe:
	kfree(top_dev);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_TOP_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.driver = {
		.name = LWIS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_id_match,
	},
};

#else  /* CONFIG_OF not defined */
static struct platform_device_id lwis_driver_id[] = {
	{
		.name = LWIS_DRIVER_NAME,
		.driver_data = 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, lwis_driver_id);

static struct platform_driver lwis_driver = { .id_table = lwis_driver_id,
					      .driver = {
						      .name = LWIS_DRIVER_NAME,
						      .owner = THIS_MODULE,
					      } };
#endif /* CONFIG_OF */

/*
 *  lwis_top_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_top_device_init(void)
{
	int ret = 0;

	pr_info("Top device initialization\n");

	ret = platform_driver_probe(&lwis_driver, lwis_top_device_probe);
	if (ret) {
		pr_err("platform_driver_probe failed - %d", ret);
	}

	return ret;
}

int lwis_top_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
