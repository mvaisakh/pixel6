/*
 * Google LWIS I/O Mapped Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioreg-dev: " fmt

#include "lwis_device_ioreg.h"

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "lwis_interrupt.h"
#include "lwis_ioreg.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-ioreg"

static int lwis_ioreg_device_enable(struct lwis_device *lwis_dev);
static int lwis_ioreg_device_disable(struct lwis_device *lwis_dev);
static int lwis_ioreg_register_read(struct lwis_device *lwis_dev,
				    struct lwis_io_entry *entry,
				    bool non_blocking);
static int lwis_ioreg_register_write(struct lwis_device *lwis_dev,
				     struct lwis_io_entry *entry,
				     bool non_blocking);

static struct lwis_device_subclass_operations ioreg_vops = {
	.register_read = lwis_ioreg_register_read,
	.register_write = lwis_ioreg_register_write,
	.device_enable = lwis_ioreg_device_enable,
	.device_disable = lwis_ioreg_device_disable,
	.event_enable = NULL,
	.event_flags_updated = NULL,
};

static int lwis_ioreg_device_enable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_ioreg_device_disable(struct lwis_device *lwis_dev)
{
	return 0;
}

static int lwis_ioreg_register_read(struct lwis_device *lwis_dev,
				    struct lwis_io_entry *entry,
				    bool non_blocking)
{
	return lwis_ioreg_io_entry_read((struct lwis_ioreg_device *)lwis_dev,
					entry, non_blocking);
}

static int lwis_ioreg_register_write(struct lwis_device *lwis_dev,
				     struct lwis_io_entry *entry,
				     bool non_blocking)
{
	return lwis_ioreg_io_entry_write((struct lwis_ioreg_device *)lwis_dev,
					 entry, non_blocking);
}

static int lwis_ioreg_device_setup(struct lwis_ioreg_device *ioreg_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_ioreg_device_parse_dt(ioreg_dev);
	if (ret) {
		pr_err("Failed to parse device tree\n");
	}
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -ENOSYS;
#endif

	return ret;
}

static int __init lwis_ioreg_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_ioreg_device *ioreg_dev;

	/* Allocate IOREG device specific data construct */
	ioreg_dev = kzalloc(sizeof(struct lwis_ioreg_device), GFP_KERNEL);
	if (!ioreg_dev) {
		pr_err("Failed to allocate IOREG device structure\n");
		return -ENOMEM;
	}

	ioreg_dev->base_dev.type = DEVICE_TYPE_IOREG;
	ioreg_dev->base_dev.vops = ioreg_vops;

	/* Call the base device probe function */
	ret = lwis_base_probe((struct lwis_device *)ioreg_dev, plat_dev);
	if (ret) {
		pr_err("Error in lwis base probe\n");
		goto error_probe;
	}

	/* Call IOREG device specific setup function */
	ret = lwis_ioreg_device_setup(ioreg_dev);
	if (ret) {
		pr_err("Error in IOREG device initialization\n");
		goto error_probe;
	}

	return 0;

error_probe:
	kfree(ioreg_dev);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_IOREG_DEVICE_COMPAT },
	{},
};
// MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.driver =
		{
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
 *  lwis_ioreg_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_ioreg_device_init(void)
{
	int ret = 0;

	pr_info("IOREG device initialization\n");

	ret = platform_driver_probe(&lwis_driver, lwis_ioreg_device_probe);
	if (ret) {
		pr_err("platform_driver_probe failed - %d", ret);
	}

	return ret;
}

int lwis_ioreg_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
