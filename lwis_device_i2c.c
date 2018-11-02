/*
 * Google LWIS I2C Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-i2c-dev: " fmt

#include "lwis_device_i2c.h"

#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "lwis_i2c.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-i2c"

#define I2C_ON_STRING "on_i2c"
#define I2C_OFF_STRING "off_i2c"

static int lwis_i2c_device_enable(struct lwis_device *lwis_dev);
static int lwis_i2c_device_disable(struct lwis_device *lwis_dev);
static int lwis_i2c_register_read(struct lwis_device *lwis_dev,
				  struct lwis_io_msg *msg, bool non_blocking);
static int lwis_i2c_register_write(struct lwis_device *lwis_dev,
				   struct lwis_io_msg *msg, bool non_blocking);

static struct lwis_device_subclass_operations i2c_vops = {
	.register_read = lwis_i2c_register_read,
	.register_write = lwis_i2c_register_write,
	.device_enable = lwis_i2c_device_enable,
	.device_disable = lwis_i2c_device_disable,
	.event_enable = NULL,
	.event_flags_updated = NULL,
};

static int lwis_i2c_device_enable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_i2c_device *i2c_dev = (struct lwis_i2c_device *) lwis_dev;

	/* Enable the I2C bus */
	ret = lwis_i2c_set_state(i2c_dev, I2C_ON_STRING);
	if (ret) {
		pr_err("Error enabling i2c bus (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int lwis_i2c_device_disable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_i2c_device *i2c_dev = (struct lwis_i2c_device *) lwis_dev;

	/* Disable the I2C bus */
	ret = lwis_i2c_set_state(i2c_dev, I2C_OFF_STRING);
	if (ret) {
		pr_err("Error disabling i2c bus (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int lwis_i2c_register_read(struct lwis_device *lwis_dev,
				  struct lwis_io_msg *msg, bool non_blocking)
{
	/* I2C does not currently support non-blocking calls at all */
	if(non_blocking) {
		return -EAGAIN;
	}
	return lwis_i2c_read_batch((struct lwis_i2c_device *) lwis_dev, msg);
}

static int lwis_i2c_register_write(struct lwis_device *lwis_dev,
				   struct lwis_io_msg *msg, bool non_blocking)
{
	/* I2C does not currently support non-blocking calls at all */
	if(non_blocking) {
		return -EAGAIN;
	}
	return lwis_i2c_write_batch((struct lwis_i2c_device *) lwis_dev, msg);
}

static int lwis_i2c_device_setup(struct lwis_i2c_device *i2c_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_i2c_device_parse_dt(i2c_dev);
	if (ret) {
		pr_err("Failed to parse device tree\n");
	}
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -ENOSYS;
#endif

	return ret;
}

static int __init lwis_i2c_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_i2c_device *i2c_dev;

	/* Allocate I2C device specific data construct */
	i2c_dev = kzalloc(sizeof(struct lwis_i2c_device), GFP_KERNEL);
	if (!i2c_dev) {
		pr_err("Failed to allocate i2c device structure\n");
		return -ENOMEM;
	}

	i2c_dev->base_dev.type = DEVICE_TYPE_I2C;
	i2c_dev->base_dev.vops = i2c_vops;

	/* Call the base device probe function */
	ret = lwis_base_probe((struct lwis_device *) i2c_dev, plat_dev);
	if (ret) {
		pr_err("Error in lwis base probe\n");
		goto error_probe;
	}

	/* Call I2C device specific setup function */
	ret = lwis_i2c_device_setup(i2c_dev);
	if (ret) {
		pr_err("Error in i2c device initialization\n");
		goto error_probe;
	}

	return 0;

error_probe:
	kfree(i2c_dev);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_I2C_DEVICE_COMPAT },
	{},
};
MODULE_DEVICE_TABLE(of, lwis_id_match);

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
 *  lwis_i2c_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
static int __init lwis_i2c_device_init(void)
{
	int ret = 0;

	pr_info("I2C device initialization\n");

	ret = platform_driver_probe(&lwis_driver, lwis_i2c_device_probe);
	if (ret) {
		pr_err("platform_driver_probe failed - %d", ret);
	}

	return ret;
}

device_initcall_sync(lwis_i2c_device_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Google-ACMA");
MODULE_DESCRIPTION("LWIS I2C Device");
