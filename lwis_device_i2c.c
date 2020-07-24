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
#include "lwis_init.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-i2c"

#define I2C_ON_STRING "on_i2c"
#define I2C_OFF_STRING "off_i2c"

static int lwis_i2c_device_enable(struct lwis_device *lwis_dev);
static int lwis_i2c_device_disable(struct lwis_device *lwis_dev);
static int lwis_i2c_register_io(struct lwis_device *lwis_dev,
				struct lwis_io_entry *entry, bool non_blocking,
				int access_size);

static struct lwis_device_subclass_operations i2c_vops = {
	.register_io = lwis_i2c_register_io,
	.device_enable = lwis_i2c_device_enable,
	.device_disable = lwis_i2c_device_disable,
	.event_enable = NULL,
	.event_flags_updated = NULL,
};

static struct lwis_event_subscribe_operations i2c_subscribe_ops = {
	.subscribe_event = NULL,
	.unsubscribe_event = NULL,
	.notify_event_subscriber = NULL,
	.release = NULL,
};

static int lwis_i2c_device_enable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_i2c_device *i2c_dev = (struct lwis_i2c_device *)lwis_dev;

	/* Enable the I2C bus */
	ret = lwis_i2c_set_state(i2c_dev, I2C_ON_STRING);
	if (ret) {
		dev_err(lwis_dev->dev, "Error enabling i2c bus (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int lwis_i2c_device_disable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_i2c_device *i2c_dev = (struct lwis_i2c_device *)lwis_dev;

	/* Disable the I2C bus */
	ret = lwis_i2c_set_state(i2c_dev, I2C_OFF_STRING);
	if (ret) {
		dev_err(lwis_dev->dev, "Error disabling i2c bus (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int lwis_i2c_register_io(struct lwis_device *lwis_dev,
				struct lwis_io_entry *entry, bool non_blocking,
				int access_size)
{
	/* I2C does not currently support non-blocking calls at all */
	if (non_blocking) {
		return -EAGAIN;
	}
	return lwis_i2c_io_entry_rw((struct lwis_i2c_device *)lwis_dev, entry);
}

static int lwis_i2c_addr_matcher(struct device *dev, void *data)
{
	struct i2c_client *client = i2c_verify_client(dev);
	int address = *(int *)data;

	/* Return 0 if error, or address doesn't match */
	if (IS_ERR_OR_NULL(client) || (client->addr != address)) {
		return 0;
	}

	/* Return 1 when address is found */
	return 1;
}

static int lwis_i2c_device_setup(struct lwis_i2c_device *i2c_dev)
{
	int ret;
	struct i2c_board_info info = {};
	struct device *dev;
	struct pinctrl *pinctrl;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_i2c_device_parse_dt(i2c_dev);
	if (ret) {
		dev_err(i2c_dev->base_dev.dev, "Failed to parse device tree\n");
		return ret;
	}
#else
	/* Non-device-tree init: Save for future implementation */
	return -ENOSYS;
#endif

	info.addr = i2c_dev->address;

	i2c_dev->client = i2c_new_device(i2c_dev->adapter, &info);

	/* New device creation failed, possibly because client with the same
	   address is defined, try to find the client instance in the adapter
	   and use it here */
	if (IS_ERR_OR_NULL(i2c_dev->client)) {
		struct device *idev;
		idev = device_find_child(&i2c_dev->adapter->dev,
					 &i2c_dev->address,
					 lwis_i2c_addr_matcher);
		i2c_dev->client = i2c_verify_client(idev);
	}

	/* Still getting error in obtaining client, return error */
	if (IS_ERR_OR_NULL(i2c_dev->client)) {
		dev_err(i2c_dev->base_dev.dev,
			"Failed to create or find i2c device\n");
		return -EINVAL;
	}

	dev = &i2c_dev->client->dev;

	/* Parent of the client is the i2c block, which is where the i2c state
	   pinctrl's are defined */
	/* TODO: Need to figure out why this is parent's parent */
	pinctrl = devm_pinctrl_get(dev->parent->parent);
	if (IS_ERR(pinctrl)) {
		dev_err(i2c_dev->base_dev.dev,
			"Cannot instantiate pinctrl instance (%d)\n",
			(int)PTR_ERR(pinctrl));
		return PTR_ERR(pinctrl);
	}
	i2c_dev->state_pinctrl = pinctrl;

	return 0;
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
	i2c_dev->base_dev.subscribe_ops = i2c_subscribe_ops;

	/* Call the base device probe function */
	ret = lwis_base_probe((struct lwis_device *)i2c_dev, plat_dev);
	if (ret) {
		pr_err("Error in lwis base probe\n");
		goto error_probe;
	}

	/* Call I2C device specific setup function */
	ret = lwis_i2c_device_setup(i2c_dev);
	if (ret) {
		dev_err(i2c_dev->base_dev.dev,
			"Error in i2c device initialization\n");
		goto error_probe;
	}

	dev_info(i2c_dev->base_dev.dev, "I2C Device Probe: Success\n");

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
	.driver =
		{
			.name = LWIS_DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = lwis_id_match,
		},
};
#else /* CONFIG_OF not defined */
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
int __init lwis_i2c_device_init(void)
{
	int ret = 0;

	pr_info("I2C device initialization\n");

	ret = platform_driver_probe(&lwis_driver, lwis_i2c_device_probe);
	if (ret) {
		pr_err("platform_driver_probe failed - %d", ret);
	}

	return ret;
}

int lwis_i2c_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
