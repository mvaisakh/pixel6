/*
 * Google LWIS IMX362 Image Sensor Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "lwis_sensor.h"

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>

#define SENSOR_NAME "LWIS_IMX362"

/*
 *  Interface Functions
 */
static int lwis_sensor_imx362_init(void)
{
	// Currently stubbed
	pr_info("IMX362: Init\n");
	return 0;
}

static struct lwis_sensor_ops imx362_ops = {
	.init = &lwis_sensor_imx362_init,
};

static int lwis_sensor_imx362_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct lwis_sensor *psensor;

	pr_info("IMX362: Probe\n");

	psensor = lwis_sensor_get_ptr(client);
	if (!psensor) {
		pr_err("IMX362: Unable obtain sensor pointer\n");
		return -ENODEV;
	}

	psensor->ops = &imx362_ops;

	lwis_sensor_parse_config(&client->dev, psensor);

	return 0;
}

static const struct of_device_id lwis_sensor_imx362_match[] = {
	{
		.compatible = "google,lwis-is-imx362",
	},
	{},
};
MODULE_DEVICE_TABLE(of, lwis_sensor_imx362_match);

static const struct i2c_device_id lwis_sensor_imx362_id_table[] = {
	{ SENSOR_NAME, 0 },
	{},
};

static struct i2c_driver lwis_sensor_imx362_driver = {
	.probe = lwis_sensor_imx362_probe,
	.driver =
	{
		.name = SENSOR_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_sensor_imx362_match,
		.suppress_bind_attrs = true,
	},
	.id_table = lwis_sensor_imx362_id_table,
};

static int __init lwis_sensor_imx362_driver_init(void)
{
	int ret;

	pr_info("IMX362: Init\n");

	ret = i2c_add_driver(&lwis_sensor_imx362_driver);
	if (ret) {
		pr_err("failed to add %s driver: %d\n",
		       lwis_sensor_imx362_driver.driver.name, ret);
	}

	return ret;
}
late_initcall_sync(lwis_sensor_imx362_driver_init);
