/*
 * Google LWIS Device Tree Parser
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "lwis_dt.h"
#include "lwis_sensor.h"

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

int lwis_sensor_parse_config_dt(struct device *pdev,
				struct lwis_sensor *psensor)
{
	int ret = 0;
	struct device_node *pdnode;

	pdnode = pdev->of_node;
	if (!pdnode) {
		pr_err("Cannot find device node\n");
		ret = -ENODEV;
		goto error;
	}

error:
	return ret;
}

int lwis_device_parse_dt(struct lwis_device *pldev)
{
	struct device *pdev;
	struct device_node *pdnode;
	struct device_node *pdnode_sensor;
	const char *compat_str;

	pdev = &(pldev->ppdev->dev);
	pdnode = pdev->of_node;

	if (!pdnode) {
		pr_err("Cannot find device node\n");
		return -ENODEV;
	}

	of_property_read_string(pdnode, "compatible", &compat_str);
	pr_info("Parsing DT: %s\n", compat_str);

	/* Look for a sensor component */
	pdnode_sensor = of_parse_phandle(pdnode, "sensor", 0);
	if (pdnode_sensor != NULL)  {
		of_property_read_string(pdnode_sensor, "compatible", &compat_str);
		pr_info("Found sensor: %s!\n", compat_str);

		pldev->psensor = kzalloc(sizeof(struct lwis_sensor),
					 GFP_KERNEL);
		if (!pldev->psensor) {
			pr_err("Failed to allocate sensor struct\n");
			return -ENOMEM;
		}

		pdnode_sensor->data = pldev->psensor;
	} else {
		pr_err("Sensor component not found\n");
	}

	return 0;
}