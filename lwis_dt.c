/*
 * Google LWIS Device Tree Parser
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-dt: " fmt

#include "lwis_dt.h"

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include "lwis_clock.h"
#include "lwis_gpio.h"
#include "lwis_regulator.h"
#include "lwis_sensor.h"

static int parse_sensor_gpio(struct device_node *pdnode, char *name,
			     struct lwis_gpio_list **gpio_list)
{
	int i;
	int ret;
	int pin;
	int count;
	enum of_gpio_flags flags;

	count = of_gpio_named_count(pdnode, name);

	/* No GPIO pins found, just return */
	if (count <= 0) {
		return 0;
	}

	*gpio_list = lwis_gpio_list_alloc(count);

	/* Parse and acquire gpio pin numbers and polarities */
	for (i = 0; i < count; ++i) {
		pin = of_get_named_gpio_flags(pdnode, name, i, &flags);
		if (pin < 0) {
			pr_err("Cannot find gpio %s[%d]\n", name, i);
			ret = pin;
			goto error_parse_gpio;
		}
		lwis_gpio_set(*gpio_list, i, pin,
			      !(flags & OF_GPIO_ACTIVE_LOW));
	}

	pr_info("%s: %s\n", __func__, name);
	lwis_gpio_print(*gpio_list);

	return 0;

	/* In case of error, free the other GPIOs that were alloc'ed */
error_parse_gpio:
	lwis_gpio_list_free(*gpio_list);
	return ret;
}

static int parse_sensor_regulators(struct device *pdev,
				   struct lwis_regulator_list **reg_list)
{
	int i;
	int ret;
	int count;
	struct device_node *pdnode;
	struct device_node *pdnode_reg;
	const char *name;

	pdnode = pdev->of_node;

	count = of_property_count_elems_of_size(pdnode, "regulators",
						sizeof(u32));

	/* No regulators found, or entry does not exist, just return */
	if (count <= 0) {
		return 0;
	}

	*reg_list = lwis_regulator_list_alloc(count);

	/* Parse regulator list and acquire the regulator pointers */
	for (i = 0; i < count; ++i) {
		pdnode_reg = of_parse_phandle(pdnode, "regulators", i);
		of_property_read_string(pdnode_reg, "regulator-name", &name);
		ret = lwis_regulator_set(*reg_list, pdev, i, (char *) name);
		if (ret) {
			pr_err("Cannot find regulator: %s\n", name);
			goto error_parse_reg;
		}
	}

	lwis_regulator_print(*reg_list);

	return 0;

	/* In case of error, free all the other regulators that were alloc'ed */
error_parse_reg:
	for (i = 0; i < count; ++i) {
		lwis_regulator_put(*reg_list, i);
	}
	lwis_regulator_list_free(*reg_list);
	return ret;
}

static int parse_sensor_clocks(struct device *pdev,
			       struct lwis_clock_list **clk_list)
{
	int i;
	int ret = 0;
	int count;
	struct device_node *pdnode;
	const char *name;
	u32 rate;

	pdnode = pdev->of_node;

	count = of_property_count_strings(pdnode, "clock-names");

	/* No clocks found, just return */
	if (count <= 0) {
		return 0;
	}

	*clk_list = lwis_clock_list_alloc(count);

	/* Parse and acquire clock pointers and frequencies, if applicable */
	for (i = 0; i < count; ++i) {
		of_property_read_string_index(pdnode, "clock-names", i,
					      &name);
		/* It is allowed to omit clock rates for some of the clocks */
		ret = of_property_read_u32_index(pdnode, "clock-rates", i,
						 &rate);
		rate = (ret == 0) ? rate : 0;

		ret = lwis_clock_set(*clk_list, pdev, i, (char *) name, rate);
		if (ret) {
			pr_err("Cannot find clock: %s\n", name);
			goto error_parse_clk;
		}
	}

	lwis_clock_print(*clk_list);

	return 0;

error_parse_clk:
	/* Put back the clock instances for the ones that were alloc'ed */
	for (i = 0; i < count; ++i) {
		lwis_clock_put(*clk_list, pdev, i);
	}
	lwis_clock_list_free(*clk_list);
	return ret;
}

int lwis_sensor_parse_config_dt(struct device *pdev,
				struct lwis_sensor *psensor)
{
	int ret = 0;
	struct device_node *pdnode;

	pdnode = pdev->of_node;
	if (!pdnode) {
		pr_err("Cannot find device node\n");
		ret = -ENODEV;
		goto error_parse_dt;
	}

	ret = parse_sensor_gpio(pdnode, "enable-gpios", &psensor->enable_gpios);
	if (ret) {
		pr_err("Error parsing enable-gpios\n");
		goto error_parse_dt;
	}

	ret = parse_sensor_gpio(pdnode, "reset-gpios", &psensor->reset_gpios);
	if (ret) {
		pr_err("Error parsing reset-gpios\n");
		goto error_parse_dt;
	}

	ret = parse_sensor_regulators(pdev, &psensor->regulators);
	if (ret) {
		pr_err("Error parsing regulators\n");
		goto error_parse_dt;
	}

	ret = parse_sensor_clocks(pdev, &psensor->clocks);
	if (ret) {
		pr_err("Error parsing clocks\n");
		goto error_parse_dt;
	}

error_parse_dt:
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
