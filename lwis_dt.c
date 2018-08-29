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

static int parse_gpios(struct lwis_device *lwis_dev, char *name,
		       struct lwis_gpio_list **gpio_list)
{
	int i;
	int ret;
	int pin;
	int count;
	enum of_gpio_flags flags;
	struct device_node *dev_node;

	dev_node = lwis_dev->plat_dev->dev.of_node;
	count = of_gpio_named_count(dev_node, name);

	/* No GPIO pins found, just return */
	if (count <= 0) {
		return 0;
	}

	*gpio_list = lwis_gpio_list_alloc(count);

	/* Parse and acquire gpio pin numbers and polarities */
	for (i = 0; i < count; ++i) {
		pin = of_get_named_gpio_flags(dev_node, name, i, &flags);
		if (pin < 0) {
			pr_err("Cannot find gpio %s[%d]\n", name, i);
			ret = pin;
			goto error_parse_gpios;
		}
		lwis_gpio_get(*gpio_list, i, pin,
			      !(flags & OF_GPIO_ACTIVE_LOW));
	}

	pr_info("%s: %s\n", __func__, name);
	lwis_gpio_print(*gpio_list);

	return 0;

	/* In case of error, free the other GPIOs that were alloc'ed */
error_parse_gpios:
	lwis_gpio_list_free(*gpio_list);
	return ret;
}

static int parse_regulators(struct lwis_device *lwis_dev)
{
	int i;
	int ret;
	int count;
	struct device_node *dev_node;
	struct device_node *dev_node_reg;
	const char *name;
	struct device *dev;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;

	count = of_property_count_elems_of_size(dev_node, "regulators",
						sizeof(u32));

	/* No regulators found, or entry does not exist, just return */
	if (count <= 0) {
		return 0;
	}

	lwis_dev->regulators = lwis_regulator_list_alloc(count);

	/* Parse regulator list and acquire the regulator pointers */
	for (i = 0; i < count; ++i) {
		dev_node_reg = of_parse_phandle(dev_node, "regulators", i);
		of_property_read_string(dev_node_reg, "regulator-name", &name);
		ret = lwis_regulator_get(lwis_dev->regulators, (char *)name,
					 dev);
		if (ret < 0) {
			pr_err("Cannot find regulator: %s\n", name);
			goto error_parse_reg;
		}
	}

	lwis_regulator_print(lwis_dev->regulators);

	return 0;

	/* In case of error, free all the other regulators that were alloc'ed */
error_parse_reg:
	for (i = 0; i < count; ++i) {
		lwis_regulator_put_by_idx(lwis_dev->regulators, i);
	}
	lwis_regulator_list_free(lwis_dev->regulators);
	lwis_dev->regulators = NULL;
	return ret;
}

static int parse_clocks(struct lwis_device *lwis_dev)
{
	int i;
	int ret = 0;
	int count;
	struct device *dev;
	struct device_node *dev_node;
	const char *name;
	u32 rate;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;

	count = of_property_count_strings(dev_node, "clock-names");

	/* No clocks found, just return */
	if (count <= 0) {
		return 0;
	}

	lwis_dev->clocks = lwis_clock_list_alloc(count);

	/* Parse and acquire clock pointers and frequencies, if applicable */
	for (i = 0; i < count; ++i) {
		of_property_read_string_index(dev_node, "clock-names", i,
					      &name);
		/* It is allowed to omit clock rates for some of the clocks */
		ret = of_property_read_u32_index(dev_node, "clock-rates", i,
						 &rate);
		rate = (ret == 0) ? rate : 0;

		ret = lwis_clock_get(lwis_dev->clocks, (char *) name, dev,
				     rate);
		if (ret < 0) {
			pr_err("Cannot find clock: %s\n", name);
			goto error_parse_clk;
		}
	}

	lwis_clock_print(lwis_dev->clocks);

	return 0;

error_parse_clk:
	/* Put back the clock instances for the ones that were alloc'ed */
	for (i = 0; i < count; ++i) {
		lwis_clock_put_by_idx(lwis_dev->clocks, i, dev);
	}
	lwis_clock_list_free(lwis_dev->clocks);
	lwis_dev->clocks = NULL;
	return ret;
}

static int parse_pinctrls(struct lwis_device *lwis_dev, char *expected_state)
{
	int ret = 0;
	int count;
	struct device *dev;
	struct device_node *dev_node;
	struct pinctrl *pc;
	struct pinctrl_state *pinctrl_state;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;

	count = of_property_count_strings(dev_node, "pinctrl-names");

	/* No pinctrl found, just return */
	if (count <= 0) {
		return 0;
	}

	/* Set up pinctrl */
	pc = devm_pinctrl_get(dev);
	if (IS_ERR(pc)) {
		pr_err("Cannot allocate pinctrl\n");
		return PTR_ERR(pc);
	}

	pinctrl_state = pinctrl_lookup_state(pc, expected_state);
	if (IS_ERR(pinctrl_state)) {
		pr_err("Cannot find pinctrl state %s\n", expected_state);
		ret = PTR_ERR(pinctrl_state);
		goto error_pinctrl_state;
	}

	lwis_dev->mclk_ctrl = pc;

	return 0;

error_pinctrl_state:
	devm_pinctrl_put(pc);
	return ret;
}

static int parse_i2c_handle(struct lwis_device *lwis_dev)
{
	struct device_node *dev_node;
	struct device_node *dev_node_i2c;

	/* Save lwis_device handle to the device node of the i2c driver, in
	   preparation for driver initialization. */
	dev_node = lwis_dev->plat_dev->dev.of_node;
	dev_node_i2c = of_parse_phandle(dev_node, "i2c-driver", 0);
	if (!dev_node_i2c) {
		pr_err("Cannot find i2c-driver node\n");
		return -ENODEV;
	}

	dev_node_i2c->data = lwis_dev;

	return 0;
}

int lwis_device_parse_dt(struct lwis_device *lwis_dev)
{
	struct device *dev;
	struct device_node *dev_node;
	const char *compat_str;
	const char *name_str;
	int ret = 0;

	dev = &(lwis_dev->plat_dev->dev);
	dev_node = dev->of_node;

	if (!dev_node) {
		pr_err("Cannot find device node\n");
		return -ENODEV;
	}

	ret = of_property_read_string(dev_node, "compatible", &compat_str);
	if (ret) {
		pr_err("Error parsing compatible string\n");
		return -EINVAL;
	}
	pr_info("Parsing DT: %s\n", compat_str);

	ret = of_property_read_string(dev_node, "node-name", &name_str);
	if (ret) {
		pr_err("Error parsing node name\n");
		return -EINVAL;
	}
	strncpy(lwis_dev->name, name_str, MAX_DEVICE_NAME_STRING);

	ret = parse_gpios(lwis_dev, "enable-gpios", &lwis_dev->enable_gpios);
	if (ret) {
		pr_err("Error parsing enable-gpios\n");
		return ret;
	}

	ret = parse_gpios(lwis_dev, "reset-gpios", &lwis_dev->reset_gpios);
	if (ret) {
		pr_err("Error parsing reset-gpios\n");
		return ret;
	}

	ret = parse_regulators(lwis_dev);
	if (ret) {
		pr_err("Error parsing regulators\n");
		return ret;
	}

	ret = parse_clocks(lwis_dev);
	if (ret) {
		pr_err("Error parsing clocks\n");
		return ret;
	}

	ret = parse_pinctrls(lwis_dev, "mclk_on");
	if (ret) {
		pr_err("Error parsing mclk pinctrls\n");
		return ret;
	}

	if (!strcmp(compat_str, LWIS_TOP_DEVICE_COMPAT)) {
		lwis_dev->type = DEVICE_TYPE_TOP;
	} else if (!strcmp(compat_str, LWIS_I2C_DEVICE_COMPAT)) {
		lwis_dev->type = DEVICE_TYPE_I2C;
		parse_i2c_handle(lwis_dev);
	} else if (!strcmp(compat_str, LWIS_MMAP_DEVICE_COMPAT)) {
		lwis_dev->type = DEVICE_TYPE_MMAP;
	} else {
		lwis_dev->type = DEVICE_TYPE_UNKNOWN;
	}

	dev_node->data = lwis_dev;

	return ret;
}
