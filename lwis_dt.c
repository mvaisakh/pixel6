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
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>

#include "lwis_clock.h"
#include "lwis_gpio.h"
#include "lwis_i2c.h"
#include "lwis_ioreg.h"
#include "lwis_regulator.h"

static int parse_gpios(struct lwis_device *lwis_dev, char *name,
		       bool *is_present)
{
	int count;
	struct device *dev;
	struct device_node *dev_node;
	struct gpio_descs *list;

	*is_present = false;

	dev = &lwis_dev->plat_dev->dev;
	dev_node = dev->of_node;

	count = gpiod_count(dev, name);

	/* No GPIO pins found, just return */
	if (count <= 0) {
		return 0;
	}

	list = lwis_gpio_list_get(dev, name);
	if (IS_ERR(list)) {
		pr_err("Error parsing GPIO list %s (%ld)\n", name,
		       PTR_ERR(list));
		return PTR_ERR(list);
	}

	/* The GPIO pins are valid, release the list as we do not need to hold
	   on to the pins yet */
	lwis_gpio_list_put(list, dev);
	*is_present = true;
	return 0;
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

error_parse_reg:
	/* In case of error, free all the other regulators that were alloc'ed */
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

		ret = lwis_clock_get(lwis_dev->clocks, (char *)name, dev, rate);
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

static int parse_interrupts(struct lwis_device *lwis_dev)
{
	int i;
	int ret;
	int count, event_infos_count;
	const char *name;
	struct device_node *dev_node;
	struct platform_device *plat_dev;
	struct of_phandle_iterator it;

	plat_dev = lwis_dev->plat_dev;
	dev_node = plat_dev->dev.of_node;

	count = platform_irq_count(plat_dev);

	/* No interrupts found, just return */
	if (count <= 0) {
		lwis_dev->irqs = NULL;
		return 0;
	}

	lwis_dev->irqs = lwis_interrupt_list_alloc(lwis_dev, count);
	if (IS_ERR(lwis_dev->irqs)) {
		pr_err("Failed to allocate IRQ list\n");
		return PTR_ERR(lwis_dev->irqs);
	}

	for (i = 0; i < count; ++i) {
		of_property_read_string_index(dev_node, "interrupt-names", i,
					      &name);
		ret = lwis_interrupt_get(lwis_dev->irqs, i, (char *)name,
					 plat_dev);
		if (ret) {
			pr_err("Cannot set irq %s\n", name);
			goto error_get_irq;
		}
	}

	event_infos_count = of_property_count_elems_of_size(
		dev_node, "interrupt-event-infos", 4);
	if (count != event_infos_count) {
		pr_err("DT numbers of irqs: %d != event infos: %d in DT\n",
		       count, event_infos_count);
		goto error_get_irq;
	}
	/* Get event infos */
	i = 0;
	of_for_each_phandle(&it, ret, dev_node, "interrupt-event-infos", 0, 0)
	{
		const char *irq_reg_space = NULL;
		u64 irq_src_reg;
		u64 irq_reset_reg;
		u64 irq_mask_reg;
		int irq_events_num;
		int int_reg_bits_num;
		u64 *irq_events;
		u32 *int_reg_bits;
		int irq_reg_bid = -1;
		int irq_reg_bid_count;
		int j;
		struct device_node *event_info = of_node_get(it.node);

		irq_events_num = of_property_count_elems_of_size(
			event_info, "irq-events", 8);
		if (irq_events_num <= 0) {
			pr_err("Error getting irq-events: %d\n",
			       irq_events_num);
			ret = -EINVAL;
			goto error_event_infos;
		}

		int_reg_bits_num = of_property_count_elems_of_size(
			event_info, "int-reg-bits", 4);
		if (irq_events_num != int_reg_bits_num ||
		    int_reg_bits_num <= 0) {
			pr_err("Error getting int-reg-bits: %d\n",
			       int_reg_bits_num);
			ret = -EINVAL;
			goto error_event_infos;
		}

		irq_events = kzalloc(sizeof(u64) * irq_events_num, GFP_KERNEL);
		if (IS_ERR_OR_NULL(irq_events)) {
			ret = -ENOMEM;
			goto error_event_infos;
		}

		int_reg_bits =
			kzalloc(sizeof(u32) * int_reg_bits_num, GFP_KERNEL);
		if (IS_ERR_OR_NULL(int_reg_bits)) {
			ret = -ENOMEM;
			kfree(irq_events);
			goto error_event_infos;
		}

		irq_events_num = of_property_read_variable_u64_array(
			event_info, "irq-events", irq_events, irq_events_num,
			irq_events_num);
		if (irq_events_num != int_reg_bits_num) {
			pr_err("Error getting irq-events: %d\n",
			       irq_events_num);
			ret = irq_events_num;
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		int_reg_bits_num = of_property_read_variable_u32_array(
			event_info, "int-reg-bits", int_reg_bits,
			int_reg_bits_num, int_reg_bits_num);
		if (irq_events_num != int_reg_bits_num) {
			pr_err("Error getting int-reg-bits: %d\n",
			       int_reg_bits_num);
			ret = int_reg_bits_num;
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_string(event_info, "irq-reg-space",
					      &irq_reg_space);
		if (ret) {
			pr_err("Error getting irq-reg-space from dt: %d\n",
			       ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		irq_reg_bid_count =
			of_property_count_strings(dev_node, "reg-names");

		if (irq_reg_bid_count <= 0) {
			pr_err("Error getting reg-names from dt: %d\n",
			       irq_reg_bid_count);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}
		for (j = 0; j < irq_reg_bid_count; j++) {
			const char *bid_name;
			ret = of_property_read_string_index(
				dev_node, "reg-names", j, &bid_name);

			if (ret) {
				break;
			}
			if (!strcmp(bid_name, irq_reg_space)) {
				irq_reg_bid = j;
				break;
			}
		}
		if (irq_reg_bid < 0) {
			pr_err("Could not find a reg bid for %s\n",
			       irq_reg_space);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_u64(event_info, "irq-src-reg",
					   &irq_src_reg);
		if (ret) {
			pr_err("Error getting irq-src-reg from dt: %d\n", ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_u64(event_info, "irq-reset-reg",
					   &irq_reset_reg);
		if (ret) {
			pr_err("Error getting irq-reset-reg from dt: %d\n",
			       ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = of_property_read_u64(event_info, "irq-mask-reg",
					   &irq_mask_reg);
		if (ret) {
			pr_err("Error getting irq-mask-reg from dt: %d\n", ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		ret = lwis_interrupt_set_event_info(
			lwis_dev->irqs, i, irq_reg_space, irq_reg_bid,
			(int64_t *)irq_events, irq_events_num, int_reg_bits,
			int_reg_bits_num, irq_src_reg, irq_reset_reg,
			irq_mask_reg);
		if (ret) {
			pr_err("Error setting event info for interrupt %d %d\n",
			       i, ret);
			kfree(irq_events);
			kfree(int_reg_bits);
			goto error_event_infos;
		}

		of_node_put(event_info);
		i++;
	}

	lwis_interrupt_print(lwis_dev->irqs);

	return 0;
error_event_infos:
	for (i = 0; i < count; ++i) {
		// TODO(yromanenko): lwis_interrupt_put
	}
error_get_irq:
	lwis_interrupt_list_free(lwis_dev->irqs);
	lwis_dev->irqs = NULL;
	return ret;
}

static int parse_phys(struct lwis_device *lwis_dev)
{
	struct device *dev;
	struct device_node *dev_node;
	int i;
	int ret;
	int count;
	const char *name;

	dev = &(lwis_dev->plat_dev->dev);
	dev_node = dev->of_node;

	count = of_count_phandle_with_args(dev_node, "phys", "#phy-cells");

	/* No PHY found, just return */
	if (count <= 0) {
		return 0;
	}

	lwis_dev->phys = lwis_phy_list_alloc(count);
	if (IS_ERR(lwis_dev->phys)) {
		pr_err("Failed to allocate PHY list\n");
		return PTR_ERR(lwis_dev->phys);
	}

	for (i = 0; i < count; ++i) {
		of_property_read_string_index(dev_node, "phy-names", i, &name);
		ret = lwis_phy_get(lwis_dev->phys, (char *)name, dev);
		if (ret < 0) {
			pr_err("Error adding PHY[%d]\n", i);
			goto error_parse_phy;
		}
	}

	lwis_phy_print(lwis_dev->phys);

	return 0;

error_parse_phy:
	for (i = 0; i < count; ++i) {
		lwis_phy_put_by_idx(lwis_dev->phys, i, dev);
	}
	lwis_phy_list_free(lwis_dev->phys);
	lwis_dev->phys = NULL;
	return ret;
}

static void parse_bitwidths(struct lwis_device *lwis_dev)
{
	int ret;
	struct device *dev;
	struct device_node *dev_node;
	u32 addr_bitwidth = 32;
	u32 value_bitwidth = 32;

	dev = &(lwis_dev->plat_dev->dev);
	dev_node = dev->of_node;

	ret = of_property_read_u32(dev_node, "reg-addr-bitwidth",
				   &addr_bitwidth);
	pr_info("Addr bitwidth set to%s: %d\n", ret ? " default" : "",
		addr_bitwidth);

	ret = of_property_read_u32(dev_node, "reg-value-bitwidth",
				   &value_bitwidth);
	pr_info("Value bitwidth set to%s: %d\n", ret ? " default" : "",
		value_bitwidth);

	lwis_dev->reg_addr_bitwidth = addr_bitwidth;
	lwis_dev->reg_value_bitwidth = value_bitwidth;
}

int lwis_base_parse_dt(struct lwis_device *lwis_dev)
{
	struct device *dev;
	struct device_node *dev_node;
	struct property *iommus;
	int iommus_len = 0;
	const char *name_str;
	int ret = 0;

	dev = &(lwis_dev->plat_dev->dev);
	dev_node = dev->of_node;

	if (!dev_node) {
		pr_err("Cannot find device node\n");
		return -ENODEV;
	}

	ret = of_property_read_string(dev_node, "node-name", &name_str);
	if (ret) {
		pr_err("Error parsing node name\n");
		return -EINVAL;
	}
	strncpy(lwis_dev->name, name_str, LWIS_MAX_DEVICE_NAME_STRING);

	pr_info("Device tree entry [%s] - begin\n", lwis_dev->name);

	ret = parse_gpios(lwis_dev, "enable", &lwis_dev->enable_gpios_present);
	if (ret) {
		pr_err("Error parsing enable-gpios\n");
		return ret;
	}

	ret = parse_gpios(lwis_dev, "reset", &lwis_dev->reset_gpios_present);
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

	ret = parse_interrupts(lwis_dev);
	if (ret) {
		pr_err("Error parsing interrupts\n");
		return ret;
	}

	ret = parse_phys(lwis_dev);
	if (ret) {
		pr_err("Error parsing phy's\n");
		return ret;
	}

	parse_bitwidths(lwis_dev);

	iommus = of_find_property(dev_node, "iommus", &iommus_len);
	lwis_dev->has_iommu = iommus && iommus_len;

	dev_node->data = lwis_dev;

	pr_info("Device tree entry [%s] - end\n", lwis_dev->name);

	return ret;
}

int lwis_i2c_device_parse_dt(struct lwis_i2c_device *i2c_dev)
{
	struct device_node *dev_node;
	struct device_node *dev_node_i2c;
	int ret;

	dev_node = i2c_dev->base_dev.plat_dev->dev.of_node;

	dev_node_i2c = of_parse_phandle(dev_node, "i2c-bus", 0);
	if (!dev_node_i2c) {
		pr_err("Cannot find i2c-bus node\n");
		return -ENODEV;
	}

	i2c_dev->adapter = of_find_i2c_adapter_by_node(dev_node_i2c);
	if (!i2c_dev->adapter) {
		pr_err("Cannot find i2c adapter\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(dev_node, "i2c-addr",
				   (u32 *)&i2c_dev->address);
	if (ret) {
		pr_err("Failed to read i2c-addr\n");
		return ret;
	}

	return 0;
}

int lwis_ioreg_device_parse_dt(struct lwis_ioreg_device *ioreg_dev)
{
	struct device_node *dev_node;
	int i;
	int ret;
	int blocks;
	int reg_tuple_size;
	const char *name;

	dev_node = ioreg_dev->base_dev.plat_dev->dev.of_node;
	reg_tuple_size = of_n_addr_cells(dev_node) + of_n_size_cells(dev_node);

	blocks = of_property_count_elems_of_size(dev_node, "reg",
						 reg_tuple_size * sizeof(u32));
	if (blocks <= 0) {
		pr_err("No register space found\n");
		return -EINVAL;
	}

	ret = lwis_ioreg_list_alloc(ioreg_dev, blocks);
	if (ret) {
		pr_err("Failed to allocate ioreg list\n");
		return ret;
	}

	for (i = 0; i < blocks; ++i) {
		of_property_read_string_index(dev_node, "reg-names", i, &name);
		ret = lwis_ioreg_get(ioreg_dev, i, (char *)name);
		if (ret) {
			pr_err("Cannot set ioreg info\n");
			goto error_ioreg;
		}
	}

	return 0;

error_ioreg:
	for (i = 0; i < blocks; ++i) {
		lwis_ioreg_put_by_idx(ioreg_dev, i);
	}
	lwis_ioreg_list_free(ioreg_dev);
	return ret;
}

int lwis_top_device_parse_dt(struct lwis_top_device *top_dev)
{
	/* To be implemented */
	return 0;
}
