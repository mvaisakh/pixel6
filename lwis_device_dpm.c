/*
 * Google LWIS Dynamic Power Managerment
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-dpm: " fmt

#include "lwis_device_dpm.h"

#include <linux/clk.h>
#include <linux/slab.h>

#include "lwis_init.h"
#include "lwis_platform.h"

#define LWIS_DRIVER_NAME "lwis-dpm"

static struct lwis_device_subclass_operations dpm_vops = {
	.register_io = NULL,
	.device_enable = NULL,
	.device_disable = NULL,
	.event_enable = NULL,
	.event_flags_updated = NULL,
};

static struct lwis_event_subscribe_operations dpm_subscribe_ops = {
	.subscribe_event = NULL,
	.unsubscribe_event = NULL,
	.notify_event_subscriber = NULL,
	.release = NULL,
};

/*
 *  lwis_dpm_update_clock: update clock settings to lwis device.
 */
int lwis_dpm_update_clock(struct lwis_device *lwis_dev,
			  struct lwis_clk_setting *clk_settings,
			  size_t num_settings)
{
	int ret = 0, i, clk_index;
	uint32_t old_clk;

	if (!lwis_dev->clocks) {
		dev_err(lwis_dev->dev, "%s has no clocks\n", lwis_dev->name);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < num_settings; ++i) {
		clk_index = clk_settings[i].clk_index;
		old_clk = clk_get_rate(lwis_dev->clocks->clk[clk_index].clk);
		if (old_clk == clk_settings[i].frequency)
			continue;

		if (clk_index >= lwis_dev->clocks->count) {
			dev_err(lwis_dev->dev, "%s clk index %d is invalid\n",
				lwis_dev->name, clk_index);
			ret = -EINVAL;
			goto out;
		}

		if (clk_index == 0 &&
		    lwis_dev->clock_family != CLOCK_FAMILY_INVALID &&
		    lwis_dev->clock_family < CLOCK_FAMILY_MAX) {
			/* convert value to KHz */
			lwis_platform_update_qos(
				lwis_dev, clk_settings[i].frequency / 1000);
		} else {
			ret = clk_set_rate(lwis_dev->clocks->clk[clk_index].clk,
					   clk_settings[i].frequency);
			if (ret) {
				dev_err(lwis_dev->dev,
					"Error updating clock %s freq: %u\n",
					lwis_dev->clocks->clk[clk_index].name,
					clk_settings[i].frequency);
				goto out;
			}
		}

		dev_info(lwis_dev->dev,
			 "Update %s freq from %u to %u, clock read back: %lu\n",
			 lwis_dev->clocks->clk[clk_index].name, old_clk,
			 clk_settings[i].frequency,
			 clk_get_rate(lwis_dev->clocks->clk[clk_index].clk));
	}
out:
	kfree(clk_settings);
	return ret;
}

static int __init lwis_dpm_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_dpm_device *dpm_dev;
	struct device *dev = &plat_dev->dev;

	/* Allocate top device specific data construct */
	dpm_dev = devm_kzalloc(dev, sizeof(struct lwis_dpm_device), GFP_KERNEL);
	if (!dpm_dev) {
		dev_err(dev, "Failed to allocate dpm device structure\n");
		return -ENOMEM;
	}

	dpm_dev->base_dev.type = DEVICE_TYPE_DPM;
	dpm_dev->base_dev.vops = dpm_vops;
	dpm_dev->base_dev.subscribe_ops = dpm_subscribe_ops;

	/* Call the base device probe function */
	ret = lwis_base_probe((struct lwis_device *)dpm_dev, plat_dev);
	if (ret)
		dev_err(dev, "Error in lwis base probe, ret: %d\n", ret);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_DPM_DEVICE_COMPAT },
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
 *  lwis_dpm_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_dpm_device_init(void)
{
	int ret = 0;

	pr_info("Dpm device initialization\n");

	ret = platform_driver_probe(&lwis_driver, lwis_dpm_device_probe);
	if (ret)
		pr_err("platform_driver_probe failed - %d\n", ret);

	return ret;
}

int lwis_dpm_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
