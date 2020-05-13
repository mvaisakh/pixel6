/*
 * Google LWIS SLC Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "lwis_device_slc.h"

#include <linux/device.h>
#include <linux/exynos_iovmm.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "pt/pt.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_DRIVER_NAME "lwis-slc"

#define SIZE_TO_KB(x) x / 1024

static int lwis_slc_enable(struct lwis_device *lwis_dev);
static int lwis_slc_disable(struct lwis_device *lwis_dev);

static struct lwis_device_subclass_operations slc_vops = {
	.register_io = NULL,
	.device_enable = lwis_slc_enable,
	.device_disable = lwis_slc_disable,
	.event_enable = NULL,
	.event_flags_updated = NULL,
};

static struct lwis_event_subscribe_operations slc_subscribe_ops = {
	.subscribe_event = NULL,
	.unsubscribe_event = NULL,
	.notify_event_subscriber = NULL,
	.release = NULL,
};

static int lwis_slc_enable(struct lwis_device *lwis_dev)
{
#ifdef CONFIG_OF
	struct lwis_slc_device *slc_dev = (struct lwis_slc_device *)lwis_dev;
	struct device_node *node = lwis_dev->plat_dev->dev.of_node;
	static size_t pt_size_kb[NUM_PT] = { 512, 768, 1024, 2048, 3072 };
	int num_pt = 0, i = 0;

	num_pt = of_property_count_strings(node, "pt_id");
	if (num_pt != NUM_PT) {
		dev_err(lwis_dev->dev, "Unexpected number of partitions\n");
		return -EINVAL;
	}

	/* Initialize SLC partitions and get a handle */
	slc_dev->pt_hnd = pt_client_register(node, NULL, NULL);
	for (i = 0; i < NUM_PT; i++) {
		slc_dev->pt[i].id = i;
		slc_dev->pt[i].size_kb = pt_size_kb[i];
		slc_dev->pt[i].ptid = PT_PTID_INVALID;
	}
	return 0;
#else  /* CONFIG_OF not defined */
	return -ENOENT;
#endif /* CONFIG_OF */
}

static int lwis_slc_disable(struct lwis_device *lwis_dev)
{
	struct lwis_slc_device *slc_dev = (struct lwis_slc_device *)lwis_dev;
	int i = 0;

	for (i = 0; i < NUM_PT; i++) {
		if (slc_dev->pt[i].ptid != PT_PTID_INVALID) {
			dev_err(slc_dev->base_dev.dev, "Disabling one id");
			pt_client_disable(slc_dev->pt_hnd, slc_dev->pt[i].id);
			slc_dev->pt[i].ptid = PT_PTID_INVALID;
		}
	}
	pt_client_unregister(slc_dev->pt_hnd);
	return 0;
}

int lwis_slc_buffer_alloc(struct lwis_device *lwis_dev,
			  struct lwis_alloc_buffer_info *alloc_info)
{
	struct lwis_slc_device *slc_dev = (struct lwis_slc_device *)lwis_dev;
	int i = 0;
	ptid_t ptid;

	for (i = 0; i < NUM_PT; i++) {
		if (slc_dev->pt[i].ptid == PT_PTID_INVALID &&
		    slc_dev->pt[i].size_kb >= SIZE_TO_KB(alloc_info->size)) {
			ptid = pt_client_enable(slc_dev->pt_hnd,
						slc_dev->pt[i].id);
			dev_info(
				slc_dev->base_dev.dev,
				"pt_client_enable called with %d and return %d",
				slc_dev->pt[i].id, (int)ptid);
			if (ptid != PT_PTID_INVALID) {
				slc_dev->pt[i].ptid = ptid;
				alloc_info->dma_fd = ptid;
				return 0;
			} else {
				dev_err(lwis_dev->dev,
					"Failed to enable partition id %d\n",
					slc_dev->pt[i].id);
				return -EPROTO;
			}
		}
	}
	dev_err(lwis_dev->dev,
		"Failed to find valid partition, largest size supported is 2048KB\n");
	return -EINVAL;
}

int lwis_slc_buffer_free(struct lwis_device *lwis_dev, ptid_t ptid)
{
	struct lwis_slc_device *slc_dev = (struct lwis_slc_device *)lwis_dev;
	int i = 0;

	dev_info(slc_dev->base_dev.dev, "lwis_slc_buffer_free ptid %d",
		 (int)ptid);
	for (i = 0; i < NUM_PT; i++) {
		if (slc_dev->pt[i].ptid == ptid) {
			dev_err(slc_dev->base_dev.dev,
				"Calling pt_client_disable id %d", i);
			pt_client_disable(slc_dev->pt_hnd, i);
			slc_dev->pt[i].ptid = PT_PTID_INVALID;
			return 0;
		}
	}
	dev_err(lwis_dev->dev, "Failed to find ptid %d\n", ptid);
	return -EINVAL;
}

static int __init lwis_slc_device_probe(struct platform_device *plat_dev)
{
	int ret = 0;
	struct lwis_slc_device *slc_dev;

	/* Allocate SLC device specific data construct */
	slc_dev = kzalloc(sizeof(struct lwis_slc_device), GFP_KERNEL);
	if (!slc_dev) {
		pr_err("Failed to allocate slc device structure\n");
		return -ENOMEM;
	}

	slc_dev->base_dev.type = DEVICE_TYPE_SLC;
	slc_dev->base_dev.vops = slc_vops;
	slc_dev->base_dev.subscribe_ops = slc_subscribe_ops;

	/* Call the base device probe function */
	ret = lwis_base_probe((struct lwis_device *)slc_dev, plat_dev);
	if (ret) {
		pr_err("Error in lwis base probe\n");
		goto error_probe;
	}

	dev_info(slc_dev->base_dev.dev, "SLC Device Probe: Success\n");

	return 0;

error_probe:
	kfree(slc_dev);
	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id lwis_id_match[] = {
	{ .compatible = LWIS_SLC_DEVICE_COMPAT },
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
 *  lwis_slc_device_init: Init function that will be called by the kernel
 *  initialization routines.
 */
int __init lwis_slc_device_init(void)
{
	int ret = 0;

	pr_info("SLC device initialization\n");

	ret = platform_driver_probe(&lwis_driver, lwis_slc_device_probe);
	if (ret) {
		pr_err("platform_driver_probe failed - %d", ret);
	}

	return ret;
}

int lwis_slc_device_deinit(void)
{
	platform_driver_unregister(&lwis_driver);
	return 0;
}
