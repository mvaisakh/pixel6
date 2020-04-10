/*
 * Google LWIS Lwis Exynos-specific platform functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/slab.h>
#include <linux/pm_qos.h>
#include <linux/iommu.h>
#include <linux/exynos_iovmm.h>
#include <linux/of.h>

#include "lwis_platform.h"
#include "lwis_platform_exynos.h"

int lwis_platform_probe(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;
	BUG_ON(!lwis_dev);
	platform = kzalloc(sizeof(struct lwis_platform), GFP_KERNEL);
	if (IS_ERR_OR_NULL(platform)) {
		return -ENOMEM;
	}
	lwis_dev->platform = platform;

	/* Enable runtime power management for the platform device */
	pm_runtime_enable(&lwis_dev->plat_dev->dev);

	return 0;
}

static int __attribute__((unused))
iovmm_fault_handler(struct iommu_domain *domain, struct device *dev,
		    unsigned long fault_addr, int fault_flag, void *token)
{
	struct lwis_device *lwis_dev = (struct lwis_device *)token;
	pr_err("IOVMM Fault - Addr: %016llx Flag: %08x Device: %p\n",
	       fault_addr, fault_flag, lwis_dev);
	return -EINVAL;
}

int lwis_platform_device_enable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_platform *platform;
	// TODO: Refactor
	const uint32_t int_cam_qos = 680000;
	const uint32_t int_qos = 107000;
	const uint32_t mif_qos = 2093000;
	const uint32_t cam_qos = 680000;
	const uint32_t hpg_qos = 1;
	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	/* Upref the runtime power management controls for the platform dev */
	ret = pm_runtime_get_sync(&lwis_dev->plat_dev->dev);
	if (ret < 0) {
		pr_err("Unable to enable platform device\n");
		return ret;
	}
	if (lwis_dev->has_iommu) {
		/* Activate IOMMU/SYSMMU for the platform device */
		ret = iovmm_activate(&lwis_dev->plat_dev->dev);
		if (ret < 0) {
			pr_err("Failed to enable IOMMU for the device: %d\n",
			       ret);
			return ret;
		}
		/* Set SYSMMU fault handler */
		iovmm_set_fault_handler(&lwis_dev->plat_dev->dev,
					iovmm_fault_handler, lwis_dev);
	}
	/* Set hardcoded DVFS levels */
	if (!pm_qos_request_active(&platform->pm_qos_int_cam))
		pm_qos_add_request(&platform->pm_qos_int_cam,
				   PM_QOS_INTCAM_THROUGHPUT, int_cam_qos);
	if (!pm_qos_request_active(&platform->pm_qos_int))
		pm_qos_add_request(&platform->pm_qos_int,
				   PM_QOS_DEVICE_THROUGHPUT, int_qos);
	if (!pm_qos_request_active(&platform->pm_qos_mem))
		pm_qos_add_request(&platform->pm_qos_mem, PM_QOS_BUS_THROUGHPUT,
				   mif_qos);
	if (!pm_qos_request_active(&platform->pm_qos_cam))
		pm_qos_add_request(&platform->pm_qos_cam, PM_QOS_CAM_THROUGHPUT,
				   cam_qos);
	if (!pm_qos_request_active(&platform->pm_qos_hpg))
		pm_qos_add_request(&platform->pm_qos_hpg, PM_QOS_CPU_ONLINE_MIN,
				   hpg_qos);


	return 0;
}

int lwis_platform_device_disable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_platform *platform;
	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	/* We can't remove fault handlers, so there's no call corresponding
	 * to the iovmm_set_fault_handler above */

	if (pm_qos_request_active(&platform->pm_qos_int_cam))
		pm_qos_remove_request(&platform->pm_qos_int_cam);
	if (pm_qos_request_active(&platform->pm_qos_int))
		pm_qos_remove_request(&platform->pm_qos_int);
	if (pm_qos_request_active(&platform->pm_qos_mem))
		pm_qos_remove_request(&platform->pm_qos_mem);
	if (pm_qos_request_active(&platform->pm_qos_cam))
		pm_qos_remove_request(&platform->pm_qos_cam);
	if (pm_qos_request_active(&platform->pm_qos_hpg))
		pm_qos_remove_request(&platform->pm_qos_hpg);

	if (lwis_dev->has_iommu) {
		/* Deactivate IOMMU/SYSMMU */
		iovmm_deactivate(&lwis_dev->plat_dev->dev);
	}

	/* Disable platform device */
	ret = pm_runtime_put_sync(&lwis_dev->plat_dev->dev);

	return ret;
}
