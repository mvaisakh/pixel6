/*
 * Google LWIS Lwis Exynos-specific platform functions
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "lwis_platform_exynos.h"

#include <linux/exynos_iovmm.h>
#include <linux/iommu.h>
#include <linux/of.h>
#include <linux/pm_qos.h>
#include <linux/slab.h>

#include "lwis_dpm.h"
#include "lwis_platform.h"

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
	pr_err("IOVMM Fault - Addr: %016lx Flag: %08x Device: %p\n",
	       fault_addr, fault_flag, lwis_dev);
	return -EINVAL;
}

int lwis_platform_device_enable(struct lwis_device *lwis_dev)
{
	int ret;
	struct lwis_platform *platform;
	// TODO: Refactor

	const uint32_t int_qos = 465000;
	const uint32_t mif_qos = 2093000;
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
	if (!pm_qos_request_active(&platform->pm_qos_mem))
		pm_qos_add_request(&platform->pm_qos_mem, PM_QOS_BUS_THROUGHPUT,
				   mif_qos);
	if (!pm_qos_request_active(&platform->pm_qos_int))
		pm_qos_add_request(&platform->pm_qos_int,
				   PM_QOS_DEVICE_THROUGHPUT, int_qos);
	if (!pm_qos_request_active(&platform->pm_qos_hpg))
		pm_qos_add_request(&platform->pm_qos_hpg, PM_QOS_CPU_ONLINE_MIN,
				   hpg_qos);

	if (lwis_dev->clock_family != CLOCK_FAMILY_INVALID &&
	    lwis_dev->clock_family < CLOCK_FAMILY_MAX) {
		lwis_platform_update_qos(lwis_dev, 67000);
	}

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

	lwis_platform_remove_qos(lwis_dev);

	if (lwis_dev->has_iommu) {
		/* Deactivate IOMMU/SYSMMU */
		iovmm_deactivate(&lwis_dev->plat_dev->dev);
	}

	/* Disable platform device */
	ret = pm_runtime_put_sync(&lwis_dev->plat_dev->dev);

	return ret;
}

int lwis_platform_update_qos(struct lwis_device *lwis_dev, uint32_t value)
{
	struct lwis_platform *platform;
	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	if (lwis_dev->last_requested_clock == value || value == 0) {
		return 0;
	}

	switch (lwis_dev->clock_family) {
	case CLOCK_FAMILY_INTCAM:
		if (!pm_qos_request_active(&platform->pm_qos_int_cam)) {
			pm_qos_add_request(&platform->pm_qos_int_cam,
					   PM_QOS_INTCAM_THROUGHPUT, value);
		} else {
			pm_qos_update_request(&platform->pm_qos_int_cam, value);
		}
		break;
	case CLOCK_FAMILY_CAM:
		if (!pm_qos_request_active(&platform->pm_qos_cam)) {
			pm_qos_add_request(&platform->pm_qos_cam,
					   PM_QOS_CAM_THROUGHPUT, value);
		} else {
			pm_qos_update_request(&platform->pm_qos_cam, value);
		}
		break;
	case CLOCK_FAMILY_TNR:
#if defined(CONFIG_SOC_GS101)
		if (!pm_qos_request_active(&platform->pm_qos_tnr)) {
			pm_qos_add_request(&platform->pm_qos_tnr,
					   PM_QOS_TNR_THROUGHPUT, value);
		} else {
			pm_qos_update_request(&platform->pm_qos_tnr, value);
		}
#endif
		break;
	default:
		dev_err(lwis_dev->dev, "%s clk family %d is invalid\n",
			lwis_dev->name, lwis_dev->clock_family);
		return -EINVAL;
		break;
	}

	dev_info(lwis_dev->dev,
		 "Updating clock for clock_family %d, freq from %u to %u\n",
		 lwis_dev->clock_family, lwis_dev->last_requested_clock, value);
	lwis_dev->last_requested_clock = value;

	return 0;
}

int lwis_platform_remove_qos(struct lwis_device *lwis_dev)
{
	struct lwis_platform *platform;
	BUG_ON(!lwis_dev);
	platform = lwis_dev->platform;
	if (!platform) {
		return -ENODEV;
	}

	if (pm_qos_request_active(&platform->pm_qos_int))
		pm_qos_remove_request(&platform->pm_qos_int);
	if (pm_qos_request_active(&platform->pm_qos_mem))
		pm_qos_remove_request(&platform->pm_qos_mem);
	if (pm_qos_request_active(&platform->pm_qos_hpg))
		pm_qos_remove_request(&platform->pm_qos_hpg);

	switch (lwis_dev->clock_family) {
	case CLOCK_FAMILY_INTCAM:
		if (pm_qos_request_active(&platform->pm_qos_int_cam)) {
			pm_qos_remove_request(&platform->pm_qos_int_cam);
		}
		lwis_dev->last_requested_clock = 0;
		break;
	case CLOCK_FAMILY_CAM:
		if (pm_qos_request_active(&platform->pm_qos_cam)) {
			pm_qos_remove_request(&platform->pm_qos_cam);
		}
		lwis_dev->last_requested_clock = 0;
		break;
	case CLOCK_FAMILY_TNR:
#if defined(CONFIG_SOC_GS101)
		if (pm_qos_request_active(&platform->pm_qos_tnr)) {
			pm_qos_remove_request(&platform->pm_qos_tnr);
		}
		lwis_dev->last_requested_clock = 0;
#endif
		break;
	default:
		break;
	}
	return 0;
}
