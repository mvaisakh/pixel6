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

#include "lwis_dpm.h"

#include <linux/clk.h>
#include <linux/slab.h>

#include "lwis_platform.h"

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
