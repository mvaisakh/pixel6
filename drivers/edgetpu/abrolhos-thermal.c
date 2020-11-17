// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU thermal driver for Abrolhos.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/gfp.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/thermal.h>

#include "abrolhos-firmware.h"
#include "abrolhos-platform.h"
#include "abrolhos-pm.h"
#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-mmu.h"
#include "edgetpu-thermal.h"

static const unsigned long state_mapping[] = {
	TPU_ACTIVE_OD,
	TPU_ACTIVE_NOM,
	TPU_ACTIVE_UD,
	TPU_ACTIVE_SUD,
	TPU_RETENTION_CLOCKS_SLOW,
	TPU_SLEEP_CLOCKS_SLOW,
	TPU_SLEEP_CLOCKS_OFF,
	TPU_DEEP_SLEEP_CLOCKS_FAST,
	TPU_DEEP_SLEEP_CLOCKS_SLOW,
	TPU_DEEP_SLEEP_CLOCKS_OFF,
	TPU_OFF,
};

/*
 * Sequence need to be kept to make power increasing
 * to make sure we always return the highest state.
 */
static const struct edgetpu_state_pwr state_pwr_map[] = {
	{ TPU_ACTIVE_OD, 198 },
	{ TPU_ACTIVE_NOM, 165 },
	{ TPU_ACTIVE_UD, 131 },
	{ TPU_ACTIVE_SUD, 102 },
	{ TPU_SLEEP_CLOCKS_SLOW, 66 },
	{ TPU_SLEEP_CLOCKS_OFF, 66 },
	{ TPU_RETENTION_CLOCKS_SLOW, 49 },
	{ TPU_DEEP_SLEEP_CLOCKS_FAST, 43 },
	{ TPU_DEEP_SLEEP_CLOCKS_SLOW, 6 },
	{ TPU_DEEP_SLEEP_CLOCKS_OFF, 6 },
	{ TPU_OFF, 0 },
};

#define find_state_pwr(i, cmp_left, cmp_right, list, out_left, out_right)      \
	do {                                                                   \
		if (cmp_left == cmp_right) {                                   \
			out_left = out_right;                                  \
			return 0;                                              \
		}                                                              \
		i++;                                                           \
	} while (i < ARRAY_SIZE(list))

static int edgetpu_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	*state = ARRAY_SIZE(state_mapping) - 1;
	return 0;
}

/* Set cooling state
 * Re-using code from abrohlos-platform.
 * TODO: move to external call
 */
static int edgetpu_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state_original)
{
	int ret;
	struct edgetpu_thermal *cooling = cdev->devdata;
	struct device *dev = cooling->dev;
	unsigned long pwr_state;

	if (WARN_ON(state_original >= ARRAY_SIZE(state_mapping))) {
		dev_err(dev, "%s: invalid cooling state %lu\n", __func__,
			state_original);
		return -EINVAL;
	}

	mutex_lock(&cooling->lock);
	pwr_state = state_mapping[state_original];
	if (state_original != cooling->cooling_state) {
		ret = exynos_acpm_set_policy(TPU_ACPM_DOMAIN, pwr_state);
		if (ret) {
			dev_err(dev, "error setting tpu policy: %d\n", ret);
			mutex_unlock(&cooling->lock);
			return ret;
		}
		cooling->cooling_state = state_original;
	}

	mutex_unlock(&cooling->lock);
	return 0;
}

static int edgetpu_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	int ret = 0;
	struct edgetpu_thermal *cooling = cdev->devdata;

	*state = cooling->cooling_state;
	if (*state >= ARRAY_SIZE(state_mapping)) {
		dev_warn(cooling->dev, "Unknown cooling state: %lu, resetting\n", *state);
		mutex_lock(&cooling->lock);

		ret = exynos_acpm_set_policy(TPU_ACPM_DOMAIN, TPU_ACTIVE_OD);
		if (ret) {
			dev_err(cooling->dev, "error setting tpu policy: %d\n", ret);
			mutex_unlock(&cooling->lock);
			return ret;
		}

		//setting back to "no cooling"
		cooling->cooling_state = 0;
		mutex_unlock(&cooling->lock);
	}

	return 0;
}

static int edgetpu_state2power_internal(unsigned long state, u32 *power,
					struct device *dev)
{
	int i = 0;

	find_state_pwr(i, state, state_pwr_map[i].state, state_pwr_map, *power,
		       state_pwr_map[i].power);
	dev_err(dev, "Unknown state req for: %lu\n", state);
	*power = 0;
	WARN_ON(1);
	return -EINVAL;
}

static int edgetpu_get_requested_power(struct thermal_cooling_device *cdev,
				       struct thermal_zone_device *tz,
				       u32 *power)
{
	unsigned long state_original;
	struct edgetpu_thermal *cooling = cdev->devdata;

	state_original = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);
	return edgetpu_state2power_internal(state_original, power,
					    cooling->dev);
}

static int edgetpu_state2power(struct thermal_cooling_device *cdev,
			       struct thermal_zone_device *tz,
			       unsigned long state, u32 *power)
{
	struct edgetpu_thermal *cooling = cdev->devdata;

	if (state >= ARRAY_SIZE(state_mapping)) {
		dev_err(cooling->dev, "%s: invalid state: %lu\n", __func__,
			state);
		return -EINVAL;
	}

	return edgetpu_state2power_internal(state_mapping[state], power,
					    cooling->dev);
}

static int edgetpu_power2state(struct thermal_cooling_device *cdev,
			       struct thermal_zone_device *tz, u32 power,
			       unsigned long *state)
{
	int i;
	struct edgetpu_thermal *cooling = cdev->devdata;

	for (i = 0; i < ARRAY_SIZE(state_pwr_map); i++) {
		if (power >= state_pwr_map[i].power) {
			*state = state_pwr_map[i].state;
			return 0;
		}
	}

	dev_err(cooling->dev, "No power2state mapping found: %d\n", power);
	WARN_ON(1);
	return -EINVAL;
}

static struct thermal_cooling_device_ops edgetpu_cooling_ops = {
	.get_max_state = edgetpu_get_max_state,
	.get_cur_state = edgetpu_get_cur_state,
	.set_cur_state = edgetpu_set_cur_state,
	.get_requested_power = edgetpu_get_requested_power,
	.state2power = edgetpu_state2power,
	.power2state = edgetpu_power2state,
};

static void tpu_thermal_exit_cooling(struct edgetpu_thermal *thermal)
{
	thermal_cooling_device_unregister(thermal->cdev);
}

static void tpu_thermal_exit(struct edgetpu_thermal *thermal)
{
	tpu_thermal_exit_cooling(thermal);
	debugfs_remove_recursive(thermal->cooling_root);
}

static void devm_tpu_thermal_release(struct device *dev, void *res)
{
	struct edgetpu_thermal *thermal = res;

	tpu_thermal_exit(thermal);
}

static int
tpu_thermal_cooling_register(struct edgetpu_thermal *thermal, char *type)
{
	struct device_node *cooling_node = NULL;

	thermal->op_data = NULL;

	mutex_init(&thermal->lock);
	cooling_node = of_find_node_by_name(NULL, "tpu-cooling");
	if (!cooling_node)
		dev_warn(thermal->dev, "failed to find cooling node\n");
	// Initialize the cooling state as 0, means "no cooling"
	thermal->cooling_state = 0;
	thermal->cdev = thermal_of_cooling_device_register(
		cooling_node, type, thermal, &edgetpu_cooling_ops);
	if (IS_ERR(thermal->cdev))
		return PTR_ERR(thermal->cdev);
	return 0;
}

static int tpu_thermal_init(struct edgetpu_thermal *thermal, struct device *dev)
{
	int err;

	thermal->dev = dev;
	thermal->cooling_root =
		debugfs_create_dir("cooling", edgetpu_fs_debugfs_dir());

	err = tpu_thermal_cooling_register(thermal, EDGETPU_COOLING_NAME);
	if (err) {
		dev_err(dev, "failed to initialize external cooling\n");
		tpu_thermal_exit(thermal);
		return err;
	}

	return 0;
}

struct edgetpu_thermal *devm_tpu_thermal_create(struct device *dev)
{
	struct edgetpu_thermal *thermal;
	int err;

	thermal = devres_alloc(devm_tpu_thermal_release, sizeof(*thermal),
			       GFP_KERNEL);
	if (!thermal)
		return ERR_PTR(-ENOMEM);

	err = tpu_thermal_init(thermal, dev);
	if (err) {
		devres_free(thermal);
		return ERR_PTR(err);
	}

	devres_add(dev, thermal);
	return thermal;
}
