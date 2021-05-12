// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU thermal driver for Abrolhos.
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
#include <linux/version.h>
#include <linux/of.h>

#include "abrolhos-platform.h"
#include "abrolhos-pm.h"
#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-mmu.h"
#include "edgetpu-thermal.h"

#define MAX_NUM_TPU_STATES 10
#define OF_DATA_NUM_MAX MAX_NUM_TPU_STATES * 2
static struct edgetpu_state_pwr state_pwr_map[MAX_NUM_TPU_STATES] = {0};

static int edgetpu_get_max_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	struct edgetpu_thermal *thermal = cdev->devdata;

	if (thermal->tpu_num_states <= 0)
		return -ENOSYS;

	*state = thermal->tpu_num_states - 1;
	return 0;
}

/*
 * Set cooling state.
 */
static int edgetpu_set_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long state_original)
{
	int ret;
	struct edgetpu_thermal *cooling = cdev->devdata;
	struct device *dev = cooling->dev;
	struct edgetpu_dev *etdev = cooling->etdev;
	unsigned long pwr_state;

	if (state_original >= cooling->tpu_num_states) {
		dev_err(dev, "%s: invalid cooling state %lu\n", __func__,
			state_original);
		return -EINVAL;
	}

	mutex_lock(&cooling->lock);
	pwr_state = state_pwr_map[state_original].state;
	if (state_original != cooling->cooling_state) {
		/*
		 * Cap the minimum state we request here.
		 * We cannot go to states below SUD until firmware/runtime
		 * handshake is added.
		 */
		if (pwr_state < TPU_ACTIVE_SUD) {
			dev_warn_ratelimited(
				dev, "Unable to go to state %lu, going to %d",
				pwr_state, TPU_ACTIVE_SUD);
			pwr_state = TPU_ACTIVE_SUD;
		}

		ret = exynos_acpm_set_policy(TPU_ACPM_DOMAIN, pwr_state);
		if (ret) {
			dev_err(dev, "error setting tpu policy: %d\n", ret);
			goto out;
		}
		cooling->cooling_state = state_original;
		ret = edgetpu_kci_notify_throttling(etdev, pwr_state);
		if (ret) {
			/*
			 * TODO(b/185596886) : After FW adds a handler for this
			 * KCI, return the correct value of ret and change the
			 * debug message to an error message
			 */
			etdev_dbg(
			etdev, "Failed to notify FW about state %lu, error:%d",
			pwr_state, ret);
			ret = 0;
		}
	} else {
		ret = -EALREADY;
	}

out:
	mutex_unlock(&cooling->lock);
	return ret;
}

static int edgetpu_get_cur_state(struct thermal_cooling_device *cdev,
				 unsigned long *state)
{
	int ret = 0;
	struct edgetpu_thermal *cooling = cdev->devdata;

	*state = cooling->cooling_state;
	if (*state >= cooling->tpu_num_states) {
		dev_warn(cooling->dev,
			 "Unknown cooling state: %lu, resetting\n", *state);
		mutex_lock(&cooling->lock);

		ret = exynos_acpm_set_policy(TPU_ACPM_DOMAIN, TPU_ACTIVE_OD);
		if (ret) {
			dev_err(cooling->dev, "error setting tpu policy: %d\n",
				ret);
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
					struct edgetpu_thermal *thermal)
{
	int i;

	for (i = 0; i < thermal->tpu_num_states; ++i) {
		if (state == state_pwr_map[i].state) {
			*power = state_pwr_map[i].power;
			return 0;
		}
	}
	dev_err(thermal->dev, "Unknown state req for: %lu\n", state);
	*power = 0;
	return -EINVAL;
}

static int edgetpu_get_requested_power(struct thermal_cooling_device *cdev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
				       struct thermal_zone_device *tz,
#endif
				       u32 *power)
{
	unsigned long state_original;
	struct edgetpu_thermal *cooling = cdev->devdata;

	state_original = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);
	return edgetpu_state2power_internal(state_original, power,
					    cooling);
}

static int edgetpu_state2power(struct thermal_cooling_device *cdev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
			       struct thermal_zone_device *tz,
#endif
			       unsigned long state, u32 *power)
{
	struct edgetpu_thermal *cooling = cdev->devdata;

	if (state >= cooling->tpu_num_states) {
		dev_err(cooling->dev, "%s: invalid state: %lu\n", __func__,
			state);
		return -EINVAL;
	}

	return edgetpu_state2power_internal(state_pwr_map[state].state, power,
					    cooling);
}

static int edgetpu_power2state(struct thermal_cooling_device *cdev,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 10, 0)
			       struct thermal_zone_device *tz,
#endif
			       u32 power, unsigned long *state)
{
	int i, penultimate_throttle_state;
	struct edgetpu_thermal *thermal = cdev->devdata;

	*state = 0;
	if (thermal->tpu_num_states < 2)
		return thermal->tpu_num_states == 1 ? 0 : -ENOSYS;

	penultimate_throttle_state = thermal->tpu_num_states - 2;
	/*
	 * argument "power" is the maximum allowed power consumption in mW as
	 * defined by the PID control loop. Check for the first state that is
	 * less than or equal to the current allowed power. state_pwr_map is
	 * descending, so lowest power consumption is last value in the array
	 * return lowest state even if it consumes more power than allowed as
	 * not all platforms can handle throttling below an active state
	 */
	for (i = penultimate_throttle_state; i >= 0; --i) {
		if (power < state_pwr_map[i].power) {
			*state = i + 1;
			break;
		}
	}
	return 0;
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
	if (!IS_ERR_OR_NULL(thermal->cdev))
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

static int tpu_thermal_parse_dvfs_table(struct edgetpu_thermal *thermal)
{
	int row_size, col_size, tbl_size, i;
	int of_data_int_array[OF_DATA_NUM_MAX];

	if (of_property_read_u32_array(thermal->dev->of_node,
                                "tpu_dvfs_table_size", of_data_int_array, 2 ))
		goto error;

	row_size = of_data_int_array[0];
	col_size = of_data_int_array[1];
	tbl_size = row_size * col_size;
	if (row_size > MAX_NUM_TPU_STATES) {
		dev_err(thermal->dev, "too many TPU states\n");
		goto error;
	}

	if (tbl_size > OF_DATA_NUM_MAX)
		goto error;

	if (of_property_read_u32_array(thermal->dev->of_node,
                                "tpu_dvfs_table", of_data_int_array, tbl_size))
		goto error;

	thermal->tpu_num_states = row_size;
	for (i = 0; i < row_size; ++i) {
		int idx = col_size * i;
		state_pwr_map[i].state = of_data_int_array[idx];
		state_pwr_map[i].power = of_data_int_array[idx+1];
	}

	return 0;

error:
	dev_err(thermal->dev, "failed to parse DVFS table\n");
	return -EINVAL;
}

static int
tpu_thermal_cooling_register(struct edgetpu_thermal *thermal, char *type)
{
	struct device_node *cooling_node = NULL;
	int err = 0;

	thermal->op_data = NULL;
	thermal->tpu_num_states = 0;

	err = tpu_thermal_parse_dvfs_table(thermal);
	if (err)
		return err;

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
	struct dentry *d;

	d = debugfs_create_dir("cooling", edgetpu_fs_debugfs_dir());
	/* don't let debugfs creation failure abort the init procedure */
	if (!d)
		dev_warn(dev, "failed to create debug fs for cooling");
	thermal->dev = dev;
	thermal->cooling_root = d;

	err = tpu_thermal_cooling_register(thermal, EDGETPU_COOLING_NAME);
	if (err) {
		dev_err(dev, "failed to initialize external cooling\n");
		tpu_thermal_exit(thermal);
		return err;
	}

	return 0;
}

struct edgetpu_thermal
*devm_tpu_thermal_create(struct device *dev, struct edgetpu_dev *etdev)
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
	thermal->etdev = etdev;
	return thermal;
}
