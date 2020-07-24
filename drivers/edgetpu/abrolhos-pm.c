// SPDX-License-Identifier: GPL-2.0
/*
 * Abrolhos EdgeTPU power management support
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include "abrolhos-platform.h"
#include "abrolhos-pm.h"
#include "edgetpu-config.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-pm.h"

/* Default power state: the lowest power state that keeps firmware running */
// static int power_state = TPU_DEEP_SLEEP_CLOCKS_SLOW;
// TODO: switch back to the correct power state once b/160361784 is fixed
static int power_state = TPU_ACTIVE_NOM;

module_param(power_state, int, 0660);

#define MAX_VOLTAGE_VAL 1250000

static struct dentry *abrolhos_pwr_debugfs_dir;

static int abrolhos_pwr_state_init(struct device *dev)
{
	int ret;

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret) {
		dev_err(dev, "pm_runtime_get_sync returned %d\n", ret);
		return ret;
	}

	ret = exynos_acpm_set_init_freq(TPU_ACPM_DOMAIN, TPU_OFF);
	if (ret) {
		dev_err(dev, "error initializing tpu state: %d\n", ret);
		return ret;
	}

	ret = pm_runtime_put_sync(dev);
	if (ret) {
		dev_err(dev, "%s: pm_runtime_put_sync returned %d\n", __func__,
			ret);
		return ret;
	}

	return ret;
}

static int abrolhos_pwr_state_set(void *data, u64 val)
{
	int ret;
	int curr_state;
	struct device *dev = (struct device *)data;

	curr_state = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);

	dev_dbg(dev, "Power state %d -> %llu\n", curr_state, val);

	if (curr_state == TPU_OFF && val > TPU_OFF) {
		ret = pm_runtime_get_sync(dev);
		if (ret) {
			dev_err(dev, "pm_runtime_get_sync returned %d\n", ret);
			return ret;
		}
	}

	ret = exynos_acpm_set_rate(TPU_ACPM_DOMAIN, (unsigned long)val);
	if (ret) {
		dev_err(dev, "error setting tpu state: %d\n", ret);
		return ret;
	}

	if (curr_state != TPU_OFF && val == TPU_OFF) {
		ret = pm_runtime_put_sync(dev);
		if (ret) {
			dev_err(dev, "%s: pm_runtime_put_sync returned %d\n",
				__func__, ret);
			return ret;
		}
	}

	return ret;
}

static int abrolhos_pwr_state_get(void *data, u64 *val)
{
	struct device *dev = (struct device *)data;

	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN, 0);
	dev_dbg(dev, "current tpu state: %llu\n", *val);

	return 0;
}

static int abrolhos_pwr_policy_set(void *data, u64 val)
{
	struct edgetpu_platform_dev *edgetpu_pdev =
		(struct edgetpu_platform_dev *)data;
	struct edgetpu_platform_pwr *platform_pwr = &edgetpu_pdev->platform_pwr;
	int ret;

	mutex_lock(&platform_pwr->policy_lock);
	ret = exynos_acpm_set_policy(TPU_ACPM_DOMAIN, val);

	if (ret) {
		dev_err(edgetpu_pdev->edgetpu_dev.dev,
			"unable to set policy %lld (ret %d)\n", val, ret);
		mutex_unlock(&platform_pwr->policy_lock);
		return ret;
	}

	platform_pwr->curr_policy = val;
	mutex_unlock(&platform_pwr->policy_lock);
	return 0;
}

static int abrolhos_pwr_policy_get(void *data, u64 *val)
{
	struct edgetpu_platform_dev *edgetpu_pdev =
		(struct edgetpu_platform_dev *)data;
	struct edgetpu_platform_pwr *platform_pwr = &edgetpu_pdev->platform_pwr;

	mutex_lock(&platform_pwr->policy_lock);
	*val = platform_pwr->curr_policy;
	mutex_unlock(&platform_pwr->policy_lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_pwr_policy, abrolhos_pwr_policy_get,
			 abrolhos_pwr_policy_set, "%llu\n");

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_pwr_state, abrolhos_pwr_state_get,
			 abrolhos_pwr_state_set, "%llu\n");

static int edgetpu_core_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_CORE_DEBUG);
	return 0;
}

static int edgetpu_core_rate_set(void *data, u64 val)
{
	unsigned long dbg_rate_req;

	dbg_rate_req = TPU_DEBUG_REQ | TPU_CLK_CORE_DEBUG;
	dbg_rate_req |= (val / 1000);

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_core_rate, edgetpu_core_rate_get,
			 edgetpu_core_rate_set, "%llu\n");

static int edgetpu_ctl_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_CTL_DEBUG);
	return 0;
}

static int edgetpu_ctl_rate_set(void *data, u64 val)
{
	unsigned long dbg_rate_req;

	dbg_rate_req = TPU_DEBUG_REQ | TPU_CLK_CTL_DEBUG;
	dbg_rate_req |= (val / 1000);

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_ctl_rate, edgetpu_ctl_rate_get,
			 edgetpu_ctl_rate_set, "%llu\n");

static int edgetpu_axi_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_AXI_DEBUG);
	return 0;
}

static int edgetpu_axi_rate_set(void *data, u64 val)
{
	unsigned long dbg_rate_req;

	dbg_rate_req = TPU_DEBUG_REQ | TPU_CLK_AXI_DEBUG;
	dbg_rate_req |= (val / 1000);

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_axi_rate, edgetpu_axi_rate_get,
			 edgetpu_axi_rate_set, "%llu\n");

static int edgetpu_apb_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_APB_DEBUG);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_apb_rate, edgetpu_apb_rate_get, NULL,
			 "%llu\n");

static int edgetpu_uart_rate_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_CLK_UART_DEBUG);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_uart_rate, edgetpu_uart_rate_get, NULL,
			 "%llu\n");

static int edgetpu_vdd_int_m_set(void *data, u64 val)
{
	struct device *dev = (struct device *)data;
	unsigned long dbg_rate_req;

	if (val > MAX_VOLTAGE_VAL) {
		dev_err(dev, "Preventing INT_M voltage > %duV",
			MAX_VOLTAGE_VAL);
		return -EINVAL;
	}

	dbg_rate_req = TPU_DEBUG_REQ | TPU_VDD_INT_M_DEBUG;
	dbg_rate_req |= val;

	return exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
}

static int edgetpu_vdd_int_m_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_VDD_INT_M_DEBUG);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_vdd_int_m, edgetpu_vdd_int_m_get,
			 edgetpu_vdd_int_m_set, "%llu\n");

static int edgetpu_vdd_tpu_set(void *data, u64 val)
{
	int ret;
	struct device *dev = (struct device *)data;
	unsigned long dbg_rate_req;

	if (val > MAX_VOLTAGE_VAL) {
		dev_err(dev, "Preventing VDD_TPU voltage > %duV",
			MAX_VOLTAGE_VAL);
		return -EINVAL;
	}

	dbg_rate_req = TPU_DEBUG_REQ | TPU_VDD_TPU_DEBUG;
	dbg_rate_req |= val;

	ret = exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
	return ret;
}

static int edgetpu_vdd_tpu_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_VDD_TPU_DEBUG);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_vdd_tpu, edgetpu_vdd_tpu_get,
			 edgetpu_vdd_tpu_set, "%llu\n");

static int edgetpu_vdd_tpu_m_set(void *data, u64 val)
{
	int ret;
	struct device *dev = (struct device *)data;
	unsigned long dbg_rate_req;

	if (val > MAX_VOLTAGE_VAL) {
		dev_err(dev, "Preventing VDD_TPU voltage > %duV",
			MAX_VOLTAGE_VAL);
		return -EINVAL;
	}

	dbg_rate_req = TPU_DEBUG_REQ | TPU_VDD_TPU_M_DEBUG;
	dbg_rate_req |= val;

	ret = exynos_acpm_set_rate(TPU_ACPM_DOMAIN, dbg_rate_req);
	return ret;
}

static int edgetpu_vdd_tpu_m_get(void *data, u64 *val)
{
	*val = exynos_acpm_get_rate(TPU_ACPM_DOMAIN,
				    TPU_DEBUG_REQ | TPU_VDD_TPU_M_DEBUG);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_tpu_vdd_tpu_m, edgetpu_vdd_tpu_m_get,
		edgetpu_vdd_tpu_m_set, "%llu\n");

static int abrolhos_get_initial_pwr_state(struct device *dev)
{
	switch (power_state) {
	case TPU_DEEP_SLEEP_CLOCKS_SLOW:
	case TPU_DEEP_SLEEP_CLOCKS_FAST:
	case TPU_RETENTION_CLOCKS_SLOW:
	case TPU_ACTIVE_SUD:
	case TPU_ACTIVE_UD:
	case TPU_ACTIVE_NOM:
	case TPU_ACTIVE_OD:
		dev_info(dev, "Initial power state: %d\n", power_state);
		break;
	case TPU_OFF:
	case TPU_DEEP_SLEEP_CLOCKS_OFF:
	case TPU_SLEEP_CLOCKS_OFF:
		dev_warn(dev, "Power state %d prevents control core booting",
			 power_state);
		/* fall-thru */
	default:
		dev_warn(dev, "Power state %d is invalid\n", power_state);
		dev_warn(dev, "defaulting to active nominal\n");
		power_state = TPU_ACTIVE_NOM;
		break;
	}
	return power_state;
}

static int abrolhos_power_up(struct edgetpu_pm *etpm)
{
	struct edgetpu_dev *etdev = etpm->etdev;
	struct device *dev = etdev->dev;
	int ret = abrolhos_pwr_state_set(dev,
					 abrolhos_get_initial_pwr_state(dev));
	if (ret)
		return ret;

	edgetpu_chip_init(etdev);

	if (etdev->kci) {
		etdev_dbg(etdev, "Resetting KCI\n");
		edgetpu_kci_reinit(etdev->kci);
	}
	if (etdev->mailbox_manager) {
		etdev_dbg(etdev, "Resetting VII mailboxes\n");
		edgetpu_mailbox_reset_vii(etdev->mailbox_manager);
	}

	switch (edgetpu_firmware_status_locked(etdev)) {
	case FW_VALID:
		ret = edgetpu_firmware_restart_locked(etdev);
		break;
	case FW_INVALID:
		ret = edgetpu_firmware_run_locked(etdev->firmware,
						  EDGETPU_DEFAULT_FIRMWARE_NAME,
						  FW_DEFAULT);
		break;
	case FW_LOADING:
	default:
		break;
	}

	return ret;
}

static void abrolhos_power_down(struct edgetpu_pm *etpm)
{

	struct edgetpu_platform_dev *edgetpu_pdev = container_of(
			etpm->etdev, struct edgetpu_platform_dev, edgetpu_dev);
	int res;

	if (etpm->etdev->kci &&
	    edgetpu_firmware_status_locked(etpm->etdev) == FW_VALID) {
		res = edgetpu_kci_shutdown(etpm->etdev->kci);
		if (res) {
			etdev_warn(etpm->etdev,
				   "Firmware shutdown request failed!\n");
			etdev_warn(
				etpm->etdev,
				"Doing forced shutdown through power policy\n");
			abrolhos_pwr_policy_set(edgetpu_pdev, TPU_OFF);
			/*
			 * TODO: experiment on hardware to verify if this delay
			 * is needed, what is a good value or an alternative way
			 * to make sure the power policy request turned the
			 * device off.
			 */
			msleep(100);
			abrolhos_pwr_policy_set(edgetpu_pdev, TPU_ACTIVE_OD);
		}
	}
	abrolhos_pwr_state_set(etpm->etdev->dev, TPU_OFF);
}

static int abrolhos_pm_after_create(struct edgetpu_pm *etpm)
{
	int ret;
	struct device *dev = etpm->etdev->dev;
	struct edgetpu_platform_dev *edgetpu_pdev = container_of(
			etpm->etdev, struct edgetpu_platform_dev, edgetpu_dev);

	ret = abrolhos_pwr_state_init(dev);
	if (ret)
		return ret;

	ret = abrolhos_pwr_state_set(dev, abrolhos_get_initial_pwr_state(dev));
	if (ret)
		return ret;

	abrolhos_pwr_debugfs_dir =
		debugfs_create_dir("power", edgetpu_dev_debugfs_dir());
	debugfs_create_file("state", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_pwr_state);
	debugfs_create_file("vdd_tpu", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_vdd_tpu);
	debugfs_create_file("vdd_tpu_m", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_vdd_tpu_m);
	debugfs_create_file("vdd_int_m", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_vdd_int_m);
	debugfs_create_file("core_rate", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_core_rate);
	debugfs_create_file("ctl_rate", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_ctl_rate);
	debugfs_create_file("axi_rate", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_axi_rate);
	debugfs_create_file("apb_rate", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_apb_rate);
	debugfs_create_file("uart_rate", 0660, abrolhos_pwr_debugfs_dir,
			dev, &fops_tpu_uart_rate);
	debugfs_create_file("policy", 0660, abrolhos_pwr_debugfs_dir,
			edgetpu_pdev, &fops_tpu_pwr_policy);

	return 0;
}

static void abrolhos_pm_before_destroy(struct edgetpu_pm *etpm)
{
	debugfs_remove_recursive(abrolhos_pwr_debugfs_dir);
	pm_runtime_disable(etpm->etdev->dev);
}

static struct edgetpu_pm_handlers abrolhos_pm_handlers = {
	.after_create = abrolhos_pm_after_create,
	.before_destroy = abrolhos_pm_before_destroy,
	.power_up = abrolhos_power_up,
	.power_down = abrolhos_power_down,
};

int abrolhos_pm_create(struct edgetpu_dev *etdev)
{
	return edgetpu_pm_create(etdev, &abrolhos_pm_handlers);
}

void abrolhos_pm_destroy(struct edgetpu_dev *etdev)
{
	edgetpu_pm_destroy(etdev);
}
