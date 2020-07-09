// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU power management interface.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/mutex.h>

#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-pm.h"

struct edgetpu_pm_private {
	const struct edgetpu_pm_handlers *handlers;
	int power_up_count;
};

int edgetpu_pm_get(struct edgetpu_pm *etpm)
{
	int ret = 0;
	int power_up_count;

	if (!etpm || !etpm->p->handlers || !etpm->p->handlers->power_up)
		return 0;
	power_up_count = etpm->p->power_up_count++;
	if (!power_up_count)
		ret = etpm->p->handlers->power_up(etpm);
	if (ret)
		etpm->p->power_up_count--;
	return ret;
}

void edgetpu_pm_put(struct edgetpu_pm *etpm)
{
	if (!etpm || !etpm->p->handlers || !etpm->p->handlers->power_down)
		return;
	if (!etpm->p->power_up_count) {
		dev_err(etpm->etdev->dev, "Unbalanced pm_put");
		WARN_ON(1);
		return;
	}
	if (!--etpm->p->power_up_count)
		etpm->p->handlers->power_down(etpm);
}

int edgetpu_pm_create(struct edgetpu_dev *etdev,
		      const struct edgetpu_pm_handlers *handlers)
{
	int ret = 0;
	struct edgetpu_pm *etpm;

	if (etdev->pm) {
		dev_err(etdev->dev,
			"Refusing to replace existing PM interface\n");
		return -EEXIST;
	}

	etpm = kzalloc(sizeof(*etpm), GFP_KERNEL);
	if (!etpm)
		return -ENOMEM;

	etpm->p = kzalloc(sizeof(*etpm->p), GFP_KERNEL);
	if (!etpm->p) {
		ret = -ENOMEM;
		goto out_free_etpm;
	}

	etpm->p->handlers = handlers;
	etpm->etdev = etdev;

	if (handlers->after_create) {
		ret = handlers->after_create(etpm);
		if (ret) {
			ret = -EINVAL;
			goto out_free_etpm_p;
		}
	}
	etdev->pm = etpm;
	return 0;
out_free_etpm_p:
	kfree(etpm->p);
out_free_etpm:
	kfree(etpm);
	return ret;
}

void edgetpu_pm_destroy(struct edgetpu_dev *etdev)
{
	const struct edgetpu_pm_handlers *handlers;

	if (!etdev->pm)
		return;
	if (etdev->pm->p) {
		handlers = etdev->pm->p->handlers;
		if (handlers && handlers->before_destroy)
			handlers->before_destroy(etdev->pm);
		kfree(etdev->pm->p);
	}
	kfree(etdev->pm);
	etdev->pm = NULL;
}

void edgetpu_pm_shutdown(struct edgetpu_dev *etdev)
{
	struct edgetpu_pm *etpm = etdev->pm;

	if (!etpm)
		return;
	edgetpu_firmware_lock(etdev);
	if (etpm->p->power_up_count) {
		etdev_warn(etdev, "Leaving %d clients behind!\n",
			   etpm->p->power_up_count);
	}
	if (etpm->p->handlers && etpm->p->handlers->power_down)
		etpm->p->handlers->power_down(etpm);
	edgetpu_firmware_unlock(etdev);
}
