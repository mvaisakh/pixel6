// SPDX-License-Identifier: GPL-2.0-only
/* exynos_drm_hibernation.c
 *
 * Copyright (C) 2020 Samsung Electronics Co.Ltd
 * Authors:
 *	Jiun Yu <jiun.yu@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/atomic.h>

#include <trace/dpu_trace.h>

#include <dqe_cal.h>

#include "exynos_drm_decon.h"
#include "exynos_drm_hibernation.h"
#include "exynos_drm_writeback.h"

#define HIBERNATION_ENTRY_MIN_TIME_MS		50
#define CAMERA_OPERATION_MASK	0xF

static bool is_camera_operating(struct exynos_hibernation *hiber)
{
	/* No need to check camera operation status. It depends on SoC */
	if (!hiber->cam_op_reg)
		return false;

	return (readl(hiber->cam_op_reg) & CAMERA_OPERATION_MASK);
}

static inline bool is_hibernation_enabled(struct exynos_hibernation *hiber)
{
	return hiber && hiber->enabled;
}

static inline bool is_hibernaton_blocked(struct exynos_hibernation *hiber)
{
	return (!is_hibernation_enabled(hiber)) ||
		(atomic_read(&hiber->block_cnt) > 0) ||
		(hiber->decon->state != DECON_STATE_ON);
}

static bool exynos_hibernation_check(struct exynos_hibernation *hiber)
{
	pr_debug("%s +\n", __func__);

	return (!is_camera_operating(hiber) &&
		pm_runtime_active(hiber->decon->dev) &&
		!dqe_reg_dimming_in_progress());
}

static inline void hibernation_block(struct exynos_hibernation *hiber)
{
	atomic_inc(&hiber->block_cnt);
}

static inline void hibernation_unblock(struct exynos_hibernation *hiber)
{
	WARN_ON(!atomic_add_unless(&hiber->block_cnt, -1, 0));
}

static void exynos_hibernation_enter(struct exynos_hibernation *hiber)
{
	struct decon_device *decon = hiber->decon;

	pr_debug("%s +\n", __func__);

	if (!decon)
		return;

	DPU_ATRACE_BEGIN("exynos_hibernation_enter");
	hibernation_block(hiber);

	if (decon->state != DECON_STATE_ON)
		goto ret;

	DPU_EVENT_LOG(DPU_EVT_ENTER_HIBERNATION_IN, decon->id, NULL);

	hiber->wb = decon_get_wb(decon);
	if (hiber->wb)
		writeback_enter_hibernation(hiber->wb);

	decon_enter_hibernation(decon);

	hiber->dsim = decon_get_dsim(decon);
	if (hiber->dsim)
		dsim_enter_ulps(hiber->dsim);

	decon->bts.ops->release_bw(decon);

	pm_runtime_put_sync(decon->dev);

	DPU_EVENT_LOG(DPU_EVT_ENTER_HIBERNATION_OUT, decon->id, NULL);

ret:
	hibernation_unblock(hiber);
	DPU_ATRACE_END("exynos_hibernation_enter");

	pr_debug("%s: DPU power %s -\n", __func__,
			pm_runtime_active(decon->dev) ? "on" : "off");
}

static int exynos_hibernation_exit(struct exynos_hibernation *hiber)
{
	struct decon_device *decon = hiber->decon;
	int ret = -EBUSY;

	pr_debug("%s +\n", __func__);

	if (!decon)
		return -ENODEV;

	hibernation_block(hiber);

	/*
	 * Cancel and/or wait for finishing previous queued hibernation entry work. It only
	 * goes to sleep when work is currently executing. If not, there is no operation here.
	 */
	kthread_cancel_delayed_work_sync(&hiber->dwork);

	mutex_lock(&hiber->lock);

	if (decon->state != DECON_STATE_HIBERNATION)
		goto ret;

	DPU_ATRACE_BEGIN("exynos_hibernation_exit");

	DPU_EVENT_LOG(DPU_EVT_EXIT_HIBERNATION_IN, decon->id, NULL);

	pm_runtime_get_sync(decon->dev);

	if (hiber->dsim) {
		dsim_exit_ulps(hiber->dsim);
		hiber->dsim = NULL;
	}

	decon_exit_hibernation(decon);

	if (hiber->wb) {
		writeback_exit_hibernation(hiber->wb);
		hiber->wb = NULL;
	}

	if (decon->partial)
		exynos_partial_restore(decon->partial);

	DPU_EVENT_LOG(DPU_EVT_EXIT_HIBERNATION_OUT, decon->id, NULL);
	DPU_ATRACE_END("exynos_hibernation_exit");

	ret = 0;
ret:
	mutex_unlock(&hiber->lock);
	hibernation_unblock(hiber);

	pr_debug("%s: DPU power %s -\n", __func__,
			pm_runtime_active(decon->dev) ? "on" : "off");

	return ret;
}

bool hibernation_block_exit(struct exynos_hibernation *hiber)
{
	const struct exynos_hibernation_funcs *funcs;
	bool ret;

	if (!hiber)
		return false;

	hibernation_block(hiber);

	if (!is_hibernation_enabled(hiber))
		return false;

	funcs = hiber->funcs;

	ret = !funcs || !funcs->exit(hiber);

	pr_debug("%s: block_cnt(%d)\n", __func__, atomic_read(&hiber->block_cnt));

	return ret;
}

void hibernation_unblock_enter(struct exynos_hibernation *hiber)
{
	struct decon_device *decon;

	if (!hiber)
		return;

	decon = hiber->decon;

	if (!decon)
		return;

	hibernation_unblock(hiber);

	if (!is_hibernaton_blocked(hiber))
		kthread_mod_delayed_work(&decon->worker, &hiber->dwork,
			msecs_to_jiffies(HIBERNATION_ENTRY_MIN_TIME_MS));

	pr_debug("%s: block_cnt(%d)\n", __func__, atomic_read(&hiber->block_cnt));
}

static const struct exynos_hibernation_funcs hibernation_funcs = {
	.check	= exynos_hibernation_check,
	.enter	= exynos_hibernation_enter,
	.exit	= exynos_hibernation_exit,
};

static void exynos_hibernation_handler(struct kthread_work *work)
{
	struct exynos_hibernation *hibernation = container_of(work,
			struct exynos_hibernation, dwork.work);
	const struct exynos_hibernation_funcs *funcs = hibernation->funcs;
	struct decon_device *decon = hibernation->decon;

	pr_debug("Display hibernation handler is called\n");

	mutex_lock(&hibernation->lock);
	if (is_hibernaton_blocked(hibernation))
		goto ret;

	/* If hibernation entry condition does NOT meet, try again later */
	if (!funcs->check(hibernation)) {
		kthread_mod_delayed_work(&decon->worker, &hibernation->dwork,
			msecs_to_jiffies(HIBERNATION_ENTRY_MIN_TIME_MS));
		goto ret;
	}

	funcs->enter(hibernation);
ret:
	mutex_unlock(&hibernation->lock);
}

struct exynos_hibernation *
exynos_hibernation_register(struct decon_device *decon)
{
	struct device_node *np, *cam_np;
	struct exynos_hibernation *hibernation;
	struct device *dev = decon->dev;

	np = dev->of_node;

	if (of_property_read_bool(np, "override-hibernation")) {
		pr_info("display hibernation is overridden\n");
		return NULL;
	}

	if (!of_property_read_bool(np, "hibernation")) {
		pr_info("display hibernation is not supported\n");
		return NULL;
	}

	hibernation = devm_kzalloc(dev, sizeof(struct exynos_hibernation),
			GFP_KERNEL);
	if (!hibernation)
		return NULL;

	cam_np = of_get_child_by_name(np, "camera-operation");
	if (!cam_np) {
		pr_info("doesn't need to get camera operation register\n");
		hibernation->cam_op_reg = NULL;
	} else {
		hibernation->cam_op_reg = of_iomap(cam_np, 0);
		if (!hibernation->cam_op_reg) {
			pr_err("failed to map camera operation register\n");
			kfree(hibernation);
			return NULL;
		}
	}

	hibernation->decon = decon;
	hibernation->funcs = &hibernation_funcs;
	hibernation->enabled = true;

	mutex_init(&hibernation->lock);

	atomic_set(&hibernation->block_cnt, 0);

	kthread_init_delayed_work(&hibernation->dwork, exynos_hibernation_handler);

	pr_info("display hibernation is supported\n");

	return hibernation;
}

void exynos_hibernation_destroy(struct exynos_hibernation *hiber)
{
	if (!is_hibernation_enabled(hiber))
		return;

	if (hiber->cam_op_reg)
		iounmap(hiber->cam_op_reg);
}
