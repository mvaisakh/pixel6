// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU software WDT interface.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-sw-watchdog.h"

static bool wdt_disable;
module_param(wdt_disable, bool, 0660);

/* Worker to execute action callback handler on watchdog bite. */
static void sw_wdt_handler_work(struct work_struct *work)
{
	struct edgetpu_sw_wdt_action_work *et_action_work =
		container_of(work, struct edgetpu_sw_wdt_action_work, work);

	if (et_action_work->edgetpu_sw_wdt_handler)
		et_action_work->edgetpu_sw_wdt_handler(et_action_work->data);
}

/*
 * Ping the f/w for a response. Reschedule the work for next beat
 * in case of response or schedule a worker for action callback in case of
 * TIMEOUT.
 */
static void sw_wdt_work(struct work_struct *work)
{
	int ret;
	struct delayed_work *dwork = to_delayed_work(work);
	struct edgetpu_sw_wdt *etdev_sw_wdt =
		container_of(dwork, struct edgetpu_sw_wdt, dwork);
	struct edgetpu_dev *etdev = etdev_sw_wdt->etdev;

	/* ping f/w */
	etdev_dbg(etdev, "sw wdt: pinging firmware\n");
	ret = edgetpu_kci_ack(etdev->kci);
	if (ret)
		etdev_dbg(etdev, "sw-watchdog ping resp:%d\n", ret);
	if (ret == -ETIMEDOUT) {
		etdev_err(etdev, "sw-watchdog response timed out\n");
		schedule_work(&etdev_sw_wdt->et_action_work.work);
	} else {
		/* reschedule to next beat. */
		schedule_delayed_work(dwork, etdev_sw_wdt->hrtbeat_jiffs);
	}
}

int edgetpu_sw_wdt_create(struct edgetpu_dev *etdev, unsigned long hrtbeat_ms)
{
	struct edgetpu_sw_wdt *etdev_sw_wdt;

	etdev_sw_wdt = kzalloc(sizeof(*etdev_sw_wdt), GFP_KERNEL);
	if (!etdev_sw_wdt)
		return -ENOMEM;

	etdev_sw_wdt->etdev = etdev;
	etdev_sw_wdt->hrtbeat_jiffs = msecs_to_jiffies(hrtbeat_ms);
	INIT_DELAYED_WORK(&etdev_sw_wdt->dwork, sw_wdt_work);
	INIT_WORK(&etdev_sw_wdt->et_action_work.work, sw_wdt_handler_work);
	etdev_sw_wdt->is_wdt_disabled = wdt_disable;
	etdev->etdev_sw_wdt = etdev_sw_wdt;
	return 0;
}

int edgetpu_sw_wdt_start(struct edgetpu_dev *etdev)
{
	struct edgetpu_sw_wdt *etdev_sw_wdt = etdev->etdev_sw_wdt;

	if (!etdev_sw_wdt)
		return -EINVAL;
	if (!etdev_sw_wdt->et_action_work.edgetpu_sw_wdt_handler)
		etdev_err(etdev, "sw wdt handler not set\n");
	if (etdev_sw_wdt->is_wdt_disabled) {
		etdev_dbg(etdev, "sw wdt disabled by module param");
		return 0;
	}
	etdev_dbg(etdev, "sw wdt: started\n");
	schedule_delayed_work(&etdev_sw_wdt->dwork,
			      etdev_sw_wdt->hrtbeat_jiffs);
	return 0;
}

void edgetpu_sw_wdt_stop(struct edgetpu_dev *etdev)
{
	if (!etdev->etdev_sw_wdt)
		return;
	etdev_dbg(etdev, "sw wdt: stopped\n");
	cancel_delayed_work_sync(&etdev->etdev_sw_wdt->dwork);
}

void edgetpu_sw_wdt_destroy(struct edgetpu_dev *etdev)
{
	/* cancel and sync work due to watchdog bite to prevent UAF */
	cancel_work_sync(&etdev->etdev_sw_wdt->et_action_work.work);
	edgetpu_sw_wdt_stop(etdev);
	kfree(etdev->etdev_sw_wdt);
	etdev->etdev_sw_wdt = NULL;
}

void edgetpu_sw_wdt_set_handler(struct edgetpu_dev *etdev,
				void (*handler_cb)(void *), void *data)
{
	struct edgetpu_sw_wdt *et_sw_wdt = etdev->etdev_sw_wdt;

	if (!et_sw_wdt)
		return;
	et_sw_wdt->et_action_work.edgetpu_sw_wdt_handler = handler_cb;
	et_sw_wdt->et_action_work.data = data;
}

void edgetpu_sw_wdt_modify_heartbeat(struct edgetpu_dev *etdev,
				     unsigned long hrtbeat_ms)
{
	struct edgetpu_sw_wdt *etdev_sw_wdt = etdev->etdev_sw_wdt;
	unsigned long hrtbeat_jiffs = msecs_to_jiffies(hrtbeat_ms);

	if (!etdev_sw_wdt)
		return;
	/*
	 * check if (et_action_work) is pending, since after watchdog bite
	 * there is no need to restart another work.
	 */
	if (work_pending(&etdev_sw_wdt->et_action_work.work))
		return;
	if (hrtbeat_jiffs != etdev_sw_wdt->hrtbeat_jiffs) {
		edgetpu_sw_wdt_stop(etdev);
		etdev_sw_wdt->hrtbeat_jiffs = hrtbeat_jiffs;
		edgetpu_sw_wdt_start(etdev);
	}
}
