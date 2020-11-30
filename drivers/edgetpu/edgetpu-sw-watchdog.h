/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU software WDT interface.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_SW_WDT_H__
#define __EDGETPU_SW_WDT_H__

#include <linux/workqueue.h>

#include "edgetpu-internal.h"

#define EDGETPU_ACTIVE_DEV_BEAT_MS 15000 /* 15 seconds */
#define EDGETPU_DORMANT_DEV_BEAT_MS 60000 /* 60 seconds */

struct edgetpu_sw_wdt_action_work {
	struct work_struct work;
	/* pending function to be called on watchdog bite. */
	void (*edgetpu_sw_wdt_handler)(void *data);
	/* optional data can be used by callback function. */
	void *data;
};

struct edgetpu_sw_wdt {
	struct delayed_work dwork;
	/* edgetpu device this watchdog is monitoring. */
	struct edgetpu_dev *etdev;
	/* time period in jiffies in pinging device firmware. */
	unsigned long hrtbeat_jiffs;
	/* work information for watchdog bite. */
	struct edgetpu_sw_wdt_action_work et_action_work;
	/* flag to mark that watchdog is disabled. */
	bool is_wdt_disabled;
};

int edgetpu_sw_wdt_create(struct edgetpu_dev *etdev, unsigned long hrtbeat_ms);
int edgetpu_sw_wdt_start(struct edgetpu_dev *etdev);
void edgetpu_sw_wdt_stop(struct edgetpu_dev *etdev);
void edgetpu_sw_wdt_destroy(struct edgetpu_dev *etdev);
/*
 * Set callback function @handler_cb and optional param @data which is to be
 * called on f/w ping timeout.
 */
void edgetpu_sw_wdt_set_handler(struct edgetpu_dev *etdev,
				void (*handler_cb)(void *), void *data);
/* Modify the time interval of heartbeat. It will also start the watchdog. */
void edgetpu_sw_wdt_modify_heartbeat(struct edgetpu_dev *etdev,
				     unsigned long hrtbeat_ms);

#endif /* __EDGETPU_SW_WDT_H__ */
