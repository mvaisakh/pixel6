// SPDX-License-Identifier: GPL-2.0
/*
 * Wakelock for the runtime to explicitly claim it's going to use the EdgeTPU
 * device.
 *
 * Copyright (C) 2021 Google, Inc.
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-wakelock.h"

/*
 * Returns the first event with a non-zero counter.
 * Returns EDGETPU_WAKELOCK_EVENT_END if all event counters are zero.
 *
 * Caller holds @wakelock->lock.
 */
static enum edgetpu_wakelock_event
wakelock_non_zero_event(struct edgetpu_wakelock *wakelock)
{
	int i;

	for (i = 0; i < EDGETPU_WAKELOCK_EVENT_END; i++)
		if (wakelock->event_count[i])
			return i;
	return EDGETPU_WAKELOCK_EVENT_END;
}

struct edgetpu_wakelock *edgetpu_wakelock_alloc(struct edgetpu_dev *etdev)
{
#ifndef EDGETPU_HAS_WAKELOCK
	return EDGETPU_NO_WAKELOCK;
#else /* !EDGETPU_HAS_WAKELOCK */
	struct edgetpu_wakelock *wakelock =
		kzalloc(sizeof(*wakelock), GFP_KERNEL);

	if (!wakelock)
		return NULL;
	wakelock->etdev = etdev;
	mutex_init(&wakelock->lock);
	/* TODO(b/180528998): init as "released" */
	/* Initialize client wakelock state to "acquired" */
	wakelock->req_count = 1;
	return wakelock;
#endif /* EDGETPU_HAS_WAKELOCK */
}

void edgetpu_wakelock_free(struct edgetpu_wakelock *wakelock)
{
	if (IS_ERR_OR_NULL(wakelock))
		return;
	kfree(wakelock);
}

bool edgetpu_wakelock_inc_event(struct edgetpu_wakelock *wakelock,
				enum edgetpu_wakelock_event evt)
{
	bool ret = true;

	if (NO_WAKELOCK(wakelock))
		return true;
	mutex_lock(&wakelock->lock);
	if (!wakelock->req_count) {
		ret = false;
		etdev_warn(
			wakelock->etdev,
			"invalid increase event %d when wakelock is released",
			evt);
	} else {
		++wakelock->event_count[evt];
		/* integer overflow.. */
		if (unlikely(wakelock->event_count[evt] == 0)) {
			--wakelock->event_count[evt];
			ret = false;
			etdev_warn_once(wakelock->etdev,
					"int overflow on increasing event %d",
					evt);
		}
	}
	mutex_unlock(&wakelock->lock);
	return ret;
}

bool edgetpu_wakelock_dec_event(struct edgetpu_wakelock *wakelock,
				enum edgetpu_wakelock_event evt)
{
	bool ret = true;

	if (NO_WAKELOCK(wakelock))
		return true;
	mutex_lock(&wakelock->lock);
	if (!wakelock->event_count[evt]) {
		ret = false;
		etdev_warn(wakelock->etdev, "event %d unbalanced decreasing",
			   evt);
	} else {
		--wakelock->event_count[evt];
	}
	mutex_unlock(&wakelock->lock);
	return ret;
}

uint edgetpu_wakelock_lock(struct edgetpu_wakelock *wakelock)
{
	if (NO_WAKELOCK(wakelock))
		return 1;
	mutex_lock(&wakelock->lock);
	return wakelock->req_count;
}

void edgetpu_wakelock_unlock(struct edgetpu_wakelock *wakelock)
{
	if (!NO_WAKELOCK(wakelock))
		mutex_unlock(&wakelock->lock);
}

int edgetpu_wakelock_acquire(struct edgetpu_wakelock *wakelock)
{
	int ret;

	if (NO_WAKELOCK(wakelock))
		return 1;
	ret = wakelock->req_count++;
	/* integer overflow */
	if (unlikely(ret < 0)) {
		wakelock->req_count--;
		return -EOVERFLOW;
	}
	return ret;
}

int edgetpu_wakelock_release(struct edgetpu_wakelock *wakelock)
{
	if (NO_WAKELOCK(wakelock))
		return 1;
	if (!wakelock->req_count) {
		etdev_warn(wakelock->etdev, "invalid wakelock release");
		return -EINVAL;
	}
	/* only need to check events when this is the last reference */
	if (wakelock->req_count == 1) {
		enum edgetpu_wakelock_event evt =
			wakelock_non_zero_event(wakelock);

		if (evt != EDGETPU_WAKELOCK_EVENT_END) {
			etdev_warn(
				wakelock->etdev,
				"event %d is happening, refusing wakelock release",
				evt);
			return -EAGAIN;
		}
	}
	return --wakelock->req_count;
}
