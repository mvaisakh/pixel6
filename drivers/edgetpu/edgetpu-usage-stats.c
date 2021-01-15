// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU usage stats
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/slab.h>
#include <linux/sysfs.h>

#include "edgetpu-internal.h"
#include "edgetpu-usage-stats.h"

#if IS_ENABLED(CONFIG_ABROLHOS)

#include "abrolhos-pm.h"

static enum tpu_pwr_state tpu_states_arr[] = {
	TPU_ACTIVE_SUD,
	TPU_ACTIVE_UD,
	TPU_ACTIVE_NOM,
	TPU_ACTIVE_OD,
};

#else /* !CONFIG_ABROLHOS */

/* All execution times will be added to the same state. */
static uint32_t tpu_states_arr[] = {
	0,
};

#endif /* CONFIG_ABROLHOS */

#define NUM_TPU_STATES ARRAY_SIZE(tpu_states_arr)

struct uid_entry {
	int32_t uid;
	uint64_t time_in_state[NUM_TPU_STATES];
	struct hlist_node node;
};

static int tpu_state_map(uint32_t state)
{
	int i;

	for (i = (NUM_TPU_STATES - 1); i >= 0; i--) {
		if (state >= tpu_states_arr[i])
			return i;
	}

	return 0;
}

/* Caller must hold usage_stats lock */
static struct uid_entry *
find_uid_entry_locked(int32_t uid, struct edgetpu_usage_stats *ustats)
{
	struct uid_entry *uid_entry;

	hash_for_each_possible(ustats->uid_hash_table, uid_entry, node, uid) {
		if (uid_entry->uid == uid)
			return uid_entry;
	}

	return NULL;
}

int edgetpu_usage_add(struct edgetpu_dev *etdev, struct tpu_usage *tpu_usage)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	struct uid_entry *uid_entry;
	int state = tpu_state_map(tpu_usage->power_state);

	if (!ustats)
		return 0;

	etdev_dbg(etdev, "%s: uid=%u state=%u dur=%u", __func__,
		  tpu_usage->uid, tpu_usage->power_state,
		  tpu_usage->duration_us);
	mutex_lock(&ustats->usage_stats_lock);

	/* Find the uid in uid_hash_table first */
	uid_entry = find_uid_entry_locked(tpu_usage->uid, ustats);
	if (uid_entry) {
		uid_entry->time_in_state[state] += tpu_usage->duration_us;
		mutex_unlock(&ustats->usage_stats_lock);
		return 0;
	}

	/* Allocate memory for this uid */
	uid_entry = kzalloc(sizeof(*uid_entry), GFP_KERNEL);
	if (!uid_entry) {
		mutex_unlock(&ustats->usage_stats_lock);
		return -ENOMEM;
	}

	uid_entry->uid = tpu_usage->uid;
	uid_entry->time_in_state[state] += tpu_usage->duration_us;

	/* Add uid_entry to the uid_hash_table */
	hash_add(ustats->uid_hash_table, &uid_entry->node, tpu_usage->uid);

	mutex_unlock(&ustats->usage_stats_lock);

	return 0;
}

void edgetpu_usage_stats_process_buffer(struct edgetpu_dev *etdev, void *buf)
{
	struct usage_tracker_header *header = buf;
	struct usage_tracker_metric *metric =
		(struct usage_tracker_metric *)(header + 1);
	int i;

	etdev_dbg(etdev, "%s: n=%u sz=%u", __func__,
		  header->num_metrics, header->metric_size);
	if (header->metric_size != sizeof(struct usage_tracker_metric)) {
		etdev_dbg(etdev, "%s: expected sz=%zu, discard", __func__,
			  sizeof(struct usage_tracker_metric));
		return;
	}

	for (i = 0; i < header->num_metrics; i++) {
		switch (metric->type) {
		case metric_type_tpu_usage:
			edgetpu_usage_add(etdev, &metric->tpu_usage);
			break;
		default:
			etdev_dbg(etdev, "%s: %d: skip unknown type=%u",
				  __func__, i, metric->type);
			break;
		}

		metric++;
	}
}

static ssize_t tpu_usage_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;
	int i;
	int ret = 0;
	unsigned int bkt;
	struct uid_entry *uid_entry;

	/* uid: TPU_ACTIVE_SUD TPU_ACTIVE_UD TPU_ACTIVE_NOM TPU_ACTIVE_OD */
	ret += scnprintf(buf, PAGE_SIZE, "uid:");

	for (i = 0; i < NUM_TPU_STATES; i++)
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, " %d",
				 tpu_states_arr[i]);

	ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");

	mutex_lock(&ustats->usage_stats_lock);

	hash_for_each(ustats->uid_hash_table, bkt, uid_entry, node) {
		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "%d:",
				 uid_entry->uid);

		for (i = 0; i < NUM_TPU_STATES; i++)
			ret += scnprintf(buf + ret, PAGE_SIZE - ret, " %lld",
					 uid_entry->time_in_state[i]);

		ret += scnprintf(buf + ret, PAGE_SIZE - ret, "\n");
	}

	mutex_unlock(&ustats->usage_stats_lock);

	return ret;
}

static void usage_stats_remove_uids(struct edgetpu_usage_stats *ustats)
{
	unsigned int bkt;
	struct uid_entry *uid_entry;
	struct hlist_node *tmp;

	mutex_lock(&ustats->usage_stats_lock);

	hash_for_each_safe(ustats->uid_hash_table, bkt, tmp, uid_entry, node) {
		hash_del(&uid_entry->node);
		kfree(uid_entry);
	}

	mutex_unlock(&ustats->usage_stats_lock);
}

/* Write to clear all entries in uid_hash_table */
static ssize_t tpu_usage_clear(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf,
			       size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;

	usage_stats_remove_uids(ustats);

	return count;
}

static DEVICE_ATTR(tpu_usage, 0644, tpu_usage_show, tpu_usage_clear);

void edgetpu_usage_stats_init(struct edgetpu_dev *etdev)
{
	struct edgetpu_usage_stats *ustats;
	int ret;

	ustats = devm_kzalloc(etdev->dev, sizeof(*etdev->usage_stats),
			      GFP_KERNEL);
	if (!ustats) {
		etdev_warn(etdev,
			   "failed to allocate memory for usage stats\n");
		return;
	}

	hash_init(ustats->uid_hash_table);
	mutex_init(&ustats->usage_stats_lock);

	etdev->usage_stats = ustats;

	ret = device_create_file(etdev->dev, &dev_attr_tpu_usage);
	if (ret)
		etdev_warn(etdev, "failed to create the usage_stats file\n");

	etdev_dbg(etdev, "%s init\n", __func__);
}

void edgetpu_usage_stats_exit(struct edgetpu_dev *etdev)
{
	struct edgetpu_usage_stats *ustats = etdev->usage_stats;

	if (ustats) {
		usage_stats_remove_uids(ustats);
		device_remove_file(etdev->dev, &dev_attr_tpu_usage);
	}

	etdev_dbg(etdev, "%s exit\n", __func__);
}
