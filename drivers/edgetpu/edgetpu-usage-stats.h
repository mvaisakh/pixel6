/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU usage stats header
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_USAGE_STATS_H__
#define __EDGETPU_USAGE_STATS_H__

#include <linux/hashtable.h>
#include <linux/mutex.h>

/* Header struct in the metric buffer. */
/* Must be kept in sync with firmware struct UsageTrackerHeader */
struct usage_tracker_header {
	uint32_t num_metrics;		/* Number of metrics being reported */
	uint32_t metric_size;		/* Size of each metric struct */
};

/*
 * Encapsulate TPU core usage information of a specific application for a
 * specific power state.
 * Must be kept in sync with firmware struct TpuUsage.
 */
struct tpu_usage {
	/* Unique identifier of the application. */
	int32_t uid;
	/* The power state of the device (values are chip dependent) */
	uint32_t power_state;
	/* Duration of usage in microseconds. */
	uint32_t duration_us;
};

/* Must be kept in sync with firmware enum class UsageTrackerMetric::Type */
enum usage_tracker_metric_type {
	metric_type_reserved = 0,
	metric_type_tpu_usage = 1,
};

/*
 * Encapsulates a single metric reported to the kernel.
 * Must be kept in sync with firmware struct UsageTrackerMetric.
 */
struct usage_tracker_metric {
	uint32_t type;
	uint8_t reserved[4];
	union {
		struct tpu_usage tpu_usage;
	};
};

#define UID_HASH_BITS 3

struct edgetpu_usage_stats {
	DECLARE_HASHTABLE(uid_hash_table, UID_HASH_BITS);
	struct mutex usage_stats_lock;
};

int edgetpu_usage_add(struct edgetpu_dev *etdev, struct tpu_usage *tpu_usage);
void edgetpu_usage_stats_process_buffer(struct edgetpu_dev *etdev, void *buf);
void edgetpu_usage_stats_init(struct edgetpu_dev *etdev);
void edgetpu_usage_stats_exit(struct edgetpu_dev *etdev);

#endif /* __EDGETPU_USAGE_STATS_H__ */
