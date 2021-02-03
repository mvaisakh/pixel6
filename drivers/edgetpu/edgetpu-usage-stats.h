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
struct edgetpu_usage_header {
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

/*
 * An enum to represent the different components we can track metrics for.
 * Must be kept in sync with firmware struct Component.
 */
enum edgetpu_usage_component {
	/* The device as a whole (TPU, R52, DMA330, etc.) */
	EDGETPU_USAGE_COMPONENT_DEVICE = 0,
	/* Just the TPU core (scalar core and tiles) */
	EDGETPU_USAGE_COMPONENT_TPU = 1,
	EDGETPU_USAGE_COMPONENT_COUNT = 2, /* number of components above */
};

/*
 * Encapsulates information about activity of a component.
 * Must be kept in sync with firmware struct ComponentActivity.
 */
struct edgetpu_component_activity {
	enum edgetpu_usage_component component;
	/* Utilization as a percentage since the last read. */
	int32_t utilization;
};

/* Must be kept in sync with firmware enum class UsageTrackerMetric::Type */
enum edgetpu_usage_metric_type {
	EDGETPU_METRIC_TYPE_RESERVED = 0,
	EDGETPU_METRIC_TYPE_TPU_USAGE = 1,
	EDGETPU_METRIC_TYPE_COMPONENT_ACTIVITY = 2,
};

/*
 * Encapsulates a single metric reported to the kernel.
 * Must be kept in sync with firmware struct UsageTrackerMetric.
 */
struct edgetpu_usage_metric {
	uint32_t type;
	uint8_t reserved[4];
	union {
		struct tpu_usage tpu_usage;
		struct edgetpu_component_activity component_activity;
	};
};

#define UID_HASH_BITS 3

struct edgetpu_usage_stats {
	DECLARE_HASHTABLE(uid_hash_table, UID_HASH_BITS);
	/* component utilization values reported by firmware */
	int32_t component_utilization[EDGETPU_USAGE_COMPONENT_COUNT];
	struct mutex usage_stats_lock;
};

int edgetpu_usage_add(struct edgetpu_dev *etdev, struct tpu_usage *tpu_usage);
void edgetpu_usage_stats_process_buffer(struct edgetpu_dev *etdev, void *buf);
void edgetpu_usage_stats_init(struct edgetpu_dev *etdev);
void edgetpu_usage_stats_exit(struct edgetpu_dev *etdev);

#endif /* __EDGETPU_USAGE_STATS_H__ */
