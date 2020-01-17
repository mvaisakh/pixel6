/*
 * Google LWIS Base Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_DEVICE_H_
#define LWIS_DEVICE_H_

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/idr.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/workqueue.h>

#include "lwis_clock.h"
#include "lwis_commands.h"
#include "lwis_gpio.h"
#include "lwis_interrupt.h"
#include "lwis_phy.h"
#include "lwis_regulator.h"

#define LWIS_TOP_DEVICE_COMPAT "google,lwis-top-device"
#define LWIS_I2C_DEVICE_COMPAT "google,lwis-i2c-device"
#define LWIS_IOREG_DEVICE_COMPAT "google,lwis-ioreg-device"

#define EVENT_HASH_BITS 8
#define BUFFER_HASH_BITS 8
#define TRANSACTION_HASH_BITS 8

/* Forward declaration for lwis_device. This is needed for the declaration for
   lwis_device_subclass_operations data struct. */
struct lwis_device;

/* Forward declaration of a platform specific struct used by platform funcs */
struct lwis_platform;

/*
 *  struct lwis_core
 *  This struct applies to all LWIS devices that are defined in the
 *  device tree.
 */
struct lwis_core {
	struct class *dev_class;
	struct idr *idr;
	struct cdev *chr_dev;
	struct mutex lock;
	dev_t lwis_devt;
	int device_major;
	struct list_head lwis_dev_list;
};

/* struct lwis_device_subclass_operations
 * This struct contains the 'virtual' functions for lwis_device subclasses
 * that are called into by various lwis_device_* code if they are not NULL
 * to allow the subclasses to customize certain behavior
 */

struct lwis_device_subclass_operations {
	/* Called by lwis_device when device register needs to be read/written
	 */
	int (*register_io)(struct lwis_device *lwis_dev,
			   struct lwis_io_entry *entry, bool non_blocking);
	/* called by lwis_device when enabling the device */
	int (*device_enable)(struct lwis_device *lwis_dev);
	/* called by lwis_device when disabling the device */
	int (*device_disable)(struct lwis_device *lwis_dev);
	/* Called by lwis_device any time a particular event_id needs to be
	 * enabled or disabled by the device
	 */
	int (*event_enable)(struct lwis_device *lwis_dev, int64_t event_id,
			    bool enabled);
	/* Called by lwis_device any time flags are updated */
	int (*event_flags_updated)(struct lwis_device *lwis_dev,
				   int64_t event_id, uint64_t old_flags,
				   uint64_t new_flags);
	/* Called by lwis_device any time an event is emitted
	 * Called with lwis_dev->lock locked and IRQs disabled */
	int (*event_emitted)(struct lwis_device *lwis_dev, int64_t event_id,
			     void **payload_ptrptr, size_t *payload_size_ptr);
};

/*
 *  struct lwis_device
 *  This struct applies to each of the LWIS devices, e.g. /dev/lwis*
 */
struct lwis_device {
	struct lwis_platform *platform;
	int id;
	enum lwis_device_types type;
	char name[LWIS_MAX_DEVICE_NAME_STRING];
	struct device *dev;
	struct platform_device *plat_dev;
	bool reset_gpios_present;
	struct gpio_descs *reset_gpios;
	bool enable_gpios_present;
	struct gpio_descs *enable_gpios;
	struct lwis_regulator_list *regulators;
	struct lwis_clock_list *clocks;
	struct pinctrl *mclk_ctrl;
	struct lwis_interrupt_list *irqs;
	struct lwis_phy_list *phys;
	struct list_head dev_list;

	/* Enabled state of the device */
	int enabled;
	/* Mutex used to synchronize access between clients */
	struct mutex client_lock;
	/* Spinlock used to synchronize access to the device struct */
	spinlock_t lock;
	/* List of clients opened for this device */
	struct list_head clients;
	/* Hash table of device-specific per-event state/control data */
	DECLARE_HASHTABLE(event_states, EVENT_HASH_BITS);
	/* Virtual function table for sub classes */
	struct lwis_device_subclass_operations vops;
	/* Does the device have IOMMU. TODO: Move to platform */
	bool has_iommu;
	/* Mutex used to synchronize register access between clients */
	struct mutex reg_rw_lock;
	/* Heartbeat timer structure */
	struct timer_list heartbeat_timer;
	/* Register-related properties */
	unsigned int reg_addr_bitwidth;
	unsigned int reg_value_bitwidth;
};

/*
 *  struct lwis_client
 *  This struct applies to each client that uses a LWIS device, i.e. each
 *  application that calls open() on a /dev/lwis* device.
 */
struct lwis_client {
	struct mutex lock;
	struct lwis_device *lwis_dev;
	/* Hash table of events controlled by userspace in this client */
	DECLARE_HASHTABLE(event_states, EVENT_HASH_BITS);
	/* Queue of pending events to be consumed by userspace */
	struct list_head event_queue;
	/* Spinlock used to synchronize access to event states and queue */
	spinlock_t event_lock;
	/* Event wait queue for waking up userspace */
	wait_queue_head_t event_wait_queue;
	/* Hash table of allocated buffers keyed by file descriptor. */
	DECLARE_HASHTABLE(allocated_buffers, BUFFER_HASH_BITS);
	/* Hash table of enrolled buffers keyed by dvaddr */
	DECLARE_HASHTABLE(enrolled_buffers, BUFFER_HASH_BITS);
	/* Hash table of transactions keyed by trigger event ID */
	DECLARE_HASHTABLE(transaction_list, TRANSACTION_HASH_BITS);
	/* Transaction task-related variables */
	struct workqueue_struct *transaction_wq;
	struct work_struct transaction_work;
	/* Spinlock  used to synchornize access to transaction data structs */
	spinlock_t transaction_lock;
	/* List of transaction triggers */
	struct list_head transaction_process_queue;
	/* Transaction counter, which also provides transacton ID */
	int64_t transaction_counter;
	/* Each device has a linked list of clients */
	struct list_head node;
};

/*
 *  lwis_base_probe: Common probe function that will be used for all types
 *  of devices.
 */
int lwis_base_probe(struct lwis_device *lwis_dev,
		    struct platform_device *plat_dev);

#endif /* LWIS_DEVICE_H_ */
