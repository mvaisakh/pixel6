/*
 * Google LWIS Base Device Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-dev: " fmt

#include "lwis_device.h"

#include <linux/device.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_buffer.h"
#include "lwis_commands.h"
#include "lwis_device.h"
#include "lwis_dt.h"
#include "lwis_event.h"
#include "lwis_init.h"
#include "lwis_ioctl.h"
#include "lwis_platform.h"
#include "lwis_transaction.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_CLASS_NAME "lwis"
#define LWIS_DEVICE_NAME "lwis"
#define LWIS_MAX_DEVICES (1U << MINORBITS)

/* Global declaration for core lwis structure */
static struct lwis_core core;

static int lwis_open(struct inode *node, struct file *fp);
static int lwis_release(struct inode *node, struct file *fp);
static long lwis_ioctl(struct file *fp, unsigned int type, unsigned long param);
static unsigned int lwis_poll(struct file *fp, poll_table *wait);

static struct file_operations lwis_fops = {
	.owner = THIS_MODULE,
	.open = lwis_open,
	.release = lwis_release,
	.unlocked_ioctl = lwis_ioctl,
	.poll = lwis_poll,
};

/*
 *  lwis_open: Opening an instance of a LWIS device
 */
static int lwis_open(struct inode *node, struct file *fp)
{
	struct lwis_device *lwis_dev;
	struct lwis_client *lwis_client;
	unsigned long flags;

	pr_info("Opening instance %d\n", iminor(node));

	/* Making sure the minor number associated with fp exists */
	mutex_lock(&core.lock);
	lwis_dev = idr_find(core.idr, iminor(node));
	mutex_unlock(&core.lock);
	if (!lwis_dev) {
		pr_err("No device %d found\n", iminor(node));
		return -ENODEV;
	}

	lwis_client = kzalloc(sizeof(struct lwis_client), GFP_KERNEL);
	if (!lwis_client) {
		pr_err("Failed to allocate lwis client\n");
		return -ENOMEM;
	}

	lwis_client->lwis_dev = lwis_dev;
	/* Initialize locks */
	mutex_init(&lwis_client->lock);
	spin_lock_init(&lwis_client->event_lock);

	/* Empty hash table for client event states */
	hash_init(lwis_client->event_states);

	/* The event queue itself is a linked list */
	INIT_LIST_HEAD(&lwis_client->event_queue);

	/* Initialize the wait queue for the event queue */
	init_waitqueue_head(&lwis_client->event_wait_queue);

	/* Empty hash table for client allocated buffers */
	hash_init(lwis_client->allocated_buffers);

	/* Empty hash table for client enrolled buffers */
	hash_init(lwis_client->enrolled_buffers);

	/* Start transaction processor task */
	lwis_transaction_init(lwis_client);

	spin_lock_irqsave(&lwis_dev->lock, flags);
	list_add(&lwis_client->node, &lwis_dev->clients);
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Storing the client handle in fp private_data for easy access */
	fp->private_data = lwis_client;

	return 0;
}

/*
 *  lwis_release: Closing an instance of a LWIS device
 */
static int lwis_release(struct inode *node, struct file *fp)
{
	struct lwis_client *lwis_client = fp->private_data;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	struct list_head *p, *n;
	unsigned long flags;

	pr_info("Closing instance %d\n", iminor(node));

	/* Let's lock the mutex while we're cleaning up to avoid other parts
	 * of the code from acquiring dangling pointers to the client or any
	 * of the client event states that are about to be freed
	 */
	mutex_lock(&lwis_client->lock);

	/* Take this lwis_client off the list of active clients */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	list_for_each_safe(p, n, &lwis_dev->clients)
	{
		if (lwis_client == list_entry(p, struct lwis_client, node)) {
			list_del(p);
		}
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Cancel all pending transactions for the client */
	lwis_transaction_client_cleanup(lwis_client);

	/* Clear event states for this client */
	lwis_client_event_states_clear(lwis_client);

	/* Disenroll and clear the table of allocated and enrolled buffers */
	lwis_client_allocated_buffers_clear(lwis_client);
	lwis_client_enrolled_buffers_clear(lwis_client);

	mutex_unlock(&lwis_client->lock);
	kfree(lwis_client);

	return 0;
}

/*
 *  lwis_ioctl: I/O control function on a LWIS device
 *
 *  List of IOCTL types are defined in lwis_commands.h
 */
static long lwis_ioctl(struct file *fp, unsigned int type, unsigned long param)
{
	int ret = 0;
	struct lwis_client *lwis_client;
	struct lwis_device *lwis_dev;

	lwis_client = fp->private_data;
	if (!lwis_client) {
		pr_err("Cannot find client instance\n");
		return -ENODEV;
	}

	lwis_dev = lwis_client->lwis_dev;
	if (!lwis_dev) {
		pr_err("Cannot find device instance\n");
		return -ENODEV;
	}

	mutex_lock(&lwis_client->lock);

	ret = lwis_ioctl_handler(lwis_client, type, param);

	mutex_unlock(&lwis_client->lock);

	if (ret && ret != -ENOENT && ret != -ETIMEDOUT && ret != -EAGAIN) {
		pr_err("Error processing IOCTL %d (%d)\n", _IOC_NR(type), ret);
	}

	return ret;
}
/*
 *  lwis_poll: Event queue status function of LWIS
 *
 */
static unsigned int lwis_poll(struct file *fp, poll_table *wait)
{
	unsigned int mask = 0;
	struct lwis_client *lwis_client;

	lwis_client = fp->private_data;
	if (!lwis_client) {
		pr_err("Cannot find client instance\n");
		mask |= POLLERR;
	}

	mutex_lock(&lwis_client->lock);

	/* Add our wait queue to the poll table */
	poll_wait(fp, &lwis_client->event_wait_queue, wait);

	/* Check if we have anything in the event list */
	if (lwis_client_event_peek_front(lwis_client, NULL) == 0) {
		mask |= POLLIN;
	}

	mutex_unlock(&lwis_client->lock);

	return mask;
}

static int lwis_base_setup(struct lwis_device *lwis_dev)
{
	int ret = 0;

#ifdef CONFIG_OF
	/* Parse device tree for device configurations */
	ret = lwis_base_parse_dt(lwis_dev);
	if (ret) {
		pr_err("Failed to parse device tree\n");
	}
#else
	/* Non-device-tree init: Save for future implementation */
	ret = -ENOSYS;
#endif

	return ret;
}

/*
 *  lwis_base_probe: Create a device instance for each of the LWIS device.
 */
int lwis_base_probe(struct lwis_device *lwis_dev,
		    struct platform_device *plat_dev)
{
	int ret = 0;

	/* Allocate a minor number to this device */
	mutex_lock(&core.lock);
	ret = idr_alloc(core.idr, lwis_dev, 0, LWIS_MAX_DEVICES, GFP_KERNEL);
	mutex_unlock(&core.lock);
	if (ret >= 0) {
		lwis_dev->id = ret;
	} else {
		pr_err("Unable to allocate minor ID (%d)\n", ret);
		return ret;
	}

	/* Initialize enabled state */
	lwis_dev->enabled = 0;

	/* Initialize client mutex */
	mutex_init(&lwis_dev->client_lock);

	/* Initialize register access mutex */
	mutex_init(&lwis_dev->reg_rw_lock);

	/* Initialize an empty list of clients */
	INIT_LIST_HEAD(&lwis_dev->clients);

	/* Initialize event state hash table */
	hash_init(lwis_dev->event_states);

	/* Initialize the spinlock */
	spin_lock_init(&lwis_dev->lock);

	lwis_dev->plat_dev = plat_dev;
	ret = lwis_base_setup(lwis_dev);
	if (ret) {
		pr_err("Error initializing LWIS device\n");
		goto error_init;
	}

	/* Upon success initialization, create device for this instance */
	lwis_dev->dev = device_create(
		core.dev_class, NULL, MKDEV(core.device_major, lwis_dev->id),
		lwis_dev, LWIS_DEVICE_NAME "-%s", lwis_dev->name);
	if (IS_ERR(lwis_dev->dev)) {
		pr_err("Failed to create device\n");
		ret = PTR_ERR(lwis_dev->dev);
		goto error_init;
	}

	/* Add this instance to the device list */
	mutex_lock(&core.lock);
	list_add(&lwis_dev->dev_list, &core.lwis_dev_list);
	mutex_unlock(&core.lock);

	platform_set_drvdata(plat_dev, lwis_dev);

	/* Call platform-specific probe function */
	lwis_platform_probe(lwis_dev);

	pr_info("Base Probe: Success\n");

	return ret;

	/* Error conditions */
error_init:
	mutex_lock(&core.lock);
	idr_remove(core.idr, lwis_dev->id);
	mutex_unlock(&core.lock);
	return ret;
}

/*
 *  lwis_register_base_device: Create device class and device major number to
 *  the class of LWIS devices.
 *
 *  This is called once when the core LWIS driver is initialized.
 */
static int __init lwis_register_base_device(void)
{
	int ret = 0;
	dev_t lwis_devt;

	/* Allocate ID management instance for device minor numbers */
	core.idr = kzalloc(sizeof(struct idr), GFP_KERNEL);
	if (!core.idr) {
		pr_err("Cannot allocate idr instance\n");
		return -ENOMEM;
	}

	mutex_lock(&core.lock);

	idr_init(core.idr);

	/* Acquire device major number and allocate the range to minor numbers
	   to the device */
	ret = alloc_chrdev_region(&lwis_devt, 0, LWIS_MAX_DEVICES,
				  LWIS_DEVICE_NAME);
	if (ret) {
		pr_err("Error in allocating chrdev region\n");
		goto error_chrdev_alloc;
	}

	core.device_major = MAJOR(lwis_devt);

	/* Create a device class*/
	core.dev_class = class_create(THIS_MODULE, LWIS_CLASS_NAME);
	if (IS_ERR(core.dev_class)) {
		pr_err("Failed to create device class\n");
		ret = PTR_ERR(core.dev_class);
		goto error_class_create;
	}

	/* Allocate a character device */
	core.chr_dev = cdev_alloc();
	if (!core.chr_dev) {
		pr_err("Failed to allocate cdev\n");
		ret = -ENOMEM;
		goto error_cdev_alloc;
	}

	core.chr_dev->ops = &lwis_fops;

	ret = cdev_add(core.chr_dev, lwis_devt, LWIS_MAX_DEVICES);
	if (ret) {
		pr_err("Failed to add cdev\n");
		goto error_cdev_alloc;
	}

	INIT_LIST_HEAD(&core.lwis_dev_list);

	mutex_unlock(&core.lock);

	return ret;

	/* Error conditions */
error_cdev_alloc:
	class_destroy(core.dev_class);
	core.dev_class = NULL;
error_class_create:
	unregister_chrdev_region(lwis_devt, LWIS_MAX_DEVICES);
error_chrdev_alloc:
	mutex_unlock(&core.lock);
	kfree(core.idr);
	core.idr = NULL;

	return ret;
}

/*
 *  lwis_base_device_init: Called during subsys_initcall routines.
 */
static int __init lwis_base_device_init(void)
{
	int ret = 0;

	pr_info("LWIS device initialization\n");

	/* Initialize the core struct */
	memset(&core, 0, sizeof(struct lwis_core));
	mutex_init(&core.lock);

	ret = lwis_register_base_device();
	if (ret) {
		pr_err("Failed to register LWIS base\n");
	}

	ret = lwis_top_device_init();
	if (ret) {
		pr_err("Failed to lwis_top_device_init\n");
	}

	ret = lwis_ioreg_device_init();
	if (ret) {
		pr_err("Failed to lwis_ioreg_device_init\n");
	}

	ret = lwis_i2c_device_init();
	if (ret) {
		pr_err("Failed to lwis_i2c_device_init\n");
	}

	return ret;
}

subsys_initcall(lwis_base_device_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Google-ACMA");
MODULE_DESCRIPTION("LWIS Base Device Driver");
