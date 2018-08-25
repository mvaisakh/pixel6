/*
 * Google LWIS Camera Driver
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/device.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_commands.h"
#include "lwis_device.h"
#include "lwis_dt.h"

#define LWIS_CLASS_NAME		"lwis"
#define LWIS_DEVICE_NAME	"lwis"
#define LWIS_DRIVER_NAME	"lwis-driver"
#define LWIS_MAX_DEVICES	(1U << MINORBITS)

static struct lwis_core core;

static int  lwis_open(struct inode *node, struct file *fp);
static int  lwis_release(struct inode *node, struct file *fp);
static long lwis_ioctl(struct file *fp, unsigned int type,
			 unsigned long param);

static struct file_operations lwis_fops = {
	.owner		= THIS_MODULE,
	.open 		= lwis_open,
	.release 	= lwis_release,
	.unlocked_ioctl = lwis_ioctl,
};

/*
 *  lwis_open: Opening an instance of a LWIS device
 */
static int lwis_open(struct inode *node, struct file *fp)
{
	struct lwis_device *pdev;
	struct lwis_client *pclient;

	pr_info("Opening instance %d\n", iminor(node));

	/* Making sure the minor number associated with fp exists */
	mutex_lock(&core.lock);
	pdev = idr_find(core.pidr, iminor(node));
	mutex_unlock(&core.lock);
	if (!pdev) {
		pr_err("No device %d found\n", iminor(node));
		return -ENODEV;
	}

	pclient = kzalloc(sizeof(struct lwis_client), GFP_KERNEL);
	if (!pclient) {
		pr_err("Failed to allocate lwis client\n");
		return -ENOMEM;
	}

	pclient->pdev = pdev;
	mutex_init(&pclient->lock);

	/* Storing the client handle in fp private_data for easy access */
	fp->private_data = pclient;

	return 0;
}

/*
 *  lwis_release: Closing an instance of a LWIS device
 */
static int lwis_release(struct inode *node, struct file *fp)
{
	struct lwis_client *pclient = fp->private_data;

	pr_info("Closing instance %d\n", iminor(node));

	kfree(pclient);

	return 0;
}

/*
 *  lwis_ioctl: I/O control function on a LWIS device
 *
 *  List of IOCTL types are defined in lwis_commands.h
 */
static long lwis_ioctl(struct file *fp, unsigned int type,
			unsigned long param)
{
	int ret = 0;
	struct lwis_client *pclient;
	struct lwis_device *pldev;

	pclient = fp->private_data;
	if (!pclient) {
		pr_err("Cannot find client instance\n");
		return -ENODEV;
	}

	pldev = pclient->pdev;
	if (!pldev) {
		pr_err("Cannot find device instance\n");
		return -ENODEV;
	}

	mutex_lock(&pclient->lock);

	switch (type) {
		case LWIS_GET_DEVICE_INFO:
		{
			struct lwis_device_info resp;
			resp.dummy0 = 12;
			resp.dummy1 = 34;
			resp.dummy2 = 56;
			ret = copy_to_user((void __user *)param, &resp,
					   sizeof(struct lwis_device_info));
			break;
		}
		case LWIS_ENROLL_BUFFER:
		{
			break;
		}
		case LWIS_SENSOR_INIT:
		{
			lwis_sensor_init(pldev->psensor);
			break;
		}
		default:
		{
			pr_err("Unknown IOCTL operation\n");
			ret = -EINVAL;
		}
	};

	mutex_unlock(&pclient->lock);

	return ret;
}

#ifdef CONFIG_OF

static const struct of_device_id lwis_id_match[] = {
	{
		.compatible = "google,lwis-device",
	},
	{},
};
MODULE_DEVICE_TABLE(of, lwis_id_match);

static struct platform_driver lwis_driver = {
	.driver = {
		.name = LWIS_DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = lwis_id_match,
	},
};

#else

static struct platform_device_id lwis_driver_id[] = {
	{
		.name		= LWIS_DRIVER_NAME,
		.driver_data	= 0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, lwis_driver_id);

static struct platform_driver lwis_driver = {
	.id_table	= lwis_driver_id,
	.driver	  = {
		.name	= LWIS_DRIVER_NAME,
		.owner	= THIS_MODULE,
	}
};

#endif  /* CONFIG_OF */

/*
 *  lwis_register_device: Create device class and device major number to the
 *  class of LWIS devices.
 *
 *  This is called once when the LWIS driver is registered as a platform device.
 */
static int __init lwis_register_device(void)
{
	int ret = 0;
	dev_t lwis_devt;

	/* Allocate ID management instance for device minor numbers */
	core.pidr = kzalloc(sizeof(struct idr), GFP_KERNEL);
	if (!core.pidr) {
		pr_err("Cannot allocate idr instance\n");
		return -ENOMEM;
	}

	mutex_lock(&core.lock);

	idr_init(core.pidr);

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
	core.pclass = class_create(THIS_MODULE, LWIS_CLASS_NAME);
	if (IS_ERR(core.pclass)) {
		pr_err("Failed to create device class\n");
		ret = PTR_ERR(core.pclass);
		goto error_class_create;
	}

	/* Allocate a character device */
	core.pcdev = cdev_alloc();
	if (!core.pcdev) {
		pr_err("Failed to allocate cdev\n");
		ret = -ENOMEM;
		goto error_cdev_alloc;
	}

	core.pcdev->ops = &lwis_fops;

	ret = cdev_add(core.pcdev, lwis_devt, LWIS_MAX_DEVICES);
	if (ret) {
		pr_err("Failed to add cdev\n");
		goto error_cdev_alloc;
	}

	mutex_unlock(&core.lock);

	return ret;

	/* Error conditions */
error_cdev_alloc:
	class_destroy(core.pclass);
	core.pclass = NULL;
error_class_create:
	unregister_chrdev_region(lwis_devt, LWIS_MAX_DEVICES);
error_chrdev_alloc:
	mutex_unlock(&core.lock);
	kfree(core.pidr);
	core.pidr = NULL;

	return ret;
}

/*
 *  lwis_probe: Create a device instance for each of the LWIS device.
 */
static int __init lwis_probe(struct platform_device *ppdev)
{
	int ret = 0;
	struct lwis_device *pldev;

	pr_info("Probe: Begin\n");

	/* Create LWIS device instance */
	pldev = kzalloc(sizeof(struct lwis_device), GFP_KERNEL);
	if (!pldev) {
		pr_err("Failed to allocate lwis_device struct\n");
		return -ENOMEM;
	}

	/* Allocate a minor number to this device */
	mutex_lock(&core.lock);
	ret = idr_alloc(core.pidr, pldev, 0, LWIS_MAX_DEVICES,
			GFP_KERNEL);
	mutex_unlock(&core.lock);
	if (ret >= 0) {
		pldev->id = ret;
		ret = 0;
	} else {
		pr_err("Unable to allocate minor ID (%d)\n", ret);
		goto error_minor_alloc;
	}

	pldev->ppdev = ppdev;
	pldev->pdev = device_create(core.pclass, NULL,
				     MKDEV(core.device_major, pldev->id),
				     pldev, LWIS_DEVICE_NAME "%d",
				     pldev->id);
	if (IS_ERR(pldev->pdev)) {
		pr_err("Failed to create device\n");
		ret = PTR_ERR(pldev->pdev);
		goto error_dev_create;
	}

	/* Parse device tree for device configurations */
	ret = lwis_device_parse_dt(pldev);
	if (ret) {
		pr_err("Failed to parse device tree\n");
		goto error_parse_dt;
	}

	platform_set_drvdata(ppdev, pldev);

	pr_info("Probe: Done\n");

	return ret;

	/* Error conditions */
error_parse_dt:
	device_destroy(core.pclass, MKDEV(core.device_major, pldev->id));
error_dev_create:
	mutex_lock(&core.lock);
	idr_remove(core.pidr, pldev->id);
	mutex_unlock(&core.lock);
error_minor_alloc:
	kfree(pldev);
	return ret;
}

/*
 *  lwis_device_init: Called during device_initcall_sync routines.
 */
static int __init lwis_device_init(void)
{
	int ret = 0;

	pr_info("Device initialization\n");

	/* Initialize the core struct */
	memset(&core, 0, sizeof(struct lwis_core));
	mutex_init(&core.lock);

	lwis_register_device();

	ret = platform_driver_probe(&lwis_driver, lwis_probe);
	if (ret) {
		pr_err("platform_driver_probe failed - %d", ret);
	}

	return ret;
}

device_initcall_sync(lwis_device_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Google-ACMA");
MODULE_DESCRIPTION("LWIS Driver");
