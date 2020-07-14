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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_buffer.h"
#include "lwis_clock.h"
#include "lwis_commands.h"
#include "lwis_debug.h"
#include "lwis_device.h"
#include "lwis_device_slc.h"
#include "lwis_dpm.h"
#include "lwis_dt.h"
#include "lwis_event.h"
#include "lwis_gpio.h"
#include "lwis_init.h"
#include "lwis_ioctl.h"
#include "lwis_periodic_io.h"
#include "lwis_pinctrl.h"
#include "lwis_platform.h"
#include "lwis_transaction.h"

#ifdef CONFIG_OF
#include "lwis_dt.h"
#endif

#define LWIS_CLASS_NAME "lwis"
#define LWIS_DEVICE_NAME "lwis"
#define LWIS_MAX_DEVICES (1U << MINORBITS)
#define MCLK_ON_STRING "mclk_on"
#define MCLK_OFF_STRING "mclk_off"

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

	/* Making sure the minor number associated with fp exists */
	mutex_lock(&core.lock);
	lwis_dev = idr_find(core.idr, iminor(node));
	mutex_unlock(&core.lock);
	if (!lwis_dev) {
		pr_err("No device %d found\n", iminor(node));
		return -ENODEV;
	}
	dev_info(lwis_dev->dev, "Opening instance %d\n", iminor(node));

	lwis_client = kzalloc(sizeof(struct lwis_client), GFP_KERNEL);
	if (!lwis_client) {
		dev_err(lwis_dev->dev, "Failed to allocate lwis client\n");
		return -ENOMEM;
	}

	lwis_client->lwis_dev = lwis_dev;
	/* Initialize locks */
	mutex_init(&lwis_client->lock);
	mutex_init(&lwis_client->periodic_io_lock);
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

	/* Start periodic io processor task */
	lwis_periodic_io_init(lwis_client);

	memset(&lwis_client->debug_info, 0, sizeof(lwis_client->debug_info));

	spin_lock_irqsave(&lwis_dev->lock, flags);
	list_add(&lwis_client->node, &lwis_dev->clients);
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Storing the client handle in fp private_data for easy access */
	fp->private_data = lwis_client;

	return 0;
}

static int lwis_release_client(struct lwis_client *lwis_client)
{
	/* Let's lock the mutex while we're cleaning up to avoid other parts
	 * of the code from acquiring dangling pointers to the client or any
	 * of the client event states that are about to be freed
	 */
	mutex_lock(&lwis_client->lock);

	/* Clear event states for this client */
	lwis_client_event_states_clear(lwis_client);

	/* Clean up all periodic io state for the client */
	lwis_periodic_io_client_cleanup(lwis_client);

	/* Cancel all pending transactions for the client */
	lwis_transaction_client_cleanup(lwis_client);

	/* Disenroll and clear the table of allocated and enrolled buffers */
	lwis_client_allocated_buffers_clear(lwis_client);
	lwis_client_enrolled_buffers_clear(lwis_client);

	mutex_unlock(&lwis_client->lock);
	kfree(lwis_client);

	return 0;
}
/*
 *  lwis_release: Closing an instance of a LWIS device
 */
static int lwis_release(struct inode *node, struct file *fp)
{
	struct lwis_client *lwis_client = fp->private_data;
	struct lwis_client *p, *n;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;
	unsigned long flags;
	int rc = 0;

	dev_info(lwis_dev->dev, "Closing instance %d\n", iminor(node));

	/* Take this lwis_client off the list of active clients */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	list_for_each_entry_safe (p, n, &lwis_dev->clients, node) {
		if (lwis_client == p)
			list_del(&lwis_client->node);
	}
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	rc = lwis_release_client(lwis_client);

	mutex_lock(&lwis_dev->client_lock);
	/* Release power if client closed without power down called */
	if (lwis_dev->enabled > 0) {
		lwis_dev->enabled--;
		if (lwis_dev->enabled == 0) {
			dev_info(lwis_dev->dev, "No more client, power down\n");
			rc = lwis_dev_power_down_locked(lwis_dev);
		}
	}
	/* Release device event states if no more client is using */
	if (lwis_dev->enabled == 0)
		lwis_device_event_states_clear_locked(lwis_dev);
	mutex_unlock(&lwis_dev->client_lock);

	return rc;
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
		lwis_ioctl_pr_err(lwis_dev, type, ret);
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
 *  lwis_assign_top_to_other: Assign top device to the devices probed before.
 */
static void lwis_assign_top_to_other(struct lwis_device *top_dev)
{
	struct lwis_device *lwis_dev;

	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev, &core.lwis_dev_list, dev_list) {
		lwis_dev->top_dev = top_dev;
	}
	mutex_unlock(&core.lock);
}

/*
 * Power up a LWIS device, should be called when lwis_dev->enabled is 0
 * lwis_dev->client_lock should be held before this function.
 * TODO(edmondchung): Not sure whether this is generic enough to handle device
 * enable yet, but this is sufficient to enable the sensor - so I will stick
 * with this for now.
 */
int lwis_dev_power_up_locked(struct lwis_device *lwis_dev)
{
	int ret;

	/* Let's do the platform-specific enable call */
	ret = lwis_platform_device_enable(lwis_dev);
	if (ret) {
		dev_err(lwis_dev->dev,
			"Platform-specific device enable fail: %d\n", ret);
		goto error_platform_enable;
	}

	if (lwis_dev->clocks) {
		/* Enable clocks */
		ret = lwis_clock_enable_all(lwis_dev->clocks);
		if (ret) {
			dev_err(lwis_dev->dev, "Error enabling clocks (%d)\n",
				ret);
			goto error_clock_enable;
		}
	}

	if (lwis_dev->enable_gpios_present) {
		struct gpio_descs *gpios;

		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev, "enable");
		if (IS_ERR_OR_NULL(gpios)) {
			dev_err(lwis_dev->dev,
				"Failed to obtain enable gpio list (%d)\n",
				PTR_ERR(gpios));
			ret = PTR_ERR(gpios);
			goto error_gpio_get;
		}

		/* Setting enable_gpios to non-NULL to indicate that this lwis
			 device is holding onto the GPIO pins. */
		lwis_dev->enable_gpios = gpios;

		/* Set enable pins to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 1);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error enabling GPIO pins (%d)\n", ret);
			goto error_gpio_set;
		}

		if (lwis_dev->enable_gpios_settle_time > 0) {
			usleep_range(lwis_dev->enable_gpios_settle_time,
				     lwis_dev->enable_gpios_settle_time);
		}
	}

	if (lwis_dev->regulators) {
		/* Enable all the regulators related to this sensor */
		ret = lwis_regulator_enable_all(lwis_dev->regulators);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error enabling regulators (%d)\n", ret);
			goto error_regulator_enable;
		}
	}

	if (lwis_dev->reset_gpios_present) {
		struct gpio_descs *gpios;

		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev, "reset");
		if (IS_ERR_OR_NULL(gpios)) {
			dev_err(lwis_dev->dev,
				"Failed to obtain reset gpio list (%d)\n",
				PTR_ERR(gpios));
			ret = PTR_ERR(gpios);
			goto error_reset_gpio_get;
		}
		/* Setting reset_gpios to non-NULL to indicate that this lwis
			 device is holding onto the GPIO pins. */
		lwis_dev->reset_gpios = gpios;

		/* Set reset pin to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 1);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Failed to set reset GPIOs to ACTIVE (%d)\n",
				ret);
			goto error_reset_gpio_set;
		}

		/* Inherited from FIMC, will see if this is needed */
		usleep_range(1500, 1500);

		/* Set reset pin to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 0);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Failed to set reset GPIOs to INACTIVE (%d)\n",
				ret);
			goto error_reset_gpio_unset;
		}
	}

	if (lwis_dev->mclk_present) {
		bool activate_mclk = true;

		lwis_dev->mclk_ctrl =
			devm_pinctrl_get(&lwis_dev->plat_dev->dev);
		if (IS_ERR(lwis_dev->mclk_ctrl)) {
			dev_err(lwis_dev->dev, "Failed to get mclk\n");
			ret = PTR_ERR(lwis_dev->mclk_ctrl);
			lwis_dev->mclk_ctrl = NULL;
			goto error_mclk_enable;
		}
		if (lwis_dev->shared_pinctrl > 0) {
			struct lwis_device *lwis_dev_it;
			/* Look up if pinctrl it's already enabled */
			mutex_lock(&core.lock);
			list_for_each_entry (lwis_dev_it, &core.lwis_dev_list,
					     dev_list) {
				if ((lwis_dev->id != lwis_dev_it->id) &&
				    (lwis_dev_it->shared_pinctrl ==
				     lwis_dev->shared_pinctrl) &&
				    lwis_dev_it->enabled) {
					activate_mclk = false;
					devm_pinctrl_put(lwis_dev->mclk_ctrl);
					lwis_dev->mclk_ctrl = NULL;
					dev_info(lwis_dev->dev,
						 "mclk already be acquired\n");
					break;
				}
			}
			mutex_unlock(&core.lock);
		}

		if (activate_mclk) {
			/* Set MCLK state to on */
			ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl,
						     MCLK_ON_STRING);
			if (ret) {
				dev_err(lwis_dev->dev,
					"Error setting %s state (%d)\n",
					MCLK_ON_STRING, ret);
				devm_pinctrl_put(lwis_dev->mclk_ctrl);
				lwis_dev->mclk_ctrl = NULL;
				goto error_mclk_enable;
			}
		}
	}

	if (lwis_dev->shared_enable_gpios_present) {
		struct gpio_descs *gpios;

		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev,
					   "shared-enable");
		if (IS_ERR_OR_NULL(gpios)) {
			if (PTR_ERR(gpios) == -EBUSY) {
				dev_warn(
					lwis_dev->dev,
					"Shared gpios requested by another device\n");
			} else {
				dev_err(lwis_dev->dev,
					"Failed to obtain shared gpio list (%d)\n",
					PTR_ERR(gpios));
				ret = PTR_ERR(gpios);
				goto error_shared_gpio_get;
			}
		} else {
			/* Set enable pins to 1 (i.e. asserted) */
			ret = lwis_gpio_list_set_output_value(gpios, 1);
			lwis_dev->shared_enable_gpios = gpios;
			if (ret) {
				dev_err(lwis_dev->dev,
					"Error enabling GPIO pins (%d)\n", ret);
				goto error_shared_gpio_set;
			}
		}
	}

	if (lwis_dev->phys) {
		/* Power on the PHY */
		ret = lwis_phy_set_power_all(lwis_dev->phys,
					     /* power_on = */ true);
		if (ret) {
			dev_err(lwis_dev->dev, "Error powering on PHY\n");
			goto error_phy_set_power;
		}
	}

	if (lwis_dev->irqs) {
		ret = lwis_interrupt_request_all_default(lwis_dev->irqs);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Failed to request interrupts (%d)\n", ret);
			goto error_irq_request;
		}
	}

	if (lwis_dev->vops.device_enable) {
		ret = lwis_dev->vops.device_enable(lwis_dev);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error executing device enable function\n");
			goto error_vops_device_enable;
		}
	}

	/* Sleeping to make sure all pins are ready to go */
	usleep_range(2000, 2000);
	return 0;
	/* Error handling */
error_vops_device_enable:
	if (lwis_dev->irqs) {
		lwis_interrupt_free_all_default(lwis_dev->irqs);
	}
error_irq_request:
	if (lwis_dev->phys) {
		lwis_phy_set_power_all(lwis_dev->phys, /* power_on = */ false);
	}
error_phy_set_power:
	if (lwis_dev->shared_enable_gpios_present &&
	    lwis_dev->shared_enable_gpios) {
		lwis_gpio_list_set_output_value(lwis_dev->shared_enable_gpios,
						0);
	}
error_shared_gpio_set:
	if (lwis_dev->shared_enable_gpios_present &&
	    lwis_dev->shared_enable_gpios) {
		/* Release "ownership" of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->shared_enable_gpios,
				   &lwis_dev->plat_dev->dev);
		lwis_dev->shared_enable_gpios = NULL;
	}
error_shared_gpio_get:
	if (lwis_dev->mclk_ctrl) {
		lwis_pinctrl_set_state(lwis_dev->mclk_ctrl, MCLK_OFF_STRING);
	}
error_mclk_enable:
	if (lwis_dev->reset_gpios_present && lwis_dev->reset_gpios) {
		/* Set reset pins to 1 (i.e. asserted) */
		lwis_gpio_list_set_output_value(lwis_dev->reset_gpios, 1);
	}
error_reset_gpio_unset:
error_reset_gpio_set:
	if (lwis_dev->reset_gpios_present && lwis_dev->reset_gpios) {
		/* Release ownership of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->reset_gpios,
				   &lwis_dev->plat_dev->dev);
		lwis_dev->reset_gpios = NULL;
	}
error_reset_gpio_get:
	if (lwis_dev->regulators) {
		/* Disable all the regulators */
		lwis_regulator_disable_all(lwis_dev->regulators);
	}
error_regulator_enable:
	if (lwis_dev->enable_gpios_present && lwis_dev->enable_gpios) {
		/* Set enable pins to 0 (i.e. deasserted) */
		lwis_gpio_list_set_output_value(lwis_dev->enable_gpios, 0);
	}
error_gpio_set:
	if (lwis_dev->enable_gpios_present && lwis_dev->enable_gpios) {
		/* Release "ownership" of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->enable_gpios,
				   &lwis_dev->plat_dev->dev);
		lwis_dev->enable_gpios = NULL;
	}
error_gpio_get:
	if (lwis_dev->clocks) {
		/* Disable all clocks */
		lwis_clock_disable_all(lwis_dev->clocks);
	}
error_clock_enable:
	/* Let's do the platform-specific disable call */
	lwis_platform_device_disable(lwis_dev);
error_platform_enable:
	return ret;
}

/*
 * Power down a LWIS device, should be called when lwis_dev->enabled become 0
 * lwis_dev->client_lock should be held before this function.
 * TODO(edmondchung): Same comment as lwis_dev_power_up_locked.
 */
int lwis_dev_power_down_locked(struct lwis_device *lwis_dev)
{
	int ret;

	if (lwis_dev->vops.device_disable) {
		ret = lwis_dev->vops.device_disable(lwis_dev);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error executing device disable function\n");
			return ret;
		}
	}

	if (lwis_dev->irqs) {
		lwis_interrupt_free_all_default(lwis_dev->irqs);
	}

	if (lwis_dev->phys) {
		/* Power on the PHY */
		ret = lwis_phy_set_power_all(lwis_dev->phys,
					     /* power_on = */ false);
		if (ret) {
			dev_err(lwis_dev->dev, "Error powering off PHY\n");
			return ret;
		}
	}

	if (lwis_dev->mclk_ctrl) {
		bool deactivate_mclk = true;

		if (lwis_dev->shared_pinctrl) {
			struct lwis_device *lwis_dev_it;
			/* Look up if pinctrl still used by other device */
			mutex_lock(&core.lock);
			list_for_each_entry (lwis_dev_it, &core.lwis_dev_list,
					     dev_list) {
				if ((lwis_dev->id != lwis_dev_it->id) &&
				    (lwis_dev_it->shared_pinctrl ==
				     lwis_dev->shared_pinctrl) &&
				    lwis_dev_it->enabled) {
					/* Move mclk owner to the device who
					   still using it */
					lwis_dev_it->mclk_ctrl =
						lwis_dev->mclk_ctrl;
					lwis_dev->mclk_ctrl = NULL;
					deactivate_mclk = false;
					break;
				}
			}
			mutex_unlock(&core.lock);
		}
		if (deactivate_mclk) {
			/* Set MCLK state to off */
			ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl,
						     MCLK_OFF_STRING);
			if (ret) {
				dev_err(lwis_dev->dev,
					"Error setting %s state (%d)\n",
					MCLK_OFF_STRING, ret);
				return ret;
			}
			devm_pinctrl_put(lwis_dev->mclk_ctrl);
			lwis_dev->mclk_ctrl = NULL;
		}
	}

	if (lwis_dev->shared_enable_gpios_present &&
	    lwis_dev->shared_enable_gpios) {
		/* Set enable pins to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(
			lwis_dev->shared_enable_gpios, 0);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error disabling GPIO pins (%d)\n", ret);
			return ret;
		}

		/* Release "ownership" of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->shared_enable_gpios,
				   &lwis_dev->plat_dev->dev);
		lwis_dev->shared_enable_gpios = NULL;
	}

	if (lwis_dev->enable_gpios_present && lwis_dev->enable_gpios) {
		/* Set enable pins to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(lwis_dev->enable_gpios,
						      0);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error disabling GPIO pins (%d)\n", ret);
			return ret;
		}

		/* Release "ownership" of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->enable_gpios,
				   &lwis_dev->plat_dev->dev);
		lwis_dev->enable_gpios = NULL;
	}

	if (lwis_dev->regulators) {
		/* Disable all the regulators */
		ret = lwis_regulator_disable_all(lwis_dev->regulators);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error disabling regulators (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->reset_gpios_present && lwis_dev->reset_gpios) {
		/* Set reset pins to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(lwis_dev->reset_gpios, 1);
		if (ret) {
			dev_err(lwis_dev->dev,
				"Error setting reset GPIOs to ACTIVE (%d)\n",
				ret);
			return ret;
		}

		/* Release ownership of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->reset_gpios,
				   &lwis_dev->plat_dev->dev);
		lwis_dev->reset_gpios = NULL;
	}

	if (lwis_dev->clocks) {
		/* Disable all clocks */
		lwis_clock_disable_all(lwis_dev->clocks);
	}

	/* Let's do the platform-specific disable call */
	ret = lwis_platform_device_disable(lwis_dev);
	if (ret) {
		dev_err(lwis_dev->dev,
			"Platform-specific device disable fail: %d\n", ret);
		return ret;
	}
	return ret;
}

/*
 *  lwis_find_top_dev: Find LWIS top device.
 */
struct lwis_device *lwis_find_top_dev()
{
	struct lwis_device *lwis_dev;

	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev, &core.lwis_dev_list, dev_list) {
		if (lwis_dev->type == DEVICE_TYPE_TOP) {
			mutex_unlock(&core.lock);
			return lwis_dev;
		}
	}
	mutex_unlock(&core.lock);
	return NULL;
}

/*
 *  lwis_find_dev_by_id: Find LWIS device by id.
 */
struct lwis_device *lwis_find_dev_by_id(int dev_id)
{
	struct lwis_device *lwis_dev;

	mutex_lock(&core.lock);
	list_for_each_entry (lwis_dev, &core.lwis_dev_list, dev_list) {
		if (lwis_dev->id == dev_id) {
			mutex_unlock(&core.lock);
			return lwis_dev;
		}
	}
	mutex_unlock(&core.lock);
	return NULL;
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
	lwis_dev->clock_family = CLOCK_FAMILY_INVALID;
	lwis_dev->last_requested_clock = 0;

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

	if (lwis_dev->type == DEVICE_TYPE_TOP) {
		lwis_dev->top_dev = lwis_dev;
		/* Assign top device to the devices probed before */
		lwis_assign_top_to_other(lwis_dev);
	} else {
		lwis_dev->top_dev = lwis_find_top_dev();
		if (lwis_dev->top_dev == NULL)
			pr_warn("Top device not probed yet");
	}

	lwis_dev->plat_dev = plat_dev;
	ret = lwis_base_setup(lwis_dev);
	if (ret) {
		pr_err("Error initializing LWIS device\n");
		goto error_init;
	}

	/* Upon success initialization, create device for this instance */
	lwis_dev->dev =
		device_create(core.dev_class, NULL,
			      MKDEV(core.device_major, lwis_dev->id), lwis_dev,
			      LWIS_DEVICE_NAME "-%s", lwis_dev->name);
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

	lwis_device_debugfs_setup(lwis_dev, core.dbg_root);
	memset(&lwis_dev->debug_info, 0, sizeof(lwis_dev->debug_info));

	dev_info(lwis_dev->dev, "Base Probe: Success\n");

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
	ret = alloc_chrdev_region(&core.lwis_devt, 0, LWIS_MAX_DEVICES,
				  LWIS_DEVICE_NAME);
	if (ret) {
		pr_err("Error in allocating chrdev region\n");
		goto error_chrdev_alloc;
	}

	core.device_major = MAJOR(core.lwis_devt);

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

	ret = cdev_add(core.chr_dev, core.lwis_devt, LWIS_MAX_DEVICES);
	if (ret) {
		pr_err("Failed to add cdev\n");
		goto error_cdev_alloc;
	}

	INIT_LIST_HEAD(&core.lwis_dev_list);

#ifdef CONFIG_DEBUG_FS
	/* Create DebugFS directory for LWIS, if avaiable */
	core.dbg_root = debugfs_create_dir("lwis", NULL);
	if (IS_ERR_OR_NULL(core.dbg_root)) {
		/* No need to return error as this is just informational that
		   DebugFS is not present */
		pr_info("Failed to create DebugFS dir - DebugFS not present?");
		core.dbg_root = NULL;
	}
#endif

	mutex_unlock(&core.lock);

	return ret;

	/* Error conditions */
error_cdev_alloc:
	class_destroy(core.dev_class);
	core.dev_class = NULL;
error_class_create:
	unregister_chrdev_region(core.lwis_devt, LWIS_MAX_DEVICES);
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

	ret = lwis_slc_device_init();
	if (ret) {
		pr_err("Failed to lwis_slc_device_init\n");
	}

	return ret;
}

/*
 *  lwis_base_device_deinit: Called when driver is unloaded.
 */
static void __exit lwis_driver_exit(void)
{
	struct lwis_device *lwis_dev, *temp;
	struct lwis_client *client, *client_temp;
	struct lwis_i2c_device *i2c_dev;

	pr_info("%s Clean up LWIS devices.\n", __func__);
	cdev_del(core.chr_dev);
	list_for_each_entry_safe (lwis_dev, temp, &core.lwis_dev_list,
				  dev_list) {
		pr_info("Destroy device %s id %d", lwis_dev->name,
			lwis_dev->id);
		lwis_device_debugfs_cleanup(lwis_dev);
		/* Disable lwis device events */
		lwis_device_event_enable(lwis_dev, LWIS_EVENT_ID_HEARTBEAT,
					 false);
		if (lwis_dev->type == DEVICE_TYPE_I2C) {
			i2c_dev = (struct lwis_i2c_device *)lwis_dev;
			i2c_unregister_device(i2c_dev->client);
		}
		/* Relase each client registered with dev */
		list_for_each_entry_safe (client, client_temp,
					  &lwis_dev->clients, node) {
			if (lwis_release_client(client))
				pr_info("Failed to release client.");
		}
		pm_runtime_disable(&lwis_dev->plat_dev->dev);
		/* Release device clock list */
		if (lwis_dev->clocks)
			lwis_clock_list_free(lwis_dev->clocks);
		/* Release device interrupt list */
		if (lwis_dev->irqs)
			lwis_interrupt_list_free(lwis_dev->irqs);
		/* Release device regulator list */
		if (lwis_dev->regulators)
			lwis_regulator_list_free(lwis_dev->regulators);
		/* Release device phy list */
		if (lwis_dev->phys)
			lwis_phy_list_free(lwis_dev->phys);
		/* Release device gpio list */
		if (lwis_dev->reset_gpios)
			lwis_gpio_list_put(lwis_dev->reset_gpios,
					   &lwis_dev->plat_dev->dev);
		if (lwis_dev->enable_gpios)
			lwis_gpio_list_put(lwis_dev->enable_gpios,
					   &lwis_dev->plat_dev->dev);
		/* Release event subscription components */
		if (lwis_dev->type == DEVICE_TYPE_TOP)
			lwis_dev->top_dev->subscribe_ops.release(lwis_dev);
		/* Destroy device */
		device_destroy(core.dev_class,
			       MKDEV(core.device_major, lwis_dev->id));
		list_del(&lwis_dev->dev_list);
		kfree(lwis_dev);
	}

	/* Unregister core lwis device */
	unregister_chrdev_region(core.lwis_devt, LWIS_MAX_DEVICES);
	class_destroy(core.dev_class);
	core.dev_class = NULL;
	kfree(core.idr);
	core.idr = NULL;

	/* Deinit device classes */
	lwis_top_device_deinit();
	lwis_i2c_device_deinit();
	lwis_ioreg_device_deinit();
	lwis_slc_device_deinit();
}

subsys_initcall(lwis_base_device_init);
module_exit(lwis_driver_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Google-ACMA");
MODULE_DESCRIPTION("LWIS Base Device Driver");
