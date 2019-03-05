/*
 * Google LWIS IOCTL Handler
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioctl: " fmt

#include "lwis_ioctl.h"

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_buffer.h"
#include "lwis_clock.h"
#include "lwis_commands.h"
#include "lwis_device_i2c.h"
#include "lwis_device_ioreg.h"
#include "lwis_event.h"
#include "lwis_gpio.h"
#include "lwis_i2c.h"
#include "lwis_ioreg.h"
#include "lwis_pinctrl.h"
#include "lwis_platform.h"
#include "lwis_regulator.h"

#define MCLK_ON_STRING "mclk_on"
#define MCLK_OFF_STRING "mclk_off"

static int ioctl_reg_read(struct lwis_device *lwis_dev,
			  struct lwis_io_msg *user_msg)
{
	int ret;
	struct lwis_io_msg k_msg;
	struct lwis_io_data *user_buf;
	struct lwis_io_data *k_buf;

	/* register read is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_read) {
		pr_err("Register read not supported on this LWIS device\n");
		return -EINVAL;
	}

	/* Copy message struct from userspace */
	ret = copy_from_user(&k_msg, (void __user *)user_msg,
			     sizeof(struct lwis_io_msg));
	if (ret) {
		return ret;
	}

	user_buf = k_msg.buf;

	/* Allocate a kernel buffer for the read data */
	k_buf = kzalloc(k_msg.num_entries * sizeof(struct lwis_io_data),
			GFP_KERNEL);
	if (!k_buf) {
		return -ENOMEM;
	}

	/* Set kernel buffer in the message */
	k_msg.buf = k_buf;

	/* Copy read data from userspace */
	ret = copy_from_user(k_buf, (void __user *)user_buf,
			     k_msg.num_entries * sizeof(struct lwis_io_data));
	if (ret) {
		goto error_i2c_read;
	}

	ret = lwis_dev->vops.register_read(lwis_dev, &k_msg, false);
	if (ret) {
		goto error_i2c_read;
	}

	/* Copy read data back to userspace */
	ret = copy_to_user((void __user *)user_buf, k_buf,
			   k_msg.num_entries * sizeof(struct lwis_io_data));

error_i2c_read:
	kfree(k_buf);
	return ret;
}

static int ioctl_reg_write(struct lwis_device *lwis_dev,
			   struct lwis_io_msg *user_msg)
{
	int ret;
	struct lwis_io_msg k_msg;
	struct lwis_io_data *user_buf;
	struct lwis_io_data *k_buf;

	/* register write is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_write) {
		pr_err("Register write not supported on this LWIS device\n");
		return -EINVAL;
	}

	/* Copy message struct from userspace */
	ret = copy_from_user(&k_msg, (void __user *)user_msg,
			     sizeof(struct lwis_io_msg));
	if (ret) {
		return ret;
	}

	user_buf = k_msg.buf;

	/* Allocate a kernel buffer for the write data */
	k_buf = kzalloc(k_msg.num_entries * sizeof(struct lwis_io_data),
			GFP_KERNEL);
	if (!k_buf) {
		return -ENOMEM;
	}

	/* Set kernel buffer in the message */
	k_msg.buf = k_buf;

	/* Copy write data from userspace */
	ret = copy_from_user(k_buf, (void __user *)user_buf,
			     k_msg.num_entries * sizeof(struct lwis_io_data));
	if (ret) {
		goto error_i2c_write;
	}

	ret = lwis_dev->vops.register_write(lwis_dev, &k_msg, false);

error_i2c_write:
	kfree(k_buf);
	return ret;
}

static int ioctl_buffer_enroll(struct lwis_client *lwis_client,
			       struct lwis_buffer_info __user *msg)
{
	unsigned long ret;
	struct lwis_buffer *buffer;

	buffer = kzalloc(sizeof(struct lwis_buffer), GFP_KERNEL);
	if (!buffer) {
		pr_err("Failed to allocate lwis_buffer struct\n");
		return -ENOMEM;
	}

	ret = copy_from_user((void *)&buffer->info, (void __user *)msg,
			     sizeof(buffer->info));
	if (ret) {
		pr_err("Failed to copy %zu bytes from user\n",
		       sizeof(buffer->info));
		kfree(buffer);
		return -EINVAL;
	}

	ret = lwis_buffer_enroll(lwis_client, buffer);

	if (ret) {
		pr_err("Failed to enroll buffer\n");
		kfree(buffer);
		return ret;
	}

	ret = copy_to_user((void __user *)msg, (void *)&buffer->info,
			   sizeof(buffer->info));
	if (ret) {
		pr_err("Failed to copy %zu bytes to user\n",
		       sizeof(buffer->info));
		lwis_buffer_disenroll(lwis_client, buffer);
		return -EINVAL;
	}

	return 0;
}

static int ioctl_buffer_disenroll(struct lwis_client *lwis_client,
				  struct lwis_buffer_info __user *msg)
{
	unsigned long ret;
	struct lwis_buffer_info info;
	struct lwis_buffer *buffer;

	ret = copy_from_user((void *)&info, (void __user *)msg, sizeof(info));
	if (ret) {
		pr_err("Failed to copy %zu bytes from user\n", sizeof(info));
		return -EINVAL;
	}

	buffer = lwis_client_enrolled_buffer_find(lwis_client, info.dma_vaddr);

	if (!buffer) {
		pr_err("Failed to find dma buffer for %llx\n", info.dma_vaddr);
		return -ENOENT;
	}

	ret = lwis_buffer_disenroll(lwis_client, buffer);
	if (ret) {
		pr_err("Failed to disenroll dma buffer for %llx\n",
		       info.dma_vaddr);
		return ret;
	}

	ret = copy_to_user((void __user *)msg, (void *)&buffer->info,
			   sizeof(buffer->info));
	if (ret) {
		pr_err("Failed to copy %zu bytes to user\n",
		       sizeof(buffer->info));
		return -EINVAL;
	}

	return 0;
}

/* TODO(edmondchung): Not sure whether this is generic enough to handle device
   enable yet, but this is sufficient to enable the sensor - so I will stick
   with this for now. */
static int ioctl_device_enable(struct lwis_device *lwis_dev)
{
	int ret = 0;

	mutex_lock(&lwis_dev->client_lock);
	if (lwis_dev->enabled > 0 && lwis_dev->enabled < INT_MAX) {
		lwis_dev->enabled++;
		mutex_unlock(&lwis_dev->client_lock);
		return 0;
	} else if (lwis_dev->enabled == INT_MAX) {
		pr_err("Enable counter overflow\n");
		ret = -EINVAL;
		goto error_locked;
	}

	lwis_dev->enabled = 1;

	/* Let's do the platform-specific enable call */
	ret = lwis_platform_device_enable(lwis_dev);
	if (ret) {
		pr_err("Platform-specific device enable fail: %d\n", ret);
		goto error_locked;
	}

	if (lwis_dev->clocks) {
		/* Enable clocks */
		ret = lwis_clock_enable_all(lwis_dev->clocks);
		if (ret) {
			pr_err("Error enabling clocks (%d)\n", ret);
			goto error_locked;
		}
	}

	if (lwis_dev->reset_gpios_present) {
		struct gpio_descs *gpios;
		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev, "reset");
		if (IS_ERR_OR_NULL(gpios)) {
			pr_err("Failed to obtain reset gpio list (%d)\n",
			       PTR_ERR(gpios));
			ret = PTR_ERR(gpios);
			goto error_locked;
		}

		/* Set reset pin to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 1);
		if (ret) {
			pr_err("Failed to set reset GPIOs to ACTIVE (%d)\n",
			       ret);
			goto error_locked;
		}

		/* Inherited from FIMC, will see if this is needed */
		usleep_range(1500, 1500);

		/* Set reset pin to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 0);
		if (ret) {
			pr_err("Failed to set reset GPIOs to INACTIVE (%d)\n",
			       ret);
			goto error_locked;
		}

		/* Setting reset_gpios to non-NULL to indicate that this lwis
		   device is holding onto the GPIO pins. */
		lwis_dev->reset_gpios = gpios;
	}

	if (lwis_dev->mclk_ctrl) {
		/* Set MCLK state to on */
		ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl,
					     MCLK_ON_STRING);
		if (ret) {
			pr_err("Error setting mclk state (%d)\n", ret);
			goto error_locked;
		}
	}

	if (lwis_dev->regulators) {
		/* Enable all the regulators related to this sensor */
		ret = lwis_regulator_enable_all(lwis_dev->regulators);
		if (ret) {
			pr_err("Error enabling regulators (%d)\n", ret);
			goto error_locked;
		}
	}

	if (lwis_dev->enable_gpios_present) {
		struct gpio_descs *gpios;
		gpios = lwis_gpio_list_get(&lwis_dev->plat_dev->dev, "enable");
		if (IS_ERR_OR_NULL(gpios)) {
			pr_err("Failed to obtain enable gpio list (%d)\n",
			       PTR_ERR(gpios));
			ret = PTR_ERR(gpios);
			goto error_locked;
		}

		/* Set enable pins to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(gpios, 1);
		if (ret) {
			pr_err("Error enabling GPIO pins (%d)\n", ret);
			goto error_locked;
		}

		/* Setting enable_gpios to non-NULL to indicate that this lwis
		   device is holding onto the GPIO pins. */
		lwis_dev->enable_gpios = gpios;
	}

	if (lwis_dev->phys) {
		/* Power on the PHY */
		ret = lwis_phy_set_power_all(lwis_dev->phys,
					     /* power_on = */ true);
		if (ret) {
			pr_err("Error powering on PHY\n");
			goto error_locked;
		}
	}

	if (lwis_dev->irqs) {
		ret = lwis_interrupt_request_all_default(lwis_dev->irqs);
		if (ret) {
			pr_err("Failed to request interrupts (%d)\n", ret);
			goto error_locked;
		}
	}

	if (lwis_dev->vops.device_enable) {
		ret = lwis_dev->vops.device_enable(lwis_dev);
		if (ret) {
			pr_err("Error executing device enable function\n");
			goto error_locked;
		}
	}

	/* Sleeping to make sure all pins are ready to go */
	usleep_range(2000, 2000);

	pr_info("Device enabled\n");
error_locked:
	mutex_unlock(&lwis_dev->client_lock);
	return ret;
}

/* TODO(edmondchung): Same comment as ioctl_device_enable. */
static int ioctl_device_disable(struct lwis_device *lwis_dev)
{
	int ret;

	mutex_lock(&lwis_dev->client_lock);
	if (lwis_dev->enabled > 1) {
		lwis_dev->enabled--;
		mutex_unlock(&lwis_dev->client_lock);
		return 0;
	} else if (lwis_dev->enabled <= 0) {
		pr_err("Disabling a device that is already disabled\n");
		ret = -EINVAL;
		goto error_locked;
	}

	lwis_dev->enabled = 0;

	if (lwis_dev->vops.device_disable) {
		ret = lwis_dev->vops.device_disable(lwis_dev);
		if (ret) {
			pr_err("Error executing device disable function\n");
			goto error_locked;
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
			pr_err("Error powering off PHY\n");
			goto error_locked;
		}
	}

	if (lwis_dev->enable_gpios_present && lwis_dev->enable_gpios) {
		/* Set enable pins to 0 (i.e. deasserted) */
		ret = lwis_gpio_list_set_output_value(lwis_dev->enable_gpios,
						      0);
		if (ret) {
			pr_err("Error disabling GPIO pins (%d)\n", ret);
			goto error_locked;
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
			pr_err("Error disabling regulators (%d)\n", ret);
			goto error_locked;
		}
	}

	if (lwis_dev->reset_gpios_present && lwis_dev->reset_gpios) {
		/* Set reset pins to 1 (i.e. asserted) */
		ret = lwis_gpio_list_set_output_value(lwis_dev->reset_gpios, 1);
		if (ret) {
			pr_err("Error setting reset GPIOs to ACTIVE (%d)\n",
			       ret);
			goto error_locked;
		}

		/* Release ownership of the GPIO pins */
		lwis_gpio_list_put(lwis_dev->reset_gpios,
				   &lwis_dev->plat_dev->dev);
		lwis_dev->reset_gpios = NULL;
	}

	if (lwis_dev->mclk_ctrl) {
		/* Set MCLK state to off */
		ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl,
					     MCLK_OFF_STRING);
		if (ret) {
			pr_err("Error setting mclk state (%d)\n", ret);
			goto error_locked;
		}
	}

	if (lwis_dev->clocks) {
		/* Disable all clocks */
		lwis_clock_disable_all(lwis_dev->clocks);
	}

	/* Let's do the platform-specific disable call */
	ret = lwis_platform_device_disable(lwis_dev);
	if (ret) {
		pr_err("Platform-specific device disable fail: %d\n", ret);
		goto error_locked;
	}

	pr_info("Device disabled\n");
error_locked:
	mutex_unlock(&lwis_dev->client_lock);
	return ret;
}

static int ioctl_event_control_get(struct lwis_client *lwis_client,
				   struct lwis_event_control __user *msg)
{
	unsigned long ret;
	struct lwis_event_control control;

	ret = copy_from_user((void *)&control, (void __user *)msg,
			     sizeof(control));
	if (ret) {
		pr_err("Failed to copy %zu bytes from user\n", sizeof(control));
		return -EINVAL;
	}

	ret = lwis_client_event_control_get(lwis_client, control.event_id,
					    &control);

	if (ret) {
		pr_err("Failed to get event: %lld (err:%ld)\n",
		       control.event_id, ret);
		return -EINVAL;
	}

	ret = copy_to_user((void __user *)msg, (void *)&control,
			   sizeof(control));
	if (ret) {
		pr_err("Failed to copy %zu bytes to user\n", sizeof(control));
		return -EINVAL;
	}

	return 0;
}

static int ioctl_event_control_set(struct lwis_client *lwis_client,
				   struct lwis_event_control __user *msg)
{
	unsigned long ret;
	struct lwis_event_control control;

	ret = copy_from_user((void *)&control, (void __user *)msg,
			     sizeof(control));
	if (ret) {
		pr_err("Failed to copy %zu bytes from user\n", sizeof(control));
		return -EINVAL;
	}

	ret = lwis_client_event_control_set(lwis_client, &control);
	return ret;
}

static int ioctl_event_dequeue(struct lwis_client *lwis_client,
			       struct lwis_event_info __user *msg)
{
	unsigned long ret, err = 0;
	struct lwis_client_event *event;
	struct lwis_event_info info_user;

	ret = copy_from_user((void *)&info_user, (void __user *)msg,
			     sizeof(info_user));
	if (ret) {
		pr_err("Failed to copy %zu bytes from user\n",
		       sizeof(info_user));
		return -EINVAL;
	}

	/* Peek at the front element */
	ret = lwis_client_event_peek_front(lwis_client, &event);
	if (ret) {
		if (ret != -ENOENT) {
			pr_err("Error dequeueing event: %ld\n", ret);
		}
		return ret;
	}

	/* We need to check if we have an adequate payload buffer */
	if (event->event_info.payload_size > info_user.payload_buffer_size) {
		/* Nope, we don't. Let's inform the user and bail */
		info_user.payload_size = event->event_info.payload_size;
		err = -ENOMEM;
	} else {
		/*
		 * Let's save the IOCTL inputs because they'll get overwritten
		 */
		size_t user_buffer_size = info_user.payload_buffer_size;
		void *user_buffer = info_user.payload_buffer;

		/* Copy over the rest of the info */
		memcpy(&info_user, &event->event_info, sizeof(info_user));

		/* Restore the IOCTL inputs */
		info_user.payload_buffer_size = user_buffer_size;
		info_user.payload_buffer = user_buffer;

		/* Here we have a payload and the buffer is big enough */
		if (event->event_info.payload_size > 0 &&
		    info_user.payload_buffer) {
			/* Copy over the payload buffer to userspace */
			ret = copy_to_user(
				(void __user *)info_user.payload_buffer,
				(void *)event->event_info.payload_buffer,
				event->event_info.payload_size);
			if (ret) {
				pr_err("Failed to copy %zu bytes to user\n",
				       event->event_info.payload_size);
				return -EINVAL;
			}
		}
	}
	/* If we didn't -ENOMEM up above, we can pop and discard the front of
	 * the event queue because we're done dealing with it. If we got the
	 * -ENOMEM case, we didn't actually dequeu this event and userspace
	 *  should try again with a bigger payload_buffer
	 */
	if (!err) {
		ret = lwis_client_event_pop_front(lwis_client, NULL);
		if (ret) {
			pr_err("Error dequeueing event: %ld\n", ret);
			return ret;
		}
	}
	/* Now let's copy the actual info struct back to user */
	ret = copy_to_user((void __user *)msg, (void *)&info_user,
			   sizeof(info_user));
	if (ret) {
		pr_err("Failed to copy %zu bytes to user\n", sizeof(info_user));
		return -EINVAL;
	}
	return err;
}

int lwis_ioctl_handler(struct lwis_client *lwis_client, unsigned int type,
		       unsigned long param)
{
	int ret = 0;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	switch (type) {
	case LWIS_GET_DEVICE_INFO:
		break;
	case LWIS_BUFFER_ENROLL:
		ret = ioctl_buffer_enroll(lwis_client,
					  (struct lwis_buffer_info *)param);
		break;
	case LWIS_BUFFER_DISENROLL:
		ret = ioctl_buffer_disenroll(lwis_client,
					     (struct lwis_buffer_info *)param);
		break;
	case LWIS_REG_READ:
		ret = ioctl_reg_read(lwis_dev, (struct lwis_io_msg *)param);
		break;
	case LWIS_REG_WRITE:
		ret = ioctl_reg_write(lwis_dev, (struct lwis_io_msg *)param);
		break;
	case LWIS_DEVICE_ENABLE:
		ret = ioctl_device_enable(lwis_dev);
		break;
	case LWIS_DEVICE_DISABLE:
		ret = ioctl_device_disable(lwis_dev);
		break;
	case LWIS_EVENT_CONTROL_GET:
		ret = ioctl_event_control_get(
			lwis_client, (struct lwis_event_control *)param);
		break;
	case LWIS_EVENT_CONTROL_SET:
		ret = ioctl_event_control_set(
			lwis_client, (struct lwis_event_control *)param);
		break;
	case LWIS_EVENT_DEQUEUE:
		ret = ioctl_event_dequeue(lwis_client,
					  (struct lwis_event_info *)param);
		break;
	default:
		pr_err("Unknown IOCTL operation\n");
		ret = -EINVAL;
	};

	return ret;
}
