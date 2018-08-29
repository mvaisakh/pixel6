/*
 * Google LWIS IOCTL Handler
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "lwis_ioctl.h"

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_clock.h"
#include "lwis_commands.h"
#include "lwis_gpio.h"
#include "lwis_i2c.h"
#include "lwis_pinctrl.h"
#include "lwis_regulator.h"

#define I2C_ON_STRING "on_i2c"
#define I2C_OFF_STRING "off_i2c"
#define MCLK_ON_STRING "mclk_on"
#define MCLK_OFF_STRING "mclk_off"

static int ioctl_reg_read(struct lwis_device *lwis_dev,
			  struct lwis_io_msg *user_msg)
{
	int ret;
	struct lwis_io_msg k_msg;
	struct lwis_io_data *user_buf;
	struct lwis_io_data *k_buf;

	/* Copy message struct from userspace */
	ret = copy_from_user(&k_msg, (void __user *) user_msg,
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
	ret = copy_from_user(k_buf, (void __user *) user_buf,
			     k_msg.num_entries * sizeof(struct lwis_io_data));
	if (ret) {
		goto error_i2c_read;
	}

	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		ret = lwis_i2c_read_batch(lwis_dev->i2c, k_msg.buf,
					  k_msg.num_entries,
					  k_msg.offset_bitwidth);
	}

	if (ret) {
		goto error_i2c_read;
	}

	/* Copy read data back to userspace */
	ret = copy_to_user((void __user *) user_buf, k_buf,
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

	/* Copy message struct from userspace */
	ret = copy_from_user(&k_msg, (void __user *) user_msg,
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
	ret = copy_from_user(k_buf, (void __user *) user_buf,
			     k_msg.num_entries * sizeof(struct lwis_io_data));
	if (ret) {
		goto error_i2c_write;
	}

	if (lwis_dev->type == DEVICE_TYPE_I2C) {
		ret = lwis_i2c_write_batch(lwis_dev->i2c, k_msg.buf,
					   k_msg.num_entries,
					   k_msg.offset_bitwidth);
	}

error_i2c_write:
	kfree(k_buf);
	return ret;
}

/* TODO(edmondchung): Not sure whether this is generic enough to handle device
   enable yet, but this is sufficient to enable the sensor - so I will stick
   with this for now. */
static int ioctl_device_enable(struct lwis_device *lwis_dev)
{
	int ret;

	if (lwis_dev->clocks) {
		/* Enable clocks */
		ret = lwis_clock_enable_all(lwis_dev->clocks);
		if (ret) {
			pr_err("Error enabling clocks (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->reset_gpios) {
		/* Set reset pin to active (since it's active low, this will set
		   the GPIO pin low) */
		ret = lwis_gpio_pin_set_level_all(lwis_dev->reset_gpios,
						  LWIS_GPIO_PIN_ACTIVE);
		if (ret) {
			pr_err("Failed to set reset GPIOs to ACTIVE (%d)\n",
			       ret);
			return ret;
		}
	}

	if (lwis_dev->i2c) {
		/* Enable the I2C bus */
		ret = lwis_i2c_set_state(lwis_dev->i2c, I2C_ON_STRING);
		if (ret) {
			pr_err("Error enabling i2c bus (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->mclk_ctrl) {
		/* Set MCLK state to on */
		ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl,
					     MCLK_ON_STRING);
		if (ret) {
			pr_err("Error setting mclk state (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->reset_gpios) {
		/* Inherited from FIMC, will see if this is needed */
		usleep_range(1500, 1500);

		/* Set reset pin to inactive */
		ret = lwis_gpio_pin_set_level_all(lwis_dev->reset_gpios,
						  LWIS_GPIO_PIN_INACTIVE);
		if (ret) {
			pr_err("Failed to set reset GPIOs to INACTIVE (%d)\n",
			       ret);
			return ret;
		}

		/* Inherited from FIMC, will see if this is needed */
		usleep_range(2000, 2000);
	}

	if (lwis_dev->regulators) {
		/* Enable all the regulators related to this sensor */
		ret = lwis_regulator_enable_all(lwis_dev->regulators);
		if (ret) {
			pr_err("Error enabling regulators (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->enable_gpios) {
		/* Set enable pins to active */
		ret = lwis_gpio_pin_set_level_all(lwis_dev->enable_gpios,
						  LWIS_GPIO_PIN_ACTIVE);
		if (ret) {
			pr_err("Error enabling GPIO pins (%d)\n", ret);
			return ret;
		}
	}

	pr_info("Device enabled\n");
	return 0;
}

/* TODO(edmondchung): Same comment as ioctl_device_enable. */
static int ioctl_device_disable(struct lwis_device *lwis_dev)
{
	int ret;

	if (lwis_dev->enable_gpios) {
		/* Set enable pins to inactive */
		ret = lwis_gpio_pin_set_level_all(lwis_dev->enable_gpios,
						  LWIS_GPIO_PIN_INACTIVE);
		if (ret) {
			pr_err("Error disabling GPIO pins (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->regulators) {
		/* Disable all the regulators */
		ret = lwis_regulator_disable_all(lwis_dev->regulators);
		if (ret) {
			pr_err("Error disabling regulators (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->reset_gpios) {
		/* Set reset pins to active */
		ret = lwis_gpio_pin_set_level_all(lwis_dev->reset_gpios,
						  LWIS_GPIO_PIN_ACTIVE);
		if (ret) {
			pr_err("Error setting reset GPIOs to ACTIVE (%d)\n",
			       ret);
			return ret;
		}
	}

	if (lwis_dev->i2c) {
		/* Disable the I2C bus */
		ret = lwis_i2c_set_state(lwis_dev->i2c, I2C_OFF_STRING);
		if (ret) {
			pr_err("Error disabling i2c bus (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->mclk_ctrl) {
		/* Set MCLK state to off */
		ret = lwis_pinctrl_set_state(lwis_dev->mclk_ctrl,
					     MCLK_OFF_STRING);
		if (ret) {
			pr_err("Error setting mclk state (%d)\n", ret);
			return ret;
		}
	}

	if (lwis_dev->clocks) {
		/* Disable all clocks */
		lwis_clock_disable_all(lwis_dev->clocks);
	}

	pr_info("Device disabled\n");

	return 0;
}

int lwis_ioctl_handler(struct lwis_device *lwis_dev, unsigned int type,
		       unsigned long param)
{
	int ret = 0;

	if (lwis_dev->type == DEVICE_TYPE_TOP) {
		if (type == LWIS_GET_DEVICE_INFO) {
			// TOP can only do GET_DEVICE_INFO
			// Saving for future implementation
		} else {
			pr_err("Invalid IOCTL operation on TOP device\n");
			return -EINVAL;
		}
	}

	switch (type) {
	case LWIS_GET_DEVICE_INFO:
		break;
	case LWIS_ENROLL_BUFFER:
		break;
	case LWIS_REG_READ:
		ret = ioctl_reg_read(lwis_dev, (struct lwis_io_msg *) param);
		break;
	case LWIS_REG_WRITE:
		ret = ioctl_reg_write(lwis_dev, (struct lwis_io_msg *) param);
		break;
	case LWIS_DEVICE_ENABLE:
		ret = ioctl_device_enable(lwis_dev);
		break;
	case LWIS_DEVICE_DISABLE:
		ret = ioctl_device_disable(lwis_dev);
		break;
	default:
		pr_err("Unknown IOCTL operation\n");
		ret = -EINVAL;
	};

	return ret;
}