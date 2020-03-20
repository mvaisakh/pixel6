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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "lwis_buffer.h"
#include "lwis_commands.h"
#include "lwis_device_i2c.h"
#include "lwis_device_ioreg.h"
#include "lwis_event.h"
#include "lwis_i2c.h"
#include "lwis_ioreg.h"
#include "lwis_platform.h"
#include "lwis_regulator.h"
#include "lwis_transaction.h"

#define IOCTL_TO_ENUM(x) _IOC_NR(x)
#define IOCTL_ARG_SIZE(x) _IOC_SIZE(x)
#define STRINGIFY(x) #x

void lwis_ioctl_pr_err(unsigned int ioctl_type, int errno)
{
	unsigned int type = IOCTL_TO_ENUM(ioctl_type);
	static char type_name[32];
	size_t exp_size;

	switch (type) {
	case IOCTL_TO_ENUM(LWIS_GET_DEVICE_INFO):
		strcpy(type_name, STRINGIFY(LWIS_GET_DEVICE_INFO));
		exp_size = IOCTL_ARG_SIZE(LWIS_GET_DEVICE_INFO);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_ALLOC):
		strcpy(type_name, STRINGIFY(LWIS_BUFFER_ALLOC));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_ALLOC);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_FREE):
		strcpy(type_name, STRINGIFY(LWIS_BUFFER_FREE));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_FREE);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_ENROLL):
		strcpy(type_name, STRINGIFY(LWIS_BUFFER_ENROLL));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_ENROLL);
		break;
	case IOCTL_TO_ENUM(LWIS_BUFFER_DISENROLL):
		strcpy(type_name, STRINGIFY(LWIS_BUFFER_DISENROLL));
		exp_size = IOCTL_ARG_SIZE(LWIS_BUFFER_DISENROLL);
		break;
	case IOCTL_TO_ENUM(LWIS_REG_READ):
		strcpy(type_name, STRINGIFY(LWIS_REG_READ));
		exp_size = IOCTL_ARG_SIZE(LWIS_REG_READ);
		break;
	case IOCTL_TO_ENUM(LWIS_REG_WRITE):
		strcpy(type_name, STRINGIFY(LWIS_REG_WRITE));
		exp_size = IOCTL_ARG_SIZE(LWIS_REG_WRITE);
		break;
	case IOCTL_TO_ENUM(LWIS_REG_IO):
		strcpy(type_name, STRINGIFY(LWIS_REG_IO));
		exp_size = IOCTL_ARG_SIZE(LWIS_REG_IO);
		break;
	case IOCTL_TO_ENUM(LWIS_DEVICE_ENABLE):
		strcpy(type_name, STRINGIFY(LWIS_DEVICE_ENABLE));
		exp_size = IOCTL_ARG_SIZE(LWIS_DEVICE_ENABLE);
		break;

	case IOCTL_TO_ENUM(LWIS_DEVICE_DISABLE):
		strcpy(type_name, STRINGIFY(LWIS_DEVICE_DISABLE));
		exp_size = IOCTL_ARG_SIZE(LWIS_DEVICE_DISABLE);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_CONTROL_GET):
		strcpy(type_name, STRINGIFY(LWIS_EVENT_CONTROL_GET));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_CONTROL_GET);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_CONTROL_SET):
		strcpy(type_name, STRINGIFY(LWIS_EVENT_CONTROL_SET));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_CONTROL_SET);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_DEQUEUE):
		strcpy(type_name, STRINGIFY(LWIS_EVENT_DEQUEUE));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_DEQUEUE);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_SUBSCRIBE):
		strcpy(type_name, STRINGIFY(LWIS_EVENT_SUBSCRIBE));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_SUBSCRIBE);
		break;
	case IOCTL_TO_ENUM(LWIS_EVENT_UNSUBSCRIBE):
		strcpy(type_name, STRINGIFY(LWIS_EVENT_UNSUBSCRIBE));
		exp_size = IOCTL_ARG_SIZE(LWIS_EVENT_UNSUBSCRIBE);
		break;
	case IOCTL_TO_ENUM(LWIS_TIME_QUERY):
		strcpy(type_name, STRINGIFY(LWIS_TIME_QUERY));
		exp_size = IOCTL_ARG_SIZE(LWIS_TIME_QUERY);
		break;
	case IOCTL_TO_ENUM(LWIS_TRANSACTION_SUBMIT):
		strcpy(type_name, STRINGIFY(LWIS_TRANSACTION_SUBMIT));
		exp_size = IOCTL_ARG_SIZE(LWIS_TRANSACTION_SUBMIT);
		break;
	case IOCTL_TO_ENUM(LWIS_TRANSACTION_CANCEL):
		strcpy(type_name, STRINGIFY(LWIS_TRANSACTION_CANCEL));
		exp_size = IOCTL_ARG_SIZE(LWIS_TRANSACTION_CANCEL);
		break;
	default:
		strcpy(type_name, "UNDEFINED");
		exp_size = 0;
		break;
	};

	if (strcmp(type_name, "UNDEFINED") &&
	    exp_size != IOCTL_ARG_SIZE(ioctl_type)) {
		pr_err("Failed to process %s (errno: %d), expecting argument with length of %d, got length of %d. Mismatch kernel version?\n",
		       type_name, errno, exp_size, IOCTL_ARG_SIZE(ioctl_type));
	} else {
		pr_err("Failed to process %s (errno: %d)\n", type_name, errno);
	}
}

static int ioctl_get_device_info(struct lwis_device *lwis_dev,
				 struct lwis_device_info *msg)
{
	int ret;
	struct lwis_device_info k_info = { .id = lwis_dev->id,
					   .type = lwis_dev->type };

	strncpy(k_info.name, lwis_dev->name, LWIS_MAX_DEVICE_NAME_STRING);

	ret = copy_to_user((void __user *)msg, &k_info, sizeof(k_info));
	if (ret) {
		pr_err("Failed to copy device info to userspace\n");
		return ret;
	}

	return 0;
}

static int lwis_reg_read(struct lwis_device *lwis_dev,
			 struct lwis_io_entry *read_entry,
			 struct lwis_io_entry *user_msg)
{
	int ret = 0;
	uint8_t *user_buf;
	bool batch_mode = false;

	if (read_entry->type == LWIS_IO_ENTRY_READ_BATCH) {
		batch_mode = true;
		/* Save the userspace buffer address */
		user_buf = read_entry->rw_batch.buf;
		/* Allocate read buffer */
		read_entry->rw_batch.buf =
			kzalloc(read_entry->rw_batch.size_in_bytes, GFP_KERNEL);
		if (!read_entry->rw_batch.buf) {
			pr_err("Failed to allocate register read buffer\n");
			return -ENOMEM;
		}
	} else if (read_entry->type != LWIS_IO_ENTRY_READ) {
		/* Type must be either READ or READ_BATCH */
		pr_err("Invalid io_entry type for REGISTER_READ\n");
		return -EINVAL;
	}

	ret = lwis_dev->vops.register_io(lwis_dev, read_entry, false,
					 lwis_dev->native_value_bitwidth);
	if (ret) {
		pr_err("Failed to read registers\n");
		goto reg_read_exit;
	}

	if (batch_mode) {
		/* Copy read data back to userspace */
		ret = copy_to_user((void __user *)user_buf,
				   read_entry->rw_batch.buf,
				   read_entry->rw_batch.size_in_bytes);
		if (ret) {
			pr_err("Failed to copy register read buffer back to "
			       "userspace\n");
		}
	} else {
		ret = copy_to_user((void __user *)user_msg, read_entry,
				   sizeof(*read_entry));
	}

reg_read_exit:
	if (batch_mode) {
		kfree(read_entry->rw_batch.buf);
	}
	return ret;
}
static int ioctl_reg_read(struct lwis_device *lwis_dev,
			  struct lwis_io_entry *user_msg)
{
	int ret = 0;
	struct lwis_io_entry k_msg;
	/* register read is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_io) {
		pr_err("Register IO not supported on this LWIS device\n");
		return -EINVAL;
	}
	/* Copy message struct from userspace */
	ret = copy_from_user(&k_msg, (void __user *)user_msg, sizeof(k_msg));
	if (ret) {
		pr_err("Failed to copy register read ioctl from userspace\n");
		return ret;
	}

	ret = lwis_reg_read(lwis_dev, &k_msg, user_msg);
	return ret;
}

static int lwis_reg_write(struct lwis_device *lwis_dev,
			  struct lwis_io_entry *write_entry)
{
	int ret = 0;
	uint8_t *user_buf;
	bool batch_mode = false;

	if (write_entry->type == LWIS_IO_ENTRY_WRITE_BATCH) {
		batch_mode = true;
		/* Save the userspace buffer address */
		user_buf = write_entry->rw_batch.buf;
		/* Allocate write buffer and copy contents from userspace */
		write_entry->rw_batch.buf = kzalloc(
			write_entry->rw_batch.size_in_bytes, GFP_KERNEL);
		if (!write_entry->rw_batch.buf) {
			pr_err("Failed to allocate register write buffer\n");
			return -ENOMEM;
		}
		ret = copy_from_user(write_entry->rw_batch.buf,
				     (void __user *)user_buf,
				     write_entry->rw_batch.size_in_bytes);
		if (ret) {
			pr_err("Failed to copy write buffer from userspace\n");
			goto reg_write_exit;
		}

	} else if (write_entry->type != LWIS_IO_ENTRY_WRITE) {
		/* Type must be either WRITE or WRITE_BATCH */
		pr_err("Invalid io_entry type for REGISTER_WRITE\n");
		return -EINVAL;
	}

	ret = lwis_dev->vops.register_io(lwis_dev, write_entry, false,
					 lwis_dev->native_value_bitwidth);
	if (ret) {
		pr_err("Failed to write registers\n");
	}

reg_write_exit:
	if (batch_mode) {
		kfree(write_entry->rw_batch.buf);
	}
	return ret;
}

static int ioctl_reg_write(struct lwis_device *lwis_dev,
			   struct lwis_io_entry *user_msg)
{
	int ret = 0;
	struct lwis_io_entry k_msg;
	/* register write is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_io) {
		pr_err("Register IO not supported on this LWIS device\n");
		return -EINVAL;
	}
	/* Copy message struct from userspace */
	ret = copy_from_user(&k_msg, (void __user *)user_msg, sizeof(k_msg));
	if (ret) {
		return ret;
	}

	ret = lwis_reg_write(lwis_dev, &k_msg);
	return ret;
}

static int lwis_reg_modify(struct lwis_device *lwis_dev,
			   struct lwis_io_entry *modify_entry)
{
	int ret = 0;

	ret = lwis_dev->vops.register_io(lwis_dev, modify_entry, false,
					 lwis_dev->native_value_bitwidth);
	if (ret) {
		pr_err("Failed to read registers for modify\n");
	}

	return ret;
}

static int ioctl_reg_io(struct lwis_device *lwis_dev,
			struct lwis_io_entries *user_msg)
{
	int ret = 0, i = 0;
	struct lwis_io_entries k_msg;
	struct lwis_io_entry *k_entries;
	uint32_t buf_size;

	/* Register io is not supported for the lwis device, return */
	if (!lwis_dev->vops.register_io) {
		pr_err("Register IO not supported on this LWIS device\n");
		return -EINVAL;
	}

	/* Copy message from userspace */
	ret = copy_from_user(&k_msg, (void __user *)user_msg, sizeof(k_msg));
	if (ret) {
		pr_err("Failed to copy io_entries header from userspace.");
		return ret;
	}
	buf_size = sizeof(struct lwis_io_entry) * k_msg.num_io_entries;
	k_entries = kzalloc(buf_size, GFP_KERNEL);
	if (!k_entries) {
		pr_err("Failed to allocate io_entries buffer\n");
		return -ENOMEM;
	}
	ret = copy_from_user(k_entries, (void __user *)k_msg.io_entries,
			     buf_size);
	if (ret) {
		pr_err("Failed to copy io_entries from userspace.");
		return ret;
	}

	/* Walk through and execute the entries */
	for (i = 0; i < k_msg.num_io_entries; i++) {
		switch (k_entries[i].type) {
		case LWIS_IO_ENTRY_MODIFY:
			ret = lwis_reg_modify(lwis_dev, &k_entries[i]);
			break;
		case LWIS_IO_ENTRY_READ:
		case LWIS_IO_ENTRY_READ_BATCH:
			ret = lwis_reg_read(lwis_dev, &k_entries[i],
					    k_msg.io_entries + i);
			break;
		case LWIS_IO_ENTRY_WRITE:
		case LWIS_IO_ENTRY_WRITE_BATCH:
			ret = lwis_reg_write(lwis_dev, &k_entries[i]);
			break;
		default:
			pr_err("Unknown io_entry operation\n");
			ret = -EINVAL;
		};

		if (ret) {
			pr_err("Register io_entry failed\n");
			goto reg_io_exit;
		}
	}

reg_io_exit:
	kfree(k_entries);
	return ret;
}
static int ioctl_buffer_alloc(struct lwis_client *lwis_client,
			      struct lwis_alloc_buffer_info __user *msg)
{
	unsigned long ret;
	struct lwis_alloc_buffer_info alloc_info;
	struct lwis_allocated_buffer *buffer;

	buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);
	if (!buffer) {
		pr_err("Failed to allocated lwis_allocated_buffer\n");
		return -ENOMEM;
	}

	ret = copy_from_user((void *)&alloc_info, (void __user *)msg,
			     sizeof(alloc_info));
	if (ret) {
		pr_err("Failed to copy %zu bytes from user\n",
		       sizeof(alloc_info));
		goto error_alloc;
	}

	ret = lwis_buffer_alloc(lwis_client, &alloc_info, buffer);
	if (ret) {
		pr_err("Failed to allocate and enroll buffer\n");
		goto error_alloc;
	}

	ret = copy_to_user((void __user *)msg, (void *)&alloc_info,
			   sizeof(alloc_info));
	if (ret) {
		pr_err("Failed to copy %zu bytes to user\n",
		       sizeof(alloc_info));
		lwis_buffer_free(lwis_client, buffer);
		goto error_alloc;
	}

	return 0;

error_alloc:
	kfree(buffer);
	return ret;
}

static int ioctl_buffer_free(struct lwis_client *lwis_client, int __user *msg)
{
	int ret;
	int fd;
	struct lwis_allocated_buffer *buffer;

	ret = copy_from_user((void *)&fd, (void __user *)msg, sizeof(fd));
	if (ret) {
		pr_err("Failed to copy file descriptor from user\n");
		return -EINVAL;
	}

	buffer = lwis_client_allocated_buffer_find(lwis_client, fd);
	if (!buffer) {
		pr_err("Cannot find allocated buffer FD %d\n", fd);
		return -ENOENT;
	}

	ret = lwis_buffer_free(lwis_client, buffer);
	if (ret) {
		pr_err("Failed to free buffer FD %d\n", fd);
		return ret;
	}

	kfree(buffer);

	return 0;
}

static int ioctl_buffer_enroll(struct lwis_client *lwis_client,
			       struct lwis_buffer_info __user *msg)
{
	unsigned long ret;
	struct lwis_enrolled_buffer *buffer;

	buffer = kzalloc(sizeof(struct lwis_enrolled_buffer), GFP_KERNEL);
	if (!buffer) {
		pr_err("Failed to allocate lwis_enrolled_buffer struct\n");
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
				  uint64_t __user *msg)
{
	unsigned long ret;
	uint64_t dma_vaddr;
	struct lwis_enrolled_buffer *buffer;

	ret = copy_from_user((void *)&dma_vaddr, (void __user *)msg,
			     sizeof(dma_vaddr));
	if (ret) {
		pr_err("Failed to copy DMA virtual address from user\n");
		return -EINVAL;
	}

	buffer = lwis_client_enrolled_buffer_find(lwis_client, dma_vaddr);

	if (!buffer) {
		pr_err("Failed to find dma buffer for %llx\n", dma_vaddr);
		return -ENOENT;
	}

	ret = lwis_buffer_disenroll(lwis_client, buffer);
	if (ret) {
		pr_err("Failed to disenroll dma buffer for %llx\n", dma_vaddr);
		return ret;
	}

	kfree(buffer);

	return 0;
}

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

	ret = lwis_dev_power_up_locked(lwis_dev);
	if (ret < 0) {
		pr_err("Failed to power up device %s\n", lwis_dev->name);
		goto error_locked;
	}

	lwis_dev->enabled++;
	pr_info("Device %s enabled\n", lwis_dev->name);
error_locked:
	mutex_unlock(&lwis_dev->client_lock);
	return ret;
}

static int ioctl_device_disable(struct lwis_client *lwis_client)
{
	int ret;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	/* Wait for all in-process transactions to complete. */
	ret = lwis_transaction_client_flush(lwis_client);
	if (ret) {
		pr_err("Failed to wait for in-process transaction to complete\n");
	}

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

	ret = lwis_dev_power_down_locked(lwis_dev);
	if (ret < 0) {
		pr_err("Failed to power down device %s\n", lwis_dev->name);
		goto error_locked;
	}

	lwis_dev->enabled--;
	pr_info("Device %s disabled\n", lwis_dev->name);
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
	struct lwis_event_entry *event;
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
		err = -EAGAIN;
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
	/* If we didn't -EAGAIN up above, we can pop and discard the front of
	 * the event queue because we're done dealing with it. If we got the
	 * -EAGAIN case, we didn't actually dequeue this event and userspace
	 * should try again with a bigger payload_buffer.
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

static int ioctl_time_query(struct lwis_client *client, int64_t __user *msg)
{
	int ret = 0;
	int64_t timestamp = ktime_to_ns(ktime_get());

	ret = copy_to_user((void __user *)msg, &timestamp, sizeof(timestamp));
	if (ret) {
		pr_err("Failed to copy timestamp to userspace\n");
	}

	return ret;
}

static int ioctl_transaction_submit(struct lwis_client *client,
				    struct lwis_transaction_info __user *msg)
{
	int i;
	int ret;
	int last_buf_alloc_idx = 0;
	size_t entry_size;
	struct lwis_transaction *k_transaction;
	struct lwis_transaction_info *user_transaction;
	struct lwis_io_entry *k_entries;
	struct lwis_io_entry *user_entries;
	uint8_t *user_buf;
	uint8_t *k_buf;

	k_transaction = kzalloc(sizeof(struct lwis_transaction), GFP_KERNEL);
	if (!k_transaction) {
		pr_err("Failed to allocate transaction info\n");
		return -ENOMEM;
	}

	user_transaction = (struct lwis_transaction_info *)msg;
	ret = copy_from_user((void *)&k_transaction->info,
			     (void __user *)user_transaction,
			     sizeof(struct lwis_transaction_info));
	if (ret) {
		pr_err("Failed to copy transaction info from user\n");
		goto error_free_transaction;
	}

	user_entries = k_transaction->info.io_entries;
	entry_size = k_transaction->info.num_io_entries *
		     sizeof(struct lwis_io_entry);
	k_entries = kzalloc(entry_size, GFP_KERNEL);
	if (!k_entries) {
		pr_err("Failed to allocate transaction entries\n");
		ret = -ENOMEM;
		goto error_free_transaction;
	}
	k_transaction->info.io_entries = k_entries;

	ret = copy_from_user((void *)k_entries, (void __user *)user_entries,
			     entry_size);
	if (ret) {
		pr_err("Failed to copy transaction entries from user\n");
		goto error_free_entries;
	}

	/* For batch writes, need to allocate kernel buffers to deep copy the
	 * write values. Don't need to do this for batch reads because memory
	 * will be allocated in the form of lwis_io_result in transaction
	 * processing.
	 */
	for (i = 0; i < k_transaction->info.num_io_entries; ++i) {
		if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH) {
			user_buf = k_entries[i].rw_batch.buf;
			k_buf = kzalloc(k_entries[i].rw_batch.size_in_bytes,
					GFP_KERNEL);
			if (!k_buf) {
				pr_err("Failed to allocate tx write buffer\n");
				ret = -ENOMEM;
				goto error_free_buf;
			}
			last_buf_alloc_idx = i;
			k_entries[i].rw_batch.buf = k_buf;
			ret = copy_from_user(
				k_buf, (void __user *)user_buf,
				k_entries[i].rw_batch.size_in_bytes);
			if (ret) {
				pr_err("Failed to copy tx write buffer from userspace\n");
				goto error_free_buf;
			}
		}
	}

	ret = lwis_transaction_submit(client, k_transaction);
	if (ret) {
		pr_err("Failed to submit transaction\n");
		goto error_free_buf;
	}

	ret = copy_to_user((void __user *)user_transaction,
			   &k_transaction->info,
			   sizeof(struct lwis_transaction_info));
	if (ret) {
		pr_err("Failed to copy transaction results to userspace\n");
		return ret;
	}

	return 0;

error_free_buf:
	for (i = 0; i < last_buf_alloc_idx; ++i) {
		if (k_entries[i].type == LWIS_IO_ENTRY_WRITE_BATCH) {
			kfree(k_entries[i].rw_batch.buf);
		}
	}
error_free_entries:
	kfree(k_entries);
error_free_transaction:
	kfree(k_transaction);
	return ret;
}

static int ioctl_transaction_cancel(struct lwis_client *client,
				    int64_t __user *msg)
{
	int ret;
	int64_t id;

	ret = copy_from_user((void *)&id, (void __user *)msg, sizeof(id));
	if (ret) {
		pr_err("Failed to copy transaction ID from user\n");
		return ret;
	}

	ret = lwis_transaction_cancel(client, id);
	if (ret) {
		pr_err("Failed to clear transaction id 0x%llx\n", id);
		return ret;
	}

	return 0;
}

static int ioctl_event_subscribe(struct lwis_client *client,
				 struct lwis_event_subscribe __user *msg)
{
	int ret = 0;
	struct lwis_event_subscribe k_subscribe;
	struct lwis_device *lwis_dev = client->lwis_dev;
	struct lwis_device *trigger_device;
	uint16_t trigger_device_id;

	ret = copy_from_user((void *)&k_subscribe, (void __user *)msg,
			     sizeof(k_subscribe));
	if (ret) {
		pr_err("Failed to copy event subscribe from user\n");
		return ret;
	}

	/* Check if top device probe failed */
	if (lwis_dev->top_dev == NULL) {
		pr_err("Top device is null");
		return -EINVAL;
	}

	if ((k_subscribe.trigger_event_id & LWIS_TRANSACTION_EVENT_FLAG) ||
	    (k_subscribe.trigger_event_id &
	     LWIS_TRANSACTION_FAILURE_EVENT_FLAG)) {
		pr_err("Not support SW event subscription");
		return -EINVAL;
	}

	trigger_device_id = k_subscribe.trigger_device_id;
	trigger_device = lwis_find_dev_by_id(trigger_device_id);

	if (!trigger_device) {
		pr_err("Device id : %d doesn't match to any device",
		       trigger_device_id);
		return -EINVAL;
	}

	/* Create event state to trigger/receiver device
	 * Because of driver initialize in user space is sequential, it's
	 * possible that receiver device subscribe an event before trigger
	 * device set it up
	 */
	if (IS_ERR_OR_NULL(lwis_device_event_state_find_or_create(
		    lwis_dev, k_subscribe.trigger_event_id)) ||
	    IS_ERR_OR_NULL(lwis_client_event_state_find_or_create(
		    client, k_subscribe.trigger_event_id)) ||
	    IS_ERR_OR_NULL(lwis_device_event_state_find_or_create(
		    trigger_device, k_subscribe.trigger_event_id))) {
		pr_err("Failed to add event id 0x%llx to trigger/receiver"
		       "device",
		       k_subscribe.trigger_event_id);

		return -EINVAL;
	}
	ret = lwis_dev->top_dev->subscribe_ops.subscribe_event(
		lwis_dev->top_dev, k_subscribe.trigger_event_id,
		trigger_device->id, lwis_dev->id);
	if (ret < 0)
		pr_err("Failed to subscribe event: 0x%x for %s",
		       k_subscribe.trigger_event_id, lwis_dev->name);

	return ret;
}

static int ioctl_event_unsubscribe(struct lwis_client *client,
				   int64_t __user *msg)
{
	int ret = 0;
	int64_t event_id;
	struct lwis_device *lwis_dev = client->lwis_dev;

	ret = copy_from_user((void *)&event_id, (void __user *)msg,
			     sizeof(event_id));
	if (ret) {
		pr_err("Failed to copy event unsubscribe from user\n");
		return ret;
	}

	/* Check if top device probe failed */
	if (lwis_dev->top_dev == NULL) {
		pr_err("Top device is null");
		return -EINVAL;
	}

	ret = lwis_dev->top_dev->subscribe_ops.unsubscribe_event(
		lwis_dev->top_dev, event_id, lwis_dev->id);
	if (ret < 0)
		pr_err("Failed to unsubscribe event: 0x%x for %s", event_id,
		       lwis_dev->name);

	return ret;
}

int lwis_ioctl_handler(struct lwis_client *lwis_client, unsigned int type,
		       unsigned long param)
{
	int ret = 0;
	bool device_disabled;
	struct lwis_device *lwis_dev = lwis_client->lwis_dev;

	mutex_lock(&lwis_dev->client_lock);
	device_disabled = (lwis_dev->enabled == 0);
	mutex_unlock(&lwis_dev->client_lock);
	if (lwis_dev->type != DEVICE_TYPE_TOP && device_disabled &&
	    type != LWIS_GET_DEVICE_INFO && type != LWIS_DEVICE_ENABLE &&
	    type != LWIS_EVENT_CONTROL_GET && type != LWIS_TIME_QUERY &&
	    type != LWIS_EVENT_DEQUEUE) {
		ret = -EBADFD;
		pr_err("Unsupported IOCTL on disabled device.\n");
		return ret;
	}

	switch (type) {
	case LWIS_GET_DEVICE_INFO:
		ret = ioctl_get_device_info(lwis_dev,
					    (struct lwis_device_info *)param);
		break;
	case LWIS_BUFFER_ALLOC:
		ret = ioctl_buffer_alloc(
			lwis_client, (struct lwis_alloc_buffer_info *)param);
		break;
	case LWIS_BUFFER_FREE:
		ret = ioctl_buffer_free(lwis_client, (int *)param);
		break;
	case LWIS_BUFFER_ENROLL:
		ret = ioctl_buffer_enroll(lwis_client,
					  (struct lwis_buffer_info *)param);
		break;
	case LWIS_BUFFER_DISENROLL:
		ret = ioctl_buffer_disenroll(lwis_client, (uint64_t *)param);
		break;
	case LWIS_REG_READ:
		ret = ioctl_reg_read(lwis_dev, (struct lwis_io_entry *)param);
		break;
	case LWIS_REG_WRITE:
		ret = ioctl_reg_write(lwis_dev, (struct lwis_io_entry *)param);
		break;
	case LWIS_REG_IO:
		ret = ioctl_reg_io(lwis_dev, (struct lwis_io_entries *)param);
		break;
	case LWIS_DEVICE_ENABLE:
		ret = ioctl_device_enable(lwis_dev);
		break;
	case LWIS_DEVICE_DISABLE:
		ret = ioctl_device_disable(lwis_client);
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
	case LWIS_EVENT_SUBSCRIBE:
		ret = ioctl_event_subscribe(
			lwis_client, (struct lwis_event_subscribe *)param);
		break;
	case LWIS_EVENT_UNSUBSCRIBE:
		ret = ioctl_event_unsubscribe(lwis_client, (int64_t *)param);
		break;
	case LWIS_TIME_QUERY:
		ret = ioctl_time_query(lwis_client, (int64_t *)param);
		break;
	case LWIS_TRANSACTION_SUBMIT:
		ret = ioctl_transaction_submit(
			lwis_client, (struct lwis_transaction_info *)param);
		break;
	case LWIS_TRANSACTION_CANCEL:
		ret = ioctl_transaction_cancel(lwis_client, (int64_t *)param);
		break;
	default:
		pr_err("Unknown IOCTL operation\n");
		ret = -EINVAL;
	};

	return ret;
}
