// SPDX-License-Identifier: GPL-2.0
/*
 * File operations for EdgeTPU ML accel chips.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include "edgetpu-config.h"
#include "edgetpu-device-group.h"
#include "edgetpu-dmabuf.h"
#include "edgetpu-dram.h"
#include "edgetpu-firmware.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mapping.h"
#include "edgetpu-pm.h"
#include "edgetpu-telemetry.h"
#include "edgetpu.h"

#define CREATE_TRACE_POINTS
#include <trace/events/edgetpu.h>

#define DRIVER_VERSION "1.0"

static struct class *edgetpu_class;
static dev_t edgetpu_basedev;
static atomic_t char_minor = ATOMIC_INIT(-1);

static struct dentry *edgetpu_debugfs_dir;

#define LOCK(client) mutex_lock(&client->group_lock)
#define UNLOCK(client) mutex_unlock(&client->group_lock)
/*
 * Locks @client->group_lock and assigns @client->group to @grp.
 * Returns -EINVAL if @client is not the leader of the group.
 */
#define LOCK_RETURN_IF_NOT_LEADER(client, grp)                                 \
	do {                                                                   \
		LOCK(client);                                                  \
		grp = client->group;                                           \
		if (!grp || !edgetpu_device_group_is_leader(grp, client)) {    \
			UNLOCK(client);                                        \
			return -EINVAL;                                        \
		}                                                              \
	} while (0)

int edgetpu_open(struct edgetpu_dev *etdev, struct file *file)
{
	struct edgetpu_client *client;
	int res;

	/* Set client pointer to NULL if error creating client. */
	file->private_data = NULL;
	mutex_lock(&etdev->open.lock);
	if (etdev->pm) {
		res = edgetpu_pm_get(etdev->pm);
		if (res) {
			dev_err(etdev->dev,
				"Failed to request device power up (%d)", res);
			mutex_unlock(&etdev->open.lock);
			return -ENODEV;
		}
	}

	client = edgetpu_client_add(etdev);
	if (IS_ERR(client)) {
		mutex_unlock(&etdev->open.lock);
		return PTR_ERR(client);
	}
	etdev->open.count++;
	mutex_unlock(&etdev->open.lock);
	file->private_data = client;
	return 0;
}

static int edgetpu_fs_open(struct inode *inode, struct file *file)
{
	struct edgetpu_dev *etdev =
		container_of(inode->i_cdev, struct edgetpu_dev, cdev);

	return edgetpu_open(etdev, file);
}

static int edgetpu_fs_release(struct inode *inode, struct file *file)
{
	struct edgetpu_client *client = file->private_data;
	struct edgetpu_dev *etdev;
	uint wakelock_count;

	if (!client)
		return 0;
	etdev = client->etdev;

	mutex_lock(&client->wakelock.lock);

	wakelock_count = client->wakelock.req_count;
	/* Set wakelock state to "released" */
	client->wakelock.req_count = 0;

	/* HACK: Can't disband a group if the device is off, turn it on */
	if (client->group && !wakelock_count) {
		wakelock_count = 1;
		edgetpu_pm_get(etdev->pm);
	}

	mutex_unlock(&client->wakelock.lock);

	edgetpu_client_remove(client);

	mutex_lock(&etdev->open.lock);
	if (etdev->open.count)
		--etdev->open.count;

	/* count was zero if client previously released its wake lock */
	if (wakelock_count)
		edgetpu_pm_put(etdev->pm);
	mutex_unlock(&etdev->open.lock);
	return 0;
}

static int edgetpu_ioctl_set_eventfd(struct edgetpu_client *client,
				     struct edgetpu_event_register __user *argp)
{
	struct edgetpu_device_group *group;
	int ret;
	struct edgetpu_event_register eventreg;

	if (copy_from_user(&eventreg, argp, sizeof(eventreg)))
		return -EFAULT;

	LOCK_RETURN_IF_NOT_LEADER(client, group);
	ret = edgetpu_group_set_eventfd(group, eventreg.event_id,
					eventreg.eventfd);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_unset_eventfd(struct edgetpu_client *client,
				       uint event_id)
{
	struct edgetpu_device_group *group;

	LOCK_RETURN_IF_NOT_LEADER(client, group);
	edgetpu_group_unset_eventfd(group, event_id);
	UNLOCK(client);
	return 0;
}

static int
edgetpu_ioctl_set_perdie_eventfd(struct edgetpu_dev *etdev,
				 struct edgetpu_event_register __user *argp)
{
	struct edgetpu_event_register eventreg;

	if (copy_from_user(&eventreg, argp, sizeof(eventreg)))
		return -EFAULT;

	switch (eventreg.event_id) {
	case EDGETPU_PERDIE_EVENT_LOGS_AVAILABLE:
		return edgetpu_telemetry_set_event(etdev, EDGETPU_TELEMETRY_LOG,
						   eventreg.eventfd);
	case EDGETPU_PERDIE_EVENT_TRACES_AVAILABLE:
		return edgetpu_telemetry_set_event(
			etdev, EDGETPU_TELEMETRY_TRACE, eventreg.eventfd);
	default:
		return -EINVAL;
	}
}

static int edgetpu_ioctl_unset_perdie_eventfd(struct edgetpu_dev *etdev,
					      uint event_id)
{
	switch (event_id) {
	case EDGETPU_PERDIE_EVENT_LOGS_AVAILABLE:
		edgetpu_telemetry_unset_event(etdev, EDGETPU_TELEMETRY_LOG);
		break;
	case EDGETPU_PERDIE_EVENT_TRACES_AVAILABLE:
		edgetpu_telemetry_unset_event(etdev, EDGETPU_TELEMETRY_TRACE);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int edgetpu_ioctl_finalize_group(struct edgetpu_client *client)
{
	struct edgetpu_device_group *group;
	int ret;

	LOCK_RETURN_IF_NOT_LEADER(client, group);
	ret = edgetpu_device_group_finalize(group);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_join_group(struct edgetpu_client *client,
				    u64 leader_fd)
{
	struct fd f = fdget(leader_fd);
	struct file *file = f.file;
	struct edgetpu_client *leader;
	int ret;
	struct edgetpu_device_group *group;

	if (!file) {
		ret = -EBADF;
		goto out;
	}
	leader = file->private_data;
	if (!leader) {
		ret = -EINVAL;
		goto out;
	}
	mutex_lock(&leader->group_lock);
	if (!leader->group ||
	    !edgetpu_device_group_is_leader(leader->group, leader)) {
		ret = -EINVAL;
		mutex_unlock(&leader->group_lock);
		goto out;
	}
	group = edgetpu_device_group_get(leader->group);
	mutex_unlock(&leader->group_lock);

	ret = edgetpu_device_group_add(group, client);
	edgetpu_device_group_put(group);
out:
	fdput(f);
	return ret;
}

static int edgetpu_ioctl_create_group(struct edgetpu_client *client,
				      struct edgetpu_mailbox_attr __user *argp)
{
	struct edgetpu_mailbox_attr attr;
	struct edgetpu_device_group *group;

	if (copy_from_user(&attr, argp, sizeof(attr)))
		return -EFAULT;

	group = edgetpu_device_group_alloc(client, &attr);
	if (IS_ERR(group))
		return PTR_ERR(group);

	edgetpu_device_group_put(group);
	return 0;
}

static int edgetpu_ioctl_map_buffer(struct edgetpu_client *client,
				    struct edgetpu_map_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	struct edgetpu_map_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	trace_edgetpu_map_buffer_start(&ibuf);

	LOCK_RETURN_IF_NOT_LEADER(client, group);
	/* to prevent group being released when we perform map/unmap later */
	group = edgetpu_device_group_get(group);
	/*
	 * Don't hold @client->group_lock on purpose since
	 * 1. We don't care whether @client still belongs to @group.
	 * 2. get_user_pages_fast called by edgetpu_device_group_map() will hold
	 *    mm->mmap_sem, we need to prevent our locks being held around it.
	 */
	UNLOCK(client);
	ret = edgetpu_device_group_map(group, &ibuf);
	if (ret)
		goto out;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_device_group_unmap(group, ibuf.die_index,
					   ibuf.device_address,
					   EDGETPU_MAP_SKIP_CPU_SYNC);
		ret = -EFAULT;
	}

out:
	edgetpu_device_group_put(group);
	trace_edgetpu_map_buffer_end(&ibuf);

	return ret;
}

static int edgetpu_ioctl_unmap_buffer(struct edgetpu_client *client,
				      struct edgetpu_map_ioctl __user *argp)
{
	struct edgetpu_map_ioctl ibuf;
	struct edgetpu_device_group *group;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	LOCK_RETURN_IF_NOT_LEADER(client, group);
	ret = edgetpu_device_group_unmap(group, ibuf.die_index,
					 ibuf.device_address, ibuf.flags);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_allocate_device_buffer_compat(
	struct edgetpu_client *leader,
	struct edgetpu_device_buffer_ioctl __user *argp)
{
#ifndef EDGETPU_HAS_DEVICE_DRAM
	return -ENOTTY;
#else
	struct edgetpu_device_buffer_ioctl ibuf;
	struct edgetpu_client *client;
	struct edgetpu_device_group *group;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	LOCK_RETURN_IF_NOT_LEADER(leader, group);
	mutex_lock(&group->lock);

	if (!edgetpu_device_group_is_finalized(group) ||
	    ibuf.die_index >= group->n_clients) {
		mutex_unlock(&group->lock);
		UNLOCK(leader);
		return -EINVAL;
	}
	client = group->members[ibuf.die_index];
	mutex_unlock(&group->lock);
	UNLOCK(leader);

	return edgetpu_device_dram_getfd(client, ibuf.size);
#endif /* EDGETPU_HAS_DEVICE_DRAM */
}

static int
edgetpu_ioctl_allocate_device_buffer(struct edgetpu_client *client, u64 size)
{
#ifndef EDGETPU_HAS_DEVICE_DRAM
	return -ENOTTY;
#else
	return edgetpu_device_dram_getfd(client, size);
#endif /* EDGETPU_HAS_DEVICE_DRAM */
}

static int edgetpu_ioctl_sync_buffer(struct edgetpu_client *client,
				     struct edgetpu_sync_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	int ret;
	struct edgetpu_sync_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	LOCK_RETURN_IF_NOT_LEADER(client, group);
	ret = edgetpu_device_group_sync_buffer(group, &ibuf);
	UNLOCK(client);
	return ret;
}

static int
edgetpu_ioctl_map_dmabuf(struct edgetpu_client *client,
			 struct edgetpu_map_dmabuf_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	struct edgetpu_map_dmabuf_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	trace_edgetpu_map_dmabuf_start(&ibuf);

	LOCK_RETURN_IF_NOT_LEADER(client, group);
	/* to prevent group being released when we perform unmap on fault */
	group = edgetpu_device_group_get(group);
	ret = edgetpu_map_dmabuf(group, &ibuf);
	UNLOCK(client);
	if (ret)
		goto out;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_unmap_dmabuf(group, ibuf.die_index,
				     ibuf.device_address);
		ret = -EFAULT;
	}

out:
	edgetpu_device_group_put(group);
	trace_edgetpu_map_dmabuf_end(&ibuf);

	return ret;
}

static int
edgetpu_ioctl_unmap_dmabuf(struct edgetpu_client *client,
			   struct edgetpu_map_dmabuf_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	int ret;
	struct edgetpu_map_dmabuf_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	LOCK_RETURN_IF_NOT_LEADER(client, group);
	ret = edgetpu_unmap_dmabuf(group, ibuf.die_index, ibuf.device_address);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_sync_fence_create(
	struct edgetpu_create_sync_fence_data __user *datap)
{
	struct edgetpu_create_sync_fence_data data;
	int ret;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	ret = edgetpu_sync_fence_create(&data);
	if (ret)
		return ret;
	if (copy_to_user((void __user *)datap, &data, sizeof(data)))
		ret = -EFAULT;
	return ret;
}

static int edgetpu_ioctl_sync_fence_signal(
	struct edgetpu_signal_sync_fence_data __user *datap)
{
	struct edgetpu_signal_sync_fence_data data;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	return edgetpu_sync_fence_signal(&data);
}

static int
edgetpu_ioctl_map_bulk_dmabuf(struct edgetpu_client *client,
			      struct edgetpu_map_bulk_dmabuf_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	struct edgetpu_map_bulk_dmabuf_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	LOCK_RETURN_IF_NOT_LEADER(client, group);
	/* to prevent group being released when we perform unmap on fault */
	group = edgetpu_device_group_get(group);
	ret = edgetpu_map_bulk_dmabuf(group, &ibuf);
	UNLOCK(client);
	if (ret)
		goto out;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_unmap_bulk_dmabuf(group, ibuf.device_address);
		ret = -EFAULT;
	}
out:
	edgetpu_device_group_put(group);
	return ret;
}

static int edgetpu_ioctl_unmap_bulk_dmabuf(
	struct edgetpu_client *client,
	struct edgetpu_map_bulk_dmabuf_ioctl __user *argp)
{
	struct edgetpu_device_group *group;
	int ret;
	struct edgetpu_map_bulk_dmabuf_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	LOCK_RETURN_IF_NOT_LEADER(client, group);
	ret = edgetpu_unmap_bulk_dmabuf(group, ibuf.device_address);
	UNLOCK(client);
	return ret;
}

static int edgetpu_ioctl_sync_fence_status(
	struct edgetpu_sync_fence_status __user *datap)
{
	struct edgetpu_sync_fence_status data;
	int ret;

	if (copy_from_user(&data, (void __user *)datap, sizeof(data)))
		return -EFAULT;
	ret = edgetpu_sync_fence_status(&data);
	if (ret)
		return ret;
	if (copy_to_user((void __user *)datap, &data, sizeof(data)))
		ret = -EFAULT;
	return ret;
}

static int edgetpu_ioctl_fw_version(struct edgetpu_dev *etdev,
				    struct edgetpu_fw_version __user *argp)
{
	if (etdev->fw_version.kci_version == EDGETPU_INVALID_KCI_VERSION)
		return -ENODEV;
	if (copy_to_user(argp, &etdev->fw_version, sizeof(*argp)))
		return -EFAULT;
	return 0;
}

static int edgetpu_ioctl_tpu_timestamp(struct edgetpu_client *client,
				       __u64 __user *argp)
{
	u64 timestamp;
	int ret = 0;

	mutex_lock(&client->wakelock.lock);
	if (!client->wakelock.req_count) {
		mutex_unlock(&client->wakelock.lock);
		ret = -EAGAIN;
	} else {
		timestamp = edgetpu_chip_tpu_timestamp(client->etdev);
		mutex_unlock(&client->wakelock.lock);
		if (copy_to_user(argp, &timestamp, sizeof(*argp)))
			ret = -EFAULT;
	}
	return ret;
}

static bool edgetpu_ioctl_check_permissions(struct file *file, uint cmd)
{
	return file->f_mode & FMODE_WRITE;
}

static int edgetpu_ioctl_release_wakelock(struct edgetpu_client *client)
{
	if (!client->etdev->pm)
		return -ENODEV;

	mutex_lock(&client->wakelock.lock);

	/* Cannot release wakelock if client has active CSR mappings */
	if (client->wakelock.csr_map_count) {
		etdev_warn(
			client->etdev,
			"%s: refusing wakelock release with %u CSR mappings\n",
			__func__, client->wakelock.csr_map_count);
		mutex_unlock(&client->wakelock.lock);
		return -EAGAIN;
	}

	/* Cannot release wakelock if it wasn't acquired */
	if (!client->wakelock.req_count) {
		etdev_warn(client->etdev, "%s: invalid wakelock release\n",
			   __func__);
		mutex_unlock(&client->wakelock.lock);
		return -EINVAL;
	}

	edgetpu_pm_put(client->etdev->pm);
	client->wakelock.req_count--;
	etdev_dbg(client->etdev,
		  "%s: wakelock req count = %u CSR map count = %u\n", __func__,
		  client->wakelock.req_count, client->wakelock.csr_map_count);
	mutex_unlock(&client->wakelock.lock);
	return 0;
}

static int edgetpu_ioctl_acquire_wakelock(struct edgetpu_client *client)
{
	int ret;

	if (!client->etdev->pm)
		return -ENODEV;

	mutex_lock(&client->wakelock.lock);

	ret = edgetpu_pm_get(client->etdev->pm);

	if (ret) {
		etdev_warn(client->etdev, "%s: pm_get failed (%d)", __func__,
			   ret);
		mutex_unlock(&client->wakelock.lock);
		return ret;
	}

	client->wakelock.req_count++;
	etdev_dbg(client->etdev,
		  "%s: wakelock req count = %u CSR map count = %u\n", __func__,
		  client->wakelock.req_count, client->wakelock.csr_map_count);
	mutex_unlock(&client->wakelock.lock);
	return 0;
}

long edgetpu_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct edgetpu_client *client = file->private_data;
	void __user *argp = (void __user *)arg;
	long ret;

	if (!client)
		return -ENODEV;

	if (!edgetpu_ioctl_check_permissions(file, cmd))
		return -EPERM;

	switch (cmd) {
	case EDGETPU_MAP_BUFFER:
		ret = edgetpu_ioctl_map_buffer(client, argp);
		break;
	case EDGETPU_UNMAP_BUFFER:
		ret = edgetpu_ioctl_unmap_buffer(client, argp);
		break;
	case EDGETPU_SET_EVENTFD:
		ret = edgetpu_ioctl_set_eventfd(client, argp);
		break;
	case EDGETPU_CREATE_GROUP:
		ret = edgetpu_ioctl_create_group(client, argp);
		break;
	case EDGETPU_JOIN_GROUP:
		ret = edgetpu_ioctl_join_group(client, (u64)argp);
		break;
	case EDGETPU_FINALIZE_GROUP:
		ret = edgetpu_ioctl_finalize_group(client);
		break;
	case EDGETPU_SET_PERDIE_EVENTFD:
		ret = edgetpu_ioctl_set_perdie_eventfd(client->etdev, argp);
		break;
	case EDGETPU_ALLOCATE_DEVICE_BUFFER_COMPAT:
		ret = edgetpu_ioctl_allocate_device_buffer_compat(client, argp);
		break;
	case EDGETPU_UNSET_EVENT:
		ret = edgetpu_ioctl_unset_eventfd(client, arg);
		break;
	case EDGETPU_UNSET_PERDIE_EVENT:
		ret = edgetpu_ioctl_unset_perdie_eventfd(client->etdev, arg);
		break;
	case EDGETPU_SYNC_BUFFER:
		ret = edgetpu_ioctl_sync_buffer(client, argp);
		break;
	case EDGETPU_MAP_DMABUF:
		ret = edgetpu_ioctl_map_dmabuf(client, argp);
		break;
	case EDGETPU_UNMAP_DMABUF:
		ret = edgetpu_ioctl_unmap_dmabuf(client, argp);
		break;
	case EDGETPU_ALLOCATE_DEVICE_BUFFER:
		ret = edgetpu_ioctl_allocate_device_buffer(client, (u64)argp);
		break;
	case EDGETPU_CREATE_SYNC_FENCE:
		ret = edgetpu_ioctl_sync_fence_create(argp);
		break;
	case EDGETPU_SIGNAL_SYNC_FENCE:
		ret = edgetpu_ioctl_sync_fence_signal(argp);
		break;
	case EDGETPU_MAP_BULK_DMABUF:
		ret = edgetpu_ioctl_map_bulk_dmabuf(client, argp);
		break;
	case EDGETPU_UNMAP_BULK_DMABUF:
		ret = edgetpu_ioctl_unmap_bulk_dmabuf(client, argp);
		break;
	case EDGETPU_SYNC_FENCE_STATUS:
		ret = edgetpu_ioctl_sync_fence_status(argp);
		break;
	case EDGETPU_RELEASE_WAKE_LOCK:
		ret = edgetpu_ioctl_release_wakelock(client);
		break;
	case EDGETPU_ACQUIRE_WAKE_LOCK:
		ret = edgetpu_ioctl_acquire_wakelock(client);
		break;
	case EDGETPU_FIRMWARE_VERSION:
		ret = edgetpu_ioctl_fw_version(client->etdev, argp);
		break;
	case EDGETPU_GET_TPU_TIMESTAMP:
		ret = edgetpu_ioctl_tpu_timestamp(client, argp);
		break;
	default:
		return -ENOTTY; /* unknown command */
	}

	return ret;
}

static long edgetpu_fs_ioctl(struct file *file, uint cmd, ulong arg)
{
	if (file->f_op != &edgetpu_fops)
		return -ENOTTY;

	return edgetpu_ioctl(file, cmd, arg);
}

/* Map a region of device/coherent memory. */
static int edgetpu_fs_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct edgetpu_client *client = file->private_data;

	if (!client)
		return -ENODEV;

	return edgetpu_mmap(client, vma);
}

static struct edgetpu_dumpregs_range common_statusregs_ranges[] = {
	{
		.firstreg = EDGETPU_REG_AON_RESET,
		.lastreg = EDGETPU_REG_AON_FORCE_QUIESCE,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_DMA_PAUSED,
		.lastreg = EDGETPU_REG_USER_HIB_DMA_PAUSED,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_FIRST_ERROR_STATUS,
		.lastreg = EDGETPU_REG_USER_HIB_FIRST_ERROR_STATUS,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_ERROR_STATUS,
		.lastreg = EDGETPU_REG_USER_HIB_ERROR_MASK,
	},
	{
		.firstreg = EDGETPU_REG_SC_CURRENTPC,
		.lastreg = EDGETPU_REG_SC_DECODEPC,
	},
	{
		.firstreg = EDGETPU_REG_SC_RUNSTATUS,
		.lastreg = EDGETPU_REG_SC_RUNSTATUS,
	},
	{
		.firstreg = EDGETPU_REG_SC_ERROR,
		.lastreg = EDGETPU_REG_SC_ERROR_MASK,
	},
	{
		.firstreg = EDGETPU_REG_SC_ERROR_INFO,
		.lastreg = EDGETPU_REG_SC_ERROR_INFO,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_OUT_ACTVQ_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_OUT_ACTVQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_INSTRQ_TAIL,
		.lastreg = EDGETPU_REG_USER_HIB_INSTRQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_IN_ACTVQ_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_IN_ACTVQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_PARAMQ_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_PARAMQ_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_SC_HOST_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_SC_HOST_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_TOPLVL_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_TOPLVL_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_FATALERR_INT_STAT,
		.lastreg = EDGETPU_REG_USER_HIB_FATALERR_INT_STAT,
	},
	{
		.firstreg = EDGETPU_REG_USER_HIB_SNAPSHOT,
		.lastreg = EDGETPU_REG_USER_HIB_SNAPSHOT,
	},
};

static struct edgetpu_dumpregs_range common_tile_statusregs_ranges[] = {
	{
		.firstreg = EDGETPU_REG_TILECONF1_DEEPSLEEP,
		.lastreg = EDGETPU_REG_TILECONF1_DEEPSLEEP,
	},
	{
		.firstreg = EDGETPU_REG_TILECONF1_ERROR_TILE,
		.lastreg = EDGETPU_REG_TILECONF1_ERROR_MASK_TILE,
	},
	{
		.firstreg = EDGETPU_REG_TILECONF1_ERROR_INFO_TILE,
		.lastreg = EDGETPU_REG_TILECONF1_ERROR_INFO_TILE,
	},
};

static void dump_statusregs_ranges(
	struct seq_file *s, struct edgetpu_dev *etdev,
	struct edgetpu_dumpregs_range *ranges, int nranges)
{
	int i;
	enum edgetpu_csrs reg;
	uint64_t val;

	for (i = 0; i < nranges; i++) {
		for (reg = ranges[i].firstreg; reg <= ranges[i].lastreg;
		     reg += sizeof(val)) {
			val = edgetpu_dev_read_64(etdev, reg);
			seq_printf(s, "0x%08x: 0x%016llx\n", reg, val);
		}
	}
}

static void dump_mboxes(struct seq_file *s, struct edgetpu_dev *etdev)
{
	enum edgetpu_csrs base;
	uint32_t val;
	int mbox_id;
	int n_p2p_mbox_dump = min(EDGETPU_NUM_P2P_MAILBOXES, 2);

	/* Dump VII mailboxes plus 2 P2P (if any) + KCI. */
	for (mbox_id = 0, base = EDGETPU_MBOX_BASE;
	     mbox_id < EDGETPU_NUM_VII_MAILBOXES + n_p2p_mbox_dump + 1;
	     mbox_id++, base += EDGETPU_MBOX_CSRS_SIZE) {
		int offset;

		for (offset = 0x0; offset <= 0x40; offset += sizeof(val)) {
			val = edgetpu_dev_read_32(etdev, base + offset);
			seq_printf(s, "0x%08x: 0x%08x\n", base + offset, val);
		}
		for (offset = 0x1000; offset <= 0x1014; offset += sizeof(val)) {
			val = edgetpu_dev_read_32(etdev, base + offset);
			seq_printf(s, "0x%08x: 0x%08x\n", base + offset, val);
		}
		for (offset = 0x1800; offset <= 0x1818; offset += sizeof(val)) {
			val = edgetpu_dev_read_32(etdev, base + offset);
			seq_printf(s, "0x%08x: 0x%08x\n", base + offset, val);
		}
	}
}

static int statusregs_show(struct seq_file *s, void *data)
{
	struct edgetpu_dev *etdev = s->private;
	int tileid;

	dump_statusregs_ranges(s, etdev, common_statusregs_ranges,
			       ARRAY_SIZE(common_statusregs_ranges));
	dump_statusregs_ranges(s, etdev, edgetpu_chip_statusregs_ranges,
			       edgetpu_chip_statusregs_nranges);
	for (tileid = 0; tileid < EDGETPU_NTILES; tileid++) {
		edgetpu_dev_write_64(etdev, EDGETPU_REG_USER_HIB_TILECONFIG1,
				     tileid);
		seq_printf(s, "tile %d:\n", tileid);
		dump_statusregs_ranges(
			s, etdev, common_tile_statusregs_ranges,
			ARRAY_SIZE(common_tile_statusregs_ranges));
		dump_statusregs_ranges(s, etdev,
				       edgetpu_chip_tile_statusregs_ranges,
				       edgetpu_chip_tile_statusregs_nranges);
	}
	dump_mboxes(s, etdev);
	return 0;
}

static int statusregs_open(struct inode *inode, struct file *file)
{
	return single_open(file, statusregs_show, inode->i_private);
}

static const struct file_operations statusregs_ops = {
	.open = statusregs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};

static int mappings_show(struct seq_file *s, void *data)
{
	struct edgetpu_dev *etdev = s->private;
	int i;

	mutex_lock(&etdev->groups_lock);

	for (i = 0; i < EDGETPU_NGROUPS; i++) {
		struct edgetpu_device_group *group = etdev->groups[i];

		if (!group)
			continue;
		edgetpu_group_mappings_show(group, s);
	}

	mutex_unlock(&etdev->groups_lock);
	edgetpu_kci_mappings_show(etdev, s);
	return 0;
}

static int mappings_open(struct inode *inode, struct file *file)
{
	return single_open(file, mappings_show, inode->i_private);
}

static const struct file_operations mappings_ops = {
	.open = mappings_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};

static void edgetpu_fs_setup_debugfs(struct edgetpu_dev *etdev)
{
	etdev->d_entry =
		debugfs_create_dir(etdev->dev_name, edgetpu_debugfs_dir);
	if (!etdev->d_entry)
		return;
	debugfs_create_file("mappings", 0440, etdev->d_entry,
			    etdev, &mappings_ops);
	debugfs_create_file("statusregs", 0440, etdev->d_entry, etdev,
			    &statusregs_ops);
}

const struct file_operations edgetpu_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.mmap = edgetpu_fs_mmap,
	.open = edgetpu_fs_open,
	.release = edgetpu_fs_release,
	.unlocked_ioctl = edgetpu_fs_ioctl,
};

/* Called from edgetpu core to add a new edgetpu device. */
int edgetpu_fs_add(struct edgetpu_dev *etdev)
{
	int ret;

	etdev->devno = MKDEV(MAJOR(edgetpu_basedev),
			     atomic_add_return(1, &char_minor));
	cdev_init(&etdev->cdev, &edgetpu_fops);
	ret = cdev_add(&etdev->cdev, etdev->devno, 1);
	if (ret) {
		dev_err(etdev->dev,
			"%s: error %d adding cdev for dev %d:%d\n",
			etdev->dev_name, ret,
			MAJOR(etdev->devno), MINOR(etdev->devno));
		return ret;
	}

	etdev->etcdev = device_create(edgetpu_class, etdev->dev, etdev->devno,
				      etdev, etdev->dev_name);
	if (IS_ERR(etdev->etcdev)) {
		ret = PTR_ERR(etdev->etcdev);
		dev_err(etdev->dev, "%s: failed to create char device: %d\n",
			etdev->dev_name, ret);
		cdev_del(&etdev->cdev);
		return ret;
	}

	edgetpu_fs_setup_debugfs(etdev);
	return 0;
}

void edgetpu_fs_remove(struct edgetpu_dev *etdev)
{
	device_destroy(edgetpu_class, etdev->devno);
	cdev_del(&etdev->cdev);
	debugfs_remove_recursive(etdev->d_entry);
}

static int syncfences_open(struct inode *inode, struct file *file)
{
	return single_open(file, edgetpu_sync_fence_debugfs_show,
			   inode->i_private);
}

static const struct file_operations syncfences_ops = {
	.open = syncfences_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.owner = THIS_MODULE,
	.release = single_release,
};

static void edgetpu_debugfs_global_setup(void)
{
	edgetpu_debugfs_dir = debugfs_create_dir("edgetpu", NULL);
	debugfs_create_file("syncfences", 0440, edgetpu_debugfs_dir, NULL,
			    &syncfences_ops);
}

int __init edgetpu_fs_init(void)
{
	int ret;

	edgetpu_class = class_create(THIS_MODULE, "edgetpu");
	if (IS_ERR(edgetpu_class)) {
		pr_err(DRIVER_NAME " error creating edgetpu class: %ld\n",
		       PTR_ERR(edgetpu_class));
		return PTR_ERR(edgetpu_class);
	}

	ret = alloc_chrdev_region(&edgetpu_basedev, 0, EDGETPU_DEV_MAX,
				  DRIVER_NAME);
	if (ret) {
		pr_err(DRIVER_NAME " char driver registration failed: %d\n",
		       ret);
		class_destroy(edgetpu_class);
		return ret;
	}
	pr_debug(DRIVER_NAME " registered major=%d\n", MAJOR(edgetpu_basedev));
	edgetpu_debugfs_global_setup();
	return 0;
}

void __exit edgetpu_fs_exit(void)
{
	debugfs_remove_recursive(edgetpu_debugfs_dir);
	unregister_chrdev_region(edgetpu_basedev, EDGETPU_DEV_MAX);
	class_destroy(edgetpu_class);
}

struct dentry *edgetpu_fs_debugfs_dir(void)
{
	return edgetpu_debugfs_dir;
}

MODULE_DESCRIPTION("Google EdgeTPU file operations");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_INFO(gitinfo, GIT_REPO_TAG);
