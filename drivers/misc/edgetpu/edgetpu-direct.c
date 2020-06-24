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
#include <linux/miscdevice.h>
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
#include "edgetpu-dram.h"
#include "edgetpu-internal.h"
#include "edgetpu-map-dmabuf.h"
#include "edgetpu-mapping.h"
#include "edgetpu-telemetry.h"
#include "edgetpu.h"

#define CREATE_TRACE_POINTS
#include <trace/events/edgetpu.h>

#define DRIVER_VERSION "1.0"

/* Claim miscdev minor numbers starting at 150 */
#define EDGETPU_MISCDEV_MINOR_BASE	150
static atomic_t misc_minor = ATOMIC_INIT(EDGETPU_MISCDEV_MINOR_BASE - 1);

static struct class *edgetpu_class;
static dev_t edgetpu_basedev;
static atomic_t char_minor = ATOMIC_INIT(-1);

static struct dentry *edgetpu_debugfs_dir;

static const struct file_operations edgetpu_fops;

/*
 * Switch device file private_data to point to the client after open,
 * providing for multiple clients.
 */
static int edgetpu_open(struct inode *inode, struct file *file)
{
	struct edgetpu_dev *etdev;
	struct edgetpu_client *client;

	if (MAJOR(inode->i_rdev) == MAJOR(edgetpu_basedev))
		etdev = container_of(inode->i_cdev, struct edgetpu_dev,
				     cdev);
	else
		etdev = container_of(file->private_data,
				     struct edgetpu_dev, miscdev);

	/* Set client pointer to NULL if error creating client. */
	file->private_data = NULL;
	mutex_lock(&etdev->open.lock);
	if (!etdev->open.enabled) {
		mutex_unlock(&etdev->open.lock);
		return -EBUSY;
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

static int etdirect_release(struct inode *inode, struct file *file)
{
	struct edgetpu_client *client = file->private_data;
	struct edgetpu_dev *etdev;

	if (!client)
		return 0;
	etdev = client->etdev;

	edgetpu_client_remove(client);

	mutex_lock(&etdev->open.lock);
	if (etdev->open.count)
		--etdev->open.count;
	mutex_unlock(&etdev->open.lock);
	return 0;
}

static int etdirect_set_eventfd(struct edgetpu_client *client,
				struct edgetpu_event_register __user *argp)
{
	struct edgetpu_event_register eventreg;

	if (copy_from_user(&eventreg, argp, sizeof(eventreg)))
		return -EFAULT;

	return edgetpu_group_set_eventfd(client->group, eventreg.event_id,
					 eventreg.eventfd);
}

static int
etdirect_set_perdie_eventfd(struct edgetpu_dev *etdev,
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

static int etdirect_unset_perdie_eventfd(struct edgetpu_dev *etdev,
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

static int etdirect_join_group(struct edgetpu_client *client, u64 leader_fd)
{
	struct fd f = fdget(leader_fd);
	struct file *file = f.file;
	struct edgetpu_client *leader;
	int ret;

	if (!file) {
		ret = -EBADF;
		goto out;
	}
	if (file->f_op != &edgetpu_fops) {
		ret = -EINVAL;
		goto out;
	}

	leader = file->private_data;
	if (!leader || !leader->group ||
	    !edgetpu_device_group_is_leader(leader->group, leader)) {
		ret = -EINVAL;
		goto out;
	}

	ret = edgetpu_device_group_add(leader->group, client);

out:
	fdput(f);
	return ret;
}

static int etdirect_create_group(struct edgetpu_client *client,
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

static int etdirect_map_buffer(struct edgetpu_device_group *group,
			       struct edgetpu_map_ioctl __user *argp)
{
	struct edgetpu_map_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	trace_edgetpu_map_buffer_start(&ibuf);

	ret = edgetpu_device_group_map(group, &ibuf);
	if (ret)
		return ret;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_device_group_unmap(group, ibuf.die_index,
					   ibuf.device_address);
		return -EFAULT;
	}

	trace_edgetpu_map_buffer_end(&ibuf);

	return 0;
}

static int etdirect_unmap_buffer(struct edgetpu_device_group *group,
				 struct edgetpu_map_ioctl __user *argp)
{
	struct edgetpu_map_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	return edgetpu_device_group_unmap(group, ibuf.die_index,
					  ibuf.device_address);
}

static int
etdirect_allocate_device_buffer_compat(struct edgetpu_device_group *group,
				struct edgetpu_device_buffer_ioctl __user *argp)
{
#ifndef EDGETPU_HAS_DEVICE_DRAM
	return -ENOTTY;
#else
	struct edgetpu_device_buffer_ioctl ibuf;
	struct edgetpu_client *client;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	mutex_lock(&group->lock);

	if (!edgetpu_device_group_is_finalized(group) ||
			ibuf.die_index >= group->n_clients) {
		mutex_unlock(&group->lock);
		return -EINVAL;
	}
	client = group->members[ibuf.die_index];
	mutex_unlock(&group->lock);

	return edgetpu_device_dram_getfd(client, ibuf.size);
#endif /* EDGETPU_HAS_DEVICE_DRAM */
}

static int
etdirect_allocate_device_buffer(struct edgetpu_client *client, u64 size)
{
#ifndef EDGETPU_HAS_DEVICE_DRAM
	return -ENOTTY;
#else
	return edgetpu_device_dram_getfd(client, size);
#endif /* EDGETPU_HAS_DEVICE_DRAM */
}

static int etdirect_sync_buffer(struct edgetpu_device_group *group,
				struct edgetpu_sync_ioctl __user *argp)
{
	struct edgetpu_sync_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	return edgetpu_device_group_sync_buffer(group, &ibuf);
}

static int etdirect_map_dmabuf(struct edgetpu_device_group *group,
			       struct edgetpu_map_dmabuf_ioctl __user *argp)
{
	struct edgetpu_map_dmabuf_ioctl ibuf;
	int ret;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	trace_edgetpu_map_dmabuf_start(&ibuf);

	/* dmabuf is exported by other drivers */
	ret = edgetpu_map_dmabuf(group, &ibuf);
	if (ret)
		return ret;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		edgetpu_unmap_dmabuf(group, ibuf.die_index,
				     ibuf.device_address);
		return -EFAULT;
	}

	trace_edgetpu_map_dmabuf_end(&ibuf);

	return 0;
}

static int etdirect_unmap_dmabuf(struct edgetpu_device_group *group,
				 struct edgetpu_map_dmabuf_ioctl __user *argp)
{
	struct edgetpu_map_dmabuf_ioctl ibuf;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;
	return edgetpu_unmap_dmabuf(group, ibuf.die_index, ibuf.device_address);
}

static bool etdirect_ioctl_check_permissions(struct file *file, uint cmd)
{
	return file->f_mode & FMODE_WRITE;
}

/*
 * Checks if the state of @client is valid to execute ioctl command @cmd.
 */
static bool etdirect_ioctl_check_group(struct edgetpu_client *client, uint cmd)
{
	/* @client must not belong to any group */
	/* Except the default client will transfer old group to new */
	if (cmd == EDGETPU_CREATE_GROUP || cmd == EDGETPU_JOIN_GROUP)
		return !client->group;

	/* Valid for any @client */
	if (cmd == EDGETPU_SET_PERDIE_EVENTFD ||
	    cmd == EDGETPU_UNSET_PERDIE_EVENT ||
	    cmd == EDGETPU_ALLOCATE_DEVICE_BUFFER)
		return true;

	if (!client->group)
		return false;

	/* Other operations can only be applied on the group lead by @client. */
	/*
	 * Note: Though this function already checks the group is not disbanded,
	 * the callbacks of ioctl still need to check the state of group is
	 * waiting/finalized with lock to prevent racing.
	 */
	return edgetpu_device_group_is_leader(client->group, client);
}

static long etdirect_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct edgetpu_client *client = file->private_data;
	void __user *argp = (void __user *)arg;
	long ret;

	if (!client)
		return -ENODEV;

	if (!etdirect_ioctl_check_permissions(file, cmd))
		return -EPERM;

	if (!etdirect_ioctl_check_group(client, cmd))
		return -EINVAL;

	switch (cmd) {
	case EDGETPU_MAP_BUFFER:
		ret = etdirect_map_buffer(client->group, argp);
		break;
	case EDGETPU_UNMAP_BUFFER:
		ret = etdirect_unmap_buffer(client->group, argp);
		break;
	case EDGETPU_UNMAP_BUFFER_COMPAT:
		ret = edgetpu_device_group_unmap(client->group, ALL_DIES,
						 (tpu_addr_t)argp);
		break;
	case EDGETPU_SET_EVENTFD:
		ret = etdirect_set_eventfd(client, argp);
		break;
	case EDGETPU_UNSET_EVENT:
		edgetpu_group_unset_eventfd(client->group, arg);
		ret = 0;
		break;
	case EDGETPU_CREATE_GROUP:
		ret = etdirect_create_group(client, argp);
		break;
	case EDGETPU_JOIN_GROUP:
		ret = etdirect_join_group(client, (u64)argp);
		break;
	case EDGETPU_FINALIZE_GROUP:
		ret = edgetpu_device_group_finalize(client->group);
		break;
	case EDGETPU_SET_PERDIE_EVENTFD:
		ret = etdirect_set_perdie_eventfd(client->etdev, argp);
		break;
	case EDGETPU_UNSET_PERDIE_EVENT:
		ret = etdirect_unset_perdie_eventfd(client->etdev, arg);
		break;
	case EDGETPU_ALLOCATE_DEVICE_BUFFER_COMPAT:
		ret = etdirect_allocate_device_buffer_compat(client->group,
							     argp);
		break;
	case EDGETPU_ALLOCATE_DEVICE_BUFFER:
		ret = etdirect_allocate_device_buffer(client, (u64)argp);
		break;
	case EDGETPU_SYNC_BUFFER:
		ret = etdirect_sync_buffer(client->group, argp);
		break;
	case EDGETPU_MAP_DMABUF:
		ret = etdirect_map_dmabuf(client->group, argp);
		break;
	case EDGETPU_UNMAP_DMABUF:
		ret = etdirect_unmap_dmabuf(client->group, argp);
		break;
	default:
		return -ENOTTY; /* unknown command */
	}

	return ret;
}

/* Map a region of device/coherent memory. */
static int etdirect_mmap(struct file *file, struct vm_area_struct *vma)
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

static void etdirect_setup_debugfs(struct edgetpu_dev *etdev)
{
	etdev->d_entry =
		debugfs_create_dir(etdev->dev_name, edgetpu_debugfs_dir);
	if (!etdev->d_entry)
		return;
	debugfs_create_file("mappings", 0660, etdev->d_entry,
			    etdev, &mappings_ops);
	debugfs_create_file("statusregs", 0660, etdev->d_entry, etdev,
			    &statusregs_ops);
}

static const struct file_operations edgetpu_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.mmap = etdirect_mmap,
	.open = edgetpu_open,
	.release = etdirect_release,
	.unlocked_ioctl = etdirect_ioctl,
};

/* Called from edgetpu core to add a new edgetpu device. */
int edgetpu_dev_add(struct edgetpu_dev *etdev)
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

	// TODO(b/156441506): remove miscdevice support
	etdev->miscdev.name = etdev->dev_name;
	etdev->miscdev.parent = etdev->dev;
	etdev->miscdev.fops = &edgetpu_fops;
	do {
		etdev->miscdev.minor = atomic_add_return(1, &misc_minor);
		ret = misc_register(&etdev->miscdev);
	} while (ret == -EBUSY);

	if (ret)
		dev_warn(etdev->dev,
			"%s: failed to register miscdevice: %d\n",
			 etdev->dev_name, ret);

	etdirect_setup_debugfs(etdev);
	return 0;
}

void edgetpu_dev_remove(struct edgetpu_dev *etdev)
{
	// TODO(b/156441506): remove miscdevice support
	misc_deregister(&etdev->miscdev);
	device_destroy(edgetpu_class, etdev->devno);
	cdev_del(&etdev->cdev);
	debugfs_remove_recursive(etdev->d_entry);
}

int __init edgetpu_dev_init(void)
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
	edgetpu_debugfs_dir = debugfs_create_dir("edgetpu", NULL);
	return 0;
}

void __exit edgetpu_dev_exit(void)
{
	debugfs_remove(edgetpu_debugfs_dir);
	unregister_chrdev_region(edgetpu_basedev, EDGETPU_DEV_MAX);
	class_destroy(edgetpu_class);
}

MODULE_DESCRIPTION("Google EdgeTPU file operations");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
