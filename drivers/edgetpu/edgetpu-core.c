// SPDX-License-Identifier: GPL-2.0
/*
 * Common support functions for Edge TPU ML accelerator host-side ops.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <asm/current.h>
#include <asm/page.h>
#include <linux/atomic.h>
#include <linux/compiler.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "edgetpu-config.h"
#include "edgetpu-debug-dump.h"
#include "edgetpu-device-group.h"
#include "edgetpu-dram.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mailbox.h"
#include "edgetpu-mcp.h"
#include "edgetpu-mmu.h"
#include "edgetpu-telemetry.h"
#include "edgetpu.h"

static atomic_t single_dev_count = ATOMIC_INIT(-1);

/* TODO(b/156444816): Check permission. */
static int edgetpu_mmap_compat(struct edgetpu_client *client,
			       struct vm_area_struct *vma)
{
	int ret;
	ulong phys_base, vma_size, map_size;

	vma_size = vma->vm_end - vma->vm_start;
	map_size = min(vma_size, client->reg_window.size);
	phys_base = client->etdev->regs.phys +
		client->reg_window.start_reg_offset;
	ret = io_remap_pfn_range(vma, vma->vm_start, phys_base >> PAGE_SHIFT,
				 map_size, vma->vm_page_prot);
	if (ret)
		etdev_dbg(client->etdev,
			  "Error remapping PFN range: %d\n", ret);
	return ret;
}

static void edgetpu_vma_open(struct vm_area_struct *vma)
{
	struct edgetpu_client *client = vma->vm_private_data;

	switch (vma->vm_pgoff) {
	case 0:
	case EDGETPU_MMAP_CSR_OFFSET >> PAGE_SHIFT:
		mutex_lock(&client->wakelock.lock);
		client->wakelock.csr_map_count++;
		mutex_unlock(&client->wakelock.lock);
		break;
	}
}

static void edgetpu_vma_close(struct vm_area_struct *vma)
{
	struct edgetpu_client *client = vma->vm_private_data;

	switch (vma->vm_pgoff) {
	case 0:
	case EDGETPU_MMAP_CSR_OFFSET >> PAGE_SHIFT:
		mutex_lock(&client->wakelock.lock);
		if (!client->wakelock.csr_map_count)
			etdev_warn(client->etdev,
				   "unbalanced vma_close on CSR mapping\n");
		else
			client->wakelock.csr_map_count--;
		etdev_dbg(client->etdev,
			  "%s: unmap CSRS. pgoff = %lX count = %u\n", __func__,
			  vma->vm_pgoff, client->wakelock.csr_map_count);
		mutex_unlock(&client->wakelock.lock);
		break;
	}
}

static const struct vm_operations_struct edgetpu_vma_ops = {
	.open = edgetpu_vma_open,
	.close = edgetpu_vma_close,
};


/* Map exported device CSRs or queue into user space. */
int edgetpu_mmap(struct edgetpu_client *client, struct vm_area_struct *vma)
{
	int ret;

	if (vma->vm_start & ~PAGE_MASK) {
		etdev_dbg(client->etdev,
			  "Base address not page-aligned: 0x%lx\n",
			  vma->vm_start);
		return -EINVAL;
	}

	etdev_dbg(client->etdev, "%s: mmap pgoff = %lX\n", __func__,
		  vma->vm_pgoff);

	vma->vm_private_data = client;
	vma->vm_ops = &edgetpu_vma_ops;

	/* Mark the VMA's pages as uncacheable. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* If backward compat map all CSRs */
	if (!vma->vm_pgoff) {
		mutex_lock(&client->wakelock.lock);
		if (!client->wakelock.req_count)
			ret = -EAGAIN;
		else
			ret = edgetpu_mmap_compat(client, vma);
		if (!ret)
			client->wakelock.csr_map_count++;
		mutex_unlock(&client->wakelock.lock);
		return ret;
	}

	/* Allow mapping log and telemetry buffers without creating a group */
	if (vma->vm_pgoff == EDGETPU_MMAP_LOG_BUFFER_OFFSET >> PAGE_SHIFT)
		return edgetpu_mmap_telemetry_buffer(
			client->etdev, EDGETPU_TELEMETRY_LOG, vma);
	if (vma->vm_pgoff == EDGETPU_MMAP_TRACE_BUFFER_OFFSET >> PAGE_SHIFT)
		return edgetpu_mmap_telemetry_buffer(
			client->etdev, EDGETPU_TELEMETRY_TRACE, vma);

	mutex_lock(&client->group_lock);
	if (!client->group) {
		mutex_unlock(&client->group_lock);
		return -EINVAL;
	}

	switch (vma->vm_pgoff) {
	case EDGETPU_MMAP_CSR_OFFSET >> PAGE_SHIFT:
		mutex_lock(&client->wakelock.lock);
		if (!client->wakelock.req_count)
			ret = -EAGAIN;
		else
			ret = edgetpu_mmap_csr(client->group, vma);
		if (!ret)
			client->wakelock.csr_map_count++;
		etdev_dbg(client->etdev, "%s: mmap CSRS. count = %u ret = %d\n",
			  __func__, client->wakelock.csr_map_count, ret);
		mutex_unlock(&client->wakelock.lock);
		break;
	case EDGETPU_MMAP_CMD_QUEUE_OFFSET >> PAGE_SHIFT:
		ret = edgetpu_mmap_queue(client->group, MAILBOX_CMD_QUEUE, vma);
		break;
	case EDGETPU_MMAP_RESP_QUEUE_OFFSET >> PAGE_SHIFT:
		ret = edgetpu_mmap_queue(client->group, MAILBOX_RESP_QUEUE,
					 vma);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&client->group_lock);
	return ret;
}

static struct edgetpu_mailbox_manager_desc mailbox_manager_desc = {
	.num_mailbox = EDGETPU_NUM_MAILBOXES,
	.num_vii_mailbox = EDGETPU_NUM_VII_MAILBOXES,
	.num_p2p_mailbox = EDGETPU_NUM_P2P_MAILBOXES,
	.get_context_csr_base = edgetpu_mailbox_get_context_csr_base,
	.get_cmd_queue_csr_base = edgetpu_mailbox_get_cmd_queue_csr_base,
	.get_resp_queue_csr_base = edgetpu_mailbox_get_resp_queue_csr_base,
};

int edgetpu_get_state_errno_locked(struct edgetpu_dev *etdev)
{
	switch (etdev->state) {
	case ETDEV_STATE_BAD:
		return -ENODEV;
	case ETDEV_STATE_FWLOADING:
		return -EAGAIN;
	case ETDEV_STATE_NOFW:
		return -EINVAL;
	default:
		break;
	}
	return 0;
}

int edgetpu_device_add(struct edgetpu_dev *etdev,
		       const struct edgetpu_mapped_resource *regs)
{
	int ret;

	etdev->regs = *regs;

	/* mcp_id and mcp_die_index fields set by caller */
	if (etdev->mcp_id < 0) {
		uint ordinal_id = atomic_add_return(1, &single_dev_count);

		if (!ordinal_id)
			snprintf(etdev->dev_name, EDGETPU_DEVICE_NAME_MAX, "%s",
				 DRIVER_NAME);
		else
			snprintf(etdev->dev_name, EDGETPU_DEVICE_NAME_MAX,
				 "%s.%u", DRIVER_NAME, ordinal_id);
	} else {
		snprintf(etdev->dev_name, EDGETPU_DEVICE_NAME_MAX,
			 "%s.%u.%u", DRIVER_NAME, etdev->mcp_id,
			 etdev->mcp_die_index);
	}

	mutex_init(&etdev->open.lock);
	mutex_init(&etdev->groups_lock);
	etdev->group_join_lockout = false;
	mutex_init(&etdev->state_lock);
	etdev->state = ETDEV_STATE_NOFW;

	ret = edgetpu_fs_add(etdev);
	if (ret) {
		dev_err(etdev->dev, "%s: edgetpu_fs_add returns %d\n",
			etdev->dev_name, ret);
		return ret;
	}

	etdev->mailbox_manager =
		edgetpu_mailbox_create_mgr(etdev, &mailbox_manager_desc);
	if (IS_ERR(etdev->mailbox_manager)) {
		ret = PTR_ERR(etdev->mailbox_manager);
		dev_err(etdev->dev,
			"%s: edgetpu_mailbox_create_mgr returns %d\n",
			etdev->dev_name, ret);
		goto remove_dev;
	}
	edgetpu_setup_mmu(etdev);

	etdev->kci = devm_kzalloc(etdev->dev, sizeof(*etdev->kci), GFP_KERNEL);
	if (!etdev->kci) {
		ret = -ENOMEM;
		goto detach_mmu;
	}

	etdev->telemetry =
		devm_kzalloc(etdev->dev, sizeof(*etdev->telemetry), GFP_KERNEL);
	if (!etdev->telemetry) {
		ret = -ENOMEM;
		goto detach_mmu;
	}

	ret = edgetpu_kci_init(etdev->mailbox_manager, etdev->kci);
	if (ret) {
		etdev_err(etdev, "edgetpu_kci_init returns %d\n", ret);
		goto detach_mmu;
	}

	ret = edgetpu_device_dram_init(etdev);
	if (ret) {
		etdev_err(etdev,
			  "failed to init on-device DRAM management: %d\n",
			  ret);
		goto remove_kci;
	}

	ret = edgetpu_debug_dump_init(etdev);
	if (ret)
		etdev_warn(etdev, "debug dump init fail: %d", ret);

	edgetpu_chip_init(etdev);
	return 0;

remove_kci:
	/* releases the resources of KCI */
	edgetpu_mailbox_remove_all(etdev->mailbox_manager);
detach_mmu:
	edgetpu_mmu_detach(etdev);
remove_dev:
	edgetpu_mark_probe_fail(etdev);
	edgetpu_fs_remove(etdev);
	return ret;
}

void edgetpu_device_remove(struct edgetpu_dev *etdev)
{
	edgetpu_chip_exit(etdev);
	edgetpu_debug_dump_exit(etdev);
	edgetpu_mailbox_remove_all(etdev->mailbox_manager);
	edgetpu_mmu_detach(etdev);
	edgetpu_fs_remove(etdev);
}

struct edgetpu_client *edgetpu_client_add(struct edgetpu_dev *etdev)
{
	struct edgetpu_client *client;

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	/* Allow entire CSR space to be mmap()'ed using 1.0 interface */
	client->reg_window.start_reg_offset = 0;
	client->reg_window.size = etdev->regs.size;
	client->pid = current->pid;
	client->tgid = current->tgid;
	client->etdev = etdev;
	mutex_init(&client->group_lock);
	mutex_init(&client->wakelock.lock);
	/* Initialize client wakelock state to "acquired" */
	client->wakelock.req_count = 1;
	/* equivalent to edgetpu_client_get() */
	refcount_set(&client->count, 1);
	return client;
}

struct edgetpu_client *edgetpu_client_get(struct edgetpu_client *client)
{
	WARN_ON_ONCE(!refcount_inc_not_zero(&client->count));
	return client;
}

void edgetpu_client_put(struct edgetpu_client *client)
{
	if (!client)
		return;
	if (refcount_dec_and_test(&client->count))
		kfree(client);
}

void edgetpu_client_remove(struct edgetpu_client *client)
{
	if (IS_ERR_OR_NULL(client))
		return;
	/*
	 * A quick check without holding client->group_lock.
	 *
	 * If client doesn't belong to a group then we are fine to not proceed.
	 * If there is a race that the client belongs to a group but is removing
	 * by another process - this will be detected by the check with holding
	 * client->group_lock later.
	 */
	if (client->group)
		edgetpu_device_group_leave(client);
	edgetpu_client_put(client);
}

int edgetpu_register_irq(struct edgetpu_dev *etdev, int irq)
{
	int ret;

	ret = devm_request_irq(etdev->dev, irq, edgetpu_chip_irq_handler,
			       IRQF_ONESHOT, etdev->dev_name, etdev);
	if (ret)
		dev_err(etdev->dev, "%s: failed to request irq %d: %d\n",
			etdev->dev_name, irq, ret);
	return ret;
}

void edgetpu_unregister_irq(struct edgetpu_dev *etdev, int irq)
{
	devm_free_irq(etdev->dev, irq, etdev);
}

int __init edgetpu_init(void)
{
	int ret;

	ret = edgetpu_fs_init();
	if (ret)
		return ret;
	edgetpu_mcp_init();
	return 0;
}

void __exit edgetpu_exit(void)
{
	edgetpu_mcp_exit();
	edgetpu_fs_exit();
}
