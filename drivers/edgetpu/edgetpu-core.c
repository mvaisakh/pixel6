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

/* Map exported device CSRs or queue into user space. */
int edgetpu_mmap(struct edgetpu_client *client, struct vm_area_struct *vma)
{
	if (vma->vm_start & ~PAGE_MASK) {
		etdev_dbg(client->etdev,
			  "Base address not page-aligned: 0x%lx\n",
			  vma->vm_start);
		return -EINVAL;
	}

	vma->vm_private_data = client->etdev;

	/* Mark the VMA's pages as uncacheable. */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* If backward compat map all CSRs */
	if (!vma->vm_pgoff)
		return edgetpu_mmap_compat(client, vma);

	/* Allow mapping log and telemetry buffers without creating a group */
	if (vma->vm_pgoff == EDGETPU_MMAP_LOG_BUFFER_OFFSET >> PAGE_SHIFT)
		return edgetpu_mmap_telemetry_buffer(
			client->etdev, EDGETPU_TELEMETRY_LOG, vma);
	if (vma->vm_pgoff == EDGETPU_MMAP_TRACE_BUFFER_OFFSET >> PAGE_SHIFT)
		return edgetpu_mmap_telemetry_buffer(
			client->etdev, EDGETPU_TELEMETRY_TRACE, vma);

	if (!client->group)
		return -EINVAL;

	if (vma->vm_pgoff == EDGETPU_MMAP_CSR_OFFSET >> PAGE_SHIFT)
		return edgetpu_mmap_csr(client->group, vma);
	if (vma->vm_pgoff == EDGETPU_MMAP_CMD_QUEUE_OFFSET >> PAGE_SHIFT)
		return edgetpu_mmap_queue(client->group, MAILBOX_CMD_QUEUE,
					  vma);
	if (vma->vm_pgoff == EDGETPU_MMAP_RESP_QUEUE_OFFSET >> PAGE_SHIFT)
		return edgetpu_mmap_queue(client->group, MAILBOX_RESP_QUEUE,
					  vma);
	return -EINVAL;
}

void edgetpu_set_open_enabled(struct edgetpu_dev *etdev, bool enabled)
{
	mutex_lock(&etdev->open.lock);
	etdev->open.enabled = enabled;
	mutex_unlock(&etdev->open.lock);
}

static struct edgetpu_mailbox_manager_desc mailbox_manager_desc = {
	.num_mailbox = EDGETPU_NUM_MAILBOXES,
	.num_vii_mailbox = EDGETPU_NUM_VII_MAILBOXES,
	.num_p2p_mailbox = EDGETPU_NUM_P2P_MAILBOXES,
	.get_context_csr_base = edgetpu_mailbox_get_context_csr_base,
	.get_cmd_queue_csr_base = edgetpu_mailbox_get_cmd_queue_csr_base,
	.get_resp_queue_csr_base = edgetpu_mailbox_get_resp_queue_csr_base,
};

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
	etdev->open.enabled = true;
	mutex_init(&etdev->groups_lock);

	ret = edgetpu_dev_add(etdev);
	if (ret) {
		dev_err(etdev->dev, "%s: edgetpu_dev_add returns %d\n",
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
		goto release_mbox_mgr;
	}

	etdev->telemetry =
		devm_kzalloc(etdev->dev, sizeof(*etdev->telemetry), GFP_KERNEL);
	if (!etdev->telemetry) {
		ret = -ENOMEM;
		goto release_mbox_mgr;
	}

	ret = edgetpu_kci_init(etdev->mailbox_manager, etdev->kci);
	if (ret) {
		etdev_err(etdev, "edgetpu_kci_init returns %d\n", ret);
		goto release_mbox_mgr;
	}

	ret = edgetpu_device_dram_init(etdev);
	if (ret) {
		etdev_err(etdev,
			  "failed to init on-device DRAM management: %d\n",
			  ret);
		goto release_mbox_mgr;
	}

	ret = edgetpu_telemetry_init(etdev);
	if (ret)
		etdev_err(etdev, "failed to init telemetry: %d\n", ret);

	edgetpu_chip_init(etdev);
	return 0;

release_mbox_mgr:
	edgetpu_mmu_detach(etdev);
	/* this also releases the resources of KCI */
	edgetpu_mailbox_release_mgr(etdev->mailbox_manager);
remove_dev:
	edgetpu_dev_remove(etdev);
	return ret;
}

void edgetpu_device_remove(struct edgetpu_dev *etdev)
{
	edgetpu_telemetry_exit(etdev);
	edgetpu_chip_exit(etdev);
	edgetpu_dev_remove(etdev);
	edgetpu_mailbox_release_mgr(etdev->mailbox_manager);
	edgetpu_mmu_detach(etdev);
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
	edgetpu_device_group_leave(client);
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

	ret = edgetpu_dev_init();
	if (ret)
		return ret;
	edgetpu_mcp_init();
	return 0;
}

void __exit edgetpu_exit(void)
{
	edgetpu_mcp_exit();
	edgetpu_dev_exit();
}
