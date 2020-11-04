// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU ML accelerator telemetry: logging and tracing.
 *
 * Copyright (C) 2019-2020 Google, Inc.
 */
#ifdef CONFIG_X86
#include <linux/printk.h>	// pr_warn used by set_memory.h
#include <asm/pgtable_types.h>
#include <asm/set_memory.h>
#endif
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "edgetpu-iremap-pool.h"
#include "edgetpu-mmu.h"
#include "edgetpu-telemetry.h"
#include "edgetpu.h"

static struct edgetpu_telemetry *
select_telemetry(struct edgetpu_telemetry_ctx *ctx,
		 enum edgetpu_telemetry_type type)
{
	switch (type) {
	case EDGETPU_TELEMETRY_TRACE:
		return &ctx->trace;
	case EDGETPU_TELEMETRY_LOG:
		return &ctx->log;
	default:
		WARN_ONCE(1, "Unrecognized EdgeTPU telemetry type: %d", type);
		/* return a valid object, don't crash the kernel */
		return &ctx->log;
	}
}

static int telemetry_init(struct edgetpu_dev *etdev,
			  struct edgetpu_telemetry *tel, const char *name,
			  struct edgetpu_coherent_mem *mem)
{
	const size_t size = EDGETPU_TELEMETRY_BUFFER_SIZE;
	const u32 flags = EDGETPU_MMU_DIE | EDGETPU_MMU_32 | EDGETPU_MMU_HOST;
	void *vaddr;
	dma_addr_t dma_addr;
	tpu_addr_t tpu_addr;

	if (mem) {
		tel->coherent_mem = *mem;
		vaddr = mem->vaddr;
		tel->caller_mem = true;
	} else {
		vaddr = dmam_alloc_coherent(etdev->dev, size, &dma_addr,
					    GFP_KERNEL);
		if (!vaddr)
			return -ENOMEM;
#ifdef CONFIG_X86
		set_memory_uc((unsigned long)vaddr, size >> PAGE_SHIFT);
#endif
		tpu_addr = edgetpu_mmu_tpu_map(etdev, dma_addr, size,
					       DMA_BIDIRECTIONAL,
					       EDGETPU_CONTEXT_KCI, flags);
		if (!tpu_addr) {
			dev_err(etdev->dev,
				"%s: failed to map buffer for '%s'\n",
				etdev->dev_name, name);
			return -ENOSPC;
		}
		tel->coherent_mem.vaddr = vaddr;
		tel->coherent_mem.dma_addr = dma_addr;
		tel->coherent_mem.tpu_addr = tpu_addr;
		tel->coherent_mem.size = size;
		tel->caller_mem = false;
	}

	rwlock_init(&tel->ctx_mem_lock);
	tel->name = name;

	tel->header = (struct edgetpu_telemetry_header *)vaddr;
	tel->header->head = 0;
	tel->header->tail = 0;
	tel->header->entries_dropped = 0;

	tel->ctx = NULL;

	spin_lock_init(&tel->state_lock);
	tel->state = EDGETPU_TELEMETRY_ENABLED;
	tel->inited = true;

	return 0;
}

static void telemetry_exit(struct edgetpu_dev *etdev,
			   struct edgetpu_telemetry *tel)
{
	ulong flags;

	if (!tel->inited)
		return;
	spin_lock_irqsave(&tel->state_lock, flags);
	/* Prevent racing with the IRQ handler */
	tel->state = EDGETPU_TELEMETRY_INVALID;
	spin_unlock_irqrestore(&tel->state_lock, flags);

	if (tel->coherent_mem.tpu_addr && !tel->caller_mem) {
		edgetpu_mmu_tpu_unmap(etdev, tel->coherent_mem.tpu_addr,
				      tel->coherent_mem.size,
				      EDGETPU_CONTEXT_KCI);
		tel->coherent_mem.tpu_addr = 0;
#ifdef CONFIG_X86
		set_memory_wb((unsigned long)tel->coherent_mem.vaddr,
			      tel->coherent_mem.size >> PAGE_SHIFT);
#endif
	}
	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = NULL;
}

static int telemetry_kci(struct edgetpu_dev *etdev,
			 struct edgetpu_telemetry *tel,
			 int (*send_kci)(struct edgetpu_kci *, u64, u32))
{
	int err;

	if (!tel->inited)
		return -ENODEV;
	etdev_dbg(etdev, "Sending KCI %s", tel->name);
	err = send_kci(etdev->kci, tel->coherent_mem.tpu_addr,
		       tel->coherent_mem.size);

	if (err < 0) {
		etdev_err(etdev, "KCI %s failed :( - %d", tel->name, err);
		return err;
	}

	if (err > 0) {
		etdev_err(etdev, "KCI %s returned %d", tel->name, err);
		return -EBADMSG;
	}
	etdev_dbg(etdev, "KCI %s Succeeded :)", tel->name);
	return 0;
}

static int telemetry_set_event(struct edgetpu_dev *etdev,
			       struct edgetpu_telemetry *tel, u32 eventfd)
{
	struct eventfd_ctx *ctx;
	ulong flags;

	if (!tel->inited)
		return -ENODEV;
	ctx = eventfd_ctx_fdget(eventfd);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	write_lock_irqsave(&tel->ctx_mem_lock, flags);
	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = ctx;
	write_unlock_irqrestore(&tel->ctx_mem_lock, flags);

	return 0;
}

static void telemetry_unset_event(struct edgetpu_dev *etdev,
				  struct edgetpu_telemetry *tel)
{
	ulong flags;

	if (!tel->inited)
		return;
	write_lock_irqsave(&tel->ctx_mem_lock, flags);
	if (tel->ctx)
		eventfd_ctx_put(tel->ctx);
	tel->ctx = NULL;
	write_unlock_irqrestore(&tel->ctx_mem_lock, flags);

	return;
}

/* Copy data out of the log buffer with wrapping */
static void copy_with_wrap(struct edgetpu_telemetry_header *header, void *dest,
			   u32 length, u32 size, void *start)
{
	const u32 wrap_bit = EDGETPU_TELEMETRY_WRAP_BIT;
	u32 remaining = 0;
	u32 head = header->head & (wrap_bit - 1);

	if (head + length < size) {
		memcpy(dest, start + head, length);
		header->head += length;
	} else {
		remaining = size - head;
		memcpy(dest, start + head, remaining);
		memcpy(dest + remaining, start, length - remaining);
		header->head = (header->head & wrap_bit) ^ wrap_bit;
		header->head |= length - remaining;
	}
}

/* Log messages from TPU CPU to dmesg */
static void edgetpu_fw_log(struct edgetpu_dev *etdev,
			   struct edgetpu_telemetry *log)
{
	struct edgetpu_telemetry_header *header = log->header;
	struct edgetpu_log_entry_header entry;
	u8 *start;
	const size_t queue_size = log->coherent_mem.size - sizeof(*header);
	const size_t max_length = queue_size - sizeof(entry);
	char *buffer = kmalloc(max_length + 1, GFP_ATOMIC);

	if (!buffer) {
		header->head = header->tail;
		etdev_err_ratelimited(etdev, "failed to allocate log buffer");
		return;
	}
	start = (u8 *)header + sizeof(*header);

	while (header->head != header->tail) {
		copy_with_wrap(header, &entry, sizeof(entry), queue_size,
			       start);
		if (entry.length == 0 || entry.length > max_length) {
			header->head = header->tail;
#if 0 /* TODO(b/170340226): add me back */
			etdev_err_ratelimited(etdev, "log queue is corrupted");
#endif
			break;
		}
		copy_with_wrap(header, buffer, entry.length, queue_size, start);
		buffer[entry.length] = 0;

		switch (entry.code) {
		case 2:
		case 1:
			etdev_dbg_ratelimited(etdev, "%s", buffer);
			break;
		case -1:
			etdev_warn_ratelimited(etdev, "%s", buffer);
			break;
		case -2:
			etdev_err_ratelimited(etdev, "%s", buffer);
			break;
		case 0:
		default:
			etdev_info_ratelimited(etdev, "%s", buffer);
			break;
		}
	}
	kfree(buffer);
}

/* Consumes the queue buffer. */
static void edgetpu_fw_trace(struct edgetpu_dev *etdev,
			     struct edgetpu_telemetry *trace)
{
	struct edgetpu_telemetry_header *header = trace->header;

	header->head = header->tail;
}

/*
 * If the buffer queue is not empty,
 * - signals the event context.
 * - calls @fallback if event is not set.
 */
static void telemetry_irq_handler(struct edgetpu_dev *etdev,
				  struct edgetpu_telemetry *tel,
				  void (*fallback)(struct edgetpu_dev *,
						   struct edgetpu_telemetry *))
{
	if (!tel->inited)
		return;
	spin_lock(&tel->state_lock);

	if (tel->state == EDGETPU_TELEMETRY_ENABLED &&
	    tel->header->head != tel->header->tail) {
		read_lock(&tel->ctx_mem_lock);
		if (tel->ctx)
			eventfd_signal(tel->ctx, 1);
		else
			fallback(etdev, tel);
		read_unlock(&tel->ctx_mem_lock);
	}

	spin_unlock(&tel->state_lock);
}

static void telemetry_mappings_show(struct edgetpu_telemetry *tel,
				    struct seq_file *s)
{
	if (!tel->inited)
		return;

	seq_printf(s, "  0x%llx %lu %s 0x%llx %pad\n",
		   tel->coherent_mem.tpu_addr,
		   tel->coherent_mem.size / PAGE_SIZE, tel->name,
		   tel->coherent_mem.host_addr, &tel->coherent_mem.dma_addr);
}

static int telemetry_mmap_buffer(struct edgetpu_dev *etdev,
				 struct edgetpu_telemetry *tel,
				 struct vm_area_struct *vma)
{
	int ret;

	if (!tel->inited)
		return -ENODEV;

	write_lock(&tel->ctx_mem_lock);

	ret = edgetpu_iremap_mmap(etdev, vma, &tel->coherent_mem);

	if (!ret)
		tel->coherent_mem.host_addr = vma->vm_start;

	write_unlock(&tel->ctx_mem_lock);

	return ret;
}

int edgetpu_telemetry_init(struct edgetpu_dev *etdev,
			   struct edgetpu_coherent_mem *log_mem,
			   struct edgetpu_coherent_mem *trace_mem)
{
	int ret;

	if (!etdev->telemetry)
		return -ENODEV;
	ret = telemetry_init(etdev, &etdev->telemetry->log, "telemetry_log",
			     log_mem);
	if (ret)
		return ret;
#if IS_ENABLED(CONFIG_EDGETPU_TELEMETRY_TRACE)
	ret = telemetry_init(etdev, &etdev->telemetry->trace, "telemetry_trace",
			     trace_mem);
	if (ret) {
		telemetry_exit(etdev, &etdev->telemetry->log);
		return ret;
	}
#endif
	return 0;
}

void edgetpu_telemetry_exit(struct edgetpu_dev *etdev)
{
	if (!etdev->telemetry)
		return;
	telemetry_exit(etdev, &etdev->telemetry->trace);
	telemetry_exit(etdev, &etdev->telemetry->log);
}

int edgetpu_telemetry_kci(struct edgetpu_dev *etdev)
{
	int ret;

	if (!etdev->telemetry)
		return -ENODEV;
	ret = telemetry_kci(etdev, &etdev->telemetry->log,
			    edgetpu_kci_map_log_buffer);
	if (ret)
		return ret;
	ret = telemetry_kci(etdev, &etdev->telemetry->trace,
			    edgetpu_kci_map_trace_buffer);
	if (ret)
		return ret;

	return 0;
}

int edgetpu_telemetry_set_event(struct edgetpu_dev *etdev,
				enum edgetpu_telemetry_type type, u32 eventfd)
{
	if (!etdev->telemetry)
		return -ENODEV;
	return telemetry_set_event(
		etdev, select_telemetry(etdev->telemetry, type), eventfd);
}

void edgetpu_telemetry_unset_event(struct edgetpu_dev *etdev,
				   enum edgetpu_telemetry_type type)
{
	if (!etdev->telemetry)
		return;
	telemetry_unset_event(etdev, select_telemetry(etdev->telemetry, type));
}

void edgetpu_telemetry_irq_handler(struct edgetpu_dev *etdev)
{
	if (!etdev->telemetry)
		return;
	telemetry_irq_handler(etdev, &etdev->telemetry->log, edgetpu_fw_log);
	telemetry_irq_handler(etdev, &etdev->telemetry->trace,
			      edgetpu_fw_trace);
}

void edgetpu_telemetry_mappings_show(struct edgetpu_dev *etdev,
				     struct seq_file *s)
{
	if (!etdev->telemetry)
		return;
	telemetry_mappings_show(&etdev->telemetry->log, s);
	telemetry_mappings_show(&etdev->telemetry->trace, s);
}

int edgetpu_mmap_telemetry_buffer(struct edgetpu_dev *etdev,
				  enum edgetpu_telemetry_type type,
				  struct vm_area_struct *vma)
{
	if (!etdev->telemetry)
		return -ENODEV;
	return telemetry_mmap_buffer(
		etdev, select_telemetry(etdev->telemetry, type), vma);
}
