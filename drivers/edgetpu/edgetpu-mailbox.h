/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Utility functions of mailbox protocol for Edge TPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_MAILBOX_H__
#define __EDGETPU_MAILBOX_H__

#include <linux/compiler.h>
#include <linux/irqreturn.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "edgetpu-internal.h"
#include "edgetpu.h"

#define CIRCULAR_QUEUE_WRAP_BIT (1 << 10)
#define CIRCULAR_QUEUE_INDEX_MASK (CIRCULAR_QUEUE_WRAP_BIT - 1)
#define CIRCULAR_QUEUE_WRAPPED(idx) ((idx) & CIRCULAR_QUEUE_WRAP_BIT)
#define CIRCULAR_QUEUE_REAL_INDEX(idx) ((idx) & CIRCULAR_QUEUE_INDEX_MASK)

/* CMD_QUEUE_SIZE register is 10 bits wide */
#define MAX_QUEUE_SIZE (CIRCULAR_QUEUE_WRAP_BIT - 1)

/* the index of mailbox for kernel control interface */
#define KERNEL_MAILBOX_INDEX 0

/* Size of CSRs start from cmd_queue_csr_base can be mmap-ed to userspace. */
#define USERSPACE_CSR_SIZE 0x1000ul

struct edgetpu_device_group;

struct edgetpu_mailbox {
	uint mailbox_id;
	struct edgetpu_dev *etdev;
	/* base offset for CSRs in struct edgetpu_mailbox_context_csr */
	u32 context_csr_base;
	/* base offset for CSRs in struct edgetpu_mailbox_cmd_queue_csr */
	u32 cmd_queue_csr_base;
	/* base offset for CSRs in struct edgetpu_mailbox_resp_queue_csr */
	u32 resp_queue_csr_base;

	/*
	 * Queue-related fields, all of them are in units of number of elements.
	 */

	u32 cmd_queue_size; /* size of cmd queue */
	u32 cmd_queue_tail; /* offset within the cmd queue */
	u32 resp_queue_size; /* size of resp queue */
	u32 resp_queue_head; /* offset within the resp queue */

	/* IRQ handler */
	void (*handle_irq)(struct edgetpu_mailbox *mailbox);

	/*
	 * Internal data. It's edgetpu_kci* for KCI mailbox,
	 * edgetpu_device_group* for VII mailboxes.
	 */
	union {
		struct edgetpu_kci *kci;
		struct edgetpu_device_group *group;
	} internal;
};

typedef struct edgetpu_coherent_mem edgetpu_queue_mem;

struct edgetpu_vii {
	/* The mailbox this VII uses, can be NULL for an uninitialized VII. */
	struct edgetpu_mailbox *mailbox;
	edgetpu_queue_mem cmd_queue_mem;
	edgetpu_queue_mem resp_queue_mem;
};

typedef u32 (*get_csr_base_t)(uint index);

struct edgetpu_mailbox_manager {
	struct edgetpu_dev *etdev;
	/* total number of mailboxes that edgetpu device could provide */
	u8 num_mailbox;
	/* indices reserved for VII, the range is [from, to) */
	u8 vii_index_from, vii_index_to;
	/* indices reserved for P2P, the range is [from, to) */
	u8 p2p_index_from, p2p_index_to;
	rwlock_t mailboxes_lock;	/* protects mailboxes */
	struct edgetpu_mailbox **mailboxes;
	/* converts index (0 ~ num_mailbox - 1) of mailbox to CSR offset */
	get_csr_base_t get_context_csr_base;
	get_csr_base_t get_cmd_queue_csr_base;
	get_csr_base_t get_resp_queue_csr_base;
};

/* the structure to configure a mailbox manager */
struct edgetpu_mailbox_manager_desc {
	u8 num_mailbox;
	u8 num_vii_mailbox;
	u8 num_p2p_mailbox;
	get_csr_base_t get_context_csr_base;
	get_csr_base_t get_cmd_queue_csr_base;
	get_csr_base_t get_resp_queue_csr_base;
};

/* Mailbox CSRs. The order and size are exactly the same as RTL defined. */

/* CSRs that can be accessed by AP kernel only, don't mmap them to userspace */
struct edgetpu_mailbox_context_csr {
	u32 context_enable;
	u32 priority;
	u32 cmd_queue_doorbell_enable;
	/* doorbell will be triggered automatically on cmd_queue_tail updated */
	u32 cmd_queue_tail_doorbell_enable;
	u32 cmd_queue_doorbell_clear;
	u32 cmd_queue_address_low;
	u32 cmd_queue_address_high;
	u32 cmd_queue_size;
	u32 resp_queue_doorbell_enable;
	u32 resp_queue_tail_doorbell_enable;
	u32 resp_queue_address_low;
	u32 resp_queue_address_high;
	u32 resp_queue_size;
	u32 config_spare_0;
	u32 config_spare_1;
	u32 config_spare_2;
	u32 config_spare_3;
} __packed;

/* CSRs that can be accessed by AP runtime */

struct edgetpu_mailbox_cmd_queue_csr {
	u32 doorbell_set;
	u32 doorbell_status;
	u32 head;
	u32 tail;
	u32 config;
	u32 error_status;
} __packed;

struct edgetpu_mailbox_resp_queue_csr {
	u32 doorbell_set;
	u32 doorbell_clear;
	u32 doorbell_status;
	u32 head;
	u32 tail;
	u32 config;
	u32 error_status;
} __packed;

/* Mailbox APIs */

/* to specify the operation is toward cmd or resp queue */
enum mailbox_queue_type {
	MAILBOX_CMD_QUEUE,
	MAILBOX_RESP_QUEUE
};

/*
 * Allocates the mailbox manager.
 *
 * Allocations are device-managed so no release function is needed to free the
 * manager.
 */
struct edgetpu_mailbox_manager *edgetpu_mailbox_create_mgr(
		struct edgetpu_dev *etdev,
		const struct edgetpu_mailbox_manager_desc *desc);

/* interrupt handler */
irqreturn_t edgetpu_mailbox_handle_irq(struct edgetpu_mailbox_manager *mgr);

/* removes the mailbox previously requested from a mailbox manager */
int edgetpu_mailbox_remove(struct edgetpu_mailbox_manager *mgr,
			   struct edgetpu_mailbox *mailbox);
/* removes all the mailboxes previously requested */
void edgetpu_mailbox_remove_all(struct edgetpu_mailbox_manager *mgr);

/* configure mailbox */

/* set cmd/resp_queue's address and size */
int edgetpu_mailbox_set_queue(struct edgetpu_mailbox *mailbox,
			      enum mailbox_queue_type type, u64 addr, u32 size);
void edgetpu_mailbox_set_priority(struct edgetpu_mailbox *mailbox,
				  u32 priority);

/* Reset mailbox queues, clear out any commands/responses left from before. */
void edgetpu_mailbox_reset(struct edgetpu_mailbox *mailbox);

/*
 * Clears any stale doorbell requests and enables the doorbell interrupts
 * at the mailbox level
 */
void edgetpu_mailbox_init_doorbells(struct edgetpu_mailbox *mailbox);

/* utility functions for KCI */

/* requests the mailbox for KCI */
struct edgetpu_mailbox *edgetpu_mailbox_kci(
		struct edgetpu_mailbox_manager *mgr);
void edgetpu_mailbox_inc_cmd_queue_tail(struct edgetpu_mailbox *mailbox,
					u32 inc);
void edgetpu_mailbox_inc_resp_queue_head(struct edgetpu_mailbox *mailbox,
					 u32 inc);

/* utility functions for VII */

/*
 * Request the mailbox with mailbox_id equals @id.
 * @id = 0 means there is no preference, @mgr will return a spare mailbox.
 *
 * -EBUSY is returned if the requested @id is used or there is no mailbox
 * available.
 */
struct edgetpu_mailbox *
edgetpu_mailbox_vii_add(struct edgetpu_mailbox_manager *mgr, uint id);
int edgetpu_mailbox_init_vii(struct edgetpu_vii *vii,
			     struct edgetpu_device_group *group,
			     const struct edgetpu_mailbox_attr *attr);
void edgetpu_mailbox_remove_vii(struct edgetpu_vii *vii);


/*
 * Reset VII mailboxes CSRs to valid values, needed after the device is power
 * gated.
 */
void edgetpu_mailbox_reset_vii(struct edgetpu_mailbox_manager *mgr);


/* For VII and P2P mailboxes to allocate/free queue memory */

int edgetpu_mailbox_alloc_queue(struct edgetpu_dev *etdev,
				struct edgetpu_mailbox *mailbox, u32 queue_size,
				u32 unit, enum mailbox_queue_type type,
				edgetpu_queue_mem *mem);
void edgetpu_mailbox_free_queue(struct edgetpu_dev *etdev,
				struct edgetpu_mailbox *mailbox,
				edgetpu_queue_mem *mem);

/*
 * Re-configure VII mailbox queues which have an active client, re-using
 * existing buffers
 */
void edgetpu_mailbox_restore_active_vii_queues(struct edgetpu_dev *etdev);

/* Return context ID for mailbox. */
static inline enum edgetpu_context_id
edgetpu_mailbox_context_id(struct edgetpu_mailbox *mailbox)
{
	return EDGETPU_CONTEXT_VII_BASE + mailbox->mailbox_id - 1;
}

/* utility functions for P2P */

int edgetpu_mailbox_p2p_batch(struct edgetpu_mailbox_manager *mgr, uint n,
			      uint skip_i, struct edgetpu_mailbox **mailboxes);

/* Utilities of circular queue operations */

/*
 * Returns the number of elements in a circular queue given its @head, @tail,
 * and @queue_size.
 */
static inline u32 circular_queue_count(u32 head, u32 tail, u32 queue_size)
{
	if (CIRCULAR_QUEUE_WRAPPED(tail) != CIRCULAR_QUEUE_WRAPPED(head))
		return queue_size - CIRCULAR_QUEUE_REAL_INDEX(head) +
		       CIRCULAR_QUEUE_REAL_INDEX(tail);
	else
		return tail - head;
}

/* Increases @index of a circular queue by @inc. */
static inline u32 circular_queue_inc(u32 index, u32 inc, u32 queue_size)
{
	u32 new_index = CIRCULAR_QUEUE_REAL_INDEX(index) + inc;

	if (unlikely(new_index >= queue_size))
		return (index + inc - queue_size) ^ CIRCULAR_QUEUE_WRAP_BIT;
	else
		return index + inc;
}

/* Macros for accessing mailbox CSRs. */

/* Read mailbox register with no memory barrier / access ordering guarantee. */
#define EDGETPU_MAILBOX_READ(mailbox, base, type, field) \
	edgetpu_dev_read_32(mailbox->etdev, base + offsetof(type, field))

/*
 * Read mailbox register with memory barrier, ensuring the register read
 * completes prior to any following CPU reads by this thread.
 */
#define EDGETPU_MAILBOX_READ_SYNC(mailbox, base, type, field) \
	edgetpu_dev_read_32_sync(mailbox->etdev, base + offsetof(type, field))

#define EDGETPU_MAILBOX_CONTEXT_READ(mailbox, field) \
	EDGETPU_MAILBOX_READ(mailbox, mailbox->context_csr_base, \
			     struct edgetpu_mailbox_context_csr, field)

#define EDGETPU_MAILBOX_CMD_QUEUE_READ(mailbox, field) \
	EDGETPU_MAILBOX_READ(mailbox, mailbox->cmd_queue_csr_base, \
			     struct edgetpu_mailbox_cmd_queue_csr, field)

/* Read response queue register, no memory barrier / access ordering. */
#define EDGETPU_MAILBOX_RESP_QUEUE_READ(mailbox, field) \
	EDGETPU_MAILBOX_READ(mailbox, mailbox->resp_queue_csr_base, \
			     struct edgetpu_mailbox_resp_queue_csr, field)

/* Read response queue register with memory barrier. */
#define EDGETPU_MAILBOX_RESP_QUEUE_READ_SYNC(mailbox, field) \
	EDGETPU_MAILBOX_READ_SYNC(mailbox, mailbox->resp_queue_csr_base, \
			     struct edgetpu_mailbox_resp_queue_csr, field)

/* Write mailbox register with no memory barrier / access ordering guarantee. */
#define EDGETPU_MAILBOX_WRITE(mailbox, base, type, field, value) \
	edgetpu_dev_write_32(mailbox->etdev, base + offsetof(type, field), \
			     value)

/*
 * Write mailbox register with memory barrier, ensuring all CPU memory writes
 * by this thread complete prior to the register write.
 */
#define EDGETPU_MAILBOX_WRITE_SYNC(mailbox, base, type, field, value) \
	edgetpu_dev_write_32_sync(mailbox->etdev, \
				  base + offsetof(type, field), \
				  value)

/* Write context register with no memory barrier / access ordering. */
#define EDGETPU_MAILBOX_CONTEXT_WRITE(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE(mailbox, mailbox->context_csr_base, \
			      struct edgetpu_mailbox_context_csr, field, value)

/* Write context register with memory barrier. */
#define EDGETPU_MAILBOX_CONTEXT_WRITE_SYNC(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE_SYNC(mailbox, mailbox->context_csr_base, \
				   struct edgetpu_mailbox_context_csr, field, \
				   value)

/* Write command queue register with no memory barrier / access ordering. */
#define EDGETPU_MAILBOX_CMD_QUEUE_WRITE(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE(mailbox, mailbox->cmd_queue_csr_base, \
			      struct edgetpu_mailbox_cmd_queue_csr, field, \
			      value)

/* Write command queue register with memory barrier. */
#define EDGETPU_MAILBOX_CMD_QUEUE_WRITE_SYNC(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE_SYNC(mailbox, mailbox->cmd_queue_csr_base, \
				   struct edgetpu_mailbox_cmd_queue_csr, \
				   field, value)

#define EDGETPU_MAILBOX_RESP_QUEUE_WRITE(mailbox, field, value) \
	EDGETPU_MAILBOX_WRITE(mailbox, mailbox->resp_queue_csr_base, \
			      struct edgetpu_mailbox_resp_queue_csr, field, \
			      value)

#endif /* __EDGETPU_MAILBOX_H__ */
