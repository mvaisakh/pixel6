/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Kernel Control Interface, implements the protocol between AP kernel and TPU
 * firmware.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_KCI_H__
#define __EDGETPU_KCI_H__

#include <linux/dma-direction.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "edgetpu-internal.h"
#include "edgetpu-mailbox.h"

/*
 * The status field in a firmware response is set to this by us when the
 * response is fetched from the queue.
 */
#define KCI_STATUS_OK			(0)
/*
 * edgetpu_kci#wait_list uses this value to record the status of responses
 * that haven't been received yet.
 */
#define KCI_STATUS_WAITING_RESPONSE	(1)
/*
 * Used when an expected response is not received, see the documentation of
 * edgetpu_kci_consume_wait_list() for details.
 *
 */
#define KCI_STATUS_NO_RESPONSE		(2)

/* command/response queue elements for KCI */

struct edgetpu_dma_descriptor {
	u64 address;
	u32 size;
	u32 flags;
};

struct edgetpu_command_element {
	u64 seq;	 /* set by edgetpu_kci_push_cmd() */
	u16 code;
	u16 reserved[3]; /* explicit padding, does not affect alignment */
	struct edgetpu_dma_descriptor dma;
};

struct edgetpu_kci_response_element {
	u64 seq;
	u16 code;
	/*
	 * Reserved on firmware side - they can't touch this.
	 * If a value is written here it will be discarded and overwritten
	 * during response processing
	 */
	u16 status;
	/*
	 * Return value is not currently needed by KCI, but firmware may set
	 * this to a watermark value to aid in debugging
	 */
	u32 retval;
} __packed;

/* VII response element */
/* The size of this structure must match the runtime definition. */
struct edgetpu_vii_response_element {
	u64 seq;
	u16 code;
	u8 reserved[6];	/* padding */
	u64 retval;
} __packed;

/*
 * Definition of code in command elements.
 * Code for KCI is a 16-bit unsigned integer.
 */
enum edgetpu_kci_code {
	KCI_CODE_ACK = 0,
	KCI_CODE_UNMAP_BUFFER = 1,
	KCI_CODE_MAP_LOG_BUFFER = 2,
	KCI_CODE_JOIN_GROUP = 3,
	KCI_CODE_LEAVE_GROUP = 4,
	KCI_CODE_MAP_TRACE_BUFFER = 5,
	KCI_CODE_FIRMWARE_FLAVOR = 6,
	KCI_CODE_SHUTDOWN = 7,
};

/*
 * Definition of code in response elements.
 * It is a 16-bit unsigned integer.
 */
enum edgetpu_kci_error {
	KCI_ERROR_OK = 0, /* Not an error; returned on success */
	KCI_ERROR_CANCELLED = 1,
	KCI_ERROR_UNKNOWN = 2,
	KCI_ERROR_INVALID_ARGUMENT = 3,
	KCI_ERROR_DEADLINE_EXCEEDED = 4,
	KCI_ERROR_NOT_FOUND = 5,
	KCI_ERROR_ALREADY_EXISTS = 6,
	KCI_ERROR_PERMISSION_DENIED = 7,
	KCI_ERROR_RESOURCE_EXHAUSTED = 8,
	KCI_ERROR_FAILED_PRECONDITION = 9,
	KCI_ERROR_ABORTED = 10,
	KCI_ERROR_OUT_OF_RANGE = 11,
	KCI_ERROR_UNIMPLEMENTED = 12,
	KCI_ERROR_INTERNAL = 13,
	KCI_ERROR_UNAVAILABLE = 14,
	KCI_ERROR_DATA_LOSS = 15,
	KCI_ERROR_UNAUTHENTICATED = 16,
};

/* Firmware flavors returned via KCI from firmware image. */
enum edgetpu_fw_flavor {
	/* used by host when cannot determine the flavor */
	FW_FLAVOR_UNKNOWN = 0,
	/* second-stage bootloader */
	FW_FLAVOR_BL1 = 1,
	/* systest app image */
	FW_FLAVOR_SYSTEST = 2,
	/* default production app image from DarwiNN team */
	FW_FLAVOR_PROD_DEFAULT = 3,
	/* custom image produced by other teams */
	FW_FLAVOR_CUSTOM = 4,
};

struct edgetpu_kci_wait_list {
	struct list_head list;
	/*
	 * Its content will be updated once a response with sequence number
	 * equal to resp->seq is received.
	 */
	struct edgetpu_kci_response_element *resp;
};

struct edgetpu_kci {
	struct edgetpu_mailbox *mailbox;
	struct mutex mailbox_lock;	/* protects mailbox */
	u64 cur_seq;
	struct edgetpu_command_element *cmd_queue;
	struct mutex cmd_queue_lock;	/* protects cmd_queue */
	tpu_addr_t cmd_queue_tpu_addr;  /* TPU device address for cmd queue */
	dma_addr_t cmd_queue_dma_addr;  /* DMA address for cmd queue */
	struct edgetpu_kci_response_element *resp_queue;
	spinlock_t resp_queue_lock;	/* protects resp_queue */
	tpu_addr_t resp_queue_tpu_addr; /* TPU device address for resp queue */
	dma_addr_t resp_queue_dma_addr; /* DMA address for resp queue */
	/* queue for waiting for the response doorbell to be rung */
	wait_queue_head_t resp_doorbell_waitq;
	/* add to this list if a command needs to wait for a response */
	struct list_head wait_list;
	spinlock_t wait_list_lock;	/* protects wait_list */
	/* queue for waiting for the wait_list to be consumed */
	wait_queue_head_t wait_list_waitq;
	struct work_struct work;	/* worker of consuming responses */
};

struct edgetpu_kci_device_group_detail {
	u8 n_dies;
	/* virtual ID from 0 ~ n_dies - 1 */
	/* ID 0 for the group master */
	u8 vid;
	u8 reserved[6]; /* padding */
};

/*
 * Initializes a KCI object.
 *
 * Will request a mailbox from @mgr and allocate cmd/resp queues.
 */
int edgetpu_kci_init(struct edgetpu_mailbox_manager *mgr,
		     struct edgetpu_kci *kci);
/*
 * Re-initializes the initialized KCI object.
 *
 * This function is used when the TPU device is reset, it re-programs CSRs
 * related to KCI mailbox.
 *
 * Returns 0 on success, -errno on error.
 */
int edgetpu_kci_reinit(struct edgetpu_kci *kci);
/*
 * Releases resources allocated by @kci.
 *
 * Note: must invoke this function after the interrupt of mailbox disabled and
 * before free the mailbox pointer.
 */
void edgetpu_kci_release(struct edgetpu_dev *etdev, struct edgetpu_kci *kci);

/*
 * Pushes an element to cmd queue.
 *
 * @cmd's seq field will be set.
 * Will update the CMD_QUEUE_TAIL CSR.
 *
 * @resp will NOT be updated immediately, instead, it will be appended to the
 * wait_list of @kci. Once the response of @cmd is received, @resp will be
 * updated. Compare the value of resp->code with KCI_CODE_WAITING_RESPONSE to
 * check if the response is received.
 * @resp can be NULL if the command doesn't need a response.
 *
 * This is a synchronous function. If the cmd queue is full, it will wait until
 * the queue is consumed.
 *
 * Returns 0 on success, or a negative errno on error.
 */
int edgetpu_kci_push_cmd(struct edgetpu_kci *kci,
			 struct edgetpu_command_element *cmd,
			 struct edgetpu_kci_response_element *resp);

/*
 * Pushes an element to cmd queue and waits for the response.
 * Returns -ETIMEDOUT if no response is received within KCI_TIMEOUT.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_send_cmd(struct edgetpu_kci *kci,
			 struct edgetpu_command_element *cmd);

/*
 * Sends the "Unmap Buffer Sync" command and waits for remote response.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_unmap_buffer(struct edgetpu_kci *kci, tpu_addr_t tpu_addr,
			     u32 size, enum dma_data_direction dir);

/*
 * Sends an ACK command and expects a response.
 * Can be used to test the firmware is running.
 *
 * Returns 0 if successful
 */
int edgetpu_kci_ack(struct edgetpu_kci *kci);

/*
 * Sends a FIRMWARE_FLAVOR command and expects a response indicating what
 * edgetpu_fw_flavor type is running.  Also serves as an initial handshake
 * with firmware at load time.
 *
 * Returns >=0 edgetpu_fw_flavor when response received from firmware,
 *         <0 on error communicating with firmware (typically -ETIMEDOUT).
 */
enum edgetpu_fw_flavor edgetpu_kci_fw_flavor(struct edgetpu_kci *kci);

/*
 * Sends the "Map Log Buffer" command and waits for remote response.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_map_log_buffer(struct edgetpu_kci *kci, tpu_addr_t tpu_addr,
			       u32 size);

/*
 * Sends the "Map Trace Buffer" command and waits for remote response.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_map_trace_buffer(struct edgetpu_kci *kci, tpu_addr_t tpu_addr,
				 u32 size);

/*
 * Sent when a group is created with @n_dies dies, and @etdev is the @vid-th
 * die in this group.
 *
 * Returns the code of response, or a negative errno on error.
 */
int edgetpu_kci_join_group(struct edgetpu_kci *kci, struct edgetpu_dev *etdev,
			   u8 n_dies, u8 vid);
/* Informs the TPU to leave the group it currently belongs to. */
int edgetpu_kci_leave_group(struct edgetpu_kci *kci);

/* debugfs mappings dump */
void edgetpu_kci_mappings_show(struct edgetpu_dev *etdev, struct seq_file *s);

/* Send shutdown request to firmware */
int edgetpu_kci_shutdown(struct edgetpu_kci *kci);

#endif /* __EDGETPU_KCI_H__ */
