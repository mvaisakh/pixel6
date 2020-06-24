/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU kernel-userspace interface definitions.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_H__
#define __EDGETPU_H__

#include <linux/ioctl.h>
#include <linux/types.h>

/* mmap offsets for mailbox CSRs, command queue, and response queue */
#define EDGETPU_MMAP_CSR_OFFSET 0x1800000
#define EDGETPU_MMAP_CMD_QUEUE_OFFSET 0x1900000
#define EDGETPU_MMAP_RESP_QUEUE_OFFSET 0x1A00000
/* mmap offsets for logging and tracing buffers */
#define EDGETPU_MMAP_LOG_BUFFER_OFFSET 0x1B00000
#define EDGETPU_MMAP_TRACE_BUFFER_OFFSET 0x1C00000

/* EdgeTPU map flag macros */

typedef __u32 edgetpu_map_flag_t;
/* The mask for specifying DMA direction in EdgeTPU map flag */
#define EDGETPU_MAP_DIR_MASK		3
/* The targeted DMA direction for the buffer */
#define EDGETPU_MAP_DMA_BIDIRECTIONAL   0
#define EDGETPU_MAP_DMA_TO_DEVICE       1
#define EDGETPU_MAP_DMA_FROM_DEVICE     2
#define EDGETPU_MAP_DMA_NONE            3
/* The address is mapped to all dies in a device group */
#define EDGETPU_MAP_MIRRORED		(0u << 2)
/* The address is mapped on the specific die */
#define EDGETPU_MAP_NONMIRRORED		(1u << 2)
/* The TPU address must be accessible to the TPU CPU */
#define EDGETPU_MAP_CPU_ACCESSIBLE	(0u << 3)
#define EDGETPU_MAP_CPU_NONACCESSIBLE	(1u << 3)
/* Skip CPU sync on unmap */
#define EDGETPU_MAP_SKIP_CPU_SYNC	(1u << 4)

struct edgetpu_map_ioctl {
	__u64 host_address;
	__u64 size;		/* size of mapping in bytes */
	__u64 device_address;	/* returned TPU VA */
	/*
	 * Flags indicating mapping attribute requests from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [1:0]   - DMA_DIRECTION:
	 *               00 = DMA_BIDIRECTIONAL
	 *               01 = DMA_TO_DEVICE
	 *               10 = DMA_FROM_DEVICE
	 *               11 = DMA_NONE
	 *   [2:2]   - Mirroredness. Mirrored across device group or local to a
	 *             specific die:
	 *               0 = map to all dies in a device group
	 *               1 = map to the @die_index-th die of the device group
	 *   [3:3]   - If the TPU address must be accessible to the TPU CPU:
	 *               0 = yes, returned @device_address must be within the
	 *                   address range addressable by the TPU CPU
	 *               1 = no, returned @device_address can be outside the TPU
	 *                   CPU-addressable range
	 *             Note: this flag may be ignored if the TPU chip does not
	 *             have the capability to internally map memory outside the
	 *             CPU-addressable range.
	 *   [4:4]   - Skip cache invalidation on unmap.
	 *               0 = Don't skip CPU sync. Default DMA API behavior.
	 *               1 = Skip CPU sync.
	 *             Note: This bit is ignored on the map call.
	 *   [31:5]  - RESERVED
	 */
	edgetpu_map_flag_t flags;
	/*
	 * Index of die in a device group. The index is decided by the order of
	 * joining the group, with value from zero to (# dies in group) - 1.
	 * Index 0 for the master die in the group.
	 *
	 * This field is ignored unless EDGETPU_MAP_NONMIRRORED is passed to
	 * flags.
	 */
	__u32 die_index;
};

#define EDGETPU_IOCTL_BASE 0xED

/* Map host buffer to TPU. */
#define EDGETPU_MAP_BUFFER \
	_IOWR(EDGETPU_IOCTL_BASE, 0, struct edgetpu_map_ioctl)

/*
 * Un-map host buffer from TPU previously mapped by EDGETPU_MAP_BUFFER.
 *
 * Only fields @device_address, @die_index, and @flags (see Note) in the third
 * argument will be used, other fields will be fetched from the kernel's
 * internal records. It is recommended to use the argument that was passed in
 * EDGETPU_MAP_BUFFER to un-map the buffer.
 *
 * Note: Only the SKIP_CPU_SYNC flag is considered, other bits in @flags are
 * fetched from the kernel's record.
 */
#define EDGETPU_UNMAP_BUFFER \
	_IOR(EDGETPU_IOCTL_BASE, 4, struct edgetpu_map_ioctl)

/*
 * Event types for which device group eventfds can be registered
 * for notifications.
 */
#define EDGETPU_EVENT_RESPDATA		0

struct edgetpu_event_register {
	__u32 event_id;
	__u32 eventfd;
};

/* Set eventfd for notification of events from kernel to the device group. */
#define EDGETPU_SET_EVENTFD \
	_IOR(EDGETPU_IOCTL_BASE, 5, struct edgetpu_event_register)

struct edgetpu_mailbox_attr {
	__u32 cmd_queue_size    : 10; /* size of cmd queue in KB */
	__u32 resp_queue_size   : 10; /* size of response queue in KB */
	__u32 priority          :  4; /* mailbox service priority */
	__u32 cmdq_tail_doorbell:  1; /* auto doorbell on cmd queue tail move */
};

/* Create a new device group with the caller as the master. */
#define EDGETPU_CREATE_GROUP \
	_IOR(EDGETPU_IOCTL_BASE, 6, struct edgetpu_mailbox_attr)

/* Join the calling fd to the device group of the supplied fd. */
#define EDGETPU_JOIN_GROUP \
	_IOR(EDGETPU_IOCTL_BASE, 7, __u32)

/* Finalize the device group with the caller as the master. */
#define EDGETPU_FINALIZE_GROUP \
	_IO(EDGETPU_IOCTL_BASE, 8)

/*
 * Event types for which per-die eventfds can be registered for
 * notifications.
 */
#define EDGETPU_PERDIE_EVENT_LOGS_AVAILABLE		0x1000
#define EDGETPU_PERDIE_EVENT_TRACES_AVAILABLE		0x1001

/* Set eventfd for notification of per-die events from kernel. */
#define EDGETPU_SET_PERDIE_EVENTFD \
	_IOR(EDGETPU_IOCTL_BASE, 9, struct edgetpu_event_register)

struct edgetpu_device_buffer_ioctl {
	/*
	 * Buffer size to be allocated in bytes.
	 *
	 * The size will be aligned with the page size.
	 */
	__u64 size;
	/*
	 * The buffer will be allocated on the @die_index-th die's device DRAM.
	 * The index is decided by the order of joining the group, with value
	 * from zero to (# dies in group) - 1.
	 */
	__u32 die_index;
};

/*
 * Deprecated, use EDGETPU_ALLOCATE_DEVICE_BUFFER.
 *
 * Return a dma-buf FD on success.
 *
 * EINVAL: If @size is zero.
 * EINVAL: If @die_index is greater than or equal to the number of dies in the
 *         device group.
 * EINVAL: If the target device group is not finalized.
 * ENODEV: If the on-device DRAM is not supported or failed on initialization.
 */
#define EDGETPU_ALLOCATE_DEVICE_BUFFER_COMPAT \
	_IOR(EDGETPU_IOCTL_BASE, 11, struct edgetpu_device_buffer_ioctl)

/* Unset event by event_id registered with EDGETPU_SET_EVENTFD. */
#define EDGETPU_UNSET_EVENT \
	_IOR(EDGETPU_IOCTL_BASE, 14, __u32)

/* Unset event by event_id registered with EDGETPU_SET_PERDIE_EVENTFD. */
#define EDGETPU_UNSET_PERDIE_EVENT \
	_IOR(EDGETPU_IOCTL_BASE, 15, __u32)

#define EDGETPU_SYNC_FOR_DEVICE		(0 << 2)
#define EDGETPU_SYNC_FOR_CPU		(1 << 2)

struct edgetpu_sync_ioctl {
	/*
	 * The starting address of the buffer to be synchronized. Must be a
	 * device address returned by EDGETPU_MAP_BUFFER.
	 */
	__u64 device_address;
	/* size in bytes to be sync'ed */
	__u64 size;
	/*
	 * offset in bytes at which the sync operation is to begin from the
	 * start of the buffer
	 */
	__u64 offset;
	/*
	 * The die index passed to EDGETPU_MAP_BUFFER if it was an
	 * EDGETPU_MAP_NONMIRRORED request, otherwise this field is ignored.
	 */
	__u32 die_index;
	/*
	 * Flags indicating sync operation requested from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [1:0]   - DMA_DIRECTION:
	 *               00 = DMA_BIDIRECTIONAL
	 *               01 = DMA_TO_DEVICE
	 *               10 = DMA_FROM_DEVICE
	 *               11 = DMA_NONE
	 *   [2:2]   - Sync direction. Sync for device or CPU.
	 *               0 = sync for device
	 *               1 = sync for CPU
	 *   [31:3]  - RESERVED
	 */
	__u32 flags;
};

/*
 * Sync the buffer previously mapped by EDGETPU_MAP_BUFFER.
 *
 * EINVAL: If a mapping for @device_address is not found.
 * EINVAL: If @size equals 0.
 * EINVAL: If @offset plus @size exceeds the mapping size.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_SYNC_BUFFER \
	_IOR(EDGETPU_IOCTL_BASE, 16, struct edgetpu_sync_ioctl)

struct edgetpu_map_dmabuf_ioctl {
	/*
	 * Offset within the dma-buf to be mapped in bytes.
	 *
	 * Must be page-aligned.
	 */
	__u64 offset;
	/* Size to be mapped in bytes. */
	__u64 size;
	/*
	 * Returned TPU VA.
	 *
	 * The address is page-aligned.
	 */
	__u64 device_address;
	/* A dma-buf FD. */
	int dmabuf_fd;
	/*
	 * Flags indicating mapping attributes. See edgetpu_map_ioctl.flags for
	 * details.
	 *
	 * Note: the SKIP_CPU_SYNC flag is ignored, the behavior of
	 * synchronization on unmap is controlled by the dma-buf exporter.
	 */
	edgetpu_map_flag_t flags;
	/*
	 * Index of die in a device group. See edgetpu_map_ioctl.die_index for
	 * details.
	 */
	__u32 die_index;
};

/*
 * Map the dma-buf FD to TPU.
 *
 * On success, @device_address is set and the syscall returns zero.
 *
 * EINVAL: If @offset is not page-aligned.
 * EINVAL: If @size is zero.
 * EINVAL: If @die_index exceeds the number of clients in the group.
 * EINVAL: If the target device group is disbanded.
 * ENOTTY: If DMA_SHARED_BUFFER is not enabled.
 */
#define EDGETPU_MAP_DMABUF \
	_IOWR(EDGETPU_IOCTL_BASE, 17, struct edgetpu_map_dmabuf_ioctl)
/*
 * Un-map address previously mapped by EDGETPU_MAP_DEVICE_BUFFER.
 *
 * Only fields @dmabuf_fd, @die_index, and @device_address in the third argument
 * will be used, other fields such as @size and @offset will be fetched from the
 * kernel's internal records. If the buffer was requested as
 * EDGETPU_MAP_MIRRORED, @die_index is ignored as well.
 *
 * EINVAL: If @device_address is not found.
 * EINVAL: If the target device group is disbanded.
 * ENOTTY: If DMA_SHARED_BUFFER is not enabled.
 */
#define EDGETPU_UNMAP_DMABUF \
	_IOR(EDGETPU_IOCTL_BASE, 18, struct edgetpu_map_dmabuf_ioctl)

/*
 * Allocate device buffer of provided @size(__u64) and
 * return a dma-buf FD on success.
 *
 * EINVAL: If @size is zero.
 * ENODEV: If the on-device DRAM is not supported or failed on initialization.
 */
#define EDGETPU_ALLOCATE_DEVICE_BUFFER \
	_IOR(EDGETPU_IOCTL_BASE, 19, __u64)

#endif /* __EDGETPU_H__ */
