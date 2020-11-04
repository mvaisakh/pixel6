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
/* Offset and mask to set the PBHA bits of IOMMU mappings */
#define EDGETPU_MAP_ATTR_PBHA_SHIFT	5
#define EDGETPU_MAP_ATTR_PBHA_MASK	0xf

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
	 *   [8:5]   - Value of PBHA bits for IOMMU mappings. For Abrolhos only.
	 *   [31:9]  - RESERVED
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
	_IOW(EDGETPU_IOCTL_BASE, 4, struct edgetpu_map_ioctl)

#define EDGETPU_UNMAP_BUFFER_COMPAT \
	_IOR(EDGETPU_IOCTL_BASE, 4, struct edgetpu_map_ioctl)

/*
 * Event types for which device group eventfds can be registered
 * for notifications.
 */
#define EDGETPU_EVENT_RESPDATA		0
#define EDGETPU_EVENT_FATAL_ERROR	1

struct edgetpu_event_register {
	__u32 event_id;
	__u32 eventfd;
};

/* Set eventfd for notification of events from kernel to the device group. */
#define EDGETPU_SET_EVENTFD \
	_IOW(EDGETPU_IOCTL_BASE, 5, struct edgetpu_event_register)

#define EDGETPU_SET_EVENTFD_COMPAT \
	_IOR(EDGETPU_IOCTL_BASE, 5, struct edgetpu_event_register)

struct edgetpu_mailbox_attr {
	/*
	 * There are limitations on these size fields, see the error cases in
	 * EDGETPU_CREATE_GROUP.
	 */

	__u32 cmd_queue_size; /* size of command queue in KB */
	__u32 resp_queue_size; /* size of response queue in KB */
	__u32 sizeof_cmd; /* size of command element in bytes */
	__u32 sizeof_resp; /* size of response element in bytes */
	__u32 priority          : 4; /* mailbox service priority */
	__u32 cmdq_tail_doorbell: 1; /* auto doorbell on cmd queue tail move */
};

/*
 * Create a new device group with the caller as the master.
 *
 * EINVAL: If the caller already belongs to a group.
 * EINVAL: If @cmd/resp_queue_size equals 0.
 * EINVAL: If @sizeof_cmd/resp equals 0.
 * EINVAL: If @cmd_queue_size * 1024 / @sizeof_cmd >= 1024, this is a hardware
 *         limitation. Same rule for the response sizes pair.
 */
#define EDGETPU_CREATE_GROUP \
	_IOW(EDGETPU_IOCTL_BASE, 6, struct edgetpu_mailbox_attr)

#define EDGETPU_CREATE_GROUP_COMPAT_2 \
	_IOR(EDGETPU_IOCTL_BASE, 6, struct edgetpu_mailbox_attr)

/* Join the calling fd to the device group of the supplied fd. */
#define EDGETPU_JOIN_GROUP \
	_IOW(EDGETPU_IOCTL_BASE, 7, __u32)

#define EDGETPU_JOIN_GROUP_COMPAT \
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
	_IOW(EDGETPU_IOCTL_BASE, 9, struct edgetpu_event_register)

#define EDGETPU_SET_PERDIE_EVENTFD_COMPAT \
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
	_IOW(EDGETPU_IOCTL_BASE, 14, __u32)

#define EDGETPU_UNSET_EVENT_COMPAT \
	_IOR(EDGETPU_IOCTL_BASE, 14, __u32)

/* Unset event by event_id registered with EDGETPU_SET_PERDIE_EVENTFD. */
#define EDGETPU_UNSET_PERDIE_EVENT \
	_IOW(EDGETPU_IOCTL_BASE, 15, __u32)

#define EDGETPU_UNSET_PERDIE_EVENT_COMPAT \
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
	_IOW(EDGETPU_IOCTL_BASE, 16, struct edgetpu_sync_ioctl)

#define EDGETPU_SYNC_BUFFER_COMPAT \
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
	 * Note: the SKIP_CPU_SYNC and PBHA flags are ignored, DMA flags to be
	 * used is controlled by the dma-buf exporter.
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
 */
#define EDGETPU_MAP_DMABUF \
	_IOWR(EDGETPU_IOCTL_BASE, 17, struct edgetpu_map_dmabuf_ioctl)
/*
 * Un-map address previously mapped by EDGETPU_MAP_DMABUF.
 *
 * Only fields @die_index and @device_address in the third argument will be
 * used, other fields such as @size and @offset will be fetched from the
 * kernel's internal records. If the buffer was requested as
 * EDGETPU_MAP_MIRRORED, @die_index is ignored as well.
 *
 * EINVAL: If @device_address is not found.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_UNMAP_DMABUF \
	_IOW(EDGETPU_IOCTL_BASE, 18, struct edgetpu_map_dmabuf_ioctl)

#define EDGETPU_UNMAP_DMABUF_COMPAT \
	_IOR(EDGETPU_IOCTL_BASE, 18, struct edgetpu_map_dmabuf_ioctl)

/*
 * Allocate device buffer of provided @size(__u64) and
 * return a dma-buf FD on success.
 *
 * EINVAL: If @size is zero.
 * ENODEV: If the on-device DRAM is not supported or failed on initialization.
 * ENOTTY: If config EDGETPU_DEVICE_DRAM is disabled.
 */
#define EDGETPU_ALLOCATE_DEVICE_BUFFER \
	_IOW(EDGETPU_IOCTL_BASE, 19, __u64)

#define EDGETPU_ALLOCATE_DEVICE_BUFFER_COMPAT_2 \
	_IOR(EDGETPU_IOCTL_BASE, 19, __u64)

/*
 * struct edgetpu_create_sync_fence_data
 * @seqno:		the seqno to initialize the fence with
 * @timeline_name:	the name of the timeline the fence belongs to
 * @fence:		returns the fd of the new sync_file with the new fence
 *
 * Timeline names can be up to 128 characters (including trailing NUL byte)
 * for edgetpu debugfs and kernel debug logs.  These names are truncated to
 * 32 characters in the data returned by the standard SYNC_IOC_FILE_INFO
 * ioctl.
 */
#define EDGETPU_SYNC_TIMELINE_NAME_LEN	128
struct edgetpu_create_sync_fence_data {
	__u32 seqno;
	char  timeline_name[EDGETPU_SYNC_TIMELINE_NAME_LEN];
	__s32 fence;
};

/*
 * Create a DMA sync fence, return the sync_file fd for the new fence.
 */
#define EDGETPU_CREATE_SYNC_FENCE \
	_IOWR(EDGETPU_IOCTL_BASE, 20, struct edgetpu_create_sync_fence_data)

/*
 * struct edgetpu_signal_sync_fence_data
 * @fence:		fd of the sync_file for the fence
 * @error:		error status errno value or zero for success
 */
struct edgetpu_signal_sync_fence_data {
	__s32 fence;
	__s32 error;
};

/*
 * Signal a DMA sync fence with optional error status.
 * Can pass a sync_file fd created by any driver.
 * Signals the first DMA sync fence in the sync file.
 */
#define EDGETPU_SIGNAL_SYNC_FENCE \
	_IOW(EDGETPU_IOCTL_BASE, 21, struct edgetpu_signal_sync_fence_data)

#define EDGETPU_SIGNAL_SYNC_FENCE_COMPAT \
	_IOR(EDGETPU_IOCTL_BASE, 21, struct edgetpu_signal_sync_fence_data)

#define EDGETPU_IGNORE_FD (-1)
#define EDGETPU_MAX_NUM_DEVICES_IN_GROUP 36
struct edgetpu_map_bulk_dmabuf_ioctl {
	__u64 size; /* Size to be mapped in bytes. */
	__u64 device_address; /* returned TPU VA */
	/*
	 * Same format as edgetpu_map_dmabuf_ioctl.flags, except:
	 *   - [2:2] Mirroredness is ignored.
	 */
	edgetpu_map_flag_t flags;
	/*
	 * The list of file descriptors backed by dma-buf.
	 *
	 * The first FD will be mapped to the first device in the target group
	 * (i.e. the master die); the second FD will be mapped to the second
	 * device and so on.
	 * Only the first N FDs will be used, where N is the number of devices
	 * in the group.
	 *
	 * Use EDGETPU_IGNORE_FD if it's not required to map on specific
	 * device(s). For example, if one passes {fd0, EDGETPU_IGNORE_FD, fd2}
	 * to this field for mapping a group with 3 devices, only the first
	 * device and the third device has the mapping on @device_address.
	 */
	__s32 dmabuf_fds[EDGETPU_MAX_NUM_DEVICES_IN_GROUP];
};

/*
 * Map a list of dma-buf FDs to devices in the group.
 *
 * On success, @device_address is set and the syscall returns zero.
 *
 * EINVAL: If @size is zero.
 * EINVAL: If the target device group is not finalized.
 * EINVAL: If any file descriptor is not backed by dma-buf.
 * EINVAL: If @size exceeds the size of any buffer.
 * EINVAL: If all file descriptors are EDGETPU_IGNORE_FD.
 */
#define EDGETPU_MAP_BULK_DMABUF \
	_IOWR(EDGETPU_IOCTL_BASE, 22, struct edgetpu_map_bulk_dmabuf_ioctl)
/*
 * Un-map address previously mapped by EDGETPU_MAP_BULK_DMABUF.
 *
 * Only field @device_address in the third argument is used, other fields such
 * as @size will be fetched from the kernel's internal records.
 *
 * EINVAL: If @device_address is not found.
 * EINVAL: If the target device group is disbanded.
 */
#define EDGETPU_UNMAP_BULK_DMABUF \
	_IOW(EDGETPU_IOCTL_BASE, 23, struct edgetpu_map_bulk_dmabuf_ioctl)

#define EDGETPU_UNMAP_BULK_DMABUF_COMPAT \
	_IOR(EDGETPU_IOCTL_BASE, 23, struct edgetpu_map_bulk_dmabuf_ioctl)

/*
 * struct edgetpu_sync_fence_status
 * @fence:		fd of the sync_file for the fence
 * @status:		returns:
 *			   0 if active
 *			   1 if signaled with no error
 *			   negative errno value if signaled with error
 */
struct edgetpu_sync_fence_status {
	__s32 fence;
	__s32 status;
};

/*
 * Retrieve DMA sync fence status.
 * Can pass a sync_file fd created by any driver.
 * Returns the status of the first DMA sync fence in the sync file.
 */
#define EDGETPU_SYNC_FENCE_STATUS \
	_IOWR(EDGETPU_IOCTL_BASE, 24, struct edgetpu_sync_fence_status)

/*
 * Release the current client's wakelock, allowing firmware to be shut down if
 * no other clients are active.
 * Groups and buffer mappings are preserved.
 * WARNING: Attempts to access any mapped CSRs before re-acquiring the wakelock
 * may crash the system.
 */
#define EDGETPU_RELEASE_WAKE_LOCK	_IO(EDGETPU_IOCTL_BASE, 25)

/*
 * Acquire the wakelock for this client, ensures firmware keeps running.
 */
#define EDGETPU_ACQUIRE_WAKE_LOCK	_IO(EDGETPU_IOCTL_BASE, 26)

struct edgetpu_fw_version {
	__u32 major_version; /* Returned firmware major version number */
	__u32 minor_version; /* Returned firmware minor version number */
	__u32 vii_version; /* Returned firmware VII version number */
	__u32 kci_version; /* Returned firmware KCI version number */
};

/*
 * Query the version information of the firmware currently loaded.
 *
 * When there is an attempt to load firmware, its version numbers are recorded
 * by the kernel and will be returned on the following EDGETPU_FIRMWARE_VERSION
 * calls. If the latest firmware attempted to load didn't exist or had an
 * invalid header, this call returns -ENODEV.
 *
 * Returns 0 on success, -errno on error.
 */
#define EDGETPU_FIRMWARE_VERSION \
	_IOR(EDGETPU_IOCTL_BASE, 27, struct edgetpu_fw_version)

#endif /* __EDGETPU_H__ */
