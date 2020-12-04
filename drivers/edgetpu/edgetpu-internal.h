/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU driver common internal definitions.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_INTERNAL_H__
#define __EDGETPU_INTERNAL_H__

#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "edgetpu.h"
#include "edgetpu-pm.h"
#include "edgetpu-thermal.h"

#define etdev_err(etdev, fmt, ...) dev_err((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_warn(etdev, fmt, ...)                                            \
	dev_warn((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_info(etdev, fmt, ...)                                            \
	dev_info((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_dbg(etdev, fmt, ...) dev_dbg((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_err_ratelimited(etdev, fmt, ...)                                 \
	dev_err_ratelimited((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_warn_ratelimited(etdev, fmt, ...)                                \
	dev_warn_ratelimited((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_info_ratelimited(etdev, fmt, ...)                                \
	dev_info_ratelimited((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_dbg_ratelimited(etdev, fmt, ...)                                 \
	dev_dbg_ratelimited((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_warn_once(etdev, fmt, ...)                                       \
	dev_warn_once((etdev)->etcdev, fmt, ##__VA_ARGS__)

/* The number of TPU tiles in an edgetpu chip */
#ifdef CONFIG_EDGETPU_FPGA
#define EDGETPU_NTILES	4
#else
#define EDGETPU_NTILES	16
#endif

/* Up to 7 concurrent device groups / workloads per device. */
#define EDGETPU_NGROUPS		7

/* 1 context per VII/group plus 1 for KCI */
#define EDGETPU_NCONTEXTS	(EDGETPU_NGROUPS + 1)

/*
 * Common-layer context IDs for non-secure TPU access, translated to chip-
 * specific values in the mmu driver.
 */
enum edgetpu_context_id {
	EDGETPU_CONTEXT_KCI = 0,	/* TPU firmware/kernel ID 0 */
	EDGETPU_CONTEXT_VII_BASE = 1,	/* groups 0-6 IDs 1-7 */
	/* contexts 8 and above not yet allocated */
};

typedef u64 tpu_addr_t;

struct edgetpu_coherent_mem {
	void *vaddr;		/* kernel VA, no allocation if NULL */
	dma_addr_t dma_addr;	/* DMA handle for downstream IOMMU, if any */
	tpu_addr_t tpu_addr;	/* DMA handle for TPU internal IOMMU, if any */
	u64 host_addr;		/* address mapped on host for debugging */
	size_t size;
};

struct edgetpu_reg_window {
	uint start_reg_offset;
	size_t size;
};

struct edgetpu_device_group;
struct edgetpu_p2p_csr_map;
struct edgetpu_remote_dram_map;

struct edgetpu_client {
	pid_t pid;
	pid_t tgid;
	/* Reference count */
	refcount_t count;
	/* protects group. */
	struct mutex group_lock;
	/*
	 * The virtual device group this client belongs to. Can be NULL if the
	 * client doesn't belong to any group.
	 */
	struct edgetpu_device_group *group;
	/*
	 * This client is the idx-th member of @group.
	 * It's meaningless if this client doesn't belong to a group.
	 */
	uint idx;
	/* the device opened by this client */
	struct edgetpu_dev *etdev;
	/* Peer CSR dma addrs for this client, if in a group with P2P */
	dma_addr_t *p2p_csrs_dma_addrs;
	/* Peer DRAM dma addrs for this client, if has on-device DRAM */
	dma_addr_t *remote_drams_dma_addrs;
	/* range of device CSRs mmap()'able */
	struct edgetpu_reg_window reg_window;
	/* Per-client request to keep device active */
	struct {
		struct mutex lock;
		uint req_count;
		uint csr_map_count;
	} wakelock;
};

struct edgetpu_mapping;
struct edgetpu_mailbox_manager;
struct edgetpu_kci;
struct edgetpu_telemetry_ctx;
struct edgetpu_mempool;

#define EDGETPU_DEVICE_NAME_MAX	64

/* ioremapped resource */
struct edgetpu_mapped_resource {
	void __iomem *mem;	/* starting virtual address */
	phys_addr_t phys;	/* starting physical address */
	resource_size_t size;	/* size in bytes */
};

enum edgetpu_dev_state {
	ETDEV_STATE_NOFW = 0,	/* no firmware running on device. */
	ETDEV_STATE_GOOD = 1,	/* healthy firmware running. */
	ETDEV_STATE_FWLOADING = 2, /* firmware is getting loaded on device. */
	ETDEV_STATE_BAD = 3,	/* firmware/device is in unusable state. */
};

/* a mark to know whether we read valid versions from the firmware header */
#define EDGETPU_INVALID_KCI_VERSION (~0u)

struct edgetpu_dev {
	struct device *dev;	   /* platform/pci bus device */
	struct device *etcdev;	   /* edgetpu class char device */
	struct cdev cdev;	   /* cdev char device structure */
	dev_t devno;		   /* char device dev_t */
	char dev_name[EDGETPU_DEVICE_NAME_MAX];
	struct {
		struct mutex lock;
		uint count;	   /* number times device currently opened */
	} open;
	struct edgetpu_mapped_resource regs; /* ioremapped CSRs */
	struct dentry *d_entry;    /* debugfs dir for this device */
	struct mutex state_lock;   /* protects state of this device */
	enum edgetpu_dev_state state;
	struct mutex groups_lock;  /* protects groups and lockout */
	struct edgetpu_device_group *groups[EDGETPU_NGROUPS];
	bool group_join_lockout;   /* disable group join while reinit */
	void *mmu_cookie;	   /* mmu driver private data */
	void *dram_cookie;	   /* on-device DRAM private data */
	struct edgetpu_mailbox_manager *mailbox_manager;
	struct edgetpu_kci *kci;
	struct edgetpu_firmware *firmware; /* firmware management */
	struct edgetpu_telemetry_ctx *telemetry;
	struct edgetpu_thermal *thermal;
	struct edgetpu_pm *pm;  /* Power management interface */
	/* Memory pool in instruction remap region */
	struct edgetpu_mempool *iremap_pool;
	int mcp_id;		/* multichip pkg id, or -1 for none */
	uint mcp_die_index;	/* physical die index w/in multichip pkg */
	u8 mcp_pkg_type;	/* multichip pkg type */
	struct edgetpu_sw_wdt *etdev_sw_wdt;	/* software watchdog */
	/* version read from the firmware binary file */
	struct edgetpu_fw_version fw_version;
	atomic_t job_count;	/* times joined to a device group */
	struct edgetpu_coherent_mem debug_dump_mem;	/* debug dump memory */
};

extern const struct file_operations edgetpu_fops;

/* Status regs dump. */
struct edgetpu_dumpregs_range {
	u32 firstreg;
	u32 lastreg;
};
extern struct edgetpu_dumpregs_range edgetpu_chip_statusregs_ranges[];
extern int edgetpu_chip_statusregs_nranges;
extern struct edgetpu_dumpregs_range edgetpu_chip_tile_statusregs_ranges[];
extern int edgetpu_chip_tile_statusregs_nranges;

static inline const char *edgetpu_dma_dir_rw_s(enum dma_data_direction dir)
{
	static const char *tbl[4] = { "rw", "r", "w", "?" };

	return tbl[dir];
}

/* edgetpu device IO functions */

static inline u32 edgetpu_dev_read_32(struct edgetpu_dev *etdev,
				      uint reg_offset)
{
	return readl_relaxed(etdev->regs.mem + reg_offset);
}

/* Read 32-bit reg with memory barrier completing before following CPU reads. */
static inline u32 edgetpu_dev_read_32_sync(struct edgetpu_dev *etdev,
					   uint reg_offset)
{
	return readl(etdev->regs.mem + reg_offset);
}

static inline u64 edgetpu_dev_read_64(struct edgetpu_dev *etdev,
				      uint reg_offset)
{
	return readq_relaxed(etdev->regs.mem + reg_offset);
}

static inline void edgetpu_dev_write_32(struct edgetpu_dev *etdev,
					uint reg_offset, u32 value)
{
	writel_relaxed(value, etdev->regs.mem + reg_offset);
}

/* Write 32-bit reg with memory barrier completing CPU writes first. */
static inline void edgetpu_dev_write_32_sync(struct edgetpu_dev *etdev,
					     uint reg_offset, u32 value)
{
	writel(value, etdev->regs.mem + reg_offset);
}

static inline void edgetpu_dev_write_64(struct edgetpu_dev *etdev,
					uint reg_offset, u64 value)
{
	writeq_relaxed(value, etdev->regs.mem + reg_offset);
}

/* External drivers can hook up to edgetpu driver using these calls. */
int edgetpu_open(struct edgetpu_dev *etdev, struct file *file);
long edgetpu_ioctl(struct file *file, uint cmd, ulong arg);

/* Bus (Platform/PCI) <-> Core API */

int __init edgetpu_init(void);
void __exit edgetpu_exit(void);
int edgetpu_device_add(struct edgetpu_dev *etdev,
		       const struct edgetpu_mapped_resource *regs);
void edgetpu_device_remove(struct edgetpu_dev *etdev);
int edgetpu_register_irq(struct edgetpu_dev *etdev, int irq);
void edgetpu_unregister_irq(struct edgetpu_dev *etdev, int irq);

/* Core -> Device FS API */

int __init edgetpu_fs_init(void);
void __exit edgetpu_fs_exit(void);
int edgetpu_fs_add(struct edgetpu_dev *etdev);
void edgetpu_fs_remove(struct edgetpu_dev *dev);
/* Get the top-level debugfs directory for the device class */
struct dentry *edgetpu_fs_debugfs_dir(void);

/* Core/Device/FS -> Chip API */

/* Chip-specific init/exit */
void edgetpu_chip_init(struct edgetpu_dev *etdev);
void edgetpu_chip_exit(struct edgetpu_dev *etdev);

/* IRQ handler */
irqreturn_t edgetpu_chip_irq_handler(int irq, void *arg);

/* Called from core to chip layer when MMU is needed during device init. */
void edgetpu_setup_mmu(struct edgetpu_dev *etdev);

/* Read TPU timestamp */
u64 edgetpu_chip_tpu_timestamp(struct edgetpu_dev *etdev);

/* Device -> Core API */

/* Add current thread as new TPU client */
struct edgetpu_client *edgetpu_client_add(struct edgetpu_dev *etdev);

/* Remove TPU client */
void edgetpu_client_remove(struct edgetpu_client *client);

/* mmap() device/queue memory */
int edgetpu_mmap(struct edgetpu_client *client, struct vm_area_struct *vma);

/* Increase reference count of @client. */
struct edgetpu_client *edgetpu_client_get(struct edgetpu_client *client);

/* Decrease reference count and free @client if count reaches zero */
void edgetpu_client_put(struct edgetpu_client *client);

/* Mark die that fails probe to allow bypassing */
void edgetpu_mark_probe_fail(struct edgetpu_dev *etdev);

/*
 * Get error code corresponding to @etdev state. Caller holds
 * etdev->state_lock.
 */
int edgetpu_get_state_errno_locked(struct edgetpu_dev *etdev);

#endif /* __EDGETPU_INTERNAL_H__ */
