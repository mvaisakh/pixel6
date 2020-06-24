/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Edge TPU driver common internal definitions.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_INTERNAL_H__
#define __EDGETPU_INTERNAL_H__

#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/irqreturn.h>
#include <linux/miscdevice.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "edgetpu.h"

#define etdev_err(etdev, fmt, ...)				\
	dev_err((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_warn(etdev, fmt, ...)				\
	dev_warn((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_info(etdev, fmt, ...)				\
	dev_info((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_dbg(etdev, fmt, ...)				\
	dev_dbg((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_err_ratelimited(etdev, fmt, ...)			\
	dev_err_ratelimited((etdev)->etcdev, fmt, ##__VA_ARGS__)
#define etdev_warn_once(etdev, fmt, ...)			\
	dev_warn_once((etdev)->etcdev, fmt, ##__VA_ARGS__)

/* The number of TPU tiles in an edgetpu chip */
#ifdef CONFIG_EDGETPU_FPGA
#define EDGETPU_NTILES	4
#else
#define EDGETPU_NTILES	16
#endif

/* Up to 7 concurrent device groups / workloads per device. */
#define EDGETPU_NGROUPS		7

/*
 * Common-layer context IDs for non-secure TPU access, translated to chip-
 * specific values in the mmu driver.
 */
enum edgetpu_context_id {
	EDGETPU_CONTEXT_KCI = 0,	/* TPU firmware/kernel ID 0 */
	EDGETPU_CONTEXT_VII_BASE = 1,	/* groups 0-6 IDs 1-7 */
	/* contexts 8 and above not yet allocated */
};

/* Firmware size is limited to 2MB */
#define EDGETPU_FW_SIZE_MAX	0x200000
/* TPU address where the firmware is expected to be located (after remap) */
#define FW_IOVA		0x80000000ll

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
};

struct edgetpu_mapping;
struct edgetpu_mailbox_manager;
struct edgetpu_kci;
struct edgetpu_telemetry_ctx;

#define EDGETPU_DEVICE_NAME_MAX	64

/* ioremapped resource */
struct edgetpu_mapped_resource {
	void __iomem *mem;	/* starting virtual address */
	phys_addr_t phys;	/* starting physical address */
	resource_size_t size;	/* size in bytes */
};

struct edgetpu_dev {
	struct device *dev;	   /* platform/pci bus device */
	struct device *etcdev;	   /* edgetpu class char device */
	struct cdev cdev;	   /* cdev char device structure */
	dev_t devno;		   /* char device dev_t */
	char dev_name[EDGETPU_DEVICE_NAME_MAX];
	/* TODO(b/156441506): remove miscdevice */
	struct miscdevice miscdev; /* misc device info */
	struct {
		struct mutex lock;
		uint count;	   /* number times device currently opened */
		bool enabled;	   /* can be opened from the runtime */
	} open;
	struct edgetpu_mapped_resource regs; /* ioremapped CSRs */
	struct dentry *d_entry;    /* debugfs dir for this device */
	struct mutex groups_lock;  /* protects groups */
	struct edgetpu_device_group *groups[EDGETPU_NGROUPS];
	void *mmu_cookie;	   /* mmu driver private data */
	void *dram_cookie;	   /* on-device DRAM private data */
	struct edgetpu_mailbox_manager *mailbox_manager;
	struct edgetpu_kci *kci;
	struct edgetpu_firmware *firmware; /* firmware management */
	struct edgetpu_telemetry_ctx *telemetry;
	int mcp_id;		/* multichip pkg id, or -1 for none */
	uint mcp_die_index;	/* physical die index w/in multichip pkg */
	u8 mcp_pkg_type;	/* multichip pkg type */
};

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

/* Bus (Platform/PCI) <-> Core API */

int __init edgetpu_init(void);
void __exit edgetpu_exit(void);
int edgetpu_device_add(struct edgetpu_dev *etdev,
		       const struct edgetpu_mapped_resource *regs);
void edgetpu_device_remove(struct edgetpu_dev *etdev);
int edgetpu_register_irq(struct edgetpu_dev *etdev, int irq);
void edgetpu_unregister_irq(struct edgetpu_dev *etdev, int irq);

/* Called from core to chip layer when MMU is needed during device init. */
void edgetpu_setup_mmu(struct edgetpu_dev *etdev);

/* Core -> Device API */

int __init edgetpu_dev_init(void);
void __exit edgetpu_dev_exit(void);
int edgetpu_dev_add(struct edgetpu_dev *etdev);
void edgetpu_dev_remove(struct edgetpu_dev *dev);

/* Core/Device -> Chip API */

/* Chip-specific init/exit */
void edgetpu_chip_init(struct edgetpu_dev *etdev);
void edgetpu_chip_exit(struct edgetpu_dev *etdev);

/* IRQ handler */
irqreturn_t edgetpu_chip_irq_handler(int irq, void *arg);

/* Return true if the device is marked as bypassed. */
bool edgetpu_chip_bypassed(struct edgetpu_dev *etdev);

/* Device -> Core API */

/* Add current thread as new TPU client */
struct edgetpu_client *edgetpu_client_add(struct edgetpu_dev *etdev);

/* Remove TPU client */
void edgetpu_client_remove(struct edgetpu_client *client);

/* mmap() device/queue memory */
int edgetpu_mmap(struct edgetpu_client *client, struct vm_area_struct *vma);

/*
 * Enable or disable @etdev being requested (opened) by the runtime.
 *
 * Opened clients are not affected by this function.
 */
void edgetpu_set_open_enabled(struct edgetpu_dev *etdev, bool enabled);

/* Increase reference count of @client. */
struct edgetpu_client *edgetpu_client_get(struct edgetpu_client *client);

/* Decrease reference count and free @client if count reaches zero */
void edgetpu_client_put(struct edgetpu_client *client);

#endif /* __EDGETPU_INTERNAL_H__ */
