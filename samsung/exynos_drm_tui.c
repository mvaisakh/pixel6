// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2020 Samsung Electronics Co., Ltd.
 * Authors:
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) "[TUI Driver] " fmt

#include <linux/init.h>
#include <linux/compiler.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/dma-buf.h>
#include <linux/ion.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sizes.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <drm/samsung_drm.h>

#include "exynos_drm_tui.h"
#include "exynos_drm_drv.h"

#define DRV_NAME "tui-driver"

struct exynos_tui_driver_buf_info {
	uint64_t pa[TUI_DRIVER_BUFFER_NUM];
	uint64_t size[TUI_DRIVER_BUFFER_NUM];
};

struct tui_driver_dma_buf {
	struct exynos_tui_driver_buf_info g_tui_buf_info;
	struct dma_buf *g_dma_buf;
	struct dma_buf_attachment *g_attachment;
	struct sg_table *g_sgt;
};

struct tui_driver_priv {
	struct device *dev;
	struct tui_driver_dma_buf buf;
	struct miscdevice *misc;
};

static uint64_t find_heapmask(void)
{
	int i, cnt = ion_query_heaps_kernel(NULL, 0);
	const char *heapname;
	struct ion_heap_data data[ION_NUM_MAX_HEAPS];

	heapname = TUI_DRIVER_HEAP_NAME;
	ion_query_heaps_kernel((struct ion_heap_data *)data, cnt);
	for (i = 0; i < cnt; i++) {
		if (!strncmp(data[i].name, heapname, MAX_HEAP_NAME))
			break;
	}

	if (i == cnt) {
		pr_err("heap %s is not found\n", heapname);
		return 0;
	}

	return 1 << data[i].heap_id;
}

static int exynos_tui_alloc_buffer(struct device *dev,
				   struct tui_driver_dma_buf *buf,
				   unsigned long framebuf_size)
{
	uint64_t heapmask;
	dma_addr_t phys_addr;

	framebuf_size = ALIGN(framebuf_size,
			      SZ_4K);
	heapmask = find_heapmask();

	/*
	 * Request additional 4K memory to make sure
	 * the physical address and size is 4K aligned
	 */
	buf->g_dma_buf = ion_alloc(framebuf_size + SZ_4K,
				   heapmask,
				   ION_EXYNOS_FLAG_PROTECTED);
	if (IS_ERR(buf->g_dma_buf)) {
		pr_err("fail to allocate dma buffer\n");
		goto err_alloc;
	}

	buf->g_attachment = dma_buf_attach(buf->g_dma_buf,
					   dev);
	if (IS_ERR(buf->g_attachment)) {
		pr_err("fail to dma buf attachment\n");
		goto err_attach;
	}

	buf->g_sgt = dma_buf_map_attachment(buf->g_attachment,
					    DMA_BIDIRECTIONAL);
	if (IS_ERR(buf->g_sgt)) {
		pr_err("fail to map attachment, err: %lld\n", (int64_t)buf->g_sgt);
		goto err_attachment;
	}

	phys_addr = sg_phys(buf->g_sgt->sgl);
	phys_addr = ALIGN(phys_addr,
			  SZ_4K);
	buf->g_tui_buf_info.pa[0] = (uint64_t)phys_addr;
	buf->g_tui_buf_info.size[0] = framebuf_size;

	return 0;

err_attachment:
	buf->g_sgt = NULL;
	dma_buf_detach(buf->g_dma_buf, buf->g_attachment);
err_attach:
	buf->g_attachment = NULL;
	dma_buf_put(buf->g_dma_buf);
err_alloc:
	buf->g_dma_buf = NULL;
	return -ENOMEM;
}

static void exynos_tui_free_buffer(struct tui_driver_dma_buf *buf)
{
	if (buf->g_sgt != NULL && buf->g_attachment != NULL) {
		dma_buf_unmap_attachment(buf->g_attachment,
					 buf->g_sgt,
					 DMA_BIDIRECTIONAL);
		buf->g_sgt = NULL;
	}
	if (buf->g_attachment != NULL && buf->g_dma_buf != NULL) {
		dma_buf_detach(buf->g_dma_buf,
			       buf->g_attachment);
		buf->g_attachment = NULL;
	}
	if (buf->g_dma_buf != NULL) {
		dma_buf_put(buf->g_dma_buf);
		buf->g_dma_buf = NULL;
	}
}

/*
 * Device file ops
 */
static int exynos_tui_open(__maybe_unused struct inode *inode, struct file *filp)
{
	struct miscdevice *mdev = filp->private_data;
	struct tui_driver_priv *priv;

	pr_debug("%s\n", __func__);

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = mdev->this_device;
	filp->private_data = priv;

	return 0;
}

static int exynos_tui_release(struct inode *inode, struct file *filp)
{
	struct tui_driver_priv *priv = filp->private_data;
	(void)inode;

	pr_debug("%s\n", __func__);

	kfree(priv);

	return 0;
}

static int exynos_tui_request_buffer(struct tui_driver_priv *priv,
			    struct tui_hw_buffer __user *argp)
{
	int ret;
	struct tui_hw_buffer buffer;

	if (copy_from_user(&buffer,
			   argp,
			   sizeof(struct tui_hw_buffer))) {
		pr_err("copy_from_user failed\n");
		ret = -EFAULT;
		goto alloc_tui_buffer_fail;
	}

	/* allocate TUI frame buffer */
	ret = exynos_tui_alloc_buffer(priv->dev,
				      &(priv->buf),
				      (unsigned long)buffer.fb_size);
	if (ret < 0) {
		pr_err("failed to allocate buffer\n");
		goto alloc_tui_buffer_fail;
	}

	buffer.fb_physical = priv->buf.g_tui_buf_info.pa[0];
	buffer.fb_size = priv->buf.g_tui_buf_info.size[0];
	pr_debug("fb_physical: %llu, fb_size: %llu\n",
			 buffer.fb_physical,
			 buffer.fb_size);

	if (copy_to_user(argp,
			 &buffer,
			 sizeof(struct tui_hw_buffer))) {
		pr_err("copy_to_user failed\n");
		ret = -EFAULT;
		goto copy_to_user_fail;
	}

	return ret;

copy_to_user_fail:
	exynos_tui_free_buffer(&(priv->buf));
alloc_tui_buffer_fail:
	return ret;
}

/* Disable TUI driver / Activate linux UI drivers */
static int exynos_tui_release_buffer(struct tui_driver_priv *priv)
{
	int ret = 0;

	exynos_tui_free_buffer(&(priv->buf));

	return ret;
}

/*
 * Ioctls
 */
static long exynos_tui_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct tui_driver_priv *priv = filp->private_data;
	long ret = 0;

	/* Handle command */
	switch (cmd) {
	case EXYNOS_START_TUI:
		/* Prepare display for TUI / Deactivate linux UI drivers */
		ret = exynos_atomic_enter_tui();
		if (ret < 0)
			pr_err("failed to enter TUI\n");
		break;
	case EXYNOS_FINISH_TUI:
		ret = exynos_atomic_exit_tui();
		if (ret < 0)
			pr_err("failed to exit TUI\n");
		break;
	case EXYNOS_TUI_REQUEST_BUFFER:
		ret = exynos_tui_request_buffer(priv,
				       (struct tui_hw_buffer __user *)arg);
		break;
	case EXYNOS_TUI_RELEASE_BUFFER:
		ret = exynos_tui_release_buffer(priv);
		break;
	default:
		ret = -ENOTTY;
		break;
	}

	return ret;
}

static const struct file_operations exynos_tui_fops = {
	.open           = exynos_tui_open,
	.release        = exynos_tui_release,
	.unlocked_ioctl = exynos_tui_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = exynos_tui_ioctl,
#endif
};

static int exynos_tui_probe(struct platform_device *pdev)
{
	struct miscdevice *misc_dev;
	static uint64_t dma_mask;
	int ret = 0;

	/* Create a char device: we want to create it anew */
	misc_dev = devm_kzalloc(&pdev->dev, sizeof(*misc_dev), GFP_KERNEL);
	if (!misc_dev)
		return -ENOMEM;

	dma_mask = DMA_BIT_MASK(48);

	misc_dev->minor = MISC_DYNAMIC_MINOR;
	misc_dev->fops = &exynos_tui_fops;
	misc_dev->name = DRV_NAME;
	misc_dev->parent = pdev->dev.parent;

	dev_set_drvdata(&pdev->dev, misc_dev);

	ret = misc_register(misc_dev);
	if (ret == 0) {
		misc_dev->this_device->dma_mask = &dma_mask;
		dma_set_coherent_mask(misc_dev->this_device, dma_mask);
	}
	return ret;
}

static int exynos_tui_remove(struct platform_device *pdev)
{
	struct miscdevice *misc_dev = dev_get_drvdata(&pdev->dev);

	misc_deregister(misc_dev);

	return 0;
}

struct platform_driver tui_driver = {
	.driver = {
		.name = DRV_NAME,
	},
	.probe = exynos_tui_probe,
	.remove = exynos_tui_remove,
};

MODULE_DESCRIPTION("Samsung SoC TUI Driver");
MODULE_LICENSE("GPL");
