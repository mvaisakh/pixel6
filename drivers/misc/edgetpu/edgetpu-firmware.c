// SPDX-License-Identifier: GPL-2.0
/*
 * Edge TPU firmware loader.
 *
 * Copyright (C) 2019-2020 Google, Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "edgetpu-firmware.h"
#include "edgetpu-firmware-util.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-shared-fw.h"
#include "edgetpu-telemetry.h"

/*
 * Descriptor for loaded firmware, either in shared buffer mode or legacy mode
 * (non-shared, custom allocated memory).
 */
struct edgetpu_firmware_desc {
	/*
	 * Mode independent buffer information. This is either passed into or
	 * updated by handlers.
	 */
	struct edgetpu_firmware_buffer buf;
	/*
	 * Shared firmware buffer when we're using shared buffer mode. This
	 * pointer to keep and release the reference count on unloading this
	 * shared firmware buffer.
	 *
	 * This is NULL when firmware is loaded in legacy mode.
	 */
	struct edgetpu_shared_fw_buffer *shared_buf;
};

struct edgetpu_firmware_private {
	const struct edgetpu_firmware_handlers *handlers;
	void *data; /* for edgetpu_firmware_(set/get)_data */

	struct mutex fw_desc_lock;
	struct edgetpu_firmware_desc fw_desc;
};

void edgetpu_firmware_set_data(struct edgetpu_firmware *et_fw, void *data)
{
	et_fw->p->data = data;
}

void *edgetpu_firmware_get_data(struct edgetpu_firmware *et_fw)
{
	return et_fw->p->data;
}

static int edgetpu_firmware_legacy_load_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc, const char *name)
{
	int ret;
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct device *dev = etdev->dev;
	const struct firmware *fw;
	size_t aligned_size;

	ret = request_firmware(&fw, name, dev);
	if (ret) {
		etdev_dbg(etdev,
			  "%s: request '%s' failed: %d\n", __func__, name, ret);
		return ret;
	}

	aligned_size = ALIGN(fw->size, fw_desc->buf.used_size_align);
	if (aligned_size > fw_desc->buf.alloc_size) {
		etdev_dbg(etdev,
			  "%s: firmware buffer too small: alloc size=0x%zx, required size=0x%zx\n",
			  __func__, fw_desc->buf.alloc_size, aligned_size);
		ret = -ENOSPC;
		goto out_release_firmware;
	}

	memcpy(fw_desc->buf.vaddr, fw->data, fw->size);
	fw_desc->buf.used_size = aligned_size;
	fw_desc->buf.name = kstrdup(name, GFP_KERNEL);

out_release_firmware:
	release_firmware(fw);
	return ret;
}

static void edgetpu_firmware_legacy_unload_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc)
{
	kfree(fw_desc->buf.name);
	fw_desc->buf.name = NULL;
	fw_desc->buf.used_size = 0;
}

static int edgetpu_firmware_shared_load_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc, const char *name)
{
	int ret;
	struct edgetpu_dev *etdev = et_fw->etdev;
	struct edgetpu_shared_fw_buffer *shared_buf;

	shared_buf = edgetpu_shared_fw_load(name, etdev);
	if (IS_ERR(shared_buf)) {
		ret = PTR_ERR(shared_buf);
		etdev_dbg(etdev, "shared buffer loading failed: %d\n", ret);
		return ret;
	}
	fw_desc->shared_buf = shared_buf;
	fw_desc->buf.vaddr = edgetpu_shared_fw_buffer_vaddr(shared_buf);
	fw_desc->buf.alloc_size = edgetpu_shared_fw_buffer_size(shared_buf);
	fw_desc->buf.used_size = fw_desc->buf.alloc_size;
	fw_desc->buf.name = edgetpu_shared_fw_buffer_name(shared_buf);
	return 0;
}

static void edgetpu_firmware_shared_unload_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc)
{
	fw_desc->buf.vaddr = NULL;
	fw_desc->buf.alloc_size = 0;
	fw_desc->buf.used_size = 0;
	fw_desc->buf.name = NULL;
	edgetpu_shared_fw_put(fw_desc->shared_buf);
	fw_desc->shared_buf = NULL;
}

static int edgetpu_firmware_do_load_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc, const char *name)
{
	/* Use shared firmware from host if not allocated a buffer space. */
	if (!fw_desc->buf.vaddr)
		return edgetpu_firmware_shared_load_locked(et_fw, fw_desc,
							   name);
	else
		return edgetpu_firmware_legacy_load_locked(et_fw, fw_desc,
							   name);
}

static void edgetpu_firmware_do_unload_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc)
{
	if (fw_desc->shared_buf)
		edgetpu_firmware_shared_unload_locked(et_fw, fw_desc);
	else
		edgetpu_firmware_legacy_unload_locked(et_fw, fw_desc);
}

static int edgetpu_firmware_load_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc, const char *name)
{
	const struct edgetpu_firmware_handlers *handlers = et_fw->p->handlers;
	struct edgetpu_dev *etdev = et_fw->etdev;
	int ret;

	if (handlers && handlers->alloc_buffer) {
		ret = handlers->alloc_buffer(et_fw, &fw_desc->buf);
		if (ret) {
			etdev_dbg(etdev, "handler alloc_buffer failed: %d\n",
				  ret);
			return ret;
		}
	}

	ret = edgetpu_firmware_do_load_locked(et_fw, fw_desc, name);
	if (ret) {
		etdev_dbg(etdev, "firmware request failed: %d\n", ret);
		goto out_free_buffer;
	}

	if (handlers && handlers->setup_buffer) {
		ret = handlers->setup_buffer(et_fw, &fw_desc->buf);
		if (ret) {
			etdev_dbg(etdev, "handler setup_buffer failed: %d\n",
				  ret);
			goto out_do_unload_locked;
		}
	}

	return 0;

out_do_unload_locked:
	edgetpu_firmware_do_unload_locked(et_fw, fw_desc);
out_free_buffer:
	if (handlers && handlers->free_buffer)
		handlers->free_buffer(et_fw, &fw_desc->buf);
	return ret;
}

static void edgetpu_firmware_unload_locked(
		struct edgetpu_firmware *et_fw,
		struct edgetpu_firmware_desc *fw_desc)
{
	const struct edgetpu_firmware_handlers *handlers = et_fw->p->handlers;

	/*
	 * Platform specific implementation for cleaning up allocated buffer.
	 */
	if (handlers && handlers->teardown_buffer)
		handlers->teardown_buffer(et_fw, &fw_desc->buf);
	edgetpu_firmware_do_unload_locked(et_fw, fw_desc);
	/*
	 * Platform specific implementation for freeing allocated buffer.
	 */
	if (handlers && handlers->free_buffer)
		handlers->free_buffer(et_fw, &fw_desc->buf);
}

static int edgetpu_firmware_ack(struct edgetpu_dev *etdev)
{
	int err;

	etdev_dbg(etdev, "Sending KCI ACK");
	err = edgetpu_kci_ack(etdev->kci);
	if (err)
		etdev_err(etdev, "KCI ACK failed :( - %d", err);
	else
		etdev_info(etdev, "KCI ACK Succeeded :)");
	return err;
}

static int edgetpu_firmware_run_locked(struct edgetpu_firmware *et_fw,
				       const char *name)
{
	const struct edgetpu_firmware_handlers *handlers = et_fw->p->handlers;
	struct edgetpu_firmware_desc new_fw_desc;
	int ret;

	memset(&new_fw_desc, 0, sizeof(new_fw_desc));
	ret = edgetpu_firmware_load_locked(et_fw, &new_fw_desc, name);
	if (ret)
		return ret;

	if (handlers && handlers->prepare_run) {
		ret = handlers->prepare_run(et_fw, &new_fw_desc.buf);
		if (ret)
			goto out_unload_new_fw;
	}

	/*
	 * Previous firmware buffer is not used anymore when R52 runs on
	 * new firmware buffer. Unload this before et_fw->p->fw_buf is
	 * overwritten by new buffer information.
	 */
	edgetpu_firmware_unload_locked(et_fw, &et_fw->p->fw_desc);
	et_fw->p->fw_desc = new_fw_desc;

	/* Give the firmware some time to initialize */
	msleep(100);
	ret = edgetpu_firmware_ack(et_fw->etdev);
	if (!ret)
		edgetpu_telemetry_kci(et_fw->etdev);

	return ret;

out_unload_new_fw:
	edgetpu_firmware_unload_locked(et_fw, &new_fw_desc);
	return ret;
}

int edgetpu_firmware_run(struct edgetpu_dev *etdev, const char *name)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;

	if (!et_fw)
		return -ENODEV;

	edgetpu_set_open_enabled(etdev, false);
	if (etdev->open.count) {
		etdev_err(etdev,
			  "failed to run firmware because device is in use");
		edgetpu_set_open_enabled(etdev, true);
		return -EBUSY;
	}

	mutex_lock(&et_fw->p->fw_desc_lock);
	ret = edgetpu_firmware_run_locked(et_fw, name);
	mutex_unlock(&et_fw->p->fw_desc_lock);

	edgetpu_set_open_enabled(etdev, true);
	return ret;
}

static ssize_t load_firmware_show(
		struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;
	const char *fw_name;

	if (!et_fw)
		return -ENODEV;

	mutex_lock(&et_fw->p->fw_desc_lock);
	fw_name = et_fw->p->fw_desc.buf.name;
	if (fw_name)
		ret = scnprintf(buf, PAGE_SIZE, "%s\n", fw_name);
	else
		ret = -ENODATA;
	mutex_unlock(&et_fw->p->fw_desc_lock);
	return ret;
}

static ssize_t load_firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct edgetpu_dev *etdev = dev_get_drvdata(dev);
	struct edgetpu_firmware *et_fw = etdev->firmware;
	int ret;
	char *name;

	if (!et_fw)
		return -ENODEV;

	name = edgetpu_fwutil_name_from_attr_buf(buf);
	if (IS_ERR(name))
		return PTR_ERR(name);

	etdev_info(etdev, "loading firmware %s\n", name);
	ret = edgetpu_chip_firmware_run(etdev, name);

	kfree(name);

	if (ret)
		return ret;
	return count;
}

static DEVICE_ATTR_RW(load_firmware);

static struct attribute *dev_attrs[] = {
	&dev_attr_load_firmware.attr,
	NULL,
};

static const struct attribute_group edgetpu_firmware_attr_group = {
	.attrs = dev_attrs,
};

int edgetpu_firmware_create(struct edgetpu_dev *etdev,
			    const struct edgetpu_firmware_handlers *handlers)
{
	struct edgetpu_firmware *et_fw;
	int ret;

	if (etdev->firmware)
		return -EBUSY;

	et_fw = kzalloc(sizeof(*et_fw), GFP_KERNEL);
	if (!et_fw)
		return -ENOMEM;
	et_fw->etdev = etdev;

	et_fw->p = kzalloc(sizeof(*et_fw->p), GFP_KERNEL);
	if (!et_fw->p) {
		ret = -ENOMEM;
		goto out_kfree_et_fw;
	}
	et_fw->p->handlers = handlers;

	mutex_init(&et_fw->p->fw_desc_lock);

	ret = device_add_group(etdev->dev, &edgetpu_firmware_attr_group);
	if (ret)
		goto out_kfree_et_fw_p;

	if (handlers && handlers->after_create) {
		ret = handlers->after_create(et_fw);
		if (ret) {
			etdev_dbg(etdev,
				  "%s: after create handler failed: %d\n",
				  __func__, ret);
			goto out_device_remove_group;
		}
	}

	etdev->firmware = et_fw;
	return 0;

out_device_remove_group:
	device_remove_group(etdev->dev, &edgetpu_firmware_attr_group);
out_kfree_et_fw_p:
	kfree(et_fw->p);
out_kfree_et_fw:
	kfree(et_fw);
	return ret;
}

void edgetpu_firmware_destroy(struct edgetpu_dev *etdev)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	const struct edgetpu_firmware_handlers *handlers;

	if (!et_fw)
		return;

	if (et_fw->p) {
		handlers = et_fw->p->handlers;
		/*
		 * Platform specific implementation, which includes stop
		 * running firmware.
		 */
		if (handlers && handlers->before_destroy)
			handlers->before_destroy(et_fw);
	}

	device_remove_group(etdev->dev, &edgetpu_firmware_attr_group);

	if (et_fw->p) {
		mutex_lock(&et_fw->p->fw_desc_lock);
		edgetpu_firmware_unload_locked(et_fw, &et_fw->p->fw_desc);
		mutex_unlock(&et_fw->p->fw_desc_lock);
	}

	etdev->firmware = NULL;

	kfree(et_fw->p);
	kfree(et_fw);
}

/* debugfs mappings dump */
void edgetpu_firmware_mappings_show(struct edgetpu_dev *etdev,
				    struct seq_file *s)
{
	struct edgetpu_firmware *et_fw = etdev->firmware;
	struct edgetpu_firmware_buffer *fw_buf;
	phys_addr_t fw_iova_target;

	if (!et_fw)
		return;
	fw_buf = &et_fw->p->fw_desc.buf;
	if (!fw_buf->vaddr)
		return;
	fw_iova_target = fw_buf->dram_tpa ? fw_buf->dram_tpa : fw_buf->dma_addr;
	seq_printf(s, "  0x%llx %lu fw - %pad\n",
		   FW_IOVA, fw_buf->alloc_size / PAGE_SIZE, &fw_iova_target);
}
