/*
 * Google LWIS DMA Buffer Utilities
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-buffer: " fmt

#include <linux/slab.h>

#include "lwis_buffer.h"
#include "lwis_device.h"
#include "lwis_device_slc.h"
#include "lwis_platform_dma.h"

#include "pt/pt.h"

int lwis_buffer_alloc(struct lwis_client *lwis_client,
		      struct lwis_alloc_buffer_info *alloc_info,
		      struct lwis_allocated_buffer *buffer)
{
	struct dma_buf *dma_buf;
	int ret = 0;

	BUG_ON(!lwis_client);
	BUG_ON(!alloc_info);

	if (alloc_info->flags & LWIS_DMA_SYSTEM_CACHE_RESERVATION) {
		if (lwis_client->lwis_dev->type == DEVICE_TYPE_SLC) {
			ret = lwis_slc_buffer_alloc(lwis_client->lwis_dev,
						    alloc_info);
			if (ret) {
				return ret;
			}
			dma_buf = NULL;
		} else {
			pr_err("Can't allocate system cache buffer on non-slc device\n");
			return -EINVAL;
		}
	} else {
		alloc_info->size = PAGE_ALIGN(alloc_info->size);
		dma_buf = lwis_platform_dma_buffer_alloc(alloc_info->size,
							 alloc_info->flags);
		if (IS_ERR_OR_NULL(dma_buf)) {
			pr_err("lwis_platform_dma_buffer_alloc failed (%ld)\n",
			       PTR_ERR(dma_buf));
			return -ENOMEM;
		}

		alloc_info->dma_fd = dma_buf_fd(dma_buf, O_CLOEXEC);
		if (alloc_info->dma_fd < 0) {
			pr_err("dma_buf_fd failed (%d)\n", alloc_info->dma_fd);
			dma_buf_put(dma_buf);
			return alloc_info->dma_fd;
		}

		alloc_info->partition_id = PT_PTID_INVALID;

		/*
		 * Increment refcount of the fd to 2. Both userspace's close(fd)
		 * and kernel's lwis_buffer_free() will decrement the refcount
		 * by 1. Whoever reaches 0 refcount frees the buffer.
		 */
		get_dma_buf(dma_buf);
	}

	buffer->fd = alloc_info->dma_fd;
	buffer->size = alloc_info->size;
	buffer->dma_buf = dma_buf;
	hash_add(lwis_client->allocated_buffers, &buffer->node, buffer->fd);

	return 0;
}

int lwis_buffer_free(struct lwis_client *lwis_client,
		     struct lwis_allocated_buffer *buffer)
{
	if (buffer->dma_buf == NULL) {
		if (lwis_client->lwis_dev->type == DEVICE_TYPE_SLC) {
			lwis_slc_buffer_free(lwis_client->lwis_dev, buffer->fd);
		} else {
			pr_err("Unexpected NULL dma_buf\n");
			return -EINVAL;
		}
	} else {
		dma_buf_put(buffer->dma_buf);
	}
	hash_del(&buffer->node);
	return 0;
}

int lwis_buffer_enroll(struct lwis_client *lwis_client,
		       struct lwis_enrolled_buffer *buffer)
{
	struct lwis_enrolled_buffer *old_buffer;
	BUG_ON(!lwis_client);
	BUG_ON(!buffer);

	buffer->dma_buf = dma_buf_get(buffer->info.fd);
	if (IS_ERR_OR_NULL(buffer->dma_buf)) {
		pr_err("Could not get dma buffer for fd: %d", buffer->info.fd);
		return -EINVAL;
	}

	buffer->dma_buf_attachment = dma_buf_attach(
		buffer->dma_buf, &lwis_client->lwis_dev->plat_dev->dev);

	if (IS_ERR_OR_NULL(buffer->dma_buf_attachment)) {
		pr_err("Could not attach dma buffer for fd: %d",
		       buffer->info.fd);
		dma_buf_put(buffer->dma_buf);
		return -EINVAL;
	}

	if (buffer->info.dma_read && buffer->info.dma_write) {
		buffer->dma_direction = DMA_BIDIRECTIONAL;
	} else if (buffer->info.dma_read) {
		buffer->dma_direction = DMA_TO_DEVICE;
	} else if (buffer->info.dma_write) {
		buffer->dma_direction = DMA_FROM_DEVICE;
	} else {
		buffer->dma_direction = DMA_NONE;
	}

	buffer->sg_table = dma_buf_map_attachment(buffer->dma_buf_attachment,
						  buffer->dma_direction);
	if (IS_ERR_OR_NULL(buffer->sg_table)) {
		pr_err("Could not map dma attachment for fd: %d",
		       buffer->info.fd);
		dma_buf_detach(buffer->dma_buf, buffer->dma_buf_attachment);
		dma_buf_put(buffer->dma_buf);
		return -EINVAL;
	}

	buffer->info.dma_vaddr =
		lwis_platform_dma_buffer_map(lwis_client->lwis_dev,
					     buffer->dma_buf_attachment, 0, 0,
					     buffer->dma_direction, 0);

	if (IS_ERR_OR_NULL((void *)buffer->info.dma_vaddr)) {
		pr_err("Could not map into IO VMM for fd: %d", buffer->info.fd);
		dma_buf_unmap_attachment(buffer->dma_buf_attachment,
					 buffer->sg_table,
					 buffer->dma_direction);
		dma_buf_detach(buffer->dma_buf, buffer->dma_buf_attachment);
		dma_buf_put(buffer->dma_buf);
		return -EINVAL;
	}

	old_buffer = lwis_client_enrolled_buffer_find(lwis_client,
						      buffer->info.dma_vaddr);

	if (old_buffer) {
		pr_err("Duplicate vaddr %llx for fd %d", buffer->info.dma_vaddr,
		       buffer->info.fd);
		lwis_platform_dma_buffer_unmap(lwis_client->lwis_dev,
					       buffer->dma_buf_attachment,
					       buffer->info.dma_vaddr);
		dma_buf_unmap_attachment(buffer->dma_buf_attachment,
					 buffer->sg_table,
					 buffer->dma_direction);
		dma_buf_detach(buffer->dma_buf, buffer->dma_buf_attachment);
		dma_buf_put(buffer->dma_buf);
		return -EINVAL;
	}

	hash_add(lwis_client->enrolled_buffers, &buffer->node,
		 buffer->info.dma_vaddr);

	return 0;
}

int lwis_buffer_disenroll(struct lwis_client *lwis_client,
			  struct lwis_enrolled_buffer *buffer)
{
	BUG_ON(!lwis_client);
	BUG_ON(!buffer);

	lwis_platform_dma_buffer_unmap(lwis_client->lwis_dev,
				       buffer->dma_buf_attachment,
				       buffer->info.dma_vaddr);
	dma_buf_unmap_attachment(buffer->dma_buf_attachment, buffer->sg_table,
				 buffer->dma_direction);
	dma_buf_detach(buffer->dma_buf, buffer->dma_buf_attachment);
	dma_buf_put(buffer->dma_buf);
	/* Delete the node from the hash table */
	hash_del(&buffer->node);
	return 0;
}

struct lwis_enrolled_buffer *
lwis_client_enrolled_buffer_find(struct lwis_client *lwis_client,
				 dma_addr_t dma_vaddr)
{
	struct lwis_enrolled_buffer *p;
	BUG_ON(!lwis_client);

	hash_for_each_possible (lwis_client->enrolled_buffers, p, node,
				dma_vaddr) {
		if (p->info.dma_vaddr == dma_vaddr) {
			return p;
		}
	}

	return NULL;
}

int lwis_client_enrolled_buffers_clear(struct lwis_client *lwis_client)
{
	/* Our hash table iterator */
	struct lwis_enrolled_buffer *buffer;
	/* Temporary vars for hash table traversal */
	struct hlist_node *n;
	int i;

	BUG_ON(!lwis_client);
	/* Iterate over the entire hash table */
	hash_for_each_safe (lwis_client->enrolled_buffers, i, n, buffer, node) {
		/* Disenroll the buffer */
		lwis_buffer_disenroll(lwis_client, buffer);
		/* Free the object */
		kfree(buffer);
	}

	return 0;
}

struct lwis_allocated_buffer *
lwis_client_allocated_buffer_find(struct lwis_client *lwis_client, int fd)
{
	struct lwis_allocated_buffer *p;

	BUG_ON(!lwis_client);

	hash_for_each_possible (lwis_client->allocated_buffers, p, node, fd) {
		if (p->fd == fd) {
			return p;
		}
	}
	return NULL;
}

int lwis_client_allocated_buffers_clear(struct lwis_client *lwis_client)
{
	struct lwis_allocated_buffer *buffer;
	struct hlist_node *n;
	int i;

	BUG_ON(!lwis_client);

	hash_for_each_safe (lwis_client->allocated_buffers, i, n, buffer,
			    node) {
		if (lwis_client->lwis_dev->type != DEVICE_TYPE_SLC) {
			lwis_buffer_free(lwis_client, buffer);
		}
		kfree(buffer);
	}
	return 0;
}
