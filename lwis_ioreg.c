/*
 * Google LWIS Register I/O Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-ioreg: " fmt

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_ioreg.h"

struct lwis_ioreg_list *lwis_ioreg_list_alloc(int num_blocks)
{
	struct lwis_ioreg_list *list;

	/* No need to allocate if num_blocks is invalid */
	if (num_blocks <= 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kzalloc(sizeof(struct lwis_ioreg_list), GFP_KERNEL);
	if (!list) {
		return ERR_PTR(-ENOMEM);
	}

	list->block =
		kzalloc(num_blocks * sizeof(struct lwis_ioreg), GFP_KERNEL);
	if (!list->block) {
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = num_blocks;

	return list;
}

void lwis_ioreg_list_free(struct lwis_ioreg_list *list)
{
	if (!list) {
		return;
	}

	if (list->block) {
		kfree(list->block);
	}

	kfree(list);
}

int lwis_ioreg_get(struct lwis_ioreg_list *list, int index,
		   struct platform_device *plat_dev)
{
	struct resource *res;
	struct lwis_ioreg *block;

	if (!plat_dev || !list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	res = platform_get_resource(plat_dev, IORESOURCE_MEM, index);
	if (!res) {
		pr_err("platform_get_resource error\n");
		return -EINVAL;
	}

	block = &list->block[index];
	block->start = res->start;
	block->size = resource_size(res);
	block->base = devm_ioremap_nocache(&plat_dev->dev, res->start,
					   resource_size(res));
	if (!block->base) {
		pr_err("Cannot map I/O register space\n");
		return -EINVAL;
	}

	return 0;
}

int lwis_ioreg_put(struct lwis_ioreg_list *list, int index,
		   struct platform_device *plat_dev)
{
	if (!plat_dev || !list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	if (!list->block[index].base) {
		return -EINVAL;
	}

	devm_iounmap(&plat_dev->dev, list->block[index].base);

	return 0;
}

static int ioreg_read_internal(unsigned int __iomem *base, int64_t offset,
			       int value_bits, uint64_t *value)
{
	switch (value_bits) {
	case 8:
		*value = readb((void *) base + offset);
		break;
	case 16:
		*value = readw((void *) base + offset);
		break;
	case 32:
		*value = readl((void *) base + offset);
		break;
	case 64:
		*value = readq((void *) base + offset);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ioreg_write_internal(unsigned int __iomem *base, int64_t offset,
				int value_bits, uint64_t value)
{
	switch (value_bits) {
	case 8:
		writeb((uint8_t) value, (void *) base + offset);
		break;
	case 16:
		writew((uint16_t) value, (void *) base + offset);
		break;
	case 32:
		writel((uint32_t) value, (void *) base + offset);
		break;
	case 64:
		writeq(value, (void *) base + offset);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int lwis_ioreg_read_batch(struct lwis_ioreg_list *list, int index,
			  struct lwis_io_data *data, int num_entries)
{
	int i;
	struct lwis_ioreg *block;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	block = &list->block[index];
	if (!block->base) {
		return -EINVAL;
	}

	if (!data || num_entries <= 0) {
		return -EINVAL;
	}

	for (i = 0; i < num_entries; ++i) {
		int ret =
			ioreg_read_internal(block->base, data[i].offset,
					    data[i].access_size, &data[i].val);
		if (ret) {
			pr_err("Invalid ioreg read\n");
			return ret;
		}
	}

	return 0;
}

int lwis_ioreg_write_batch(struct lwis_ioreg_list *list, int index,
			   struct lwis_io_data *data, int num_entries)
{
	int i;
	struct lwis_ioreg *block;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	block = &list->block[index];
	if (!block->base) {
		return -EINVAL;
	}

	if (!data || num_entries <= 0) {
		return -EINVAL;
	}

	for (i = 0; i < num_entries; ++i) {
		int ret =
			ioreg_write_internal(block->base, data[i].offset,
					     data[i].access_size, data[i].val);
		if (ret) {
			pr_err("Invalid ioreg write\n");
			return ret;
		}
	}

	return 0;
}

int lwis_ioreg_read(struct lwis_ioreg_list *list, int index, int64_t offset,
		    int value_bits, uint64_t *value)
{
	struct lwis_ioreg *block;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	block = &list->block[index];
	if (!block->base) {
		return -EINVAL;
	}

	return ioreg_read_internal(block->base, offset, value_bits, value);
}

int lwis_ioreg_write(struct lwis_ioreg_list *list, int index, int64_t offset,
		     int value_bits, uint64_t value)
{
	struct lwis_ioreg *block;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	block = &list->block[index];
	if (!block->base) {
		return -EINVAL;
	}

	return ioreg_write_internal(block->base, offset, value_bits, value);
}
