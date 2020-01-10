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

static int find_block_idx_by_name(struct lwis_ioreg_list *list, char *name)
{
	int i;
	for (i = 0; i < list->count; ++i) {
		if (!strcmp(list->block[i].name, name)) {
			return i;
		}
	}
	return -ENOENT;
}

int lwis_ioreg_list_alloc(struct lwis_ioreg_device *ioreg_dev, int num_blocks)
{
	struct lwis_ioreg_list *list;

	BUG_ON(!ioreg_dev);

	/* No need to allocate if num_blocks is invalid */
	if (num_blocks <= 0) {
		return -EINVAL;
	}

	list = &ioreg_dev->reg_list;
	list->block =
		kzalloc(num_blocks * sizeof(struct lwis_ioreg), GFP_KERNEL);
	if (!list->block) {
		return -ENOMEM;
	}

	list->count = num_blocks;

	return 0;
}

void lwis_ioreg_list_free(struct lwis_ioreg_device *ioreg_dev)
{
	struct lwis_ioreg_list *list;

	BUG_ON(!ioreg_dev);

	list = &ioreg_dev->reg_list;
	if (list->block) {
		kfree(list->block);
		list->block = NULL;
		list->count = 0;
	}
}

int lwis_ioreg_get(struct lwis_ioreg_device *ioreg_dev, int index, char *name)
{
	struct resource *res;
	struct lwis_ioreg *block;
	struct lwis_ioreg_list *list;
	struct platform_device *plat_dev;

	BUG_ON(!ioreg_dev);

	plat_dev = ioreg_dev->base_dev.plat_dev;
	list = &ioreg_dev->reg_list;
	if (index < 0 || index >= list->count) {
		return -EINVAL;
	}

	res = platform_get_resource(plat_dev, IORESOURCE_MEM, index);
	if (!res) {
		pr_err("platform_get_resource error\n");
		return -EINVAL;
	}

	block = &list->block[index];
	block->name = name;
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

int lwis_ioreg_put_by_idx(struct lwis_ioreg_device *ioreg_dev, int index)
{
	struct lwis_ioreg_list *list;
	struct device *dev;

	BUG_ON(!ioreg_dev);

	dev = &ioreg_dev->base_dev.plat_dev->dev;
	list = &ioreg_dev->reg_list;
	if (index < 0 || index >= list->count) {
		return -EINVAL;
	}

	if (!list->block[index].base) {
		return -EINVAL;
	}

	devm_iounmap(dev, list->block[index].base);

	return 0;
}

int lwis_ioreg_put_by_name(struct lwis_ioreg_device *ioreg_dev, char *name)
{
	int i, bidx;
	struct lwis_ioreg_list *list;
	struct device *dev;

	BUG_ON(!ioreg_dev);

	dev = &ioreg_dev->base_dev.plat_dev->dev;
	list = &ioreg_dev->reg_list;
	bidx = find_block_idx_by_name(list, name);
	if (bidx < 0) {
		return bidx;
	}
	if (list->block[bidx].base == NULL) {
		return -EINVAL;
	}

	devm_iounmap(dev, list->block[i].base);
	return 0;
}

static int ioreg_read_internal(unsigned int __iomem *base, uint64_t offset,
			       int value_bits, uint64_t *value)
{
	switch (value_bits) {
	case 8:
		*value = readb((void *)base + offset);
		break;
	case 16:
		*value = readw((void *)base + offset);
		break;
	case 32:
		*value = readl((void *)base + offset);
		break;
	case 64:
		*value = readq((void *)base + offset);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ioreg_write_internal(unsigned int __iomem *base, uint64_t offset,
				int value_bits, uint64_t value)
{
	switch (value_bits) {
	case 8:
		writeb((uint8_t)value, (void *)base + offset);
		break;
	case 16:
		writew((uint16_t)value, (void *)base + offset);
		break;
	case 32:
		writel((uint32_t)value, (void *)base + offset);
		break;
	case 64:
		writeq(value, (void *)base + offset);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int lwis_ioreg_read_batch(struct lwis_ioreg_device *ioreg_dev,
			  struct lwis_io_entry *entries, int num_entries,
			  bool non_blocking)
{
	int i;
	int ret = 0;
	struct lwis_ioreg *block;
	struct lwis_ioreg_list *list;
	int index;

	BUG_ON(!ioreg_dev);

	list = &ioreg_dev->reg_list;

	if (!entries || num_entries <= 0) {
		return -EINVAL;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	for (i = 0; i < num_entries; ++i) {
		index = entries[i].rw.bid;
		if (index < 0 || index >= list->count) {
			pr_err("Invalid ioreg read block index %d\n", index);
			ret = -EINVAL;
			goto read_func_end;
		}
		block = &list->block[index];
		if (block->base == NULL) {
			pr_err("Invalid ioreg read block base undefined\n");
			ret = -EINVAL;
			goto read_func_end;
		}
		ret = ioreg_read_internal(
			block->base, entries[i].rw.offset,
			ioreg_dev->base_dev.reg_value_bitwidth,
			&entries[i].rw.val);
		if (ret) {
			pr_err("Invalid ioreg read:\n");
			pr_err("Offset: 0x%x, Base: %p\n", entries[i].rw.offset,
			       block->base);
			goto read_func_end;
		}
	}

read_func_end:
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	return ret;
}

int lwis_ioreg_write_batch(struct lwis_ioreg_device *ioreg_dev,
			   struct lwis_io_entry *entries, int num_entries,
			   bool non_blocking)
{
	int i;
	int ret = 0;
	struct lwis_ioreg *block;
	struct lwis_ioreg_list *list;
	int index;

	BUG_ON(!ioreg_dev);

	list = &ioreg_dev->reg_list;

	if (!entries || num_entries <= 0) {
		return -EINVAL;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	for (i = 0; i < num_entries; ++i) {
		index = entries[i].rw.bid;
		if (index < 0 || index >= list->count) {
			pr_err("Invalid ioreg write block index %d\n", index);
			ret = -EINVAL;
			goto write_func_end;
		}
		block = &list->block[index];
		if (block->base == NULL) {
			pr_err("Invalid ioreg write block base undefined\n");
			ret = -EINVAL;
			goto write_func_end;
		}
		ret = ioreg_write_internal(
			block->base, entries[i].rw.offset,
			ioreg_dev->base_dev.reg_value_bitwidth,
			entries[i].rw.val);
		if (ret) {
			pr_err("Invalid ioreg write\n");
			pr_err("Offset: 0x%x, Value: 0x%x, Base: %p\n",
			       entries[i].rw.offset, entries[i].rw.val,
			       block->base);
			goto write_func_end;
		}
	}

write_func_end:
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	return ret;
}

int lwis_ioreg_read_by_block_idx(struct lwis_ioreg_device *ioreg_dev, int index,
				 uint64_t offset, uint64_t *value,
				 bool non_blocking)
{
	struct lwis_ioreg *block;
	struct lwis_ioreg_list *list;
	int ret;

	BUG_ON(!ioreg_dev);

	list = &ioreg_dev->reg_list;
	if (index < 0 || index >= list->count) {
		return -EINVAL;
	}

	block = &list->block[index];
	if (!block->base) {
		return -EINVAL;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	ret = ioreg_read_internal(block->base, offset,
				  ioreg_dev->base_dev.reg_value_bitwidth,
				  value);
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	return ret;
}

int lwis_ioreg_read_by_block_name(struct lwis_ioreg_device *ioreg_dev,
				  char *name, uint64_t offset, uint64_t *value,
				  bool non_blocking)
{
	int i, bidx;
	int ret;
	struct lwis_ioreg_list *list;

	BUG_ON(!ioreg_dev);

	list = &ioreg_dev->reg_list;
	bidx = find_block_idx_by_name(list, name);
	if (bidx < 0) {
		return bidx;
	}
	if (list->block[bidx].base == NULL) {
		return -EINVAL;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	ret = ioreg_read_internal(list->block[i].base, offset,
				  ioreg_dev->base_dev.reg_value_bitwidth,
				  value);
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	return ret;
}

int lwis_ioreg_write_by_block_idx(struct lwis_ioreg_device *ioreg_dev,
				  int index, uint64_t offset, uint64_t value,
				  bool non_blocking)
{
	struct lwis_ioreg *block;
	struct lwis_ioreg_list *list;
	int ret;

	BUG_ON(!ioreg_dev);

	list = &ioreg_dev->reg_list;
	if (index < 0 || index >= list->count) {
		return -EINVAL;
	}

	block = &list->block[index];
	if (!block->base) {
		return -EINVAL;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	ret = ioreg_write_internal(block->base, offset,
				   ioreg_dev->base_dev.reg_value_bitwidth,
				   value);
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	return ret;
}

int lwis_ioreg_write_by_block_name(struct lwis_ioreg_device *ioreg_dev,
				   char *name, uint64_t offset, uint64_t value,
				   bool non_blocking)
{
	int i, bidx;
	int ret;
	struct lwis_ioreg_list *list;

	BUG_ON(!ioreg_dev);

	list = &ioreg_dev->reg_list;
	bidx = find_block_idx_by_name(list, name);
	if (bidx < 0) {
		return bidx;
	}
	if (list->block[bidx].base == NULL) {
		return -EINVAL;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	ret = ioreg_write_internal(list->block[i].base, offset,
				   ioreg_dev->base_dev.reg_value_bitwidth,
				   value);
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	return ret;
}
