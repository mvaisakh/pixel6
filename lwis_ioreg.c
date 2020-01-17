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

static struct lwis_ioreg *get_block_by_idx(struct lwis_ioreg_device *ioreg_dev,
					   int index)
{
	struct lwis_ioreg *block;
	struct lwis_ioreg_list *list;

	list = &ioreg_dev->reg_list;
	if (index < 0 || index >= list->count) {
		return ERR_PTR(-EINVAL);
	}

	block = &list->block[index];
	if (!block->base) {
		return ERR_PTR(-EINVAL);
	}

	return block;
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

static int ioreg_read_batch_internal(unsigned int __iomem *base,
				     uint64_t offset, int value_bits,
				     size_t size_in_bytes, uint8_t *buf)
{
	int i;

	if (size_in_bytes & (value_bits / 8) - 1) {
		pr_err("Read buf size (%d) not divisible by bitwidth (%d)\n",
		       size_in_bytes, value_bits);
		return -EINVAL;
	}

	switch (value_bits) {
	case 8:
		for (i = 0; i < size_in_bytes; ++i) {
			*(buf + i) = readb((void *)base + offset + i);
		}
		break;
	case 16:
		for (i = 0; i < size_in_bytes; i += 2) {
			*(uint16_t *)(buf + i) =
				readw((void *)base + offset + i);
		}
		break;
	case 32:
		for (i = 0; i < size_in_bytes; i += 4) {
			*(uint32_t *)(buf + i) =
				readl((void *)base + offset + i);
		}
		break;
	case 64:
		for (i = 0; i < size_in_bytes; i += 8) {
			*(uint64_t *)(buf + i) =
				readq((void *)base + offset + i);
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ioreg_write_batch_internal(unsigned int __iomem *base,
				      uint64_t offset, int value_bits,
				      size_t size_in_bytes, uint8_t *buf)
{
	int i;

	if (size_in_bytes & (value_bits / 8) - 1) {
		pr_err("Write buf size (%d) not divisible by bitwidth (%d)\n",
		       size_in_bytes, value_bits);
		return -EINVAL;
	}

	switch (value_bits) {
	case 8:
		for (i = 0; i < size_in_bytes; ++i) {
			writeb(*(buf + i), (void *)base + offset + i);
		}
		break;
	case 16:
		for (i = 0; i < size_in_bytes; i += 2) {
			writew(*(uint16_t *)(buf + i),
			       (void *)base + offset + i);
		}
		break;
	case 32:
		for (i = 0; i < size_in_bytes; i += 4) {
			writel(*(uint32_t *)(buf + i),
			       (void *)base + offset + i);
		}
		break;
	case 64:
		for (i = 0; i < size_in_bytes; i += 8) {
			writeq(*(uint64_t *)(buf + i),
			       (void *)base + offset + i);
		}
		break;
	default:
		return -EINVAL;
	}

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

int lwis_ioreg_io_entry_rw(struct lwis_ioreg_device *ioreg_dev,
			   struct lwis_io_entry *entry, bool non_blocking)
{
	int ret = 0;
	int index;
	struct lwis_ioreg *block;
	uint64_t reg_value;

	BUG_ON(!ioreg_dev);
	BUG_ON(!entry);

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	/* Non-blocking because we already locked here */
	if (entry->type == LWIS_IO_ENTRY_READ) {
		ret = lwis_ioreg_read(ioreg_dev, entry->rw.bid,
				      entry->rw.offset, &entry->rw.val,
				      /*non_blocking=*/true);
	} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH) {
		index = entry->rw_batch.bid;
		block = get_block_by_idx(ioreg_dev, index);
		if (IS_ERR_OR_NULL(block)) {
			ret = PTR_ERR(block);
			goto rw_func_end;
		}
		ret = ioreg_read_batch_internal(
			block->base, entry->rw_batch.offset,
			ioreg_dev->base_dev.reg_value_bitwidth,
			entry->rw_batch.size_in_bytes, entry->rw_batch.buf);
		if (ret) {
			pr_err("Invalid ioreg batch read at:\n");
			pr_err("Offset: 0x%x, Base: %p\n",
			       entry->rw_batch.offset, block->base);
		}
	} else if (entry->type == LWIS_IO_ENTRY_WRITE) {
		ret = lwis_ioreg_write(ioreg_dev, entry->rw.bid,
				       entry->rw.offset, entry->rw.val,
				       /*non_blocking=*/true);
	} else if (entry->type == LWIS_IO_ENTRY_WRITE_BATCH) {
		index = entry->rw_batch.bid;
		block = get_block_by_idx(ioreg_dev, index);
		if (IS_ERR_OR_NULL(block)) {
			ret = PTR_ERR(block);
			goto rw_func_end;
		}
		ret = ioreg_write_batch_internal(
			block->base, entry->rw_batch.offset,
			ioreg_dev->base_dev.reg_value_bitwidth,
			entry->rw_batch.size_in_bytes, entry->rw_batch.buf);
		if (ret) {
			pr_err("Invalid ioreg batch write at:\n");
			pr_err("Offset: 0x%x, Base: %p\n",
			       entry->rw_batch.offset, block->base);
		}
	} else if (entry->type == LWIS_IO_ENTRY_MODIFY) {
		ret = lwis_ioreg_read(ioreg_dev, entry->mod.bid,
				      entry->mod.offset, &reg_value,
				      /*non_blocking=*/true);
		if (ret) {
			goto rw_func_end;
		}
		reg_value &= ~entry->mod.val_mask;
		reg_value |= entry->mod.val_mask & entry->mod.val;
		ret = lwis_ioreg_write(ioreg_dev, entry->mod.bid,
				       entry->mod.offset, reg_value,
				       /*non_blocking=*/true);
	} else {
		pr_err("Invalid IO entry type: %d\n", entry->type);
		ret = -EINVAL;
	}

rw_func_end:
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	return ret;
}

int lwis_ioreg_read(struct lwis_ioreg_device *ioreg_dev, int index,
		    uint64_t offset, uint64_t *value, bool non_blocking)
{
	struct lwis_ioreg *block;
	int ret;

	BUG_ON(!ioreg_dev);

	block = get_block_by_idx(ioreg_dev, index);
	if (IS_ERR_OR_NULL(block)) {
		return PTR_ERR(block);
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

int lwis_ioreg_write(struct lwis_ioreg_device *ioreg_dev, int index,
		     uint64_t offset, uint64_t value, bool non_blocking)
{
	struct lwis_ioreg *block;
	int ret;

	BUG_ON(!ioreg_dev);

	block = get_block_by_idx(ioreg_dev, index);
	if (IS_ERR_OR_NULL(block)) {
		return PTR_ERR(block);
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
