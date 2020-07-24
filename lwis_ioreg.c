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

#include <linux/bitops.h>
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

static int validate_offset(struct lwis_ioreg *block, uint64_t offset,
			   size_t size_in_bytes)
{
	if (offset + size_in_bytes > block->size) {
		pr_err("Accessing invalid address! Block size is %d.\n",
		       block->size);
		pr_err("Offset %llu, size_in_bytes %zu, will be out of bound.\n",
		       offset, size_in_bytes);
		return -EFAULT;
	}
	return 0;
}

static int validate_access_size(int access_size, int native_value_bitwidth)
{
	if (access_size != native_value_bitwidth) {
		if (access_size != 8 && access_size != 16 &&
		    access_size != 32 && access_size != 64) {
			pr_err("Access size must be 8, 16, 32 or 64 - Actual %d\n",
			       access_size);
			return -EINVAL;
		}
		if (access_size > native_value_bitwidth) {
			pr_err("Access size (%d) > bitwidth (%d) is not supported yet\n",
			       access_size, native_value_bitwidth);
			return -ENOSYS;
		}
	}
	return 0;
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
	int bidx;
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

	devm_iounmap(dev, list->block[bidx].base);
	return 0;
}

static int ioreg_read_batch_internal(void __iomem *base, uint64_t offset,
				     int value_bits, size_t size_in_bytes,
				     uint8_t *buf)
{
	int i;
	uint8_t *addr = (uint8_t *)base + offset;

	if (size_in_bytes & (value_bits / 8) - 1) {
		pr_err("Read buf size (%zu) not divisible by %d (bitwidth = %d)\n",
		       size_in_bytes, value_bits / 8, value_bits);
		return -EINVAL;
	}

	switch (value_bits) {
	case 8:
		for (i = 0; i < size_in_bytes; ++i) {
			*(buf + i) = readb((void __iomem *)(addr + i));
		}
		break;
	case 16:
		for (i = 0; i < size_in_bytes; i += 2) {
			*(uint16_t *)(buf + i) =
				readw((void __iomem *)(addr + i));
		}
		break;
	case 32:
		for (i = 0; i < size_in_bytes; i += 4) {
			*(uint32_t *)(buf + i) =
				readl((void __iomem *)(addr + i));
		}
		break;
	case 64:
		for (i = 0; i < size_in_bytes; i += 8) {
			*(uint64_t *)(buf + i) =
				readq((void __iomem *)(addr + i));
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ioreg_write_batch_internal(void __iomem *base, uint64_t offset,
				      int value_bits, size_t size_in_bytes,
				      uint8_t *buf)
{
	int i;
	uint8_t *addr = (uint8_t *)base + offset;

	if (size_in_bytes & (value_bits / 8) - 1) {
		pr_err("Write buf size (%zu) not divisible by %d (bitwidth = %d)\n",
		       size_in_bytes, value_bits / 8, value_bits);
		return -EINVAL;
	}

	switch (value_bits) {
	case 8:
		for (i = 0; i < size_in_bytes; ++i) {
			writeb(*(buf + i), (void __iomem *)(addr + i));
		}
		break;
	case 16:
		for (i = 0; i < size_in_bytes; i += 2) {
			writew(*(uint16_t *)(buf + i),
			       (void __iomem *)(addr + i));
		}
		break;
	case 32:
		for (i = 0; i < size_in_bytes; i += 4) {
			writel(*(uint32_t *)(buf + i),
			       (void __iomem *)(addr + i));
		}
		break;
	case 64:
		for (i = 0; i < size_in_bytes; i += 8) {
			writeq(*(uint64_t *)(buf + i),
			       (void __iomem *)(addr + i));
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ioreg_read_internal(void __iomem *base, uint64_t offset,
			       int value_bits, uint64_t *value)
{
	void __iomem *addr = (void __iomem *)((uint8_t *)base + offset);
	switch (value_bits) {
	case 8:
		*value = readb(addr);
		break;
	case 16:
		*value = readw(addr);
		break;
	case 32:
		*value = readl(addr);
		break;
	case 64:
		*value = readq(addr);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ioreg_write_internal(void __iomem *base, uint64_t offset,
				int value_bits, uint64_t value)
{
	void __iomem *addr = (void __iomem *)((uint8_t *)base + offset);
	switch (value_bits) {
	case 8:
		writeb((uint8_t)value, addr);
		break;
	case 16:
		writew((uint16_t)value, addr);
		break;
	case 32:
		writel((uint32_t)value, addr);
		break;
	case 64:
		writeq(value, addr);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int lwis_ioreg_io_entry_rw(struct lwis_ioreg_device *ioreg_dev,
			   struct lwis_io_entry *entry, bool non_blocking,
			   int access_size)
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
				      /*non_blocking=*/true, access_size);
	} else if (entry->type == LWIS_IO_ENTRY_READ_BATCH) {
		index = entry->rw_batch.bid;
		block = get_block_by_idx(ioreg_dev, index);
		if (IS_ERR_OR_NULL(block)) {
			ret = PTR_ERR(block);
			goto rw_func_end;
		}

		ret = validate_offset(block, entry->rw_batch.offset,
				      entry->rw_batch.size_in_bytes);
		if (ret) {
			goto rw_func_end;
		}

		ret = ioreg_read_batch_internal(
			block->base, entry->rw_batch.offset,
			ioreg_dev->base_dev.native_value_bitwidth,
			entry->rw_batch.size_in_bytes, entry->rw_batch.buf);
		if (ret) {
			pr_err("Invalid ioreg batch read at:\n");
			pr_err("Offset: 0x%llx, Base: %pK\n",
			       entry->rw_batch.offset, block->base);
		}
	} else if (entry->type == LWIS_IO_ENTRY_WRITE) {
		ret = lwis_ioreg_write(ioreg_dev, entry->rw.bid,
				       entry->rw.offset, entry->rw.val,
				       /*non_blocking=*/true, access_size);
	} else if (entry->type == LWIS_IO_ENTRY_WRITE_BATCH) {
		index = entry->rw_batch.bid;
		block = get_block_by_idx(ioreg_dev, index);
		if (IS_ERR_OR_NULL(block)) {
			ret = PTR_ERR(block);
			goto rw_func_end;
		}

		ret = validate_offset(block, entry->rw_batch.offset,
				      entry->rw_batch.size_in_bytes);
		if (ret) {
			goto rw_func_end;
		}

		ret = ioreg_write_batch_internal(
			block->base, entry->rw_batch.offset,
			ioreg_dev->base_dev.native_value_bitwidth,
			entry->rw_batch.size_in_bytes, entry->rw_batch.buf);
		if (ret) {
			pr_err("Invalid ioreg batch write at:\n");
			pr_err("Offset: 0x%08llx, Base: %pK\n",
			       entry->rw_batch.offset, block->base);
		}
	} else if (entry->type == LWIS_IO_ENTRY_MODIFY) {
		ret = lwis_ioreg_read(ioreg_dev, entry->mod.bid,
				      entry->mod.offset, &reg_value,
				      /*non_blocking=*/true, access_size);
		if (ret) {
			goto rw_func_end;
		}
		reg_value &= ~entry->mod.val_mask;
		reg_value |= entry->mod.val_mask & entry->mod.val;
		ret = lwis_ioreg_write(ioreg_dev, entry->mod.bid,
				       entry->mod.offset, reg_value,
				       /*non_blocking=*/true, access_size);
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
		    uint64_t offset, uint64_t *value, bool non_blocking,
		    int access_size)
{
	struct lwis_ioreg *block;
	int ret;
	uint64_t internal_offset = offset;
	unsigned int native_value_bitwidth;
	uint64_t offset_mask;

	BUG_ON(!ioreg_dev);

	block = get_block_by_idx(ioreg_dev, index);
	if (IS_ERR_OR_NULL(block)) {
		return PTR_ERR(block);
	}

	// Access_size is bitwidth
	// and validate_offset expects size of bytes
	ret = validate_offset(block, offset, access_size / 8);
	if (ret) {
		return ret;
	}

	native_value_bitwidth = ioreg_dev->base_dev.native_value_bitwidth;
	ret = validate_access_size(access_size, native_value_bitwidth);
	if (ret) {
		pr_err("Invalid access size\n");
		return ret;
	}
	if (access_size != native_value_bitwidth) {
		offset_mask = native_value_bitwidth / BITS_PER_BYTE - 1;
		internal_offset = offset & ~offset_mask;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	ret = ioreg_read_internal(block->base, internal_offset,
				  native_value_bitwidth, value);
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}

	if (access_size != native_value_bitwidth) {
		*value >>= (offset - internal_offset) * BITS_PER_BYTE;
		if (access_size < BITS_PER_TYPE(uint64_t)) {
			*value &= ((1ULL << access_size) - 1);
		}
	}

	return ret;
}

int lwis_ioreg_write(struct lwis_ioreg_device *ioreg_dev, int index,
		     uint64_t offset, uint64_t value, bool non_blocking,
		     int access_size)
{
	struct lwis_ioreg *block;
	int ret;
	uint64_t internal_offset = offset;
	unsigned int native_value_bitwidth;
	uint64_t read_value;
	uint64_t offset_mask;
	uint64_t value_mask;

	BUG_ON(!ioreg_dev);

	block = get_block_by_idx(ioreg_dev, index);
	if (IS_ERR_OR_NULL(block)) {
		return PTR_ERR(block);
	}

	// Access_size is bitwidth
	// and validate_offset expects size of bytes
	ret = validate_offset(block, offset, access_size / 8);
	if (ret) {
		return ret;
	}

	native_value_bitwidth = ioreg_dev->base_dev.native_value_bitwidth;
	ret = validate_access_size(access_size, native_value_bitwidth);
	if (ret) {
		pr_err("Invalid access size\n");
		return ret;
	}

	if (!non_blocking) {
		mutex_lock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	if (access_size != native_value_bitwidth) {
		offset_mask = native_value_bitwidth / BITS_PER_BYTE - 1;
		internal_offset = offset & ~offset_mask;
		value_mask = ((1 << access_size) - 1);
		value_mask <<= (offset - internal_offset) * BITS_PER_BYTE;
		/* We need to read-modify-write in this case */
		ioreg_read_internal(block->base, internal_offset,
				    native_value_bitwidth, &read_value);
		value <<= (offset - internal_offset) * 8;
		value = (value & value_mask) | (read_value & ~value_mask);
	}
	ret = ioreg_write_internal(block->base, internal_offset,
				   native_value_bitwidth, value);
	if (!non_blocking) {
		mutex_unlock(&ioreg_dev->base_dev.reg_rw_lock);
	}
	return ret;
}
