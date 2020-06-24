// SPDX-License-Identifier: GPL-2.0
/*
 * Provides functions for mapping buffers backed by dma-buf onto EdgeTPU
 * devices.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "edgetpu-device-group.h"
#include "edgetpu-dram.h"
#include "edgetpu-internal.h"
#include "edgetpu-map-dmabuf.h"
#include "edgetpu-mapping.h"
#include "edgetpu-mmu.h"
#include "edgetpu.h"

#if IS_ENABLED(CONFIG_DMA_SHARED_BUFFER)

#if IS_ENABLED(CONFIG_ION_EXYNOS) && IS_ENABLED(CONFIG_EXYNOS_IOVMM)
#define WORKAROUND_EXYNOS_IOVMM
#endif

/*
 * Records objects for mapping a dma-buf to an edgetpu_dev.
 */
struct dmabuf_map_entry {
	struct dma_buf_attachment *attachment;
	/* SG table returned by dma_buf_map_attachment() */
	struct sg_table *sgt;
#ifdef WORKAROUND_EXYNOS_IOVMM
	/* modified @sgt for the workaround */
	struct sg_table *mapped_sgt;
#endif
	/* the DMA addresses mapped to */
	struct {
		dma_addr_t addr;
		size_t len;
	} *dma_addrs;
	uint n; /* length of @dma_addrs */
};

/*
 * Records the mapping and other fields needed for mapping a dma-buf to a device
 * group.
 */
struct edgetpu_dmabuf_map {
	struct edgetpu_mapping map;
	u64 offset;
	u64 size; /* size of this mapping in bytes */
	u32 mmu_flags;
	struct dma_buf *dmabuf;
	/*
	 * The length of array @entries will be
	 * - 1, for a non-mirrored mapping request
	 * - number of dies in @group, otherwise
	 */
	struct dmabuf_map_entry *entries;
	uint num_entries;
};

static int etdev_add_translations(struct edgetpu_dev *etdev,
				  tpu_addr_t tpu_addr,
				  struct dmabuf_map_entry *entry,
				  enum dma_data_direction dir,
				  enum edgetpu_context_id ctx_id)
{
	const int prot = __dma_dir_to_iommu_prot(dir);
	uint i;
	u64 offset = 0;
	int ret;

	for (i = 0; i < entry->n; i++) {
		ret = edgetpu_mmu_add_translation(etdev, tpu_addr + offset,
						  entry->dma_addrs[i].addr,
						  entry->dma_addrs[i].len, prot,
						  ctx_id);
		if (ret)
			goto rollback;
		offset += entry->dma_addrs[i].len;
	}
	return 0;

rollback:
	edgetpu_mmu_remove_translation(etdev, tpu_addr, offset, ctx_id);
	return ret;
}

/* Maps to the first entry in @dmap. */
static int etdev_map_dmabuf(struct edgetpu_dev *etdev,
			    struct edgetpu_dmabuf_map *dmap,
			    enum dma_data_direction dir, tpu_addr_t *tpu_addr_p)
{
	struct edgetpu_device_group *group = dmap->map.priv;
	const enum edgetpu_context_id ctx_id = edgetpu_group_context_id(group);
	struct dmabuf_map_entry *entry = &dmap->entries[0];
	tpu_addr_t tpu_addr;
	int ret;

	if (entry->n == 1) {
		/*
		 * Easy case - only one DMA address, we can use chip-dependent
		 * tpu_map to map and acquire the TPU VA.
		 */
		tpu_addr = edgetpu_mmu_tpu_map(etdev, entry->dma_addrs[0].addr,
					       dmap->size, dir, ctx_id,
					       dmap->mmu_flags);
		if (!tpu_addr)
			return -ENOSPC;
	} else {
		/*
		 * Maps multiple DMA addresses, only chips with an internal MMU
		 * can handle this.
		 */
		tpu_addr =
			edgetpu_mmu_alloc(etdev, dmap->size, dmap->mmu_flags);
		if (!tpu_addr)
			return -ENOSPC;
		ret = etdev_add_translations(etdev, tpu_addr, entry, dir,
					     ctx_id);
		if (ret) {
			edgetpu_mmu_free(etdev, tpu_addr, dmap->size);
			return ret;
		}
	}

	*tpu_addr_p = tpu_addr;
	return 0;
}

/* reverts etdev_map_dmabuf() */
static void etdev_unmap_dmabuf(struct edgetpu_dev *etdev,
			       struct edgetpu_dmabuf_map *dmap,
			       tpu_addr_t tpu_addr)
{
	struct edgetpu_device_group *group = dmap->map.priv;
	const enum edgetpu_context_id ctx_id = edgetpu_group_context_id(group);
	struct dmabuf_map_entry *entry = &dmap->entries[0];

	if (entry->n == 1) {
		edgetpu_mmu_tpu_unmap(etdev, tpu_addr, dmap->size, ctx_id);
	} else {
		edgetpu_mmu_remove_translation(etdev, tpu_addr, dmap->size,
					       ctx_id);
		edgetpu_mmu_free(etdev, tpu_addr, dmap->size);
	}
}

/* handles mirrored mapping request */
static int group_map_dmabuf(struct edgetpu_device_group *group,
			    struct edgetpu_dmabuf_map *dmap,
			    enum dma_data_direction dir, tpu_addr_t *tpu_addr_p)
{
	const enum edgetpu_context_id ctx_id = edgetpu_group_context_id(group);
	struct edgetpu_dev *etdev = group->etdev;
	tpu_addr_t tpu_addr;
	uint i;
	int ret;

	ret = etdev_map_dmabuf(etdev, dmap, dir, &tpu_addr);
	if (ret)
		return ret;
	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		ret = etdev_add_translations(etdev, tpu_addr, &dmap->entries[i],
					     dir, ctx_id);
		if (ret)
			goto err_remove;
	}
	*tpu_addr_p = tpu_addr;
	return 0;

err_remove:
	while (i > 1) {
		i--;
		etdev = edgetpu_device_group_nth_etdev(group, i);
		edgetpu_mmu_remove_translation(etdev, tpu_addr, dmap->size,
					       ctx_id);
	}
	etdev_unmap_dmabuf(group->etdev, dmap, tpu_addr);

	return ret;
}

/* reverts group_map_dmabuf() */
static void group_unmap_dmabuf(struct edgetpu_device_group *group,
			       struct edgetpu_dmabuf_map *dmap,
			       tpu_addr_t tpu_addr)
{
	const enum edgetpu_context_id ctx_id = edgetpu_group_context_id(group);
	struct edgetpu_dev *etdev;
	uint i;

	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		edgetpu_mmu_remove_translation(etdev, tpu_addr, dmap->size,
					       ctx_id);
	}
	edgetpu_mmu_tpu_unmap(group->etdev, tpu_addr, dmap->size, ctx_id);
}

/*
 * Clean resources recorded in @dmap.
 *
 * Caller holds the lock of group (map->priv) and ensures the group is in
 * the finalized state.
 */
static void dmabuf_map_callback_release(struct edgetpu_mapping *map)
{
	struct edgetpu_dmabuf_map *dmap =
		container_of(map, struct edgetpu_dmabuf_map, map);
	struct edgetpu_device_group *group = map->priv;
	const enum dma_data_direction dir = edgetpu_host_dma_dir(map->dir);
	const tpu_addr_t tpu_addr = map->device_address;
	struct edgetpu_dev *etdev;
	uint i;

	if (tpu_addr) {
		if (IS_MIRRORED(map->flags)) {
			group_unmap_dmabuf(group, dmap, tpu_addr);
		} else {
			etdev = edgetpu_device_group_nth_etdev(group,
							       map->die_index);
			etdev_unmap_dmabuf(etdev, dmap, tpu_addr);
		}
	}
	for (i = 0; i < dmap->num_entries; i++) {
		struct dmabuf_map_entry *entry = &dmap->entries[i];

#ifdef WORKAROUND_EXYNOS_IOVMM
		if (entry->mapped_sgt) {
			dma_unmap_sg(entry->attachment->dev,
				     entry->mapped_sgt->sgl,
				     entry->mapped_sgt->orig_nents, dir);
			sg_free_table(entry->mapped_sgt);
			kfree(entry->mapped_sgt);
		}
#endif
		kfree(entry->dma_addrs);
		if (entry->sgt)
			dma_buf_unmap_attachment(entry->attachment, entry->sgt,
						 dir);
		if (entry->attachment)
			dma_buf_detach(dmap->dmabuf, entry->attachment);
	}
	dma_buf_put(dmap->dmabuf);
	edgetpu_device_group_put(group);
	kfree(dmap->entries);
	kfree(dmap);
}

static void entry_show_dma_addrs(struct dmabuf_map_entry *entry,
				 struct seq_file *s)
{
	if (entry->n == 1) {
		seq_printf(s, "%pad\n", &entry->dma_addrs[0].addr);
	} else {
		uint i;

		seq_puts(s, "[");
		for (i = 0; i < entry->n; i++) {
			if (i)
				seq_puts(s, ", ");
			seq_printf(s, "%pad", &entry->dma_addrs[i].addr);
		}
		seq_puts(s, "]\n");
	}
}

static void dmabuf_map_callback_show(struct edgetpu_mapping *map,
				     struct seq_file *s)
{
	struct edgetpu_dmabuf_map *dmap =
		container_of(map, struct edgetpu_dmabuf_map, map);

	if (IS_MIRRORED(dmap->map.flags))
		seq_printf(
			s,
			"  <%s> mirrored: iova=0x%llx pages=%llu %s offset=0x%llx",
			dmap->dmabuf->exp_name, map->device_address,
			dmap->size / PAGE_SIZE, edgetpu_dma_dir_rw_s(map->dir),
			dmap->offset);
	else
		seq_printf(
			s,
			"  <%s> die %u: iova=0x%llx pages=%llu %s offset=0x%llx",
			dmap->dmabuf->exp_name, map->die_index,
			map->device_address, dmap->size / PAGE_SIZE,
			edgetpu_dma_dir_rw_s(map->dir), dmap->offset);

	edgetpu_device_dram_dmabuf_info_show(dmap->dmabuf, s);
	seq_puts(s, " dma=");
	entry_show_dma_addrs(&dmap->entries[0], s);
}

/*
 * Allocates and properly sets fields of an edgetpu_dmabuf_map.
 *
 * Caller holds group->lock and checks @group is finalized.
 *
 * Returns the pointer on success, or NULL on failure.
 */
static struct edgetpu_dmabuf_map *
alloc_dmabuf_map(struct edgetpu_device_group *group, edgetpu_map_flag_t flags)
{
	struct edgetpu_dmabuf_map *dmap = kzalloc(sizeof(*dmap), GFP_KERNEL);
	struct edgetpu_mapping *map;
	uint n;

	if (!dmap)
		return NULL;
	if (IS_MIRRORED(flags))
		n = group->n_clients;
	else
		n = 1;
	dmap->entries = kcalloc(n, sizeof(*dmap->entries), GFP_KERNEL);
	if (!dmap->entries)
		goto err_free;
	dmap->num_entries = n;
	dmap->mmu_flags = map_to_mmu_flags(flags) | EDGETPU_MMU_DMABUF;
	map = &dmap->map;
	map->flags = flags;
	map->dir = flags & EDGETPU_MAP_DIR_MASK;
	map->release = dmabuf_map_callback_release;
	map->show = dmabuf_map_callback_show;
	map->priv = edgetpu_device_group_get(group);
	return dmap;

err_free:
	kfree(dmap->entries);
	kfree(dmap);
	return NULL;
}

/*
 * Set @entry with one DMA address if we can use that address to present the DMA
 * addresses in @sgt.
 *
 * Returns 0 if succeeded.
 * Returns -EINVAL if the DMA addresses in @sgt is not contiguous.
 */
static int entry_set_one_dma(struct dmabuf_map_entry *entry,
			     const struct sg_table *sgt, u64 offset, u64 size)
{
	int i;
	struct scatterlist *sg;
	dma_addr_t addr;

	addr = sg_dma_address(sgt->sgl);
	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		if (sg_dma_len(sg) == 0)
			break;
		if (sg_dma_address(sg) != addr)
			return -EINVAL;
		addr += sg_dma_len(sg);
	}

	entry->dma_addrs = kmalloc(sizeof(*entry->dma_addrs), GFP_KERNEL);
	if (!entry->dma_addrs)
		return -ENOMEM;
	entry->n = 1;
	entry->dma_addrs[0].addr = sg_dma_address(sgt->sgl) + offset;
	entry->dma_addrs[0].len = size;

	return 0;
}

#ifdef WORKAROUND_EXYNOS_IOVMM

static struct sg_table *dup_sg_table(const struct sg_table *sgt)
{
	struct sg_table *new_sgt;
	int i;
	struct scatterlist *sg, *new_sg;
	int ret;

	new_sgt = kmalloc(sizeof(*new_sgt), GFP_KERNEL);
	if (!new_sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(new_sgt, sgt->nents, GFP_KERNEL);
	if (ret) {
		kfree(new_sgt);
		return ERR_PTR(ret);
	}
	new_sg = new_sgt->sgl;
	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		memcpy(new_sg, sg, sizeof(*sg));
		new_sg = sg_next(new_sg);
	}

	return new_sgt;
}

/*
 * Here assumes we are mapping the dmabuf backed by Exynos' ION, which doesn't
 * follow the standard dma-buf framework.
 *
 * This workaround is needed since dma_buf_map_attachment() of Exynos ION
 * doesn't call dma_map_sg(), and it expects drivers call ion_iovmm_map()
 * manually. However our EdgeTPU is not backed by Exynos's IOVMM, so we simply
 * call dma_map_sg() before setting @entry->dma_addrs.
 */
static int entry_set_dma_addrs(struct dmabuf_map_entry *entry, u64 offset,
			       u64 size, enum dma_data_direction dir)
{
	struct dma_buf_attachment *attach = entry->attachment;
	struct sg_table *sgt;
	int ret;

	sgt = dup_sg_table(entry->sgt);
	if (IS_ERR(sgt))
		return PTR_ERR(sgt);
	sgt->nents = dma_map_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
	if (sgt->nents == 0) {
		dev_err(attach->dev, "%s: dma_map_sg failed", __func__);
		ret = -EINVAL;
		goto err_free;
	}
	ret = entry_set_one_dma(entry, sgt, offset, size);
	if (ret) {
		dev_err(attach->dev,
			"%s: cannot map to one DMA addr, nents=%u, ret=%d",
			__func__, sgt->nents, ret);
		goto err_unmap;
	}
	entry->mapped_sgt = sgt;

	return 0;

err_unmap:
	dma_unmap_sg(attach->dev, sgt->sgl, sgt->orig_nents, dir);
err_free:
	sg_free_table(sgt);
	kfree(sgt);
	return ret;
}

#else /* !WORKAROUND_EXYNOS_IOVMM */

/*
 * Allocates @entry->dma_addrs and assigns DMA addresses in @sgt start from
 * @offset with size @size to @entry->dma_addrs.
 */
static int entry_set_dma_addrs(struct dmabuf_map_entry *entry, u64 offset,
			       u64 size, enum dma_data_direction dir)
{
	struct sg_table *sgt = entry->sgt;
	struct scatterlist *sg;
	u64 cur_offset = 0;
	uint n = 0;
	uint i;

	if (!entry_set_one_dma(entry, sgt, offset, size))
		return 0;
	/* calculate the number of sg covered by [offset, offset + size) */
	for_each_sg(sgt->sgl, sg, sgt->nents, i) {
		if (offset < cur_offset + sg_dma_len(sg))
			n++;
		if (offset + size <= cur_offset + sg_dma_len(sg))
			break;
		cur_offset += sg_dma_len(sg);
	}
	if (n == 0)
		return -EINVAL;
	entry->dma_addrs =
		kmalloc_array(n, sizeof(*entry->dma_addrs), GFP_KERNEL);
	if (!entry->dma_addrs)
		return -ENOMEM;
	entry->n = n;
	cur_offset = 0;
	i = 0;
	for (sg = sgt->sgl; sg;
	     cur_offset += sg_dma_len(sg), sg = sg_next(sg)) {
		u64 remain_size = offset + size - cur_offset;

		/* hasn't touched the first covered sg */
		if (offset >= cur_offset + sg_dma_len(sg))
			continue;
		entry->dma_addrs[i].addr = sg_dma_address(sg);
		entry->dma_addrs[i].len = sg_dma_len(sg);
		/* offset exceeds current sg */
		if (offset > cur_offset) {
			entry->dma_addrs[i].addr += offset - cur_offset;
			entry->dma_addrs[i].len -= offset - cur_offset;
		}
		if (remain_size <= sg_dma_len(sg)) {
			entry->dma_addrs[i].len -= sg_dma_len(sg) - remain_size;
			break;
		}
		i++;
	}

	return 0;
}

#endif /* WORKAROUND_EXYNOS_IOVMM */

/*
 * Performs dma_buf_attach + dma_buf_map_attachment of @dmabuf to @etdev, and
 * sets @entry per the attaching result.
 *
 * Fields of @entry will be set on success.
 */
static int etdev_attach_dmabuf_to_entry(struct edgetpu_dev *etdev,
					struct dma_buf *dmabuf,
					struct dmabuf_map_entry *entry,
					u64 offset, u64 size,
					enum dma_data_direction dir)
{
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	int ret;

	attachment = dma_buf_attach(dmabuf, etdev->dev);
	if (IS_ERR(attachment))
		return PTR_ERR(attachment);
	sgt = dma_buf_map_attachment(attachment, dir);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		goto err_detach;
	}
	entry->attachment = attachment;
	entry->sgt = sgt;
	ret = entry_set_dma_addrs(entry, offset, size, dir);
	if (ret)
		goto err_unmap;

	return 0;

err_unmap:
	dma_buf_unmap_attachment(attachment, sgt, dir);
err_detach:
	dma_buf_detach(dmabuf, attachment);
	entry->sgt = NULL;
	entry->attachment = NULL;
	return ret;
}

int edgetpu_map_dmabuf(struct edgetpu_device_group *group,
		       struct edgetpu_map_dmabuf_ioctl *arg)
{
	int ret = -EINVAL;
	struct dma_buf *dmabuf;
	edgetpu_map_flag_t flags = arg->flags;
	const u64 offset = arg->offset;
	const u64 size = arg->size;
	const enum dma_data_direction dir =
		edgetpu_host_dma_dir(flags & EDGETPU_MAP_DIR_MASK);
	struct edgetpu_dev *etdev;
	struct edgetpu_dmabuf_map *dmap;
	tpu_addr_t tpu_addr;
	uint i;

	/* offset is not page-aligned */
	if (offset_in_page(offset))
		return -EINVAL;
	/* size == 0 or overflow */
	if (offset + size <= offset)
		return -EINVAL;
	dmabuf = dma_buf_get(arg->dmabuf_fd);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);
	if (offset + size > dmabuf->size)
		goto err_put;

	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group))
		goto err_unlock_group;

	dmap = alloc_dmabuf_map(group, flags);
	if (!dmap) {
		ret = -ENOMEM;
		goto err_unlock_group;
	}

	get_dma_buf(dmabuf);
	dmap->dmabuf = dmabuf;
	dmap->offset = offset;
	dmap->size = size;
	if (IS_MIRRORED(flags)) {
		for (i = 0; i < group->n_clients; i++) {
			etdev = edgetpu_device_group_nth_etdev(group, i);
			ret = etdev_attach_dmabuf_to_entry(etdev, dmabuf,
							   &dmap->entries[i],
							   offset, size, dir);
			if (ret)
				goto err_release_map;
		}
		ret = group_map_dmabuf(group, dmap, dir, &tpu_addr);
		if (ret)
			goto err_release_map;
		dmap->map.die_index = ALL_DIES;
	} else {
		etdev = edgetpu_device_group_nth_etdev(group, arg->die_index);
		if (!etdev) {
			ret = -EINVAL;
			goto err_release_map;
		}
		ret = etdev_attach_dmabuf_to_entry(
			etdev, dmabuf, &dmap->entries[0], offset, size, dir);
		if (ret)
			goto err_release_map;
		ret = etdev_map_dmabuf(etdev, dmap, dir, &tpu_addr);
		if (ret)
			goto err_release_map;
		dmap->map.die_index = arg->die_index;
	}
	dmap->map.device_address = tpu_addr;
	ret = edgetpu_mapping_add(&group->dmabuf_mappings, &dmap->map);
	if (ret)
		goto err_release_map;
	arg->device_address = tpu_addr;
	mutex_unlock(&group->lock);
	dma_buf_put(dmabuf);
	return 0;

err_release_map:
	/* also releases entries if they are set */
	dmabuf_map_callback_release(&dmap->map);
err_unlock_group:
	mutex_unlock(&group->lock);
err_put:
	dma_buf_put(dmabuf);

	return ret;
}

int edgetpu_unmap_dmabuf(struct edgetpu_device_group *group, u32 die_index,
			 tpu_addr_t tpu_addr)
{
	struct edgetpu_mapping_root *mappings = &group->dmabuf_mappings;
	struct edgetpu_mapping *map;
	int ret = -EINVAL;

	mutex_lock(&group->lock);
	/* the group is disbanded means all the mappings have been released */
	if (!edgetpu_device_group_is_finalized(group))
		goto out_unlock;
	edgetpu_mapping_lock(mappings);
	map = edgetpu_mapping_find_locked(mappings, die_index, tpu_addr);
	if (!map)
		map = edgetpu_mapping_find_locked(mappings, ALL_DIES, tpu_addr);
	/* the mapping is not found */
	if (!map) {
		edgetpu_mapping_unlock(mappings);
		goto out_unlock;
	}
	edgetpu_mapping_unlink(mappings, map);
	edgetpu_mapping_unlock(mappings);
	dmabuf_map_callback_release(map);
	ret = 0;
out_unlock:
	mutex_unlock(&group->lock);
	return ret;
}

#else /* !CONFIG_DMA_SHARED_BUFFER */

int edgetpu_map_dmabuf(struct edgetpu_device_group *group,
		       struct edgetpu_map_dmabuf_ioctl *arg)
{
	return -ENOTTY;
}

int edgetpu_unmap_dmabuf(struct edgetpu_device_group *group, u32 die_index,
			 tpu_addr_t tpu_addr)
{
	return -ENOTTY;
}

#endif /* CONFIG_DMA_SHARED_BUFFER */
