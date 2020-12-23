// SPDX-License-Identifier: GPL-2.0
/*
 * EdgeTPU support for dma-buf.
 *
 * Copyright (C) 2020 Google, Inc.
 */

#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/ktime.h>
#include <linux/list.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sync_file.h>
#include <linux/time64.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include "edgetpu-device-group.h"
#include "edgetpu-dmabuf.h"
#include "edgetpu-dram.h"
#include "edgetpu-internal.h"
#include "edgetpu-mapping.h"
#include "edgetpu-mmu.h"
#include "edgetpu.h"

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
	/*
	 * The array length of @dmabufs is
	 *   * 1, for normal dmabuf mappings
	 *   * number of devices in @group, for bulk mappings
	 */
	struct dma_buf **dmabufs;
	/*
	 * The length of array @entries will be
	 *   * 1, for non-mirrored normal mapping requests
	 *   * number of devices in @group, otherwise
	 */
	struct dmabuf_map_entry *entries;
	uint num_entries;
};

/*
 * edgetpu implementation of DMA fence
 *
 * @fence:		the base DMA fence
 * @lock:		spinlock protecting updates to @fence
 * @timeline_name:	name of the timeline associated with the fence
 *
 * It is likely timelines will become a separate object in the future,
 * but for now there's a unique named timeline associated with each fence.
 */
struct edgetpu_dma_fence {
	struct dma_fence fence;
	spinlock_t lock;
	char timeline_name[EDGETPU_SYNC_TIMELINE_NAME_LEN];
	struct list_head etfence_list;
};

/* List of all edgetpu fence objects for debugging. */
static LIST_HEAD(etfence_list_head);
static DEFINE_SPINLOCK(etfence_list_lock);

static const struct dma_fence_ops edgetpu_dma_fence_ops;

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

/*
 * Maps to the first entry in @dmap.
 *
 * If the first entry is not fetched (this could happen in bulk mappings),
 * a TPU VA is still allocated according to @dmap->mmu_flags but not mapped.
 *
 * Caller holds @group->lock.
 */
static int etdev_map_dmabuf(struct edgetpu_dev *etdev,
			    struct edgetpu_dmabuf_map *dmap,
			    enum dma_data_direction dir, tpu_addr_t *tpu_addr_p)
{
	struct edgetpu_device_group *group = dmap->map.priv;
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct dmabuf_map_entry *entry = &dmap->entries[0];
	tpu_addr_t tpu_addr;
	int ret;

	if (entry->n == 0) {
		/*
		 * This could only happen in bulk mappings, when the dmabuf for
		 * master device is not set (runtime passes EDGETPU_IGNORE_FD).
		 */
		tpu_addr =
			edgetpu_mmu_alloc(etdev, dmap->size, dmap->mmu_flags);
		if (!tpu_addr)
			return -ENOSPC;
	} else if (entry->n == 1) {
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

/*
 * Reverts etdev_map_dmabuf().
 *
 * Caller holds @group->lock.
 */
static void etdev_unmap_dmabuf(struct edgetpu_dev *etdev,
			       struct edgetpu_dmabuf_map *dmap,
			       tpu_addr_t tpu_addr)
{
	struct edgetpu_device_group *group = dmap->map.priv;
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct dmabuf_map_entry *entry = &dmap->entries[0];

	if (entry->n == 0) {
		edgetpu_mmu_free(etdev, tpu_addr, dmap->size);
	} else if (entry->n == 1) {
		edgetpu_mmu_tpu_unmap(etdev, tpu_addr, dmap->size, ctx_id);
	} else {
		edgetpu_mmu_remove_translation(etdev, tpu_addr, dmap->size,
					       ctx_id);
		edgetpu_mmu_free(etdev, tpu_addr, dmap->size);
	}
}

/*
 * Handles mirrored mapping request.
 *
 * Caller holds @group->lock.
 */
static int group_map_dmabuf(struct edgetpu_device_group *group,
			    struct edgetpu_dmabuf_map *dmap,
			    enum dma_data_direction dir, tpu_addr_t *tpu_addr_p)
{
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct edgetpu_dev *etdev = group->etdev;
	tpu_addr_t tpu_addr;
	uint i;
	int ret;

	ret = etdev_map_dmabuf(etdev, dmap, dir, &tpu_addr);
	if (ret)
		return ret;
	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		if (dmap->entries[i].n == 0) {
			edgetpu_mmu_reserve(etdev, tpu_addr, dmap->size);
			continue;
		}
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
		if (dmap->entries[i].n == 0)
			edgetpu_mmu_free(etdev, tpu_addr, dmap->size);
		else
			edgetpu_mmu_remove_translation(etdev, tpu_addr,
						       dmap->size, ctx_id);
	}
	etdev_unmap_dmabuf(group->etdev, dmap, tpu_addr);

	return ret;
}

/*
 * Reverts group_map_dmabuf().
 *
 * Caller holds @group->lock.
 */
static void group_unmap_dmabuf(struct edgetpu_device_group *group,
			       struct edgetpu_dmabuf_map *dmap,
			       tpu_addr_t tpu_addr)
{
	const enum edgetpu_context_id ctx_id =
		edgetpu_group_context_id_locked(group);
	struct edgetpu_dev *etdev;
	uint i;

	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		edgetpu_mmu_remove_translation(etdev, tpu_addr, dmap->size,
					       ctx_id);
	}
	etdev_unmap_dmabuf(group->etdev, dmap, tpu_addr);
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
			dma_buf_detach(dmap->dmabufs[0], entry->attachment);
	}
	dma_buf_put(dmap->dmabufs[0]);
	edgetpu_device_group_put(group);
	kfree(dmap->dmabufs);
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
			dmap->dmabufs[0]->exp_name, map->device_address,
			dmap->size / PAGE_SIZE, edgetpu_dma_dir_rw_s(map->dir),
			dmap->offset);
	else
		seq_printf(
			s,
			"  <%s> die %u: iova=0x%llx pages=%llu %s offset=0x%llx",
			dmap->dmabufs[0]->exp_name, map->die_index,
			map->device_address, dmap->size / PAGE_SIZE,
			edgetpu_dma_dir_rw_s(map->dir), dmap->offset);

	edgetpu_device_dram_dmabuf_info_show(dmap->dmabufs[0], s);
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
	dmap->dmabufs = kcalloc(1, sizeof(*dmap->dmabufs), GFP_KERNEL);
	if (!dmap->dmabufs)
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
	kfree(dmap->dmabufs);
	kfree(dmap->entries);
	kfree(dmap);
	return NULL;
}

/*
 * Clean resources recorded in edgetpu_dmabuf_map, bulk version.
 *
 * Caller holds the lock of group (map->priv) and ensures the group is in
 * the finalized state.
 */
static void dmabuf_bulk_map_callback_release(struct edgetpu_mapping *map)
{
	struct edgetpu_dmabuf_map *bmap =
		container_of(map, struct edgetpu_dmabuf_map, map);
	struct edgetpu_device_group *group = map->priv;
	const enum dma_data_direction dir = edgetpu_host_dma_dir(map->dir);
	const tpu_addr_t tpu_addr = map->device_address;
	int i;

	if (tpu_addr)
		group_unmap_dmabuf(group, bmap, tpu_addr);
	for (i = 0; i < group->n_clients; i++) {
		struct dmabuf_map_entry *entry = &bmap->entries[i];

		kfree(entry->dma_addrs);
		if (entry->sgt)
			dma_buf_unmap_attachment(entry->attachment, entry->sgt,
						 dir);
		if (entry->attachment)
			dma_buf_detach(bmap->dmabufs[i], entry->attachment);
		if (bmap->dmabufs[i])
			dma_buf_put(bmap->dmabufs[i]);
	}
	edgetpu_device_group_put(group);
	kfree(bmap->dmabufs);
	kfree(bmap->entries);
	kfree(bmap);
}

static void dmabuf_bulk_map_callback_show(struct edgetpu_mapping *map,
					  struct seq_file *s)
{
	struct edgetpu_dmabuf_map *bmap =
		container_of(map, struct edgetpu_dmabuf_map, map);
	int i;

	seq_printf(s, "  bulk: iova=0x%llx pages=%llu %s\n",
		   map->device_address, bmap->size / PAGE_SIZE,
		   edgetpu_dma_dir_rw_s(map->dir));
	for (i = 0; i < bmap->num_entries; i++) {
		if (!bmap->dmabufs[i]) {
			seq_printf(s, "   %2d: ignored\n", i);
			continue;
		}
		seq_printf(s, "   %2d: <%s>", i, bmap->dmabufs[i]->exp_name);
		edgetpu_device_dram_dmabuf_info_show(bmap->dmabufs[i], s);
		seq_puts(s, " dma=");
		entry_show_dma_addrs(&bmap->entries[i], s);
	}
}

/*
 * Allocates and properly sets fields of an edgetpu_dmabuf_map, bulk version.
 *
 * Caller holds group->lock and checks @group is finalized.
 *
 * Returns the pointer on success, or NULL on failure.
 */
static struct edgetpu_dmabuf_map *
alloc_dmabuf_bulk_map(struct edgetpu_device_group *group,
		      edgetpu_map_flag_t flags)
{
	struct edgetpu_dmabuf_map *bmap = kzalloc(sizeof(*bmap), GFP_KERNEL);
	struct edgetpu_mapping *map;
	const uint n = group->n_clients;

	if (!bmap)
		return NULL;
	bmap->entries = kcalloc(n, sizeof(*bmap->entries), GFP_KERNEL);
	if (!bmap->entries)
		goto err_free;
	bmap->dmabufs = kcalloc(n, sizeof(*bmap->dmabufs), GFP_KERNEL);
	if (!bmap->dmabufs)
		goto err_free;
	/* mirroredness is ignored in bulk mappings */
	bmap->mmu_flags = map_to_mmu_flags(flags & ~EDGETPU_MAP_NONMIRRORED) |
			  EDGETPU_MMU_DMABUF | EDGETPU_MMU_VDG;
	bmap->num_entries = n;
	map = &bmap->map;
	map->die_index = ALL_DIES;
	map->flags = flags;
	map->dir = flags & EDGETPU_MAP_DIR_MASK;
	map->release = dmabuf_bulk_map_callback_release;
	map->show = dmabuf_bulk_map_callback_show;
	map->priv = edgetpu_device_group_get(group);
	return bmap;

err_free:
	kfree(bmap->dmabufs);
	kfree(bmap->entries);
	kfree(bmap);
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
	if (!edgetpu_group_finalized_and_attached(group))
		goto err_unlock_group;

	dmap = alloc_dmabuf_map(group, flags);
	if (!dmap) {
		ret = -ENOMEM;
		goto err_unlock_group;
	}

	get_dma_buf(dmabuf);
	dmap->dmabufs[0] = dmabuf;
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
	if (!edgetpu_group_finalized_and_attached(group))
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
	/* use the callback to handle both normal and bulk requests */
	map->release(map);
	ret = 0;
out_unlock:
	mutex_unlock(&group->lock);
	return ret;
}

int edgetpu_map_bulk_dmabuf(struct edgetpu_device_group *group,
			    struct edgetpu_map_bulk_dmabuf_ioctl *arg)
{
	const enum dma_data_direction dir =
		edgetpu_host_dma_dir(arg->flags & EDGETPU_MAP_DIR_MASK);
	int ret = -EINVAL;
	struct edgetpu_dmabuf_map *bmap;
	struct dma_buf *dmabuf;
	struct edgetpu_dev *etdev;
	tpu_addr_t tpu_addr;
	int i;

	if (arg->size == 0)
		return -EINVAL;
	mutex_lock(&group->lock);
	if (!edgetpu_group_finalized_and_attached(group))
		goto err_unlock_group;
	/* checks not all FDs are ignored */
	for (i = 0; i < group->n_clients; i++)
		if (arg->dmabuf_fds[i] != EDGETPU_IGNORE_FD)
			break;
	if (i == group->n_clients)
		goto err_unlock_group;
	bmap = alloc_dmabuf_bulk_map(group, arg->flags);
	if (!bmap) {
		ret = -ENOMEM;
		goto err_unlock_group;
	}
	for (i = 0; i < group->n_clients; i++) {
		if (arg->dmabuf_fds[i] != EDGETPU_IGNORE_FD) {
			dmabuf = dma_buf_get(arg->dmabuf_fds[i]);
			if (IS_ERR(dmabuf)) {
				ret = PTR_ERR(dmabuf);
				goto err_release_bmap;
			}
			if (arg->size > dmabuf->size)
				goto err_release_bmap;
			bmap->dmabufs[i] = dmabuf;
		}
	}
	bmap->size = arg->size;
	for (i = 0; i < group->n_clients; i++) {
		if (!bmap->dmabufs[i])
			continue;
		etdev = edgetpu_device_group_nth_etdev(group, i);
		ret = etdev_attach_dmabuf_to_entry(etdev, bmap->dmabufs[i],
						   &bmap->entries[i], 0,
						   arg->size, dir);
		if (ret)
			goto err_release_bmap;
	}
	ret = group_map_dmabuf(group, bmap, dir, &tpu_addr);
	if (ret)
		goto err_release_bmap;
	bmap->map.device_address = tpu_addr;
	ret = edgetpu_mapping_add(&group->dmabuf_mappings, &bmap->map);
	if (ret)
		goto err_release_bmap;
	arg->device_address = tpu_addr;
	mutex_unlock(&group->lock);
	return 0;

err_release_bmap:
	/* also releases entries if they are set */
	dmabuf_bulk_map_callback_release(&bmap->map);
err_unlock_group:
	mutex_unlock(&group->lock);
	return ret;
}

/* reverts edgetpu_map_bulk_dmabuf */
int edgetpu_unmap_bulk_dmabuf(struct edgetpu_device_group *group,
			      tpu_addr_t tpu_addr)
{
	return edgetpu_unmap_dmabuf(group, ALL_DIES, tpu_addr);
}

static struct edgetpu_dma_fence *to_etfence(struct dma_fence *fence)
{
	struct edgetpu_dma_fence *etfence;

	etfence = container_of(fence, struct edgetpu_dma_fence, fence);
	if (fence->ops != &edgetpu_dma_fence_ops)
		return NULL;

	return etfence;
}

static const char *edgetpu_dma_fence_get_driver_name(struct dma_fence *fence)
{
	return "edgetpu";
}

static const char *edgetpu_dma_fence_get_timeline_name(struct dma_fence *fence)
{
	struct edgetpu_dma_fence *etfence = to_etfence(fence);

	return etfence->timeline_name;
}

static void edgetpu_dma_fence_release(struct dma_fence *fence)
{
	struct edgetpu_dma_fence *etfence = to_etfence(fence);
	unsigned long flags;

	if (!fence)
		return;

	spin_lock_irqsave(&etfence_list_lock, flags);
	list_del(&etfence->etfence_list);
	spin_unlock_irqrestore(&etfence_list_lock, flags);
	kfree(etfence);
}

static bool edgetpu_dma_fence_enable_signaling(struct dma_fence *fence)
{
	return true;
}

static const struct dma_fence_ops edgetpu_dma_fence_ops = {
	.get_driver_name = edgetpu_dma_fence_get_driver_name,
	.get_timeline_name = edgetpu_dma_fence_get_timeline_name,
	.wait = dma_fence_default_wait,
	.enable_signaling = edgetpu_dma_fence_enable_signaling,
	.release = edgetpu_dma_fence_release,
};

/* the data type of fence->seqno is u64 in 5.1 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 1, 0)
#define SEQ_FMT "%u"
#else
#define SEQ_FMT "%llu"
#endif

int edgetpu_sync_fence_create(struct edgetpu_create_sync_fence_data *datap)
{
	int fd;
	int ret;
	struct edgetpu_dma_fence *etfence;
	struct sync_file *sync_file;
	unsigned long flags;

	etfence = kzalloc(sizeof(*etfence), GFP_KERNEL);
	if (!etfence)
		return -ENOMEM;

	spin_lock_init(&etfence->lock);
	memcpy(&etfence->timeline_name, &datap->timeline_name,
	       EDGETPU_SYNC_TIMELINE_NAME_LEN - 1);

	dma_fence_init(&etfence->fence, &edgetpu_dma_fence_ops,
		       &etfence->lock, dma_fence_context_alloc(1),
		       datap->seqno);

	sync_file = sync_file_create(&etfence->fence);
	dma_fence_put(&etfence->fence);
	if (!sync_file) {
		ret = -ENOMEM;
		goto err_freefence;
	}

	fd = get_unused_fd_flags(O_CLOEXEC);
	datap->fence = fd;
	fd_install(fd, sync_file->file);
	spin_lock_irqsave(&etfence_list_lock, flags);
	list_add_tail(&etfence->etfence_list, &etfence_list_head);
	spin_unlock_irqrestore(&etfence_list_lock, flags);
	return 0;

err_freefence:
	kfree(etfence);
	return ret;
}

int edgetpu_sync_fence_signal(struct edgetpu_signal_sync_fence_data *datap)
{
	struct dma_fence *fence;
	int errno;
	int ret;

	fence = sync_file_get_fence(datap->fence);
	if (!fence)
		return -EINVAL;

	errno = datap->error;
	if (errno > 0)
		errno = -errno;

	spin_lock_irq(fence->lock);
	pr_debug("%s: %s-%s%llu-" SEQ_FMT " errno=%d\n", __func__,
		 fence->ops->get_driver_name(fence),
		 fence->ops->get_timeline_name(fence), fence->context,
		 fence->seqno, errno);
	if (errno)
		dma_fence_set_error(fence, errno);
	ret = dma_fence_signal_locked(fence);
	spin_unlock_irq(fence->lock);
	dma_fence_put(fence);
	return ret;
}

int edgetpu_sync_fence_status(struct edgetpu_sync_fence_status *datap)
{
	struct dma_fence *fence;

	fence = sync_file_get_fence(datap->fence);
	if (!fence)
		return -EINVAL;

	datap->status = dma_fence_get_status(fence);
	dma_fence_put(fence);
	return 0;
}

static const char *sync_status_str(int status)
{
	if (status < 0)
		return "error";

	if (status > 0)
		return "signaled";

	return "active";
}

int edgetpu_sync_fence_debugfs_show(struct seq_file *s, void *unused)
{
	struct list_head *pos;

	spin_lock_irq(&etfence_list_lock);
	list_for_each(pos, &etfence_list_head) {
		struct edgetpu_dma_fence *etfence =
			container_of(pos, struct edgetpu_dma_fence,
				     etfence_list);
		struct dma_fence *fence = &etfence->fence;

		spin_lock_irq(&etfence->lock);
		seq_printf(s, "%s-%s %llu-" SEQ_FMT " %s",
			   edgetpu_dma_fence_get_driver_name(fence),
			   etfence->timeline_name, fence->context, fence->seqno,
			   sync_status_str(dma_fence_get_status_locked(fence)));

		if (test_bit(DMA_FENCE_FLAG_TIMESTAMP_BIT, &fence->flags)) {
			struct timespec64 ts64 =
				ktime_to_timespec64(fence->timestamp);

			seq_printf(s, " @%lld.%09ld", (s64)ts64.tv_sec,
				   ts64.tv_nsec);
		}

		if (fence->error)
			seq_printf(s, " err=%d", fence->error);

		seq_putc(s, '\n');
		spin_unlock_irq(&etfence->lock);
	}

	spin_unlock_irq(&etfence_list_lock);
	return 0;
}
