// SPDX-License-Identifier: GPL-2.0
/*
 * Implements utilities for virtual device group of EdgeTPU.
 *
 * Copyright (C) 2019 Google, Inc.
 */

#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/eventfd.h>
#include <linux/iommu.h>
#include <linux/kconfig.h>
#include <linux/mm.h>
#include <linux/refcount.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "edgetpu-async.h"
#include "edgetpu-config.h"
#include "edgetpu-device-group.h"
#include "edgetpu-dram.h"
#include "edgetpu-internal.h"
#include "edgetpu-kci.h"
#include "edgetpu-mapping.h"
#include "edgetpu-mmu.h"
#include "edgetpu.h"

#ifdef EDGETPU_HAS_P2P_MAILBOX
#include "edgetpu-p2p-mailbox.h"
#endif

#define for_each_list_client(c, group) \
	list_for_each_entry(c, &group->clients, list)

#define for_each_list_client_safe(c, n, group) \
	list_for_each_entry_safe(c, n, &group->clients, list)

/* Records the mapping and other fields needed for a host buffer mapping */
struct edgetpu_host_map {
	struct edgetpu_mapping map;
	/*
	 * SG tables used for dma_map_sg() on each die in the device group.
	 *
	 * It's an array of SG tables if the mapping is a mirrored request,
	 * otherwise it's a NULL pointer.
	 * The first slot of @sg_tables is never used since the leader of a
	 * group uses @map->sgt as its SG table.
	 */
	struct sg_table *sg_tables;
};

#if !IS_ENABLED(CONFIG_ABROLHOS)

/* parameter to be used in async KCI jobs */
struct kci_worker_param {
	struct edgetpu_device_group *group;
	uint idx;
};

static int edgetpu_kci_join_group_worker(struct kci_worker_param *param)
{
	struct edgetpu_device_group *group = param->group;
	uint i = param->idx;
	struct edgetpu_dev *etdev = edgetpu_device_group_nth_etdev(group, i);

	etdev_dbg(etdev, "%s: join group %u %u/%u\n", __func__,
		  group->workload_id, i, group->n_clients);
	return edgetpu_kci_join_group(etdev->kci, etdev, group->n_clients, i);
}

static void edgetpu_kci_leave_group_worker(struct kci_worker_param *param)
{
	struct edgetpu_device_group *group = param->group;
	uint i = param->idx;
	struct edgetpu_dev *etdev = edgetpu_device_group_nth_etdev(group, i);

	etdev_dbg(etdev, "%s: leave group %u\n", __func__, group->workload_id);
	edgetpu_kci_leave_group(etdev->kci);
}

#endif /* CONFIG_ABROLHOS */

/*
 * Asynchronously sends LEAVE_GROUP KCI to all devices in @group.
 *
 * Caller holds group->lock.
 */
static void edgetpu_device_group_kci_leave(struct edgetpu_device_group *group)
{
#if IS_ENABLED(CONFIG_ABROLHOS)
	return;
#else
	struct kci_worker_param *params =
		kmalloc_array(group->n_clients, sizeof(*params), GFP_KERNEL);
	struct edgetpu_async_ctx *ctx = edgetpu_async_alloc_ctx();
	int i;
	int err;

	if (!params || !ctx)
		goto out_free;
	for (i = 0; i < group->n_clients; i++) {
		params[i].group = group;
		params[i].idx = i;
		err = edgetpu_async_add_job(
			ctx, &params[i],
			(edgetpu_async_job_t)edgetpu_kci_leave_group_worker);
		if (err) {
			etdev_err(group->etdev,
				  "%s: failed to create async job: %d\n",
				  __func__, err);
			goto out_free;
		}
	}
	err = edgetpu_async_wait(ctx);
	if (err)
		etdev_err(group->etdev,
			  "%s: failed to execute jobs: %d\n", __func__, err);
out_free:
	edgetpu_async_free_ctx(ctx);
	kfree(params);
#endif
}

/*
 * Asynchronously sends JOIN_GROUP KCI command to each device in @group.
 *
 * Caller holds group->lock.
 */
static int
edgetpu_device_group_kci_finalized(struct edgetpu_device_group *group)
{
#if IS_ENABLED(CONFIG_ABROLHOS)
	return 0;
#else
	struct kci_worker_param *params =
		kmalloc_array(group->n_clients, sizeof(*params), GFP_KERNEL);
	struct edgetpu_async_ctx *ctx = edgetpu_async_alloc_ctx();
	struct edgetpu_async_ctx *ctx_for_leave = edgetpu_async_alloc_ctx();
	uint i;
	int ret, val;

	if (!params || !ctx || !ctx_for_leave) {
		ret = -ENOMEM;
		goto out_free;
	}
	for (i = 0; i < group->n_clients; i++) {
		params[i].group = group;
		params[i].idx = i;
		ret = edgetpu_async_add_job(
			ctx, &params[i],
			(edgetpu_async_job_t)edgetpu_kci_join_group_worker);
		if (ret)
			goto out_free;
	}
	ret = edgetpu_async_wait(ctx);
	if (ret)
		goto out_free;
	for_each_async_ret(ctx, val, i) {
		if (val)
			goto out_leave;
	}
	ret = 0;
	goto out_free;

out_leave:
	for_each_async_ret(ctx, val, i) {
		if (val == 0)
			edgetpu_async_add_job(
				ctx_for_leave, &params[i],
				(edgetpu_async_job_t)
					edgetpu_kci_leave_group_worker);

		else if (val > 0)
			ret = -EBADMSG;
		else
			ret = val;
	}
	val = edgetpu_async_wait(ctx_for_leave);
	/* ENOMEM */
	if (val)
		ret = val;

out_free:
	edgetpu_async_free_ctx(ctx_for_leave);
	edgetpu_async_free_ctx(ctx);
	kfree(params);
	return ret;
#endif
}

/*
 * Returns the leader of @group.
 *
 * Must be called with the lock held.
 */
static struct edgetpu_client *edgetpu_device_group_leader(
		struct edgetpu_device_group *group)
{
	if (group->n_clients < 1 || edgetpu_device_group_is_disbanded(group))
		return NULL;
	return list_first_entry(&group->clients, struct edgetpu_list_client,
				list)->client;
}

static int group_alloc_members(struct edgetpu_device_group *group)
{
	struct edgetpu_list_client *c;
	int i = 0;

	group->members = kcalloc(group->n_clients, sizeof(*group->members),
				 GFP_KERNEL);
	if (!group->members)
		return -ENOMEM;
	for_each_list_client(c, group) {
		group->members[i] = c->client;
		i++;
	}

	return 0;
}

static void group_release_members(struct edgetpu_device_group *group)
{
	kfree(group->members);
	group->members = NULL;
}

int edgetpu_group_set_eventfd(struct edgetpu_device_group *group, uint event_id,
			      int eventfd)
{
	struct eventfd_ctx *ctx = eventfd_ctx_fdget(eventfd);
	ulong flags;

	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	if (event_id >= EDGETPU_EVENT_COUNT)
		return -EINVAL;

	write_lock_irqsave(&group->events.lock, flags);
	if (group->events.eventfds[event_id])
		eventfd_ctx_put(group->events.eventfds[event_id]);
	group->events.eventfds[event_id] = ctx;
	write_unlock_irqrestore(&group->events.lock, flags);
	return 0;
}

void edgetpu_group_unset_eventfd(struct edgetpu_device_group *group,
				 uint event_id)
{
	ulong flags;

	if (event_id >= EDGETPU_EVENT_COUNT)
		return;

	write_lock_irqsave(&group->events.lock, flags);
	if (group->events.eventfds[event_id])
		eventfd_ctx_put(group->events.eventfds[event_id]);
	group->events.eventfds[event_id] = NULL;
	write_unlock_irqrestore(&group->events.lock, flags);
}

static void edgetpu_group_clear_events(struct edgetpu_device_group *group)
{
	int event_id;
	ulong flags;

	write_lock_irqsave(&group->events.lock, flags);
	for (event_id = 0; event_id < EDGETPU_EVENT_COUNT; event_id++) {
		if (group->events.eventfds[event_id])
			eventfd_ctx_put(group->events.eventfds[event_id]);
		group->events.eventfds[event_id] = NULL;
	}
	write_unlock_irqrestore(&group->events.lock, flags);
}

void edgetpu_group_notify(struct edgetpu_device_group *group, uint event_id)
{
	if (event_id >= EDGETPU_EVENT_COUNT)
		return;

	read_lock(&group->events.lock);
	if (group->events.eventfds[event_id])
		eventfd_signal(group->events.eventfds[event_id], 1);
	read_unlock(&group->events.lock);
}

/*
 * Releases all resources the group allocated and mark the group as disbanded.
 *
 * @group must have a valid leader and members, this method uses its clients to
 * release VII and P2P mailboxes, buffer mappings, etc.
 *
 * The lock of group must be held.
 */
static void edgetpu_device_group_release(struct edgetpu_device_group *group)
{
	edgetpu_mappings_clear_group(group);
	edgetpu_group_clear_events(group);
	if (edgetpu_device_group_is_finalized(group)) {
		edgetpu_device_group_kci_leave(group);
		edgetpu_group_remove_remote_dram(group);
#ifdef EDGETPU_HAS_P2P_MAILBOX
		edgetpu_p2p_mailbox_release(group);
#endif
		group_release_members(group);
	}
	edgetpu_mailbox_remove_vii(&group->vii);
	group->status = EDGETPU_DEVICE_GROUP_DISBANDED;
}

/* Checks if two clients are allowed to be grouped. */
static bool edgetpu_clients_groupable(const struct edgetpu_client *client1,
				      const struct edgetpu_client *client2)
{
	struct edgetpu_dev *etdev1 = client1->etdev, *etdev2 = client2->etdev;

	return etdev1->mcp_id == etdev2->mcp_id &&
	       etdev1->mcp_die_index != etdev2->mcp_die_index;
}

/*
 * Finds an empty slot of @etdev->groups and assigns @group to it.
 *
 * Returns the non-negative index of etdev->groups on success.
 * Returns -EBUSY if no empty slot found.
 */
static int edgetpu_dev_add_group(struct edgetpu_dev *etdev,
				 struct edgetpu_device_group *group)
{
	int i;

	mutex_lock(&etdev->groups_lock);
	for (i = 0; i < EDGETPU_NGROUPS; i++) {
		if (!etdev->groups[i])
			break;
	}

	if (i >= EDGETPU_NGROUPS) {
		mutex_unlock(&etdev->groups_lock);
		return -EBUSY;
	}
	etdev->groups[i] = edgetpu_device_group_get(group);
	mutex_unlock(&etdev->groups_lock);

	return i;
}

void edgetpu_device_group_put(struct edgetpu_device_group *group)
{
	if (!group)
		return;
	if (refcount_dec_and_test(&group->ref_count))
		kfree(group);
}

struct edgetpu_device_group *edgetpu_device_group_alloc(
		struct edgetpu_client *client,
		const struct edgetpu_mailbox_attr *attr)
{
	static uint cur_workload_id;
	int ret;
	struct edgetpu_device_group *group;

	/* the client already belongs to a group */
	if (client->group)
		return ERR_PTR(-EINVAL);
	if (edgetpu_chip_bypassed(client->etdev))
		return ERR_PTR(-EINVAL);

	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group)
		return ERR_PTR(-ENOMEM);

	refcount_set(&group->ref_count, 1);
	group->workload_id = cur_workload_id++;
	INIT_LIST_HEAD(&group->clients);
	group->n_clients = 0;
	group->status = EDGETPU_DEVICE_GROUP_WAITING;
	group->etdev = client->etdev;
	mutex_init(&group->lock);
	rwlock_init(&group->events.lock);
	edgetpu_mapping_init(&group->host_mappings);
	edgetpu_mapping_init(&group->dmabuf_mappings);
	/* adds @client as the first entry */
	ret = edgetpu_device_group_add(group, client);
	if (ret) {
		etdev_dbg(group->etdev,
			  "%s: group %u add failed ret=%d\n",
			  __func__, group->workload_id, ret);
		goto error_put_group;
	}

	ret = edgetpu_mailbox_init_vii(&group->vii, group, attr);
	if (ret) {
		etdev_dbg(group->etdev,
			  "%s: group %u init vii failed ret=%d\n",
			  __func__, group->workload_id, ret);
		edgetpu_device_group_leave(client);
		goto error_put_group;
	}

	return group;

error_put_group:
	edgetpu_device_group_put(group);
	return ERR_PTR(ret);
}

int edgetpu_device_group_add(struct edgetpu_device_group *group,
			     struct edgetpu_client *client)
{
	struct edgetpu_list_client *c;
	int i;
	int ret = 0;

	if (client->group != NULL)
		return -EINVAL;
	if (edgetpu_chip_bypassed(client->etdev))
		return 0;

	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_waiting(group)) {
		ret = -EINVAL;
		goto out;
	}

	for_each_list_client(c, group) {
		if (!edgetpu_clients_groupable(c->client, client)) {
			ret = -EINVAL;
			goto out;
		}
	}

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (!c) {
		ret = -ENOMEM;
		goto out;
	}

	i = edgetpu_dev_add_group(client->etdev, group);
	if (i < 0) {
		kfree(c);
		ret = i;
		goto out;
	}

	c->client = edgetpu_client_get(client);
	list_add_tail(&c->list, &group->clients);
	client->idx = group->n_clients;
	group->n_clients++;
	client->group = edgetpu_device_group_get(group);
	etdev_dbg(client->etdev, "%s: added group %u\n",
		  __func__, group->workload_id);

out:
	mutex_unlock(&group->lock);
	return ret;
}

void edgetpu_device_group_leave(struct edgetpu_client *client)
{
	struct edgetpu_device_group *group = client->group;
	struct edgetpu_list_client *cur, *nxt;
	bool will_disband = false;
	int i;

	if (!group)
		return;

	mutex_lock(&group->lock);
	/*
	 * Disband the group if the leader leaves, or it's finalized and any
	 * member leaves.
	 */
	if (edgetpu_device_group_is_waiting(group)) {
		if (edgetpu_device_group_leader(group) == client)
			will_disband = true;
	} else if (edgetpu_device_group_is_finalized(group)) {
		will_disband = true;
	}

	if (will_disband)
		/* release the group before removing any members */
		edgetpu_device_group_release(group);

	/* removes the client from the list */
	for_each_list_client_safe(cur, nxt, group) {
		if (cur->client == client) {
			list_del(&cur->list);
			kfree(cur);
			edgetpu_client_put(client);
			group->n_clients--;
			break;
		}
	}
	edgetpu_device_group_put(client->group);
	client->group = NULL;
	mutex_unlock(&group->lock);
	/* remove the group from the client device */
	mutex_lock(&client->etdev->groups_lock);
	for (i = 0; i < EDGETPU_NGROUPS; i++) {
		if (client->etdev->groups[i] == group) {
			edgetpu_device_group_put(client->etdev->groups[i]);
			client->etdev->groups[i] = NULL;
			break;
		}
	}
	mutex_unlock(&client->etdev->groups_lock);
}

bool edgetpu_device_group_is_leader(struct edgetpu_device_group *group,
				    const struct edgetpu_client *client)
{
	bool ret;

	mutex_lock(&group->lock);
	ret = (edgetpu_device_group_leader(group) == client);
	mutex_unlock(&group->lock);
	return ret;
}

int edgetpu_device_group_finalize(struct edgetpu_device_group *group)
{
	int ret = 0;

	mutex_lock(&group->lock);
	/* do nothing if the group is finalized */
	if (edgetpu_device_group_is_finalized(group))
		goto err_unlock;

	if (!edgetpu_device_group_is_waiting(group)) {
		ret = -EINVAL;
		goto err_unlock;
	}

	ret = group_alloc_members(group);
	if (ret)
		goto err_unlock;

#ifdef EDGETPU_HAS_P2P_MAILBOX
	ret = edgetpu_p2p_mailbox_setup(group);
	if (ret)
		goto err_release_members;
#endif

	ret = edgetpu_group_setup_remote_dram(group);
	if (ret)
		goto err_release_p2p;

	ret = edgetpu_device_group_kci_finalized(group);
	if (ret)
		goto err_remove_remote_dram;

	group->status = EDGETPU_DEVICE_GROUP_FINALIZED;
	mutex_unlock(&group->lock);
	return 0;

err_remove_remote_dram:
	edgetpu_group_remove_remote_dram(group);
err_release_p2p:
#ifdef EDGETPU_HAS_P2P_MAILBOX
	edgetpu_p2p_mailbox_release(group);
err_release_members:
#endif
	group_release_members(group);
err_unlock:
	mutex_unlock(&group->lock);
	return ret;
}

/*
 * Requests all devices except the leader in @group to map
 * @hmap->map.device_address -> corresponding @hmap->sg_tables[].
 *
 * The caller holds the group lock.
 *
 * Returns 0 on success.
 * Returns a negative errno on error, no mapping operations are performed in
 * this case.
 */
static int edgetpu_device_group_map_iova_sgt(struct edgetpu_device_group *group,
					     struct edgetpu_host_map *hmap)
{
	struct edgetpu_dev *etdev;
	const struct edgetpu_mapping *map = &hmap->map;
	enum edgetpu_context_id context_id = edgetpu_group_context_id(group);
	uint i;
	int ret;

	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		ret = edgetpu_mmu_map_iova_sgt(etdev, map->device_address,
					       &hmap->sg_tables[i], map->dir,
					       context_id);
		if (ret)
			goto rollback;
	}

	return 0;

rollback:
	while (i > 1) {
		i--;
		etdev = edgetpu_device_group_nth_etdev(group, i);
		edgetpu_mmu_unmap_iova_sgt(etdev, map->device_address,
					   &hmap->sg_tables[i], map->dir,
					   context_id);
	}
	return ret;
}

/*
 * Removes previously added mapping.
 *
 * The caller holds the group lock.
 */
static void
edgetpu_device_group_unmap_iova_sgt(struct edgetpu_device_group *group,
				    struct edgetpu_host_map *hmap)
{
	const struct edgetpu_mapping *map = &hmap->map;
	struct edgetpu_dev *etdev;
	enum edgetpu_context_id context_id = edgetpu_group_context_id(group);
	uint i;

	for (i = 1; i < group->n_clients; i++) {
		etdev = edgetpu_device_group_nth_etdev(group, i);
		edgetpu_mmu_unmap_iova_sgt(etdev, map->device_address,
					   &hmap->sg_tables[i], map->dir,
					   context_id);
	}
}

/*
 * Unmap a mapping specified by @map. Unmaps from IOMMU and unpins pages,
 * frees mapping node, which is invalid upon return.
 *
 * Caller locks group->host_mappings.
 */
static void edgetpu_unmap_node(struct edgetpu_mapping *map)
{
	struct edgetpu_device_group *group = map->priv;
	enum edgetpu_context_id context_id = edgetpu_group_context_id(group);
	struct edgetpu_host_map *hmap =
		container_of(map, struct edgetpu_host_map, map);
	struct edgetpu_dev *etdev;
	struct sg_page_iter sg_iter;
	uint i;

	etdev_dbg(group->etdev, "%s: %u: die=%d, iova=0x%llx\n", __func__,
		  group->workload_id, map->die_index, map->device_address);

	if (map->device_address) {
		if (IS_MIRRORED(map->flags)) {
			etdev = group->etdev;
			edgetpu_device_group_unmap_iova_sgt(group, hmap);
		} else {
			etdev = edgetpu_device_group_nth_etdev(group,
							       map->die_index);
		}
		edgetpu_mmu_unmap(etdev, map, context_id);
	}

	for_each_sg_page(map->sgt.sgl, &sg_iter, map->sgt.nents, 0) {
		struct page *page = sg_page_iter_page(&sg_iter);

		if (map->dir == DMA_FROM_DEVICE ||
		    map->dir == DMA_BIDIRECTIONAL)
			set_page_dirty(page);

		put_page(page);
	}

	sg_free_table(&map->sgt);
	if (IS_MIRRORED(map->flags)) {
		for (i = 1; i < group->n_clients; i++)
			sg_free_table(&hmap->sg_tables[i]);
		kfree(hmap->sg_tables);
	}
	edgetpu_device_group_put(map->priv);
	kfree(hmap);
}

static void edgetpu_host_map_show(struct edgetpu_mapping *map,
				  struct seq_file *s)
{
	struct scatterlist *sg;
	int i;
	size_t cur_offset = 0;

	for_each_sg(map->sgt.sgl, sg, map->sgt.nents, i) {
		dma_addr_t phys_addr = sg_phys(sg);
		dma_addr_t dma_addr = sg_dma_address(sg);

		if (IS_MIRRORED(map->flags))
			seq_puts(s, "  mirrored: ");
		else
			seq_printf(s, "  die %u: ", map->die_index);
		seq_printf(s, "0x%llx %lu %s 0x%llx %pap %pad\n",
			   map->device_address + cur_offset,
			   sg_dma_len(sg) / PAGE_SIZE,
			   edgetpu_dma_dir_rw_s(map->dir),
			   map->host_address + cur_offset, &phys_addr,
			   &dma_addr);
		cur_offset += sg_dma_len(sg);
	}
}

/*
 * Pins the user-space address @host_addr and returns the pinned pages.
 * @pnum_pages is set to the number of pages.
 *
 * Returns -errno if failed on pinning @size bytes.
 */
static struct page **pin_user_pages(struct edgetpu_device_group *group,
				    u64 host_addr, u64 size,
				    enum dma_data_direction dir,
				    uint *pnum_pages)
{
	uint num_pages;
	ulong offset;
	struct edgetpu_dev *etdev = group->etdev;
	struct page **pages;
	int i;
	int ret;

	num_pages = size / PAGE_SIZE;
	offset = host_addr & (PAGE_SIZE - 1);
	if (offset)
		num_pages++;

	etdev_dbg(etdev, "%s: hostaddr=0x%llx pages=%u dir=%x\n",
		  __func__, host_addr, num_pages, dir);
	pages = kcalloc(num_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return ERR_PTR(-ENOMEM);

	/*
	 * DMA Buffers appear to be always dirty, so mark pages as always writeable
	 */
	ret = get_user_pages_fast(host_addr & PAGE_MASK, num_pages, 1, pages);
	if (ret < 0) {
		etdev_dbg(etdev, "get user pages failed %u:%pK-%u: %d\n",
			  group->workload_id, (void *)host_addr, num_pages,
			  ret);
		num_pages = 0;
		goto error;
	}
	if (ret < num_pages) {
		etdev_dbg(etdev,
			  "get user pages partial %u:%pK npages=%u pinned=%d\n",
			  group->workload_id, (void *)host_addr, num_pages,
			  ret);
		num_pages = ret;
		ret = -EFAULT;
		goto error;
	}

	*pnum_pages = num_pages;

	return pages;

error:
	for (i = 0; i < num_pages; i++)
		put_page(pages[i]);
	kfree(pages);

	return ERR_PTR(ret);
}

/*
 * Allocates an edgetpu_host_map with the user-space address @host_addr.
 */
static struct edgetpu_host_map *
alloc_mapping_from_useraddr(struct edgetpu_device_group *group, u64 host_addr,
			    u64 size, edgetpu_map_flag_t flags)
{
	uint num_pages = 0;
	struct edgetpu_dev *etdev = group->etdev;
	struct page **pages;
	struct edgetpu_host_map *hmap;
	const enum dma_data_direction dir = flags & EDGETPU_MAP_DIR_MASK;
	int n;
	struct sg_table *sgt;
	int i;
	int ret;

	pages = pin_user_pages(group, host_addr, size, dir, &num_pages);
	if (IS_ERR(pages))
		return (void *)pages;

	hmap = kzalloc(sizeof(*hmap), GFP_KERNEL);
	if (!hmap) {
		ret = -ENOMEM;
		goto error;
	}

	hmap->map.host_address = host_addr;
	hmap->map.dir = dir;
	hmap->map.priv = edgetpu_device_group_get(group);
	hmap->map.release = edgetpu_unmap_node;
	hmap->map.show = edgetpu_host_map_show;
	hmap->map.flags = flags;

	if (IS_MIRRORED(flags)) {
		hmap->sg_tables = kcalloc(group->n_clients,
					  sizeof(*hmap->sg_tables), GFP_KERNEL);
		if (!hmap->sg_tables) {
			ret = -ENOMEM;
			goto error;
		}
		n = group->n_clients;
	} else {
		n = 1;
	}

	for (i = 0; i < n; i++) {
		if (i == 0)
			sgt = &hmap->map.sgt;
		else
			sgt = &hmap->sg_tables[i];
		ret = sg_alloc_table_from_pages(sgt, pages, num_pages,
						host_addr & (PAGE_SIZE - 1),
						size, GFP_KERNEL);
		if (ret) {
			etdev_dbg(etdev,
				  "%s: sg_alloc_table_from_pages failed %u:%pK-%u: %d\n",
				  __func__, group->workload_id,
				  (void *)host_addr, num_pages, ret);
			goto error_free_sgt;
		}
	}

	kfree(pages);
	return hmap;

error_free_sgt:
	while (i > 0) {
		i--;
		if (i == 0)
			sgt = &hmap->map.sgt;
		else
			sgt = &hmap->sg_tables[i];
		sg_free_table(sgt);
	}
error:
	for (i = 0; i < num_pages; i++)
		put_page(pages[i]);
	kfree(pages);
	if (hmap) {
		edgetpu_device_group_put(hmap->map.priv);
		kfree(hmap->sg_tables);
		kfree(hmap);
	}

	return ERR_PTR(ret);
}

/*
 * Find the scatterlist covering range [start, end).
 *
 * Returns NULL if:
 * - @start is larger than the whole SG table
 */
static struct scatterlist *find_sg_within(const struct sg_table *sgt, u64 start,
					  u64 end, int *nelems)
{
	struct scatterlist *sg, *sg_to_sync = NULL;
	size_t cur_offset = 0;
	int i;

	*nelems = 0;
	for_each_sg(sgt->sgl, sg, sgt->orig_nents, i) {
		if (end <= cur_offset)
			break;
		if (cur_offset <= start && start < cur_offset + sg->length)
			sg_to_sync = sg;
		if (sg_to_sync)
			(*nelems)++;
		cur_offset += sg->length;
	}

	return sg_to_sync;
}

static int group_sync_host_map(struct edgetpu_device_group *group,
			       struct edgetpu_host_map *hmap, u64 offset,
			       u64 size, enum dma_data_direction dir,
			       bool for_cpu)
{
	const u64 end = offset + size;
	typeof(dma_sync_sg_for_cpu) *sync =
		for_cpu ? dma_sync_sg_for_cpu : dma_sync_sg_for_device;
	struct edgetpu_dev *etdev;
	struct sg_table *sgt;
	struct scatterlist *sg;
	int i;
	int nelems;

	sgt = &hmap->map.sgt;
	sg = find_sg_within(sgt, offset, end, &nelems);
	if (!sg)
		return -EINVAL;

	if (IS_MIRRORED(hmap->map.flags)) {
		sync(group->etdev->dev, sg, nelems, dir);
		for (i = 1; i < group->n_clients; i++) {
			etdev = edgetpu_device_group_nth_etdev(group, i);
			sg = find_sg_within(&hmap->sg_tables[i], offset, end,
					    &nelems);
			if (WARN_ON(!sg))
				return -EINVAL;
			sync(etdev->dev, sg, nelems, dir);
		}
	} else {
		etdev = edgetpu_device_group_nth_etdev(group,
						       hmap->map.die_index);
		sync(etdev->dev, sg, nelems, dir);
	}

	return 0;
}

int edgetpu_device_group_map(struct edgetpu_device_group *group,
			     struct edgetpu_map_ioctl *arg)
{
	int ret = -EINVAL;
	u64 host_addr = arg->host_address;
	u64 size = arg->size;
	edgetpu_map_flag_t flags = arg->flags;
	struct edgetpu_host_map *hmap;
	struct edgetpu_mapping *map;
	struct edgetpu_dev *etdev;
	enum edgetpu_context_id context_id = edgetpu_group_context_id(group);
	const u32 mmu_flags = map_to_mmu_flags(flags) | EDGETPU_MMU_HOST;

	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group)) {
		ret = -EINVAL;
		goto error_unlock_group;
	}
	if (!IS_MIRRORED(flags)) {
		if (arg->die_index >= group->n_clients) {
			ret = -EINVAL;
			goto error_unlock_group;
		}
	}

	hmap = alloc_mapping_from_useraddr(group, host_addr, size, flags);
	if (IS_ERR(hmap)) {
		ret = PTR_ERR(hmap);
		goto error_unlock_group;
	}

	map = &hmap->map;
	if (IS_MIRRORED(flags)) {
		map->die_index = ALL_DIES;
		etdev = group->etdev;
		ret = edgetpu_mmu_map(etdev, map, context_id, mmu_flags);
		if (ret)
			goto error_release_map;
		ret = edgetpu_device_group_map_iova_sgt(group, hmap);
		if (ret) {
			etdev_dbg(etdev,
				  "group add translation failed %u:0x%llx\n",
				  group->workload_id, map->device_address);
			goto error_release_map;
		}
	} else {
		map->die_index = arg->die_index;
		etdev = edgetpu_device_group_nth_etdev(group, map->die_index);
		ret = edgetpu_mmu_map(etdev, map, context_id, mmu_flags);
		if (ret)
			goto error_release_map;
	}

	ret = edgetpu_mapping_add(&group->host_mappings, map);
	if (ret) {
		etdev_dbg(etdev, "duplicate mapping %u:0x%llx\n",
			  group->workload_id, map->device_address);
		goto error_release_map;
	}

	mutex_unlock(&group->lock);
	arg->device_address = map->device_address;

	return 0;

error_release_map:
	edgetpu_mapping_lock(&group->host_mappings);
	/* this will free @hmap */
	edgetpu_unmap_node(map);
	edgetpu_mapping_unlock(&group->host_mappings);

error_unlock_group:
	mutex_unlock(&group->lock);
	return ret;
}

int edgetpu_device_group_unmap(struct edgetpu_device_group *group,
			       u32 die_index, tpu_addr_t tpu_addr)
{
	struct edgetpu_mapping *map;
	int ret = 0;

	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group)) {
		ret = -EINVAL;
		goto unlock_group;
	}

	edgetpu_mapping_lock(&group->host_mappings);
	map = edgetpu_mapping_find_locked(&group->host_mappings, ALL_DIES,
					  tpu_addr);
	if (!map)
		map = edgetpu_mapping_find_locked(&group->host_mappings,
						  die_index, tpu_addr);
	if (!map) {
		edgetpu_mapping_unlock(&group->host_mappings);
		etdev_dbg(group->etdev,
			  "%s: mapping not found for workload %u: 0x%llx\n",
			  __func__, group->workload_id, tpu_addr);
		ret = -EINVAL;
		goto unlock_group;
	}

	edgetpu_mapping_unlink(&group->host_mappings, map);
	edgetpu_unmap_node(map);
	edgetpu_mapping_unlock(&group->host_mappings);
unlock_group:
	mutex_unlock(&group->lock);
	return ret;
}

int edgetpu_device_group_sync_buffer(struct edgetpu_device_group *group,
				     const struct edgetpu_sync_ioctl *arg)
{
	struct edgetpu_mapping *map;
	int ret = 0;
	tpu_addr_t tpu_addr = arg->device_address;
	enum dma_data_direction dir = arg->flags & EDGETPU_MAP_DIR_MASK;
	struct edgetpu_host_map *hmap;

	/* invalid if size == 0 or overflow */
	if (arg->offset + arg->size <= arg->offset)
		return -EINVAL;

	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group)) {
		ret = -EINVAL;
		goto unlock_group;
	}

	edgetpu_mapping_lock(&group->host_mappings);
	map = edgetpu_mapping_find_locked(&group->host_mappings, ALL_DIES,
					  tpu_addr);
	if (!map)
		map = edgetpu_mapping_find_locked(&group->host_mappings,
						  arg->die_index, tpu_addr);
	if (!map) {
		ret = -EINVAL;
		goto unlock_mapping;
	}

	hmap = container_of(map, struct edgetpu_host_map, map);
	ret = group_sync_host_map(group, hmap, arg->offset, arg->size, dir,
				  arg->flags & EDGETPU_SYNC_FOR_CPU);
unlock_mapping:
	edgetpu_mapping_unlock(&group->host_mappings);
unlock_group:
	mutex_unlock(&group->lock);
	return ret;
}

void edgetpu_mappings_clear_group(struct edgetpu_device_group *group)
{
	edgetpu_mapping_clear(&group->host_mappings);
	edgetpu_mapping_clear(&group->dmabuf_mappings);
}

void edgetpu_group_mappings_show(struct edgetpu_device_group *group,
				 struct seq_file *s)
{
	seq_printf(s, "workload %u", group->workload_id);
	switch (group->status) {
	case EDGETPU_DEVICE_GROUP_WAITING:
	case EDGETPU_DEVICE_GROUP_FINALIZED:
		break;
	case EDGETPU_DEVICE_GROUP_DISBANDED:
		seq_puts(s, ": disbanded\n");
		return;
	}
	seq_printf(s, " context %u:\n", edgetpu_group_context_id(group));

	if (group->host_mappings.count) {
		seq_puts(s, "host buffer mappings:\n");
		edgetpu_mappings_show(&group->host_mappings, s);
	}
	if (group->dmabuf_mappings.count) {
		seq_puts(s, "dma-buf buffer mappings:\n");
		edgetpu_mappings_show(&group->dmabuf_mappings, s);
	}

	if (group->vii.cmd_queue_mem.vaddr) {
		seq_puts(s, "VII queues:\n");
		seq_printf(s, "  0x%llx %lu cmdq 0x%llx %pad\n",
			   group->vii.cmd_queue_mem.tpu_addr,
			   group->vii.cmd_queue_mem.size / PAGE_SIZE,
			   group->vii.cmd_queue_mem.host_addr,
			   &group->vii.cmd_queue_mem.dma_addr);
		seq_printf(s, "  0x%llx %lu rspq 0x%llx %pad\n",
			   group->vii.resp_queue_mem.tpu_addr,
			   group->vii.resp_queue_mem.size / PAGE_SIZE,
			   group->vii.resp_queue_mem.host_addr,
			   &group->vii.resp_queue_mem.dma_addr);
	}
#ifdef EDGETPU_HAS_P2P_MAILBOX
	if (group->p2p_mailbox_matrix) {
		seq_puts(s, "P2P queues:\n");
		edgetpu_p2p_mailbox_show(group, s);
	}
#endif
}

int edgetpu_mmap_csr(struct edgetpu_device_group *group,
		     struct vm_area_struct *vma)
{
	struct edgetpu_dev *etdev = group->etdev;
	int ret = 0;
	ulong phys_base, vma_size, map_size;

	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group)) {
		ret = -EINVAL;
		goto out;
	}

	vma_size = vma->vm_end - vma->vm_start;
	map_size = min(vma_size, USERSPACE_CSR_SIZE);
	phys_base = etdev->regs.phys + group->vii.mailbox->cmd_queue_csr_base;
	ret = io_remap_pfn_range(vma, vma->vm_start, phys_base >> PAGE_SHIFT,
				 map_size, vma->vm_page_prot);
	if (ret)
		etdev_dbg(etdev, "Error remapping PFN range: %d\n", ret);

out:
	mutex_unlock(&group->lock);
	return ret;
}

int edgetpu_mmap_queue(struct edgetpu_device_group *group,
		       enum mailbox_queue_type type,
		       struct vm_area_struct *vma)
{
	struct edgetpu_dev *etdev = group->etdev;
	int ret = 0;
	edgetpu_queue_mem *queue_mem;

	mutex_lock(&group->lock);
	if (!edgetpu_device_group_is_finalized(group)) {
		ret = -EINVAL;
		goto out;
	}

	if (type == MAILBOX_CMD_QUEUE)
		queue_mem = &(group->vii.cmd_queue_mem);
	else
		queue_mem = &(group->vii.resp_queue_mem);

	if (!queue_mem->vaddr) {
		ret = -ENXIO;
		goto out;
	}

	vma->vm_pgoff = 0;
	ret = dma_mmap_coherent(etdev->dev, vma, queue_mem->vaddr,
				queue_mem->dma_addr, queue_mem->size);
	if (!ret)
		queue_mem->host_addr = vma->vm_start;

out:
	mutex_unlock(&group->lock);
	return ret;
}
