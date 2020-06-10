/*
 * Google LWIS Debug Utilities
 *
 * Copyright (c) 2020 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/err.h>
#include <linux/fs.h>
#include <linux/hashtable.h>
#include <linux/list.h>

#include "lwis_debug.h"
#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_transaction.h"
#include "lwis_util.h"

#ifdef CONFIG_DEBUG_FS
static void list_transactions(struct lwis_client *client, char *k_buf,
			      size_t k_buf_size)
{
	/* Temporary buffer to be concatenated to the main buffer. */
	char tmp_buf[128] = {};
	int i;
	unsigned long flags;
	struct lwis_transaction_event_list *transaction_list;
	struct lwis_transaction *transaction;
	struct list_head *it_tran;

	spin_lock_irqsave(&client->transaction_lock, flags);
	if (hash_empty(client->transaction_list)) {
		strlcat(k_buf, "No transactions pending\n", k_buf_size);
		goto exit;
	}
	strlcat(k_buf, "Pending Transactions:\n", k_buf_size);
	hash_for_each(client->transaction_list, i, transaction_list, node)
	{
		if (list_empty(&transaction_list->list)) {
			strlcat(k_buf,
				"No pending transaction for this event\n",
				k_buf_size);
			continue;
		}
		list_for_each(it_tran, &transaction_list->list)
		{
			transaction =
				list_entry(it_tran, struct lwis_transaction,
					   event_list_node);
			snprintf(
				tmp_buf, sizeof(tmp_buf),
				"ID: 0x%llx Trigger Event: 0x%llx Count: 0x%llx\n",
				transaction->info.id,
				transaction->info.trigger_event_id,
				transaction->info.trigger_event_counter);
			strlcat(k_buf, tmp_buf, k_buf_size);
			snprintf(tmp_buf, sizeof(tmp_buf),
				 "  Emit Success: 0x%llx Error: %llx\n",
				 transaction->info.emit_success_event_id,
				 transaction->info.emit_error_event_id);
			strlcat(k_buf, tmp_buf, k_buf_size);
		}
	}
exit:
	spin_unlock_irqrestore(&client->transaction_lock, flags);
}

static ssize_t dev_info_read(struct file *fp, char __user *user_buf,
			     size_t count, loff_t *position)
{
	/* Main buffer for all information. */
	char k_buf[256] = {};
	struct lwis_device *lwis_dev = fp->f_inode->i_private;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	snprintf(k_buf, sizeof(k_buf), "%s Device Name: %s ID: %d State: %s\n",
		 lwis_device_type_to_string(lwis_dev->type), lwis_dev->name,
		 lwis_dev->id, lwis_dev->enabled ? "Enabled" : "Disabled");

	return simple_read_from_buffer(user_buf, count, position, k_buf,
				       strlen(k_buf));
}

static ssize_t event_states_read(struct file *fp, char __user *user_buf,
				 size_t count, loff_t *position)
{
	/* Main buffer for all information. */
	char k_buf[2048] = {};
	/* Temporary buffer to be concatenated to the main buffer. */
	char tmp_buf[64] = {};
	struct lwis_device *lwis_dev = fp->f_inode->i_private;
	int i;
	int idx = 0;
	unsigned long flags;
	struct lwis_device_event_state *state;
	bool enabled_event_present = false;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&lwis_dev->lock, flags);
	if (hash_empty(lwis_dev->event_states)) {
		strlcat(k_buf, "  No events being monitored\n", sizeof(k_buf));
		goto exit;
	}
	strlcat(k_buf, "Enabled Device Events:\n", sizeof(k_buf));
	hash_for_each(lwis_dev->event_states, i, state, node)
	{
		if (state->enable_counter > 0) {
			snprintf(tmp_buf, sizeof(tmp_buf),
				 "[%2d] ID: 0x%llx Counter: 0x%llx\n", idx++,
				 state->event_id, state->event_counter);
			strlcat(k_buf, tmp_buf, sizeof(k_buf));
			enabled_event_present = true;
		}
	}
	if (!enabled_event_present) {
		strlcat(k_buf, "No enabled events\n", sizeof(k_buf));
	}

exit:
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	return simple_read_from_buffer(user_buf, count, position, k_buf,
				       strlen(k_buf));
}

static ssize_t transaction_info_read(struct file *fp, char __user *user_buf,
				     size_t count, loff_t *position)
{
	/* Main buffer for all information. */
	char k_buf[2048] = {};
	/* Temporary buffer to be concatenated to the main buffer. */
	char tmp_buf[64] = {};
	struct lwis_device *lwis_dev = fp->f_inode->i_private;
	struct lwis_client *client;
	int idx = 0;

	if (lwis_dev == NULL) {
		pr_err("Unknown LWIS device pointer\n");
		return -EINVAL;
	}

	if (list_empty(&lwis_dev->clients)) {
		snprintf(k_buf, sizeof(k_buf), "No clients opened\n");
		goto exit;
	}
	list_for_each_entry(client, &lwis_dev->clients, node)
	{
		snprintf(tmp_buf, sizeof(tmp_buf), "Client %d:\n", idx);
		strlcat(k_buf, tmp_buf, sizeof(k_buf));
		list_transactions(client, k_buf, sizeof(k_buf));
		idx++;
	}

exit:
	return simple_read_from_buffer(user_buf, count, position, k_buf,
				       strlen(k_buf));
}

static struct file_operations dev_info_fops = {
	.owner = THIS_MODULE,
	.read = dev_info_read,
};

static struct file_operations event_states_fops = {
	.owner = THIS_MODULE,
	.read = event_states_read,
};

static struct file_operations transaction_info_fops = {
	.owner = THIS_MODULE,
	.read = transaction_info_read,
};

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev,
			      struct dentry *dbg_root)
{
	struct dentry *dbg_dir;
	struct dentry *dbg_dev_info_file;
	struct dentry *dbg_event_file;
	struct dentry *dbg_transaction_file;

	/* DebugFS not present, just return */
	if (dbg_root == NULL) {
		return 0;
	}

	dbg_dir = debugfs_create_dir(lwis_dev->name, dbg_root);
	if (IS_ERR_OR_NULL(dbg_dir)) {
		dev_err(lwis_dev->dev,
			"Failed to create DebugFS directory - %d",
			PTR_ERR(dbg_dir));
		return PTR_ERR(dbg_dir);
	}

	dbg_dev_info_file = debugfs_create_file("dev_info", 0444, dbg_dir,
						lwis_dev, &dev_info_fops);
	if (IS_ERR_OR_NULL(dbg_dev_info_file)) {
		dev_warn(lwis_dev->dev,
			 "Failed to create DebugFS dev_info - %d",
			 PTR_ERR(dbg_dev_info_file));
		dbg_dev_info_file = NULL;
	}

	dbg_event_file = debugfs_create_file("event_state", 0444, dbg_dir,
					     lwis_dev, &event_states_fops);
	if (IS_ERR_OR_NULL(dbg_event_file)) {
		dev_warn(lwis_dev->dev,
			 "Failed to create DebugFS event_state - %d",
			 PTR_ERR(dbg_event_file));
		dbg_event_file = NULL;
	}

	dbg_transaction_file =
		debugfs_create_file("transaction_info", 0444, dbg_dir, lwis_dev,
				    &transaction_info_fops);
	if (IS_ERR_OR_NULL(dbg_transaction_file)) {
		dev_warn(lwis_dev->dev,
			 "Failed to create DebugFS transaction_info - %d",
			 PTR_ERR(dbg_transaction_file));
		dbg_transaction_file = NULL;
	}

	lwis_dev->dbg_dir = dbg_dir;
	lwis_dev->dbg_dev_info_file = dbg_dev_info_file;
	lwis_dev->dbg_event_file = dbg_event_file;
	lwis_dev->dbg_transaction_file = dbg_transaction_file;

	return 0;
}

int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev)
{
	/* DebugFS not present, just return */
	if (lwis_dev->dbg_dir == NULL) {
		return 0;
	}
	debugfs_remove_recursive(lwis_dev->dbg_dir);
	lwis_dev->dbg_dir = NULL;
	lwis_dev->dbg_dev_info_file = NULL;
	lwis_dev->dbg_event_file = NULL;
	lwis_dev->dbg_transaction_file = NULL;
	return 0;
}

#else /* CONFIG_DEBUG_FS */

int lwis_device_debugfs_setup(struct lwis_device *lwis_dev,
			      struct dentry *dbg_root)
{
	return 0;
}

int lwis_device_debugfs_cleanup(struct lwis_device *lwis_dev)
{
	return 0;
}

#endif