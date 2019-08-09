/*
 * Google LWIS Transaction Processor
 *
 * Copyright (c) 2019 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_TRANSACTION_H_
#define LWIS_TRANSACTION_H_

#include "lwis_commands.h"
#include "lwis_device.h"

/* Transaction entry. Each entry belongs to two queues:
 * 1) Event list: Transactions are sorted by event IDs. This is to search for
 *    the appropriate transactions to trigger.
 * 2) Process queue: When it's time to process, the transaction will be put
 *    into a queue.
 */
struct lwis_transaction {
	struct lwis_transaction_info info;
	struct lwis_transaction_response_header *resp;
	struct list_head event_list_node;
	struct list_head process_queue_node;
};

struct lwis_transaction_event_list {
	uint64_t event_id;
	struct list_head list;
	struct hlist_node node;
};

int lwis_transaction_init(struct lwis_client *client);
int lwis_transaction_client_cleanup(struct lwis_client *client);

int lwis_transaction_submit(struct lwis_client *client,
			    struct lwis_transaction *transaction);
int lwis_transaction_event_trigger(struct lwis_client *client,
				   int64_t event_id, uint64_t event_counter,
				   struct list_head *pending_events);
int lwis_transaction_cancel(struct lwis_client *client, uint64_t id);

#endif /* LWIS_TRANSACTION_H_ */