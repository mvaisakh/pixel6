/*
 * Google LWIS Event Utilities
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-event: " fmt

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_device.h"
#include "lwis_event.h"
#include "lwis_transaction.h"
#include "lwis_util.h"

/*
 * lwis_client_event_state_find_locked: Looks through the provided client's
 * event state list and tries to find a lwis_client_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Assumes: lwis_client->event_lock is locked
 * Alloc: No
 * Returns: client event state object, if found, NULL otherwise
 */
static struct lwis_client_event_state *
lwis_client_event_state_find_locked(struct lwis_client *lwis_client,
				    int64_t event_id)
{
	/* Our hash iterator */
	struct lwis_client_event_state *p;

	/* Iterate through the hash bucket for this event_id */
	hash_for_each_possible (lwis_client->event_states, p, node, event_id) {
		/* If it's indeed the right one, return it */
		if (p->event_control.event_id == event_id) {
			return p;
		}
	}

	return NULL;
}

/*
 * lwis_client_event_state_find: Looks through the provided client's
 * event state list and tries to find a lwis_client_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Locks: lwis_client->event_lock
 * Alloc: No
 * Returns: client event state object, if found, NULL otherwise
 */
static struct lwis_client_event_state *
lwis_client_event_state_find(struct lwis_client *lwis_client, int64_t event_id)
{
	/* Our return value  */
	struct lwis_client_event_state *state;
	/* Flags for IRQ disable */
	unsigned long flags;

	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_client->event_lock, flags);
	state = lwis_client_event_state_find_locked(lwis_client, event_id);
	/* Unlock and restore */
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	return state;
}

/*
 * lwis_client_event_state_find_or_create: Looks through the provided client's
 * event state list and tries to find a lwis_client_event_state object with the
 * matching event_id. If not found, creates the object with 0 flags and adds it
 * to the list
 *
 * Locks: lwis_client->event_lock
 * Alloc: Maybe
 * Returns: client event state object on success, errno on error
 */
struct lwis_client_event_state *
lwis_client_event_state_find_or_create(struct lwis_client *lwis_client,
				       int64_t event_id)
{
	struct lwis_client_event_state *new_state;
	/* Flags for IRQ disable */
	unsigned long flags;

	/* Try to find a state first, if it already exists */
	struct lwis_client_event_state *state =
		lwis_client_event_state_find(lwis_client, event_id);

	/* If it doesn't, we'll have to create one */
	if (unlikely(state == NULL)) {
		/* Allocate a new state object */
		new_state = kzalloc(sizeof(struct lwis_client_event_state),
				    GFP_KERNEL);
		/* Oh no, ENOMEM */
		if (!new_state) {
			pr_err("Could not allocate memory\n");
			return ERR_PTR(-ENOMEM);
		}
		/* Set the event_id and initialize flags to 0 which pretty much
		 * means everything is disabled. Overall it's expected that
		 * having no client event state entry is equivalent to having
		 * one with flags == 0
		 */
		new_state->event_control.event_id = event_id;
		new_state->event_control.flags = 0;

		/* Critical section for adding to the hash table */
		spin_lock_irqsave(&lwis_client->event_lock, flags);
		/* Let's avoid the race condition in case somebody else beat us
		 * here, and verify that this event_id is still not in the hash
		 * table.
		 */
		state = lwis_client_event_state_find_locked(lwis_client,
							    event_id);
		/* Ok, it's not there */
		if (state == NULL) {
			/* Let's add the new state object */
			hash_add(lwis_client->event_states, &new_state->node,
				 event_id);
			state = new_state;
		} else {
			/* Ok, we now suddenly have a valid state so we need to
			 * free the one we allocated, and pretend like nothing
			 * bad happened.
			 */
			kfree(new_state);
		}

		/* End critical section */
		spin_unlock_irqrestore(&lwis_client->event_lock, flags);
	}
	return state;
}
/*
 * lwis_device_event_state_find_locked: Looks through the provided device's
 * event state list and tries to find a lwis_device_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Assumes: lwis_dev->lock is locked
 * Alloc: No
 * Returns: device event state object, if found, NULL otherwise
 */
static struct lwis_device_event_state *
lwis_device_event_state_find_locked(struct lwis_device *lwis_dev,
				    int64_t event_id)
{
	/* Our hash iterator */
	struct lwis_device_event_state *p;

	/* Iterate through the hash bucket for this event_id */
	hash_for_each_possible (lwis_dev->event_states, p, node, event_id) {
		/* If it's indeed the right one, return it */
		if (p->event_id == event_id) {
			return p;
		}
	}

	return NULL;
}

/*
 * save_device_event_state_to_history_locked: Saves the emitted events in a
 * history buffer for better debugability.
 *
 * Assumes: lwis_dev->lock is locked
 * Alloc : No
 * Returns: None
 */
static void
save_device_event_state_to_history_locked(struct lwis_device *lwis_dev,
					  struct lwis_device_event_state *state)
{
	lwis_dev->debug_info
		.event_hist[lwis_dev->debug_info.cur_event_hist_idx] = *state;
	lwis_dev->debug_info.cur_event_hist_idx++;
	if (lwis_dev->debug_info.cur_event_hist_idx >=
	    EVENT_DEBUG_HISTORY_SIZE) {
		lwis_dev->debug_info.cur_event_hist_idx = 0;
	}
}

/*
 * lwis_device_event_state_find: Looks through the provided device's
 * event state list and tries to find a lwis_device_event_state object with the
 * matching event_id. If not found, returns NULL
 *
 * Locks: lwis_dev->lock
 * Alloc: No
 * Returns: device event state object, if found, NULL otherwise
 */
struct lwis_device_event_state *
lwis_device_event_state_find(struct lwis_device *lwis_dev, int64_t event_id)
{
	/* Our return value  */
	struct lwis_device_event_state *state;
	/* Flags for IRQ disable */
	unsigned long flags;

	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	state = lwis_device_event_state_find_locked(lwis_dev, event_id);
	/* Unlock and restore */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	return state;
}

struct lwis_device_event_state *
lwis_device_event_state_find_or_create(struct lwis_device *lwis_dev,
				       int64_t event_id)
{
	struct lwis_device_event_state *new_state;
	/* Flags for IRQ disable */
	unsigned long flags;

	/* Try to find a state first, if it already exists */
	struct lwis_device_event_state *state =
		lwis_device_event_state_find(lwis_dev, event_id);

	/* If it doesn't, we'll have to create one */
	if (unlikely(state == NULL)) {
		/* Allocate a new state object */
		new_state = kzalloc(sizeof(struct lwis_device_event_state),
				    GFP_KERNEL);
		/* Oh no, ENOMEM */
		if (!new_state) {
			pr_err("Could not allocate memory\n");
			return ERR_PTR(-ENOMEM);
		}
		/* Set the event_id and initialize ref counter  to 0 which means
		 * off by default
		 */
		new_state->event_id = event_id;
		new_state->enable_counter = 0;
		new_state->event_counter = 0;
		new_state->subscriber_count = 0;

		/* Critical section for adding to the hash table */
		spin_lock_irqsave(&lwis_dev->lock, flags);
		/* Let's avoid the race condition in case somebody else beat us
		 * here, and verify that this event_id is still not in the hash
		 * table.
		 */
		state = lwis_device_event_state_find_locked(lwis_dev, event_id);
		/* Ok, it's not there */
		if (state == NULL) {
			/* Let's add the new state object */
			hash_add(lwis_dev->event_states, &new_state->node,
				 event_id);
			state = new_state;
		} else {
			/* Ok, we now suddenly have a valid state so we need to
			 * free the one we allocated, and pretend like nothing
			 * bad happened.
			 */
			kfree(new_state);
		}

		/* End critical section */
		spin_unlock_irqrestore(&lwis_dev->lock, flags);
	}
	return state;
}

int lwis_client_event_control_set(struct lwis_client *lwis_client,
				  const struct lwis_event_control *control)
{
	int ret = 0;
	struct lwis_client_event_state *state;
	uint64_t old_flags, new_flags;
	/* Find, or create, a client event state objcet for this event_id */
	state = lwis_client_event_state_find_or_create(lwis_client,
						       control->event_id);
	if (IS_ERR_OR_NULL(state)) {
		pr_err("Failed to find or create new client event state\n");
		return -ENOMEM;
	}

	old_flags = state->event_control.flags;
	new_flags = control->flags;
	if (old_flags != new_flags) {
		state->event_control.flags = new_flags;
		ret = lwis_device_event_flags_updated(lwis_client->lwis_dev,
						      control->event_id,
						      old_flags, new_flags);
		if (ret) {
			pr_err("Updating device flags failed: %d\n", ret);
		}
	}

	return ret;
}

int lwis_client_event_control_get(struct lwis_client *lwis_client,
				  int64_t event_id,
				  struct lwis_event_control *control)
{
	struct lwis_client_event_state *state;

	state = lwis_client_event_state_find_or_create(lwis_client, event_id);

	if (IS_ERR_OR_NULL(state)) {
		pr_err("Failed to create new event state\n");
		return -ENOMEM;
	}

	control->flags = state->event_control.flags;

	return 0;
}

int lwis_client_event_pop_front(struct lwis_client *lwis_client,
				struct lwis_event_entry **event_out)
{
	/* Our client event object */
	struct lwis_event_entry *event;
	/* Return value and flags for irqsave */
	unsigned long ret = 0;
	unsigned long flags;

	/* Critical section while modifying event queue */
	spin_lock_irqsave(&lwis_client->event_lock, flags);
	if (!list_empty(&lwis_client->event_queue)) {
		/* Get the front of the list */
		event = list_first_entry(&lwis_client->event_queue,
					 struct lwis_event_entry, node);
		/* Delete from the queue */
		list_del(&event->node);
		/* Copy it over */
		if (event_out) {
			*event_out = event;
		} else {
			/* The caller didn't receive it, so can't free it*/
			kfree(event);
		}
	} else {
		ret = -ENOENT;
	}
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	return ret;
}

int lwis_client_event_peek_front(struct lwis_client *lwis_client,
				 struct lwis_event_entry **event_out)
{
	/* Our client event object */
	struct lwis_event_entry *event;
	/* Return value and flags for irqsave */
	unsigned long ret = 0;
	unsigned long flags;

	/* Critical section while modifying event queue */
	spin_lock_irqsave(&lwis_client->event_lock, flags);
	if (!list_empty(&lwis_client->event_queue)) {
		/* Get the front of the list */
		event = list_first_entry(&lwis_client->event_queue,
					 struct lwis_event_entry, node);
		/* Copy it over */
		if (event_out) {
			*event_out = event;
		}
	} else {
		ret = -ENOENT;
	}
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	return ret;
}

/*
 * lwis_client_event_push_back: Inserts new event into the client event queue
 * to be later consumed by userspace. Takes ownership of *event (does not copy,
 * will be freed on the other side)
 *
 * Also wakes up any readers for this client (select() callers, etc.)
 *
 * Locks: lwis_client->event_lock
 *
 * Alloc: No
 * Returns: 0 on success
 */
static int lwis_client_event_push_back(struct lwis_client *lwis_client,
				       struct lwis_event_entry *event)
{
	unsigned long flags;

	if (!event) {
		pr_err("NULL event provided\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&lwis_client->event_lock, flags);

	list_add_tail(&event->node, &lwis_client->event_queue);

	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	wake_up_interruptible(&lwis_client->event_wait_queue);

	return 0;
}

int lwis_client_event_states_clear(struct lwis_client *lwis_client)
{
	/* Our hash table iterator */
	struct lwis_client_event_state *state;
	/* Temporary vars for hash table traversal */
	struct hlist_node *n;
	int i;
	/* Flags for irqsave */
	unsigned long flags;
	/* Disable IRQs and lock the event lock */
	spin_lock_irqsave(&lwis_client->event_lock, flags);
	/* Iterate over the entire hash table */
	hash_for_each_safe (lwis_client->event_states, i, n, state, node) {
		/* Delete the node from the hash table */
		hash_del(&state->node);
		/* Update the device state with zero flags */
		lwis_device_event_flags_updated(lwis_client->lwis_dev,
						state->event_control.event_id,
						state->event_control.flags, 0);
		/* Free the object */
		kfree(state);
	}
	/* Restore the lock */
	spin_unlock_irqrestore(&lwis_client->event_lock, flags);

	return 0;
}

int lwis_device_event_states_clear_locked(struct lwis_device *lwis_dev)
{
	struct lwis_device_event_state *state;
	struct hlist_node *n;
	int i;

	hash_for_each_safe (lwis_dev->event_states, i, n, state, node) {
		hash_del(&state->node);
		kfree(state);
	}

	return 0;
}

int lwis_device_event_flags_updated(struct lwis_device *lwis_dev,
				    int64_t event_id, uint64_t old_flags,
				    uint64_t new_flags)
{
	struct lwis_device_event_state *state;
	int ret = 0;
	bool call_enable_cb = false, event_enabled = false;
	/* Flags for irqsave */
	unsigned long flags;
	/* Find our state */
	state = lwis_device_event_state_find_or_create(lwis_dev, event_id);
	/* Could not find or create one */
	if (IS_ERR_OR_NULL(state)) {
		pr_err("Could not find or create device event state\n");
		return PTR_ERR(state);
	}
	/* Disable IRQs and lock the lock */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	/* Are we turning on the event? */
	if (!(old_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE) &&
	    (new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE)) {
		state->enable_counter++;
		event_enabled = true;
		call_enable_cb = (state->enable_counter == 1);

	} else if ((old_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE) &&
		   !(new_flags & LWIS_EVENT_CONTROL_FLAG_IRQ_ENABLE)) {
		state->enable_counter--;
		event_enabled = false;
		call_enable_cb = (state->enable_counter == 0);
	}
	/* Restore the lock */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	/* Handle event being enabled or disabled */
	if (call_enable_cb) {
		/* Call our handler dispatcher */
		ret = lwis_device_event_enable(lwis_dev, event_id,
					       event_enabled);
		if (ret) {
			pr_err("Failed to %s event: %lld (err:%d)\n",
			       event_enabled ? "enable" : "disable", event_id,
			       ret);
			return ret;
		}

		/* Reset hw event counter if hw event has been disabled */
		if (!event_enabled)
			state->event_counter = 0;
	}

	/* Reset sw event counter when it's going to disable */
	if ((event_id & LWIS_TRANSACTION_EVENT_FLAG ||
	     event_id & LWIS_TRANSACTION_FAILURE_EVENT_FLAG) &&
	    new_flags == 0)
		state->event_counter = 0;

	/* Check if our specialization cares about flags updates */
	if (lwis_dev->vops.event_flags_updated) {
		ret = lwis_dev->vops.event_flags_updated(lwis_dev, event_id,
							 old_flags, new_flags);
		if (ret) {
			pr_err("Failed updating flags:"
			       " %lld %llx -> %llx (err:%d)\n",
			       event_id, old_flags, new_flags, ret);
		}
	}

	return ret;
}

static void lwis_device_event_heartbeat_timer(struct timer_list *t)
{
	struct lwis_device *lwis_dev = from_timer(lwis_dev, t, heartbeat_timer);

	lwis_device_event_emit(lwis_dev, LWIS_EVENT_ID_HEARTBEAT, NULL, 0,
			       /*in_irq=*/false);

	mod_timer(t, jiffies + msecs_to_jiffies(1000));
}

int lwis_device_event_enable(struct lwis_device *lwis_dev, int64_t event_id,
			     bool enabled)
{
	int ret = -EINVAL, err = 0;
	/* Generic events */
	if (event_id < LWIS_EVENT_ID_START_OF_SPECIALIZED_RANGE) {
		ret = 0;
		switch (event_id) {
		case LWIS_EVENT_ID_HEARTBEAT: {
			if (enabled) {
				timer_setup(&lwis_dev->heartbeat_timer,
					    lwis_device_event_heartbeat_timer,
					    0);
				mod_timer(&lwis_dev->heartbeat_timer, jiffies);
			} else {
				del_timer(&lwis_dev->heartbeat_timer);
			}
			break;
		}
		default: {
			/* We treat this as a real error because there really
			 * shouldn't be anything else handling generic events */
			ret = err = -ENOENT;
			pr_err("Unknown generic event: %lld\n", event_id);
		}
		};
		/* Non-transaction events */
	} else if ((event_id & LWIS_TRANSACTION_EVENT_FLAG) == 0) {
		if (lwis_dev->irqs) {
			ret = lwis_interrupt_event_enable(lwis_dev->irqs,
							  event_id, enabled);
			if (ret && ret != -EINVAL) {
				pr_err("Failed to %s IRQ event: %lld (e:%d)\n",
				       enabled ? "enable" : "disable", event_id,
				       ret);
				err = ret;
			}
		}
	}
	/* Check if our specialization cares about event updates */
	if (!err && lwis_dev->vops.event_enable) {
		ret = lwis_dev->vops.event_enable(lwis_dev, event_id, enabled);
		if (ret && ret != -EINVAL) {
			pr_err("Failed to %s event: %lld (err:%d)\n",
			       enabled ? "enable" : "disable", event_id, ret);
			err = ret;
		}
	}
	return err ? err : ret;
}

static int lwis_device_event_emit_impl(struct lwis_device *lwis_dev,
				       int64_t event_id, void *payload,
				       size_t payload_size,
				       struct list_head *pending_events,
				       bool in_irq)
{
	struct lwis_client_event_state *client_event_state;
	struct lwis_device_event_state *device_event_state;
	struct lwis_event_entry *event;
	/* Our iterators */
	struct lwis_client *lwis_client;
	struct list_head *p, *n;
	int64_t timestamp;
	int64_t event_counter;
	/* Flags for IRQ disable */
	unsigned long flags;
	bool has_subscriber;

	device_event_state = lwis_device_event_state_find(lwis_dev, event_id);
	if (IS_ERR_OR_NULL(device_event_state)) {
		pr_err("Device event state not found %llx\n", event_id);
		return -EINVAL;
	}
	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_dev->lock, flags);

	/* Increment the event counter */
	device_event_state->event_counter++;
	/* Save event counter to local variable */
	event_counter = device_event_state->event_counter;
	/* Saves this event to history buffer */
	save_device_event_state_to_history_locked(lwis_dev, device_event_state);

	/* Unlock and restore device lock */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Latch timestamp */
	timestamp = ktime_to_ns(lwis_get_time());

	/* Emit event to subscriber via top device */
	spin_lock_irqsave(&lwis_dev->lock, flags);
	has_subscriber = (device_event_state->subscriber_count > 0);
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	if (has_subscriber)
		lwis_dev->top_dev->subscribe_ops.notify_event_subscriber(
			lwis_dev->top_dev, event_id, event_counter, timestamp,
			in_irq);

	/* Run internal handler if any */
	if (lwis_dev->vops.event_emitted) {
		int ret = lwis_dev->vops.event_emitted(lwis_dev, event_id,
						       &payload, &payload_size);
		if (ret) {
			pr_warn("Warning: vops.event_emitted returned %d\n",
				ret);
		}
	}

	/* Notify clients */
	list_for_each_safe (p, n, &lwis_dev->clients) {
		bool emit = false;
		lwis_client = list_entry(p, struct lwis_client, node);

		/* Lock the event lock instead */
		spin_lock_irqsave(&lwis_client->event_lock, flags);
		client_event_state = lwis_client_event_state_find_locked(
			lwis_client, event_id);

		if (!IS_ERR_OR_NULL(client_event_state)) {
			if (client_event_state->event_control.flags &
			    LWIS_EVENT_CONTROL_FLAG_QUEUE_ENABLE) {
				emit = true;
			}
		}

		/* Restore the event lock */
		spin_unlock_irqrestore(&lwis_client->event_lock, flags);
		if (emit) {
			event = kzalloc(sizeof(struct lwis_event_entry) +
						payload_size,
					GFP_ATOMIC);
			event->event_info.event_id = event_id;
			event->event_info.event_counter = event_counter;
			event->event_info.timestamp_ns = timestamp;
			event->event_info.payload_size = payload_size;
			if (payload_size > 0) {
				event->event_info.payload_buffer =
					(void *)((uint8_t *)event +
						 sizeof(struct lwis_event_entry));
				memcpy(event->event_info.payload_buffer,
				       payload, payload_size);
			} else {
				event->event_info.payload_buffer = NULL;
			}
			lwis_client_event_push_back(lwis_client, event);
		}

		/* Trigger transactions, if there's any that matches this event
		   ID and counter */
		if (lwis_transaction_event_trigger(lwis_client, event_id,
						   event_counter,
						   pending_events, in_irq)) {
			pr_warn("Failed to process transactions: "
				"Event ID: 0x%llx Counter: %lld\n",
				event_id, event_counter);
		}
	}

	return 0;
}

int lwis_device_event_emit(struct lwis_device *lwis_dev, int64_t event_id,
			   void *payload, size_t payload_size, bool in_irq)
{
	int ret;
	struct list_head pending_events;

	/* Container to store events that are triggered as a result of this
	   event. */
	INIT_LIST_HEAD(&pending_events);

	/* Emit the original event */
	ret = lwis_device_event_emit_impl(lwis_dev, event_id, payload,
					  payload_size, &pending_events,
					  in_irq);
	if (ret) {
		pr_err("lwis_device_event_emit_impl failed: event ID %llx\n",
		       event_id);
		return ret;
	}

	/* Emit pending events */
	return lwis_pending_events_emit(lwis_dev, &pending_events, in_irq);
}

int lwis_pending_event_push(struct list_head *pending_events, int64_t event_id,
			    void *payload, size_t payload_size)
{
	struct lwis_event_entry *event;

	event = kzalloc(sizeof(struct lwis_event_entry) + payload_size,
			GFP_ATOMIC);
	event->event_info.event_id = event_id;
	event->event_info.payload_size = payload_size;
	if (payload_size > 0) {
		event->event_info.payload_buffer =
			(void *)((uint8_t *)event +
				 sizeof(struct lwis_event_entry));
		memcpy(event->event_info.payload_buffer, payload, payload_size);
	} else {
		event->event_info.payload_buffer = NULL;
	}

	list_add_tail(&event->node, pending_events);

	return 0;
}

int lwis_pending_events_emit(struct lwis_device *lwis_dev,
			     struct list_head *pending_events, bool in_irq)
{
	int return_val = 0, emit_result = 0;
	struct lwis_event_entry *event;

	while (!list_empty(pending_events)) {
		event = list_first_entry(pending_events,
					 struct lwis_event_entry, node);
		emit_result = lwis_device_event_emit_impl(
			lwis_dev, event->event_info.event_id,
			event->event_info.payload_buffer,
			event->event_info.payload_size, pending_events, in_irq);
		if (emit_result) {
			return_val = emit_result;
			pr_warn("lwis_device_pending_event_emit error on "
				"ID 0x%llx\n",
				event->event_info.event_id);
		}
		list_del(&event->node);
		kfree(event);
	}
	return return_val;
}

int lwis_device_event_subscribed(struct lwis_device *lwis_dev, int64_t event_id)
{
	int ret = 0;
	unsigned long flags;
	struct lwis_device_event_state *event_state;

	spin_lock_irqsave(&lwis_dev->lock, flags);
	event_state = lwis_device_event_state_find_locked(lwis_dev, event_id);
	if (event_state == NULL) {
		pr_err("Event not found in trigger device");
		ret = -EINVAL;
		goto out;
	}
	event_state->subscriber_count++;
out:
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	return ret;
}
int lwis_device_event_unsubscribed(struct lwis_device *lwis_dev,
				   int64_t event_id)
{
	int ret = 0;
	unsigned long flags;
	struct lwis_device_event_state *event_state;

	spin_lock_irqsave(&lwis_dev->lock, flags);
	event_state = lwis_device_event_state_find_locked(lwis_dev, event_id);
	if (event_state == NULL) {
		pr_err("Event not found in trigger device");
		ret = -EINVAL;
		goto out;
	}
	event_state->subscriber_count--;
out:
	spin_unlock_irqrestore(&lwis_dev->lock, flags);
	return ret;
}

void lwis_device_external_event_emit(struct lwis_device *lwis_dev,
				     int64_t event_id, int64_t event_counter,
				     int64_t timestamp, bool in_irq)
{
	struct lwis_client_event_state *client_event_state;
	struct lwis_device_event_state *device_event_state;
	struct lwis_event_entry *event;
	/* Our iterators */
	struct lwis_client *lwis_client;
	struct list_head *p, *n;
	struct list_head pending_events;
	/* Flags for IRQ disable */
	unsigned long flags;
	bool emit = false;

	INIT_LIST_HEAD(&pending_events);

	device_event_state = lwis_device_event_state_find(lwis_dev, event_id);
	if (IS_ERR_OR_NULL(device_event_state)) {
		pr_err("Device external event state not found %llx\n",
		       event_id);
		return;
	}
	/* Lock and disable to prevent event_states from changing */
	spin_lock_irqsave(&lwis_dev->lock, flags);

	/* Update event counter */
	device_event_state->event_counter = event_counter;

	/* Unlock and restore device lock */
	spin_unlock_irqrestore(&lwis_dev->lock, flags);

	/* Notify clients */
	list_for_each_safe (p, n, &lwis_dev->clients) {
		emit = false;
		lwis_client = list_entry(p, struct lwis_client, node);

		/* Lock the event lock instead */
		spin_lock_irqsave(&lwis_client->event_lock, flags);
		client_event_state = lwis_client_event_state_find_locked(
			lwis_client, event_id);

		if (!IS_ERR_OR_NULL(client_event_state)) {
			if (client_event_state->event_control.flags &
			    LWIS_EVENT_CONTROL_FLAG_QUEUE_ENABLE) {
				emit = true;
			}
		}
		/* Restore the event lock */
		spin_unlock_irqrestore(&lwis_client->event_lock, flags);

		if (emit) {
			event = kzalloc(sizeof(struct lwis_event_entry),
					GFP_ATOMIC);
			event->event_info.event_id = event_id;
			event->event_info.event_counter = event_counter;
			event->event_info.timestamp_ns = timestamp;
			event->event_info.payload_size = 0;
			event->event_info.payload_buffer = NULL;
			lwis_client_event_push_back(lwis_client, event);
		}

		if (lwis_transaction_event_trigger(lwis_client, event_id,
						   event_counter,
						   &pending_events, in_irq))
			pr_warn("Failed to process transactions: "
				"external event ID: 0x%llx counter: %lld\n",
				event_id, event_counter);
	}

	lwis_pending_events_emit(lwis_dev, &pending_events, in_irq);
}
