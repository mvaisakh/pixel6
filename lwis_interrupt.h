/*
 * Google LWIS Interrupt Handler
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_INTERRUPT_H_
#define LWIS_INTERRUPT_H_

#include <linux/interrupt.h>
#include <linux/platform_device.h>

struct lwis_interrupt {
	int irq;
	char *name;
};

/*
 *  struct lwis_interrupt_list
 *  This is to store the list of interrupts the LWIS device uses
 */
struct lwis_interrupt_list {
	struct lwis_interrupt *irq;
	int count;
};

/*
 *  lwis_interrupt_list_alloc: Allocate an instance of the lwis_interrupt_list
 *  and initialize the data structures according to the number of interrupts
 *  specified.
 */
struct lwis_interrupt_list *lwis_interrupt_list_alloc(int count);

/*
 *  lwis_interrupt_list_free: Deallocate the lwis_interrupt_list structure.
 */
void lwis_interrupt_list_free(struct lwis_interrupt_list *list);

/*
 *  lwis_interrupt_get: Register the interrupt by name.
 *  Returns: index number (>= 0) if success, -ve if error
 */
int lwis_interrupt_get(struct lwis_interrupt_list *list, int index, char *name,
		       struct platform_device *plat_dev);

/*
 *  lwis_interrupt_request_by_idx: Request interrupt by index, which also
 *  register an interrupt handler.
 *  Returns: 0 if success, -ve if error
 */
int lwis_interrupt_request_by_idx(struct lwis_interrupt_list *list, int index,
				  irq_handler_t handler, void *dev);

/*
 *  lwis_interrupt_request_by_name: Request interrupt by name, which also
 *  register an interrupt handler.
 *  Returns: 0 if success, -ve if error
 */
int lwis_interrupt_request_by_name(struct lwis_interrupt_list *list, char *name,
				   irq_handler_t handler, void *dev);

/*
 *  lwis_interrupt_free_by_idx: Free interrupt by index.
 *  Returns: 0 if success, -ve if error
 */
void lwis_interrupt_free_by_idx(struct lwis_interrupt_list *list, int index,
				void *dev);

/*
 *  lwis_interrupt_free_by_name: Free interrupt by name.
 *  Returns: 0 if success, -ve if error
 */
void lwis_interrupt_free_by_name(struct lwis_interrupt_list *list, char *name,
				 void *dev);

/*
 *  lwis_interrupt_print: Debug function to print all the interrupts in the
 *  supplied list.
 */
void lwis_interrupt_print(struct lwis_interrupt_list *list);

#endif /* LWIS_INTERRUPT_H_ */