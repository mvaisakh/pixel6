/*
 * Google LWIS Interrupt Handler
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME "-int: " fmt

#include "lwis_interrupt.h"

#include <linux/kernel.h>
#include <linux/slab.h>

struct lwis_interrupt_list *lwis_interrupt_list_alloc(int count)
{
	struct lwis_interrupt_list *list;

	/* No need to allocate if count is invalid */
	if (count <= 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kzalloc(sizeof(struct lwis_interrupt_list), GFP_KERNEL);
	if (!list) {
		pr_err("Failed to allocate IRQ list\n");
		return ERR_PTR(-ENOMEM);
	}

	list->irq = kzalloc(count * sizeof(struct lwis_interrupt),
			    GFP_KERNEL);
	if (!list->irq) {
		pr_err("Failed to allocate IRQs\n");
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = count;

	return list;
}

void lwis_interrupt_list_free(struct lwis_interrupt_list *list)
{
	if (!list) {
		return;
	}

	if (list->irq) {
		kfree(list->irq);
	}

	kfree(list);
}

int lwis_interrupt_get(struct lwis_interrupt_list *list, int index, char *name,
		       struct platform_device *plat_dev)
{
	int irq;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	irq = platform_get_irq(plat_dev, index);
	if (irq <= 0) {
		pr_err("Error retriving interrupt %s at %d\n", name, index);
		return -EINVAL;
	}

	list->irq[index].irq = irq;
	list->irq[index].name = name;

	return 0;
}

int lwis_interrupt_request_by_idx(struct lwis_interrupt_list *list, int index,
				  irq_handler_t handler, void *dev)
{
	char irq_name[16];

	if (!list) {
		return -EINVAL;
	}

	snprintf(irq_name, 16, "lwis-irq%d", index);

	return request_irq(list->irq[index].irq, handler, IRQF_GIC_MULTI_TARGET,
			   irq_name, dev);
}

int lwis_interrupt_request_by_name(struct lwis_interrupt_list *list, char *name,
				   irq_handler_t handler, void *dev)
{
	int i;

	if (!list) {
		return -EINVAL;
	}

	for (i = 0; i < list->count; ++i) {
		if (!strcmp(list->irq[i].name, name)) {
			return lwis_interrupt_request_by_idx(list, i, handler,
							     dev);
		}
	}

	pr_err("Interrupt %s not found\n", name);
	return -ENOENT;
}

void lwis_interrupt_free_by_idx(struct lwis_interrupt_list *list, int index,
				void *dev)
{
	if (!list) {
		return;
	}

	free_irq(list->irq[index].irq, dev);
}

void lwis_interrupt_free_by_name(struct lwis_interrupt_list *list, char *name,
				 void *dev)
{
	int i;

	if (!list) {
		return;
	}

	for (i = 0; i < list->count; ++i) {
		if (!strcmp(list->irq[i].name, name)) {
			lwis_interrupt_free_by_idx(list, i, dev);
			return;
		}
	}

	pr_err("Interrupt %s not found\n", name);
}

void lwis_interrupt_print(struct lwis_interrupt_list *list)
{
	int i;
	for (i = 0; i < list->count; ++i) {
		pr_info("%s: irq: %s\n", __func__, list->irq[i].name);
	}
}