/*
 * Google LWIS Regulator Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/slab.h>

#include "lwis_regulator.h"

struct lwis_regulator_list *lwis_regulator_list_alloc(int num_regs)
{
	struct lwis_regulator_list *list;

	if (num_regs < 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kzalloc(sizeof(struct lwis_regulator_list), GFP_KERNEL);
	if (!list) {
		return ERR_PTR(-ENOMEM);
	}

	list->reg = kzalloc(num_regs * sizeof(struct lwis_regulator),
			    GFP_KERNEL);
	if (!list->reg) {
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = num_regs;

	return list;
}

void lwis_regulator_list_free(struct lwis_regulator_list *list)
{
	if (!list) {
		return;
	}

	if (list->reg) {
		kfree(list->reg);
	}

	kfree(list);
}

int lwis_regulator_set(struct lwis_regulator_list *list, struct device *pdev,
		       int index, char *name)
{
	struct regulator *reg;

	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	reg = devm_regulator_get_optional(pdev, name);
	if (IS_ERR(reg)) {
		return PTR_ERR(reg);
	}

	list->reg[index].reg = reg;
	list->reg[index].name = name;

	return 0;
}

int lwis_regulator_put(struct lwis_regulator_list *list, int index)
{
	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(list->reg[index].reg)) {
		return -EINVAL;
	}

	devm_regulator_put(list->reg[index].reg);
	list->reg[index].reg = NULL;

	return 0;
}

int lwis_regulator_enable(struct lwis_regulator_list *list, int index)
{
	return regulator_enable(list->reg[index].reg);
}

int lwis_regulator_disable(struct lwis_regulator_list *list, int index)
{
	return regulator_disable(list->reg[index].reg);
}

void lwis_regulator_print(struct lwis_regulator_list *list)
{
	int i;

	for (i = 0; i < list->count; ++i) {
		pr_info("%s: reg: %s\n", __func__, list->reg[i].name);
	}
}
