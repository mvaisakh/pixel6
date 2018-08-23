/*
 * Google LWIS Clock Interface
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

#include "lwis_clock.h"

struct lwis_clock_list *lwis_clock_list_alloc(int num_clks)
{
	struct lwis_clock_list *list;

	/* No need to allocate if num_clks is invalid */
	if (num_clks <= 0) {
		return ERR_PTR(-EINVAL);
	}

	list = kzalloc(sizeof(struct lwis_clock_list), GFP_KERNEL);
	if (!list) {
		return ERR_PTR(-ENOMEM);
	}

	list->clk = kzalloc(num_clks * sizeof(struct lwis_clock), GFP_KERNEL);
	if (!list->clk) {
		kfree(list);
		return ERR_PTR(-ENOMEM);
	}

	list->count = num_clks;

	return list;
}

void lwis_clock_list_free(struct lwis_clock_list *list)
{
	if (!list) {
		return;
	}

	if (list->clk) {
		kfree(list->clk);
	}

	kfree(list);
}

int lwis_clock_set(struct lwis_clock_list *list, struct device *pdev,
		   int index, char *name, int rate)
{
	struct clk *clk;

	if (!pdev || !list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	clk = devm_clk_get(pdev, name);

	if (IS_ERR(clk)) {
		return PTR_ERR(clk);
	}

	list->clk[index].clk = clk;
	list->clk[index].name = name;
	list->clk[index].rate = rate;

	return 0;
}

int lwis_clock_put(struct lwis_clock_list *list, struct device *pdev, int index)
{
	if (!list || index < 0 || index >= list->count) {
		return -EINVAL;
	}

	if (IS_ERR_OR_NULL(list->clk[index].clk)) {
		return -EINVAL;
	}

	devm_clk_put(pdev, list->clk[index].clk);
	list->clk[index].clk = NULL;

	return 0;
}

int lwis_clock_enable(struct lwis_clock_list *list, int index)
{
	int ret = 0;

	ret = clk_prepare_enable(list->clk[index].clk);
	if (ret) {
		return ret;
	}

	if (list->clk[index].rate > 0) {
		ret = clk_set_rate(list->clk[index].clk, list->clk[index].rate);
	}

	return ret;
}

void lwis_clock_disable(struct lwis_clock_list *list, int index)
{
	clk_disable_unprepare(list->clk[index].clk);
}

void lwis_clock_print(struct lwis_clock_list *list)
{
	int i;
	for (i = 0; i < list->count; ++i) {
		pr_info("%s: %s: rate: %d\n", __func__, list->clk[i].name,
			list->clk[i].rate);
	}
}

