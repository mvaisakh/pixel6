/*
 * Google LWIS Clock Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_CLOCK_H_
#define LWIS_CLOCK_H_

#include <linux/clk.h>
#include <linux/device.h>

/*
 *  LWIS Clock Structures
 */

struct lwis_clock {
	struct clk *clk;
	char *name;
	int rate;
};

struct lwis_clock_list {
	struct lwis_clock *clk;
	int count;
};

/*
 *  LWIS Clock Interface Functions
 */

/*
 *  lwis_clock_list_alloc: Allocate an instance of the lwis_clock_list
 *  and initialize the data structures according to the number of clocks
 *  specified.
 *  NOTE: This does not register the clock structs.
 */
struct lwis_clock_list *lwis_clock_list_alloc(int num_clks);

/*
 *  lwis_clock_list_free: Deallocate the lwis_clock_list structure.
 */
void lwis_clock_list_free(struct lwis_clock_list *list);

/*
 *  lwis_clock_set: Register the clock by name and store its assigned
 *  clock rate.
 */
int lwis_clock_set(struct lwis_clock_list *list, struct device *pdev,
		   int index, char *name, int rate);

/*
 *  lwis_clock_put: Unregister the clock by index.
 */
int lwis_clock_put(struct lwis_clock_list *list, struct device *pdev,
		   int index);

/*
 *  lwis_clock_enable: Enable clock by index.
 */
int lwis_clock_enable(struct lwis_clock_list *list, int index);

/*
 *  lwis_clock_enable_all: Enable all clocks.
 */
int lwis_clock_enable_all(struct lwis_clock_list *list);

/*
 *  lwis_clock_disable: Disable clock by index.
 */
void lwis_clock_disable(struct lwis_clock_list *list, int index);

/*
 *  lwis_clock_disable_all: Disable all clocks.
 */
void lwis_clock_disable_all(struct lwis_clock_list *list);

/*
 *  lwis_clock_print: Debug function to print all the clocks in the
 *  supplied list.
 */
void lwis_clock_print(struct lwis_clock_list *list);

#endif  /* LWIS_CLOCK_H_ */