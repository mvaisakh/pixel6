/*
 * Google LWIS Regulator Interface
 *
 * Copyright (c) 2018 Google, LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef LWIS_REGULATOR_H_
#define LWIS_REGULATOR_H_

#include <linux/regulator/consumer.h>

struct lwis_regulator {
	struct regulator *reg;
	char *name;
};

struct lwis_regulator_list {
	struct lwis_regulator *reg;
	int count;
};

/*
 *  lwis_regulator_list_alloc: Allocate an instance of the lwis_regulator_list
 *  and initialize the data structures according to the number of regulators
 *  specified.
 *  NOTE: This does not register the regulator structs.
 */
struct lwis_regulator_list *lwis_regulator_list_alloc(int num_regs);

/*
 *  lwis_regulator_list_free: Deallocate the lwis_regulator_list structure.
 */
void lwis_regulator_list_free(struct lwis_regulator_list *list);

/*
 *  lwis_regulator_set: Register the regulator by name.
 */
int lwis_regulator_set(struct lwis_regulator_list *list, struct device *pdev,
		       int index, char *name);

/*
 *  lwis_regulator_put: Unregister the regulator by index.
 */
int lwis_regulator_put(struct lwis_regulator_list *list, int index);

/*
 *  lwis_regulator_enable: Turn on/enable the regulator by index.
 */
int lwis_regulator_enable(struct lwis_regulator_list *list, int index);

/*
 *  lwis_regulator_enable_all: Turn on/enable all the regulators.
 */
int lwis_regulator_enable_all(struct lwis_regulator_list *list);

/*
 *  lwis_regulator_disable: Turn off/disable the regulator by index.
 */
int lwis_regulator_disable(struct lwis_regulator_list *list, int index);

/*
 *  lwis_regulator_disable_all: Turn off/disable all the regulators.
 */
int lwis_regulator_disable_all(struct lwis_regulator_list *list);

/*
 *  lwis_regulator_print: Debug function to print all the regulators in the
 *  supplied list.
 */
void lwis_regulator_print(struct lwis_regulator_list *list);

#endif  /* LWIS_REGULATOR_H_ */