/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Asynchronous jobs management for EdgeTPU driver.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_ASYNC_H__
#define __EDGETPU_ASYNC_H__

#include <linux/async.h>
#include <linux/list.h>
#include <linux/mutex.h>

typedef void *(*edgetpu_async_job_t)(void *);

struct edgetpu_async_ctx {
	/* constant fields after initialized */

	struct async_domain async_domain;

	/* fields need to be protected by @lock */

	struct mutex lock;
	/* Jobs to be executed, added by edgetpu_async_add_job() */
	struct list_head jobs;
	/* Number of jobs added by edgetpu_async_add_job(). */
	uint n_jobs;
	/*
	 * Return values of jobs. This field is available after
	 * edgetpu_async_wait() is called.
	 */
	void **ret;
};

/*
 * Helper to loop through the return values. Use this if and only if
 * edgetpu_async_wait(ctx) is executed successfully.
 *
 * Example:
 *   for_each_async_ret(ctx, ret, i) { ... }
 */
#define for_each_async_ret(ctx, val, i)                                        \
	for (i = 0, val = (typeof(val))(size_t)((ctx)->ret[i]);                \
	     i < (ctx)->n_jobs;                                                \
	     ++i < (ctx)->n_jobs ?                                             \
		     (val = (typeof(val))(size_t)((ctx)->ret[i])) :            \
		     0)

/*
 * Allocates and initializes a context for further use.
 *
 * Use edgetpu_async_free_ctx() to release the returned pointer.
 *
 * Returns NULL when out-of-memory.
 */
struct edgetpu_async_ctx *edgetpu_async_alloc_ctx(void);

/*
 * Registers a new job to the context.
 *
 * @job: the function to be executed. Accepts @data as its argument and its
 * return value will be recorded in @ctx.
 *
 * Note: @job is not executed within this call. Call edgetpu_async_wait() to
 * execute all registered jobs.
 *
 * Returns 0 on success.
 * Returns -EINVAL if edgetpu_async_wait() is called.
 */
int edgetpu_async_add_job(struct edgetpu_async_ctx *ctx, void *data,
			  edgetpu_async_job_t job);
/*
 * Executes all registered jobs and wait until they finished.
 *
 * No job is executed again if call this function twice.
 *
 * Use macro for_each_async_ret as a helper to check the return values of jobs.
 *
 * Returns 0 when all jobs are executed.
 * Returns -ENOMEM when out-of-memory, no job is executed.
 */
int edgetpu_async_wait(struct edgetpu_async_ctx *ctx);

/*
 * Frees the resources allocated from edgetpu_async_alloc_ctx().
 *
 * The pointer of @ctx is freed in this function.
 *
 * It's safe that @ctx is NULL, nothing is done in this case.
 *
 * If this function is called before calling edgetpu_async_wait(), no job is
 * ever executed.
 */
void edgetpu_async_free_ctx(struct edgetpu_async_ctx *ctx);

#endif /* __EDGETPU_ASYNC_H__ */
