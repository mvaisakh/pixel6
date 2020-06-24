/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU multi-chip package management.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_MCP_H__
#define __EDGETPU_MCP_H__

#include <linux/init.h>
#include <linux/mutex.h>
#include <linux/types.h>

#include "edgetpu-config.h"
#include "edgetpu-internal.h"

#ifdef EDGETPU_HAS_MCP

struct edgetpu_mcp {
	/* constant fields after initialization */

	u8 id;		/* the MCP ID matches etdev->mcp_id */
	u8 pkgtype;	/* package type, definition is chip-dependent */
	u8 total_num;	/* total number of etdevs expected by this MCP */

	/* fields need to be locked before accessing */

	struct mutex lock;
	u8 cur_num;	/* current number of etdevs registered */
	/*
	 * Array of etdevs in this MCP, its length is @total_num.
	 *
	 * @etdevs[i] equals NULL means the etdev with mcp_die_index = i is not
	 * added yet.
	 */
	struct edgetpu_dev **etdevs;
};

/*
 * Registers @etdev to the MCP manager.
 *
 * @etdev->mcp_pkg_type, @etdev->mcp_id, and @etdev->mcp_die_index must be set
 * before this call.
 *
 * Returns 0 on success, or -errno on error.
 */
int edgetpu_mcp_add_etdev(struct edgetpu_dev *etdev);

/*
 * Reverts edgetpu_mcp_add_etdev().
 *
 * @etdev->mcp_id and @etdev->mcp_die_index must be the same when called
 * edgetpu_mcp_add_etdev().
 */
void edgetpu_mcp_remove_etdev(struct edgetpu_dev *etdev);

/*
 * Invokes @callback with each (currently) registered MCP.
 *
 * If @stop_on_err is true, this function stops when @callback returned non-zero
 * value. And that value is also returned by this function.
 * If @stop_on_err is false, @callback will be called exactly the number of
 * registered MCPs times, and this function will always return 0.
 *
 * @data can be any value, it will be directly passed to @callback.
 *
 * @callback is expected to return 0 on success, -errno otherwise.
 *
 * Don't call this or other edgetpu_mcp_* functions recursively to prevent dead
 * locking.
 *
 * It's @callback's responsibility to hold edgetpu_mcp's lock when access
 * non-constant fields of edgetpu_mcp.
 */
int edgetpu_mcp_each(bool stop_on_err, void *data,
		     int (*callback)(struct edgetpu_mcp *, void *));

/*
 * Returns the MCP object @etdev belongs to.
 *
 * Returns NULL when such object is not found.
 *
 * Note: The returned pointer will be released when the last etdev is removed.
 * Don't use the returned pointer after edgetpu_mcp_remove_etdev() is called.
 */
struct edgetpu_mcp *edgetpu_mcp_of_etdev(struct edgetpu_dev *etdev);

/* Returns the next available MCP ID. */
int edgetpu_mcp_next_id(void);

/* To allocate / release structures for MCP management */
void __init edgetpu_mcp_init(void);
void __exit edgetpu_mcp_exit(void);

#else /* !EDGETPU_HAS_MCP */

static inline void __init edgetpu_mcp_init(void)
{
}

static inline void __exit edgetpu_mcp_exit(void)
{
}

#endif /* EDGETPU_HAS_MCP */

#endif /* __EDGETPU_MCP_H__ */
