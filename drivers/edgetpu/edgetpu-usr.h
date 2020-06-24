/* SPDX-License-Identifier: GPL-2.0 */
/*
 * EdgeTPU Ultra Short Reach utilities.
 *
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef __EDGETPU_USR_H__
#define __EDGETPU_USR_H__

#include "edgetpu-config.h"
#include "edgetpu-internal.h"
#include "edgetpu-mcp.h"

#ifdef EDGETPU_HAS_VN

/*
 * Trains the USR links of all devices in @mcp.
 *
 * Caller holds @mcp->lock.
 */
void edgetpu_usr_init_mcp(struct edgetpu_mcp *mcp);

void edgetpu_usr_init_single(struct edgetpu_dev *etdev);

#else /* !EDGETPU_HAS_VN */

static inline void edgetpu_usr_init_mcp(struct edgetpu_mcp *mcp)
{
}

#endif /* EDGETPU_HAS_VN */

#endif /* __EDGETPU_USR_H__ */
