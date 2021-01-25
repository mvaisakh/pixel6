/* SPDX-License-Identifier: GPL-2.0 OR Apache-2.0 */
/*
 * Google Whitechapel Audio Metrics Driver
 *
 * Copyright (c) 2021 Google LLC
 *
 */

#ifndef _AUDIOMETRICS_H
#define _AUDIOMETRICS_H

#include "uapi/audiometrics_api.h"

enum {
	CODEC_STATE_UNKNOWN = -99,
	CODEC_STATE_ONLINE = 0,
};

enum {
	WDSP_STAT_CRASH = 0,
	WDSP_STAT_DOWN,
	WDSP_STAT_UP,
};

#endif /* _AUDIOMETRICS_H */
