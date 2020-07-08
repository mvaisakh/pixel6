// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2020 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef MAX77759_H_
#define MAX77759_H_

#include <linux/i2c.h>

#define SCNPRINTF(...) scnprintf(__VA_ARGS__)
#include "max77759_regs.h"

#define MAX77759_CHG_INT_COUNT 2
#define MAX77759_PMIC_PMIC_ID_MW	0x3b

/*
 * b/156527175: workaround for read only MAX77759_CHG_DETAILS_03
 * MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7 is used to detect exit from fshipmode.
 * The register (MAX77759_PMIC_TOPSYS_INT_MASK) is type S and the bit is reset
 * to 1 on power loss. The reports MAX77759_CHG_DETAILS_03 when the bit
 * is 1 and report 0 when the bit is set to 0.
 */
#define MAX77759_FSHIP_EXIT_DTLS	  MAX77759_PMIC_TOPSYS_INT_MASK
#define MAX77759_FSHIP_EXIT_DTLS_RD \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7
#define MAX77759_FSHIP_EXIT_DTLS_RD_SHIFT \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_SHIFT
#define MAX77759_FSHIP_EXIT_DTLS_RD_MASK \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_MASK
#define MAX77759_FSHIP_EXIT_DTLS_RD_CLEAR \
				MAX77759_PMIC_TOPSYS_INT_MASK_SPR_7_CLEAR

int max777x9_pmic_reg_read(struct i2c_client *client,
			   u8 addr, u8 *val, int len);
int max777x9_pmic_reg_write(struct i2c_client *client,
			    u8 addr, const u8 *val, int len);
int max777x9_pmic_reg_update(struct i2c_client *client,
			     u8 reg, u8 mask, u8 value);

/* mux configuration in MAX77759_PMIC_CONTROL_FG */
#define THMIO_MUX_BATT_PACK	0
#define THMIO_MUX_USB_TEMP	1
#define THMIO_MUX_BATT_ID	2


#endif