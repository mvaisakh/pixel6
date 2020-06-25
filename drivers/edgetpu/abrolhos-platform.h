/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Platform device driver for the Google Edge TPU ML accelerator.
 *
 * Copyright (C) 2019 Google, Inc.
 */
#ifndef __EDGETPU_PLATFORM_H__
#define __EDGETPU_PLATFORM_H__

#include "edgetpu-internal.h"
#include "abrolhos-pwr.h"

#define TPU_ACPM_DOMAIN			14

#define TPU_DEBUG_REQ			(1 << 31)
#define TPU_VDD_TPU_DEBUG		(0 << 28)
#define TPU_VDD_TPU_M_DEBUG		(1 << 28)
#define TPU_VDD_INT_M_DEBUG		(2 << 28)
#define TPU_CLK_CORE_DEBUG		(3 << 28)
#define TPU_CLK_CTL_DEBUG		(4 << 28)
#define TPU_CLK_AXI_DEBUG		(5 << 28)
#define TPU_CLK_APB_DEBUG		(6 << 28)
#define TPU_CLK_UART_DEBUG		(7 << 28)
#define TPU_DEBUG_VALUE_MASK	((1 << 28) - 1)

#define OSCCLK_RATE		24576000
#define PLL_SHARED0_DIV0	1066000000
#define PLL_SHARED1_DIV2	933000000
#define PLL_SHARED2			800000000
#define PLL_SHARED3			666000000
#define PLL_SHARED0_DIV3	711000000
#define PLL_SHARED1_DIV3	622000000
#define PLL_SHARED0_DIV4	533000000
#define PLL_SHARED2_DIV2	400000000
#define PLL_SHARED3_DIV2	333000000

#define TPU_CMU_TOP_REG 0x1E080000
#define MUX_CLKCMU_TPU_TPU			(TPU_CMU_TPU_REG + 0x10FC)
#define DIV_CLKCMU_TPU_TPU			(TPU_CMU_TPU_REG + 0x18EC)

#define MUX_CLKCMU_TPU_TPUCTL		(TPU_CMU_TPU_REG + 0x1100)
#define DIV_CLKCMU_TPU_TPUCTL		(TPU_CMU_TPU_REG + 0x18F0)

#define MUX_CLKCMU_TPU_BUS			(TPU_CMU_TPU_REG + 0x10F8)
#define DIV_CLKCMU_TPU_BUS			(TPU_CMU_TPU_REG + 0x18E8)

#define MUX_CLKCMU_TPU_UART			(TPU_CMU_TPU_REG + 0x1104)
#define DIV_CLKCMU_TPU_UART			(TPU_CMU_TPU_REG + 0x18F4)

#define TPU_CMU_TPU_REG 0x1CC00000

#define PLL_CON0_PLL_TPU			(TPU_CMU_TPU_REG + 0x0100)
#define PLL_CON2_PLL_TPU			(TPU_CMU_TPU_REG + 0x0108)
#define MUX_CLK_TPU_TPU				(TPU_CMU_TPU_REG + 0x1000)
#define MUX_CLKCMU_TPU_TPU_USER		(TPU_CMU_TPU_REG + 0x0620)
#define DIV_CLK_TPU_TPU				(TPU_CMU_TPU_REG + 0x1804)

#define MUX_CLK_TPU_TPUCTL			(TPU_CMU_TPU_REG + 0x1004)
#define MUX_CLKCMU_TPU_TPUCTL_USER	(TPU_CMU_TPU_REG + 0x0610)
#define DIV_CLK_TPU_TPUCTL			(TPU_CMU_TPU_REG + 0x1808)

#define MUX_CLKCMU_TPU_BUS_USER		(TPU_CMU_TPU_REG + 0x0600)

#define DIV_CLK_TPU_BUSP			(TPU_CMU_TPU_REG + 0x1800)

#define MUX_CLKCMU_TPU_UART_USER	(TPU_CMU_TPU_REG + 0x0630)

#define MUX_USER_SEL_MASK		0x1
#define MUX_USER_OSCCLK_SEL		0
#define MUX_USER_CMUCLK_SEL		1

#define MUX_CMU_SEL_MASK		0x7
#define MUX_CMU_S0_D0_SEL		0
#define MUX_CMU_S1_D2_SEL		1
#define MUX_CMU_S2_SEL			2
#define MUX_CMU_S3_SEL			3
#define MUX_CMU_S0_D3_SEL		4
#define MUX_CMU_S1_D3_SEL		5
#define MUX_CMU_S0_D4_SEL		6

#define MUX_CMU_UART_SEL_MASK		0x3
#define MUX_CMU_UART_S0_D4_SEL		0
#define MUX_CMU_UART_S2_D2_SEL		1
#define MUX_CMU_UART_S3_D2_SEL		2

#define DIV_CMU_RATIO_MASK		0xf
#define DIV_USER_RATIO_MASK		0x7

struct edgetpu_platform_pwr {
	struct mutex policy_lock;
	enum tpu_pwr_state curr_policy;
};

struct edgetpu_platform_dev {
	struct edgetpu_dev edgetpu_dev;
	struct edgetpu_platform_pwr platform_pwr;
	int irq;
	void *fw_region_vaddr;
	size_t fw_region_size;
	dma_addr_t csr_iova;
	size_t csr_size;
};

#endif /* __EDGETPU_PLATFORM_H__ */
