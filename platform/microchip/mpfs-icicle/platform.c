/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Microchip SoC Products Group.
 *
 * Authors:
 *   Lewis Hanly <lewis.hanly@microchip.com>
 */

#include <sbi/riscv_encoding.h>
#include <sbi/sbi_const.h>
#include <sbi/sbi_hart.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_console.h>
#include <plat/irqchip/plic.h>
#include <plat/sys/clint.h>

#include "mss_uart_regs.h"
#include "mss_uart.h"


#define MPFS_HART_COUNT			 	5
#define MPFS_HART_STACK_SIZE	 	8192

#define MPFS_SYS_CLK		     	1000000000


#define MPFS_CLINT_ADDR			 	(0x02000000U)

#define MPFS_PLIC_ADDR				(0x0C000000U)
#define MPFS_PLIC_NUM_SOURCES		0x35
#define MPFS_PLIC_NUM_PRIORITIES	7


#define MPFS_UART1_ADDR				(0x10011000U)
#define MPFS_UART_BAUDRATE			115200

/**
 * The MPFS SoC has 5 HARTs but HART ID 0 doesn't have S mode. enable only
 * HARTs 1 to 4.
 */
#ifndef MPFS_ENABLED_HART_MASK
#define MPFS_ENABLED_HART_MASK (1 << 1 | 1 << 2 | 1 << 3 | 1 << 4)
#endif

#define MPFS_HARITD_DISABLED			~(MPFS_ENABLED_HART_MASK)

mss_uart_instance_t *g_uart= &g_mss_uart0_lo ;



static u32 mpfs_pmp_region_count(u32 hartid)
{
	return 1;
}

static int mpfs_pmp_region_info(u32 hartid, u32 index,
				 ulong *prot, ulong *addr, ulong *log2size)
{
	int ret = 0;

	switch (index) {
	case 0:
		*prot = PMP_R | PMP_W | PMP_X;
		*addr = 0;
		*log2size = __riscv_xlen;
		break;
	default:
		ret = -1;
		break;
	};

	return ret;
}

static int mpfs_console_init(void)
{
	MSS_UART_init(g_uart, MPFS_UART_BAUDRATE,
	        MSS_UART_DATA_8_BITS | MSS_UART_NO_PARITY | MSS_UART_ONE_STOP_BIT,
	        MSS_UART0_INTR_LOCAL);



	return 0;
}

int mpfs_uart_getc(void)
{
	size_t bufsize = 0;
	uint8_t bufchar = 0;
	bufsize = MSS_UART_get_rx(g_uart, &bufchar, 1);

	if(bufsize != 1)
		bufchar = 0;

	return (int)bufchar;
}

void mpfs_uart_putc(char ch)
{
	MSS_UART_polled_tx(g_uart,(const uint8_t *)&ch, 1);
}


static int mpfs_irqchip_init(bool cold_boot)
{
	int rc;
	u32 hartid = sbi_current_hartid();

	if (cold_boot) {
		rc = plic_cold_irqchip_init(MPFS_PLIC_ADDR,
					    MPFS_PLIC_NUM_SOURCES,
					    MPFS_HART_COUNT);
		if (rc)
			return rc;
	}

	return plic_warm_irqchip_init(hartid,
			(hartid) ? (2 * hartid - 1) : 0,
			(hartid) ? (2 * hartid) : -1);
}

static int mpfs_ipi_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_ipi_init(MPFS_CLINT_ADDR,
					 MPFS_HART_COUNT);
		if (rc)
			return rc;

	}

	return clint_warm_ipi_init();
}

static int mpfs_timer_init(bool cold_boot)
{
	int rc;

	if (cold_boot) {
		rc = clint_cold_timer_init(MPFS_CLINT_ADDR,
					   MPFS_HART_COUNT);
		if (rc)
			return rc;
	}

	return clint_warm_timer_init();
}

static int mpfs_system_down(u32 type)
{
	/* For now nothing to do. */
	return 0;
}

struct sbi_platform platform = {
	.name = "Microchip MPFS Icicle",
	.features = SBI_PLATFORM_DEFAULT_FEATURES,
	.hart_count = MPFS_HART_COUNT,
	.hart_stack_size = MPFS_HART_STACK_SIZE,
	.disabled_hart_mask = MPFS_HARITD_DISABLED,
	.pmp_region_count = mpfs_pmp_region_count,
	.pmp_region_info = mpfs_pmp_region_info,
	.console_putc = mpfs_uart_putc,
	.console_getc = mpfs_uart_getc,
	.console_init = mpfs_console_init,	
	.irqchip_init = mpfs_irqchip_init,
	.ipi_send = clint_ipi_send,
	.ipi_sync = clint_ipi_sync,
	.ipi_clear = clint_ipi_clear,
	.ipi_init = mpfs_ipi_init,
	.timer_value = clint_timer_value,
	.timer_event_stop = clint_timer_event_stop,
	.timer_event_start = clint_timer_event_start,
	.timer_init = mpfs_timer_init,
	.system_reboot = mpfs_system_down,
	.system_shutdown = mpfs_system_down
};
