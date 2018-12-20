/*
 * Copyright (c) 2018 Western Digital Corporation or its affiliates.
 *
 * Authors:
 *   Anup Patel <anup.patel@wdc.com>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef __SBI_SYSTEM_H__
#define __SBI_SYSTEM_H__

#include <sbi/sbi_types.h>

struct sbi_scratch;

int sbi_system_warm_early_init(struct sbi_scratch *scratch, u32 hartid);

int sbi_system_warm_final_init(struct sbi_scratch *scratch, u32 hartid);

int sbi_system_cold_early_init(struct sbi_scratch *scratch);

int sbi_system_cold_final_init(struct sbi_scratch *scratch);

void __attribute__((noreturn)) sbi_system_reboot(struct sbi_scratch *scratch,
						 u32 type);

void __attribute__((noreturn)) sbi_system_shutdown(struct sbi_scratch *scratch,
						   u32 type);

#endif