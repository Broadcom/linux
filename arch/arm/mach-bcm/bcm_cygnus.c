/*
 * Copyright (C) 2014-2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <asm/mach/arch.h>
#include <asm/mach/bcm_optee_smc.h>
#include <asm/mach/map.h>
#include <asm/proc-fns.h>
#include "cygnus_pm.h"

static const char * const bcm_cygnus_dt_compat[] __initconst = {
	"brcm,cygnus",
	NULL,
};

static void cygnus_early_init(void)
{
	struct arm_smccc_res res;
	unsigned int cmd;

	cmd = FAST_SMC(SSAPI_ENABLE_L2_CACHE);
	arm_smccc_smc(cmd, machine_desc->l2c_aux_val,
		      machine_desc->l2c_aux_mask, 0, 0, 0, 0, 0, &res);
}

DT_MACHINE_START(BCM_CYGNUS_DT, "Broadcom Cygnus SoC")
	.init_early	= cygnus_early_init,
	.init_late	= cygnus_pm_init,
	.l2c_aux_val	= 0,
	.l2c_aux_mask	= ~0,
	.dt_compat = bcm_cygnus_dt_compat,
MACHINE_END
