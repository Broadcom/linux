/*
 * Copyright (C) 2016 Broadcom
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

#include <linux/kernel.h>
#include <linux/suspend.h>
#include "iproc_pm.h"

static int iproc_suspend_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_MEM:
		cpu_do_idle();
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct platform_suspend_ops iproc_suspend_ops = {
	.enter = iproc_suspend_enter,
	.valid = suspend_valid_only_mem,
};

void __init iproc_pm_init(void)
{
	suspend_set_ops(&iproc_suspend_ops);
}
