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
#include <linux/linkage.h>

	.align

/*
 * Kernel resume entry. Invalidates the L1 cache then resume.
 */
ENTRY(cygnus_pm_cpu_resume)
	bl  v7_invalidate_l1
	b   cpu_resume
ENDPROC(cygnus_pm_cpu_resume)
