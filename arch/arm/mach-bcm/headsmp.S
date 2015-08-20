/*
 * Copyright (C) 2015 Broadcom Corporation
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

/*
 * iProc specific entry point for secondary CPUs.  This provides
 * a "holding pen" into which all secondary cores are held until
 * we are ready for them to initialise.
 */
ENTRY(nsp_secondary_startup)
	mrc     p15, 0, r0, c0, c0, 5
	and     r0, r0, #15
	adr     r4, 1f
	ldmia   r4, {r5, r6}
	sub     r4, r4, r5
	add     r6, r6, r4
pen:	ldr     r7, [r6]
	cmp     r7, r0
	bne     pen

	b    secondary_startup

1:	.long   .
	.long   pen_release

ENDPROC(nsp_secondary_startup)
