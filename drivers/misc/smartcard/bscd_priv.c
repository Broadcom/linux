/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/sched.h>

#include "bscd_datatypes.h"
#include "bscd_isopriv.h"
#include "bscd_priv.h"
#include "sci_regs.h"

#define BSTD_UNUSED(x)
#define BSCD_INTERRUPT_DEBUG

/* Population count of 1's in a byte */
static const unsigned char bscd_p_popcnt[] = {
	0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7,
	4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8
};

static const struct p_df_sc_struct p_df_arr[10][14] = {
#ifdef BSCD_EMV_TEST
	 /* D = 0 */
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },

	 /* D = 1 */
	{{0x01, 0x0B, 0x1F, 0x08}, {0x02, 0x17, 0x1F, 0x08},
	 {0x01, 0x11, 0x1F, 0x08}, {0x01, 0x17, 0x1F, 0x08},
	 {0x01, 0x23, 0x1F, 0x08}, {0x01, 0x2F, 0x1F, 0x08},
	 {0x01, 0x3B, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x0F, 0x20, 0x08},
	 {0x01, 0x17, 0x20, 0x08}, {0x01, 0x1F, 0x20, 0x08},
	 {0x01, 0x2F, 0x20, 0x08}, {0x01, 0x3F, 0x20, 0x08} },

	 /* D = 2 */
	{{0x01, 0x05, 0x1F, 0x08}, {0x02, 0x0B, 0x1F, 0x08},
	 {0x01, 0x08, 0x1F, 0x08}, {0x01, 0x0B, 0x1F, 0x08},
	 {0x01, 0x11, 0x1F, 0x08}, {0x01, 0x17, 0x1F, 0x08},
	 {0x01, 0x1D, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x07, 0x20, 0x08},
	 {0x01, 0x0B, 0x20, 0x08}, {0x01, 0x0F, 0x20, 0x08},
	 {0x01, 0x17, 0x20, 0x08}, {0x01, 0x1F, 0x20, 0x08} },

	 /* D = 3 */
	{{0x01, 0x02, 0x1F, 0x08}, {0x02, 0x05, 0x1F, 0x08},
	 {0x02, 0x08, 0x1F, 0x08}, {0x01, 0x05, 0x1F, 0x08},
	 {0x01, 0x08, 0x1F, 0x08}, {0x01, 0x0B, 0x1F, 0x08},
	 {0x01, 0x0E, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x03, 0x20, 0x08},
	 {0x01, 0x05, 0x20, 0x08}, {0x01, 0x07, 0x20, 0x08},
	 {0x01, 0x0B, 0x20, 0x08}, {0x01, 0x0F, 0x20, 0x08} },

	 /* D = 4 */
	{{0x02, 0x02, 0x1F, 0x08}, {0x02, 0x02, 0x1F, 0x08},
	 {0x04, 0x08, 0x1F, 0x04}, {0x01, 0x02, 0x1F, 0x08},
	 {0x02, 0x08, 0x1F, 0x08}, {0x01, 0x05, 0x1F, 0x08},
	 {0x02, 0x0E, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x01, 0x20, 0x08},
	 {0x01, 0x02, 0x20, 0x08}, {0x01, 0x03, 0x20, 0x08},
	 {0x01, 0x05, 0x20, 0x08}, {0x01, 0x07, 0x20, 0x08} },

	 /* D = 5 */
	{{0x04, 0x02, 0x1F, 0x04}, {0x04, 0x02, 0x1F, 0x04},
	 {0x08, 0x08, 0x1F, 0x02}, {0x02, 0x02, 0x1F, 0x08},
	 {0x04, 0x08, 0x1F, 0x04}, {0x01, 0x02, 0x1F, 0x08},
	 {0x04, 0x0E, 0x1F, 0x04}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x01, 0x20, 0x08},
	 {0x02, 0x02, 0x20, 0x08}, {0x01, 0x01, 0x20, 0x08},
	 {0x01, 0x02, 0x20, 0x08}, {0x01, 0x03, 0x20, 0x08} },

	 /* D = 6 */
	{{0x08, 0x02, 0x1F, 0x02}, {0x08, 0x02, 0x1F, 0x02},
	 {0x10, 0x08, 0x1F, 0x01}, {0x04, 0x02, 0x1F, 0x04},
	 {0x08, 0x08, 0x1F, 0x02}, {0x02, 0x02, 0x1F, 0x08},
	 {0x08, 0x0E, 0x1F, 0x02}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x04, 0x01, 0x20, 0x04},
	 {0x04, 0x02, 0x20, 0x04}, {0x02, 0x01, 0x20, 0x08},
	 {0x02, 0x02, 0x20, 0x08}, {0x01, 0x01, 0x20, 0x08} },

	 /* D = 7 */
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },

	 /* D = 8 */
	{{0x02, 0x01, 0x1F, 0x08}, {0x02, 0x01, 0x1F, 0x08},
	 {0x02, 0x02, 0x1F, 0x08}, {0x02, 0x03, 0x1F, 0x08},
	 {0x02, 0x05, 0x1F, 0x08}, {0x02, 0x07, 0x1F, 0x08},
	 {0x02, 0x09, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x03, 0x03, 0x20, 0x08},
	 {0x02, 0x03, 0x20, 0x08}, {0x03, 0x07, 0x20, 0x08},
	 {0x02, 0x07, 0x20, 0x08}, {0x03, 0x0F, 0x20, 0x08} },

	 /* D = 9 */
	{{0x05, 0x02, 0x1F, 0x04}, {0x05, 0x02, 0x1F, 0x04},
	 {0x0A, 0x08, 0x1F, 0x02}, {0x05, 0x05, 0x1F, 0x04},
	 {0x05, 0x08, 0x1F, 0x04}, {0x05, 0x0B, 0x1F, 0x04},
	 {0x02, 0x05, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x05, 0x03, 0x20, 0x04},
	 {0x05, 0x05, 0x20, 0x04}, {0x05, 0x07, 0x20, 0x04},
	 {0x05, 0x0B, 0x20, 0x04}, {0x05, 0x0F, 0x20, 0x04} }

#elif defined(BSCD_DSS_ICAM)

	/* D = 0 */
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },

	/* D = 1 */
	{{0x01, 0x0B, 0x1F, 0x08}, {0x01, 0x0B, 0x1F, 0x08},
	 {0x01, 0x11, 0x1F, 0x08}, {0x01, 0x17, 0x1F, 0x04},
	 {0x01, 0x23, 0x1F, 0x08}, {0x01, 0x2F, 0x1F, 0x02},
	 {0x01, 0x3B, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x0F, 0x20, 0x08},
	 {0x01, 0x17, 0x20, 0x08}, {0x01, 0x1F, 0x20, 0x08},
	 {0x01, 0x2F, 0x20, 0x08}, {0x01, 0x3F, 0x20, 0x08} },

	 /* D = 2 */
	{{0x01, 0x05, 0x1F, 0x08}, {0x01, 0x05, 0x1F, 0x08},
	 {0x01, 0x08, 0x1F, 0x08}, {0x01, 0x0B, 0x1F, 0x04},
	 {0x01, 0x11, 0x1F, 0x08}, {0x01, 0x17, 0x1F, 0x02},
	 {0x01, 0x1D, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x07, 0x20, 0x08},
	 {0x01, 0x0B, 0x20, 0x08}, {0x01, 0x0F, 0x20, 0x08},
	 {0x01, 0x17, 0x20, 0x08}, {0x01, 0x1F, 0x20, 0x08} },

	 /* D = 3 */
	{{0x01, 0x02, 0x1F, 0x08}, {0x01, 0x02, 0x1F, 0x08},
	 {0x02, 0x08, 0x1F, 0x08}, {0x01, 0x05, 0x1F, 0x04},
	 {0x01, 0x08, 0x1F, 0x08}, {0x01, 0x0B, 0x1F, 0x02},
	 {0x01, 0x0E, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x03, 0x20, 0x08},
	 {0x01, 0x05, 0x20, 0x08}, {0x01, 0x07, 0x20, 0x08},
	 {0x01, 0x0B, 0x20, 0x08}, {0x01, 0x0F, 0x20, 0x08} },

	 /* D = 4 */
	{{0x02, 0x02, 0x1F, 0x04}, {0x02, 0x02, 0x1F, 0x04},
	 {0x04, 0x08, 0x1F, 0x04}, {0x01, 0x02, 0x1F, 0x04},
	 {0x02, 0x08, 0x1F, 0x08}, {0x01, 0x05, 0x1F, 0x02},
	 {0x02, 0x0E, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x01, 0x20, 0x08},
	 {0x01, 0x02, 0x20, 0x08}, {0x01, 0x03, 0x20, 0x08},
	 {0x01, 0x05, 0x20, 0x08}, {0x01, 0x07, 0x20, 0x08} },

	 /* D = 5 */
	{{0x04, 0x02, 0x1F, 0x02}, {0x04, 0x02, 0x1F, 0x02},
	 {0x08, 0x08, 0x1F, 0x02}, {0x02, 0x02, 0x1F, 0x02},
	 {0x04, 0x08, 0x1F, 0x04}, {0x01, 0x02, 0x1F, 0x02},
	 {0x04, 0x0E, 0x1F, 0x04}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x01, 0x20, 0x08},
	 {0x02, 0x02, 0x20, 0x08}, {0x01, 0x01, 0x20, 0x08},
	 {0x01, 0x02, 0x20, 0x08}, {0x01, 0x03, 0x20, 0x08} },

	 /* D = 6 */
	{{0x08, 0x02, 0x1F, 0x02}, {0x08, 0x02, 0x1F, 0x02},
	 {0x10, 0x08, 0x1F, 0x01}, {0x04, 0x02, 0x1F, 0x04},
	 {0x08, 0x08, 0x1F, 0x02}, {0x02, 0x02, 0x1F, 0x08},
	 {0x08, 0x0E, 0x1F, 0x02}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x04, 0x01, 0x20, 0x04},
	 {0x04, 0x02, 0x20, 0x04}, {0x02, 0x01, 0x20, 0x08},
	 {0x02, 0x02, 0x20, 0x08}, {0x01, 0x01, 0x20, 0x08} },

	/* D = 7 */
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },

	/* D = 8 */
	{{0x02, 0x01, 0x1F, 0x08}, {0x02, 0x01, 0x1F, 0x08},
	 {0x02, 0x02, 0x1F, 0x08}, {0x02, 0x03, 0x1F, 0x08},
	 {0x02, 0x05, 0x1F, 0x08}, {0x02, 0x07, 0x1F, 0x08},
	 {0x02, 0x09, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x03, 0x03, 0x20, 0x08},
	 {0x02, 0x03, 0x20, 0x08}, {0x03, 0x07, 0x20, 0x08},
	 {0x02, 0x07, 0x20, 0x08}, {0x03, 0x0F, 0x20, 0x08} },

	 /* D = 9 */
	{{0x05, 0x02, 0x1F, 0x04}, {0x05, 0x02, 0x1F, 0x04},
	 {0x0A, 0x08, 0x1F, 0x02}, {0x05, 0x05, 0x1F, 0x04},
	 {0x05, 0x08, 0x1F, 0x04}, {0x05, 0x0B, 0x1F, 0x04},
	 {0x02, 0x05, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x05, 0x03, 0x20, 0x04},
	 {0x05, 0x05, 0x20, 0x04}, {0x05, 0x07, 0x20, 0x04},
	 {0x05, 0x0B, 0x20, 0x04}, {0x05, 0x0F, 0x20, 0x04} }

#elif (BSCD_INTERNAL_CLOCK_FREQ_40MHz == BSCD_INTERNAL_CLOCK_FREQ)

	 /* DI = 0, Reserved for Future Use */
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },

	 /* DI = 1, D=1 */
	{{0x0a, 0x77, 0x1F, 0x01}, {0x0a, 0x77, 0x1F, 0x01},
	 {0x01, 0x11, 0x1F, 0x08}, {0x01, 0x17, 0x1F, 0x08},
	 {0x01, 0x23, 0x1F, 0x08}, {0x01, 0x2F, 0x1F, 0x08},
	 {0x01, 0x3B, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x1F, 0x20, 0x08},
	 {0x01, 0x17, 0x20, 0x08}, {0x01, 0x1F, 0x20, 0x08},
	 {0x01, 0x2F, 0x20, 0x08}, {0x01, 0x3F, 0x20, 0x08} },/*F=1536, 2048*/

	 /* DI = 2, D=2 */
	{{0x02, 0x0B, 0x1F, 0x08}, {0x02, 0x0B, 0x1F, 0x08},
	 {0x01, 0x08, 0x1F, 0x08}, {0x01, 0x0B, 0x1F, 0x08},
	 {0x01, 0x11, 0x1F, 0x08}, {0x01, 0x17, 0x1F, 0x08},
	 {0x01, 0x1D, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x0F, 0x20, 0x08},
	 {0x01, 0x0B, 0x20, 0x08}, {0x01, 0x0F, 0x20, 0x08},
	 {0x01, 0x17, 0x20, 0x08}, {0x01, 0x1F, 0x20, 0x08} },/*F=1536, 2048*/

	 /* DI = 3, D=4 */
	{{0x02, 0x05, 0x1F, 0x08}, {0x02, 0x05, 0x1F, 0x08},
	 {0x02, 0x08, 0x1F, 0x05}, {0x01, 0x05, 0x1F, 0x08},
	 {0x01, 0x08, 0x1F, 0x08}, {0x01, 0x0B, 0x1F, 0x08},
	 {0x01, 0x0E, 0x1F, 0x08}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x07, 0x20, 0x08},
	 {0x01, 0x05, 0x20, 0x08}, {0x01, 0x07, 0x20, 0x08},
	 {0x01, 0x0B, 0x20, 0x08}, {0x01, 0x0F, 0x20, 0x08} },/*F=1536, 2048*/

	 /* DI = 4, D=8 */
	{{0x02, 0x02, 0x1F, 0x06}, {0x02, 0x02, 0x1F, 0x05},
	 {0x04, 0x08, 0x1F, 0x02}, {0x01, 0x02, 0x1F, 0x08},
	 {0x02, 0x08, 0x1F, 0x05}, {0x01, 0x05, 0x1F, 0x08},
	 {0x02, 0x0E, 0x1F, 0x05}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x03, 0x20, 0x08},
	 {0x01, 0x02, 0x20, 0x08}, {0x01, 0x03, 0x20, 0x08},
	 {0x01, 0x05, 0x20, 0x08}, {0x01, 0x07, 0x20, 0x08} },/*F=1536, 2048*/

	 /* DI = 5, D=16 */
	{{0x04, 0x02, 0x1F, 0x04}, {0x04, 0x02, 0x1F, 0x04},
	 {0x08, 0x08, 0x1F, 0x01}, {0x02, 0x02, 0x1F, 0x04},
	 {0x04, 0x08, 0x1F, 0x02}, {0x01, 0x02, 0x1F, 0x08},
	 {0x04, 0x0E, 0x1F, 0x02}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x01, 0x20, 0x05},
	 {0x02, 0x02, 0x20, 0x04}, {0x01, 0x01, 0x20, 0x08},
	 {0x01, 0x02, 0x20, 0x08}, {0x01, 0x03, 0x20, 0x08} },/*F=1536, 2048*/

	 /* DI = 6, D=32 */
	{{0x08, 0x02, 0x1F, 0x02}, {0x08, 0x02, 0x1F, 0x02},
	 {0x10, 0x08, 0x1F, 0x01}, {0x04, 0x02, 0x1F, 0x02},
	 {0x08, 0x08, 0x1F, 0x01}, {0x02, 0x02, 0x1F, 0x03},
	 {0x08, 0x0E, 0x1F, 0x01}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x04, 0x01, 0x20, 0x03},
	 {0x04, 0x02, 0x20, 0x02}, {0x02, 0x01, 0x20, 0x03},
	 {0x02, 0x02, 0x20, 0x03}, {0x01, 0x01, 0x20, 0x06} },/*F=1536, 2048*/

	 /* DI = 7, RFU */
	{{0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0x08, 0x01, 0x20, 0x01},
	 {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0} },

	 /* DI = 8, D=12 */
	{{0x02, 0x01, 0x1F, 0x06}, {0x02, 0x01, 0x1F, 0x06},
	 {0x02, 0x02, 0x1F, 0x04}, {0x02, 0x03, 0x1F, 0x04},
	 {0x02, 0x05, 0x1F, 0x04}, {0x02, 0x07, 0x1F, 0x04},
	 {0x02, 0x09, 0x1F, 0x04}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x03, 0x03, 0x20, 0x03},
	 {0x02, 0x03, 0x20, 0x04}, {0x03, 0x07, 0x20, 0x03},
	 {0x02, 0x07, 0x20, 0x04}, {0x03, 0x0F, 0x20, 0x03} },/*F=1536, 2048*/

	 /* DI = 9, D=20 */
	{{0x05, 0x02, 0x1F, 0x03}, {0x05, 0x02, 0x1F, 0x03},
	 {0x0A, 0x08, 0x1F, 0x01}, {0x05, 0x05, 0x1F, 0x02},
	 {0x05, 0x08, 0x1F, 0x02}, {0x05, 0x0B, 0x1F, 0x02},
	 {0x02, 0x05, 0x1F, 0x04}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x05, 0x03, 0x20, 0x02},
	 {0x05, 0x05, 0x20, 0x02}, {0x05, 0x07, 0x20, 0x02},
	 {0x05, 0x0B, 0x20, 0x02}, {0x05, 0x0F, 0x20, 0x02} } /*F=1536, 2048*/

#elif (BSCD_INTERNAL_CLOCK_FREQ_27MHz == BSCD_INTERNAL_CLOCK_FREQ)

	/* DI = 0, Reserved for Future Use */
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },

	 /* DI = 1, D=1 */
	{{0x01, 0x0B, 0x1F, 0x06}, {0x01, 0x0B, 0x1F, 0x06},
	 {0x01, 0x11, 0x1F, 0x06}, {0x01, 0x17, 0x1F, 0x06},
	 {0x01, 0x23, 0x1F, 0x06}, {0x01, 0x2F, 0x1F, 0x06},
	 {0x01, 0x3B, 0x1F, 0x06}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x0F, 0x20, 0x06},
	 {0x01, 0x17, 0x20, 0x06}, {0x01, 0x1F, 0x20, 0x06},
	 {0x01, 0x2F, 0x20, 0x06}, {0x01, 0x3F, 0x20, 0x06} },

	 /* DI = 2, D=2 */
	{{0x01, 0x05, 0x1F, 0x06}, {0x01, 0x05, 0x1F, 0x06},
	 {0x01, 0x08, 0x1F, 0x06}, {0x01, 0x0B, 0x1F, 0x06},
	 {0x01, 0x11, 0x1F, 0x06}, {0x01, 0x17, 0x1F, 0x06},
	 {0x01, 0x1D, 0x1F, 0x06}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x07, 0x20, 0x06},
	 {0x01, 0x0B, 0x20, 0x06}, {0x01, 0x0F, 0x20, 0x06},
	 {0x01, 0x17, 0x20, 0x06}, {0x01, 0x1F, 0x20, 0x06} },

	 /* DI = 3, D=4 */
	{{0x01, 0x02, 0x1F, 0x06}, {0x01, 0x02, 0x1F, 0x06},
	 {0x02, 0x08, 0x1F, 0x03}, {0x01, 0x05, 0x1F, 0x06},
	 {0x01, 0x08, 0x1F, 0x06}, {0x01, 0x0B, 0x1F, 0x06},
	 {0x01, 0x0E, 0x1F, 0x06}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x03, 0x20, 0x06},
	 {0x01, 0x05, 0x20, 0x06}, {0x01, 0x07, 0x20, 0x06},
	 {0x01, 0x0B, 0x20, 0x06}, {0x01, 0x0F, 0x20, 0x06} },

	 /* DI = 4, D=8 */
	{{0x02, 0x02, 0x1F, 0x03}, {0x02, 0x02, 0x1F, 0x03},
	 {0x04, 0x08, 0x1F, 0x02}, {0x01, 0x02, 0x1F, 0x06},
	 {0x02, 0x08, 0x1F, 0x03}, {0x01, 0x05, 0x1F, 0x06},
	 {0x02, 0x0E, 0x1F, 0x03}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x01, 0x01, 0x20, 0x06},
	 {0x01, 0x02, 0x20, 0x06}, {0x01, 0x03, 0x20, 0x06},
	 {0x01, 0x05, 0x20, 0x06}, {0x01, 0x07, 0x20, 0x06} },

	 /* DI = 5, D=16 */
	{{0x04, 0x02, 0x1F, 0x02}, {0x04, 0x02, 0x1F, 0x02},
	 {0x08, 0x08, 0x1F, 0x01}, {0x02, 0x02, 0x1F, 0x03},
	 {0x04, 0x08, 0x1F, 0x02}, {0x01, 0x02, 0x1F, 0x06},
	 {0x04, 0x0E, 0x1F, 0x02}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x02, 0x01, 0x20, 0x03},
	 {0x02, 0x02, 0x20, 0x03}, {0x01, 0x01, 0x20, 0x06},
	 {0x01, 0x02, 0x20, 0x06}, {0x01, 0x03, 0x20, 0x06} },

	 /* DI = 6, D=32 */
	{{0x08, 0x02, 0x1F, 0x01}, {0x08, 0x02, 0x1F, 0x01},
	 {0x10, 0x08, 0x1F, 0x01}, {0x04, 0x02, 0x1F, 0x01},
	 {0x08, 0x08, 0x1F, 0x01}, {0x02, 0x02, 0x1F, 0x03},
	 {0x08, 0x0E, 0x1F, 0x01}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x04, 0x01, 0x20, 0x02},
	 {0x04, 0x02, 0x20, 0x02}, {0x02, 0x01, 0x20, 0x03},
	 {0x02, 0x02, 0x20, 0x03}, {0x01, 0x01, 0x20, 0x06} },

	 /* DI = 7, RFU */
	{{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
	 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0} },

	 /* DI = 8, D=12 */
	{{0x02, 0x01, 0x1F, 0x03}, {0x02, 0x01, 0x1F, 0x03},
	 {0x02, 0x02, 0x1F, 0x03}, {0x02, 0x03, 0x1F, 0x03},
	 {0x02, 0x05, 0x1F, 0x03}, {0x02, 0x07, 0x1F, 0x03},
	 {0x02, 0x09, 0x1F, 0x03}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x03, 0x03, 0x20, 0x02},
	 {0x02, 0x03, 0x20, 0x03}, {0x03, 0x07, 0x20, 0x02},
	 {0x02, 0x07, 0x20, 0x03}, {0x03, 0x0F, 0x20, 0x02} },

	 /* DI = 9, D=20 */
	{{0x05, 0x02, 0x1F, 0x02}, {0x05, 0x02, 0x1F, 0x02},
	 {0x0A, 0x08, 0x1F, 0x01}, {0x05, 0x05, 0x1F, 0x02},
	 {0x05, 0x08, 0x1F, 0x02}, {0x05, 0x0B, 0x1F, 0x02},
	 {0x02, 0x05, 0x1F, 0x03}, {0x00, 0x00, 0x00, 0x00},
	 {0x00, 0x00, 0x00, 0x00}, {0x05, 0x03, 0x20, 0x02},
	 {0x05, 0x05, 0x20, 0x02}, {0x05, 0x07, 0x20, 0x02},
	 {0x05, 0x0B, 0x20, 0x02}, {0x05, 0x0F, 0x20, 0x02} }
#else
	error "SCI input clock unknown!"
#endif
};

static const int p_ffactor_arr[14] = {372, 372, 558, 744, 1116, 1488, 1860,
				      -1,  -1, 512, 768, 1024, 1536, 2048};

static const signed char p_dfactor_arr[10] = {
		-1, 1, 2, 4, 8, 16, 32, 64, 12, 20};

/* The value of SCA_SC_CLK_CMD_1 of sc_clk_div */
static const unsigned char p_sc_clk_div[33] = {
	0x00, 0x00, 0x10, 0x20,  0x30, 0x40, 0x00, 0x00,
	0x50, 0x00, 0x60, 0x00,  0x00, 0x00, 0x00, 0x00,
	0x70, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00,
	0x00};

unsigned char p_get_clk_div(unsigned char in_dfactor, unsigned char in_ffactor)
{
	return p_df_arr[in_dfactor][in_ffactor].sc_clk_div;
}

unsigned char p_get_etuclk_div(unsigned char in_dfactor,
			       unsigned char in_ffactor)
{
	return p_df_arr[in_dfactor][in_ffactor].sc_etuclk_div;
}

unsigned char p_get_iso_baud_rate_adjustor(unsigned char in_dfactor)
{
	return  p_dfactor_arr[in_dfactor];
}

unsigned int p_get_iso_clkrate_conversion_factor(unsigned char in_ffactor)
{
	return  p_ffactor_arr[in_ffactor];
}

unsigned char p_map_scclk_div_to_mask_value(unsigned char in_clk_div)
{
	/*
	 * 0000 - SC_CLK divider = 1.
	 * 0001 - SC_CLK divider = 2.
	 * 0010 - SC_CLK divider = 3.
	 * 0011 - SC_CLK divider = 4.
	 * 0100 - SC_CLK divider = 5.
	 * 0101 - SC_CLK divider = 8.
	 * 0110 - SC_CLK divider = 10.
	 * 0111 - SC_CLK divider = 16.
	 * 1000 - SC_CLK divider = 32.
	 */
	if (in_clk_div == 1 || in_clk_div == 2  || in_clk_div == 3  ||
	    in_clk_div == 4  || in_clk_div == 5 || in_clk_div == 8  ||
	    in_clk_div == 10 || in_clk_div == 16 || in_clk_div == 32)
		return p_sc_clk_div[in_clk_div];
	pr_err("Invalid SC_CLK divider = %d\n", in_clk_div);
	return 0;
}

unsigned char p_get_prescale(unsigned char in_dfactor, unsigned char in_ffactor)
{
	return p_df_arr[in_dfactor][in_ffactor].sc_prescale;
}

unsigned char p_get_baud_div(unsigned char in_dfactor, unsigned char in_ffactor)
{
	return p_df_arr[in_dfactor][in_ffactor].sc_baud_div;
}

int p_adjust_wwt(struct bscd_chnl_settings *chnl_settings,
		 unsigned char in_work_wait_time_integer)
{
	int err = BERR_SUCCESS;
	unsigned char  baud_rate_adjustor;

	pr_devel("dfactor = %d\n", chnl_settings->dfactor);
	pr_devel("baudrate = %lu\n", chnl_settings->curr_baud_rate);
	if (chnl_settings->curr_baud_rate == 0) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("etu in us= %lu\n", 1000000/chnl_settings->curr_baud_rate);
	baud_rate_adjustor = p_get_iso_baud_rate_adjustor(
					chnl_settings->dfactor);

	/* EMV2000 */
	if (chnl_settings->sc_std == std_emv2000)
		chnl_settings->work_wait_time.value =
			BSCD_ISO_WORK_WAIT_TIME_DEFAULT_FACTOR *
			baud_rate_adjustor *
			in_work_wait_time_integer + baud_rate_adjustor *
			BSCD_DEFAULT_EXTRA_WORK_WAITING_TIME_EMV2000 +
			BSCD_EMV2000_WORK_WAIT_TIME_DELTA;
	else
		chnl_settings->work_wait_time.value =
			BSCD_ISO_WORK_WAIT_TIME_DEFAULT_FACTOR *
			baud_rate_adjustor *
			in_work_wait_time_integer;

	chnl_settings->work_wait_time.unit = unit_etu;
bscd_p_done_label:
	return err;
}

/* This modify registers */
int p_fd_adjust(struct p_chnl_hdl *hdl, unsigned char in_ffactor,
		unsigned char in_dfactor)
{
	int err = BERR_SUCCESS;
	unsigned int clk_cmd;

	/* Set BCM to adjust the clock and bit rate */
	hdl->cur_chs.prescale =
				p_get_prescale(in_dfactor, in_ffactor) *
				hdl->cur_chs.external_clk_div +
			(hdl->cur_chs.external_clk_div - 1);

	writeb(hdl->cur_chs.prescale, hdl->baddr + BSCD_P_PRESCALE);
	hdl->cur_chs.baud_div = p_get_baud_div(in_dfactor, in_ffactor);
	hdl->cur_chs.sc_clk_div = p_get_clk_div(in_dfactor, in_ffactor);

	if (hdl->cur_chs.baud_div == BSCD_DEFAULT_BAUD_DIV) {
		clk_cmd = CLKCMD_CLK_ENA_MASK |
			p_map_scclk_div_to_mask_value(hdl->cur_chs.sc_clk_div) |
			((hdl->cur_chs.etu_clk_div - 1) << 1);
	} else {
		clk_cmd = CLKCMD_CLK_ENA_MASK |
			p_map_scclk_div_to_mask_value(hdl->cur_chs.sc_clk_div) |
			((hdl->cur_chs.etu_clk_div - 1) << 1)  |
			CLKCMD_BAUD_DIV_MASK;
	}
	writeb(clk_cmd, hdl->baddr + BSCD_P_CLK_CMD);
	return err;
}

/* This does NOT modify registers */
int p_fd_adjust_without_reg_update(struct bscd_chnl_settings *chnl_settings,
				   unsigned char in_ffactor,
				   unsigned char in_dfactor)
{
	int err = BERR_SUCCESS;

	/* Set BCM to adjust the clock and bit rate  */
	if ((in_dfactor >= 1) && (in_dfactor <= 9)) {
		if (p_get_iso_baud_rate_adjustor(in_dfactor) ==
						((unsigned char) -1)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		chnl_settings->dfactor = in_dfactor;
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	if ((in_ffactor >= 1) && (in_ffactor <= 13)) {
		if (p_get_iso_clkrate_conversion_factor(in_ffactor) ==
						((unsigned char) -1)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		chnl_settings->ffactor   = in_ffactor;
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	chnl_settings->etu_clk_div = p_get_etuclk_div(in_dfactor, in_ffactor);
	chnl_settings->sc_clk_div  = p_get_clk_div(in_dfactor, in_ffactor);
	chnl_settings->prescale = p_get_prescale(in_dfactor, in_ffactor) *
					chnl_settings->external_clk_div +
					(chnl_settings->external_clk_div - 1);

	chnl_settings->baud_div = p_get_baud_div(in_dfactor, in_ffactor);

	err = p_adjust_wwt(chnl_settings,
			BSCD_ISO_DEFAULT_WORK_WAIT_TIME_INTEGER);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

bscd_p_done_label:
	return err;
}

/* Default ISR Callback Functions */
void chnl_p_card_insert_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	pr_devel("%s\n", "default  chnl_p_card_insert_cbisr");
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.card_wait);
}

void  chnl_p_card_remove_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	pr_devel("%s\n", "default  chnl_p_card_remove_cbisr");
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true) {
		set_event(hdl->chnl_wait_event.card_wait);
		set_event(hdl->chnl_wait_event.rcv_wait);
		set_event(hdl->chnl_wait_event.tdone_wait);
		set_event(hdl->chnl_wait_event.timer_wait);
	}
}

void chnl_p_rcv_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.rcv_wait);
}

void chnl_p_atr_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.atr_start);
}

void chnl_p_wait_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true) {
		set_event(hdl->chnl_wait_event.rcv_wait);
		set_event(hdl->chnl_wait_event.tdone_wait);
	}

}

void chnl_p_retry_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true) {
		set_event(hdl->chnl_wait_event.tdone_wait);
		set_event(hdl->chnl_wait_event.rcv_wait);
	}
}

void chnl_p_timer_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true) {
		set_event(hdl->chnl_wait_event.atr_start);
		set_event(hdl->chnl_wait_event.rcv_wait);
		set_event(hdl->chnl_wait_event.timer_wait);
	}
}

void chnl_p_rparity_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.rcv_wait);
}

void chnl_p_tparity_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.tdone_wait);
}

void chnl_p_cwt_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.rcv_wait);
}

void chnl_p_bgtcb_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true) {
		set_event(hdl->chnl_wait_event.tdone_wait);
		set_event(hdl->chnl_wait_event.rcv_wait);
	}
}

void chnl_p_rlen_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.rcv_wait);
}

void chnl_p_rready_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.rcv_wait);
}

void chnl_p_tdone_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true)
		set_event(hdl->chnl_wait_event.tdone_wait);
}

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
void chnl_p_event1_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true) {
		set_event(hdl->chnl_wait_event.rcv_wait);
		set_event(hdl->chnl_wait_event.event1_wait);
	}
}
#endif

void chnl_p_event2_cbisr(struct p_chnl_hdl *hdl, void *inp_data)
{
	BSTD_UNUSED(inp_data);
	if (hdl->is_open == true) {
		set_event(hdl->chnl_wait_event.event2_wait);
		set_event(hdl->chnl_wait_event.rcv_wait);
	}
}

#ifdef BSCD_USE_POLLING
void bscd_chnl_p_read_status(void *inp_param1, /* Device channel handle */
			     int in_param2     /* reserved */)
{
	unsigned int intr_en1 = 0, intr_en2 = 0;
	struct p_chnl_hdl *hdl = inp_param1;

	/* Read Smartcard Interrupt Status & Mask Register */
	intr_en1 = readb(hdl->baddr + BSCD_P_INTR_EN_1);
	hdl->intr_status1 = intr_en1 &
				readb(hdl->baddr + BSCD_P_INTR_STAT_1);
	intr_en2 = readb(hdl->baddr + BSCD_P_INTR_EN_2);
	hdl->intr_status2  = intr_en2 &
				readb(hdl->baddr + BSCD_P_INTR_STAT_2);
}
#endif

int chnl_p_wait_for_card_insertion(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int status1;

	pr_devel("Ready to receive card insertion pres_intr interrupt\n");
	spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
	bscd_chnl_p_read_status(hdl, 0);
#endif
	status1 = hdl->status1 & STATUS1_CARD_PRES_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);

	do {

		if (status1 != STATUS1_CARD_PRES_MASK) {
			err =  wait_for_event(
#ifdef BSCD_USE_POLLING
					hdl,
#endif
					hdl->chnl_wait_event.card_wait,
					BKNI_INFINITE);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
		}
		spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
		bscd_chnl_p_read_status(hdl, 0);
#endif
		status1 = hdl->status1 & STATUS1_CARD_PRES_MASK;
		spin_unlock_irqrestore(&hdl->lock, flag);

	} while (status1 != STATUS1_CARD_PRES_MASK);

	pr_devel("Received card insertion pres_intr interrupt\n");
	if (status1 == STATUS1_CARD_PRES_MASK) {
		hdl->chnl_status.card_present = true;
		hdl->intr_status1 &= ~INTSTAT1_PRES_INTR_MASK;
		pr_devel("Smart Card Inserted\n");
	}
bscd_p_done_label:
	return err;
}

int chnl_p_wait_for_card_remove(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int status1;

	pr_devel("Ready to receive card removal pres_intr interrupt\n");

	spin_lock_irqsave(&hdl->lock, flag);
	status1 = hdl->status1 & STATUS1_CARD_PRES_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);

	do {
		if (status1 == STATUS1_CARD_PRES_MASK) {
			err = wait_for_event(
#ifdef BSCD_USE_POLLING
					hdl,
#endif
					hdl->chnl_wait_event.card_wait,
							BKNI_INFINITE);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
		}
		spin_lock_irqsave(&hdl->lock, flag);
		status1 = hdl->status1 & STATUS1_CARD_PRES_MASK;
		spin_unlock_irqrestore(&hdl->lock, flag);
	} while (status1 == STATUS1_CARD_PRES_MASK);

	pr_devel("Received card removal pres_intr interrupt\n");
	if (status1 != STATUS1_CARD_PRES_MASK) {
		hdl->chnl_status.card_present = false;
		hdl->intr_status1 &= ~INTSTAT1_PRES_INTR_MASK;
		pr_devel("Smart Card Removed\n");
	}

bscd_p_done_label:
	return err;
}

int chnl_p_wait_for_timer_event(struct p_chnl_hdl *hdl,
				unsigned int in_chk_card_removal)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int intr_status1;

	pr_devel("Ready to receive scard_timer_wait interrupt\n");
	pr_devel("TIMER_CMD:0x%x\n", readl(hdl->baddr + BSCD_P_TIMER_CMD));
	pr_devel("INTR_EN_1:0x%x\n", readl(hdl->baddr + BSCD_P_INTR_EN_1));

	spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
	bscd_chnl_p_read_status(hdl, 0);
#endif
	intr_status1 = hdl->intr_status1;
	spin_unlock_irqrestore(&hdl->lock, flag);

	pr_devel("intr_status1 = 0x%x\n", intr_status1);
	do {
		if (in_chk_card_removal && (hdl->is_card_removed == true) &&
			((intr_status1 & INTSTAT1_PRES_INTR_MASK) ==
						INTSTAT1_PRES_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1 &= ~INTSTAT1_PRES_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RST_CHNL_REQD;
			hdl->is_card_removed = false;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceTimerEvent:SC REMOVED error\n");
				err = BSCD_STATUS_FAILED;
				goto bscd_p_done_label;
		} else if ((intr_status1 & INTSTAT1_TIMER_INTR_MASK) !=
						INTSTAT1_TIMER_INTR_MASK) {
			pr_devel("No timer intr, wait: ");
			pr_devel("%d ms\n", hdl->cur_chs.timeout.value);
			err = wait_for_event(
#ifdef BSCD_USE_POLLING
						hdl,
#endif
						hdl->chnl_wait_event.timer_wait,
						hdl->cur_chs.timeout.value);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
		}
		spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
		bscd_chnl_p_read_status(hdl, 0);
#endif
		intr_status1 = hdl->intr_status1;
		spin_unlock_irqrestore(&hdl->lock, flag);
		pr_devel("Re-read SCA_SC_INTR_STAT_1=0x%x\n", intr_status1);
	} while ((intr_status1 & INTSTAT1_TIMER_INTR_MASK) !=
					INTSTAT1_TIMER_INTR_MASK);

	spin_lock_irqsave(&hdl->lock, flag);
	hdl->intr_status1  &= ~INTSTAT1_TIMER_INTR_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);

bscd_p_done_label:
	return err;
}

int chnl_p_wait_for_atr_start(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int intr_status1, intr_status2;

	pr_devel("Rdy to recv sc_atr_start intr,Slot:%d\n", hdl->chnl_number);

	spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
	chnl_p_intr_handler_isr(hdl, 0);
#endif
	intr_status1 = hdl->intr_status1;
	intr_status2 = hdl->intr_status2 & INTSTAT2_ATR_INTR_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);

	pr_devel("intr_status1 = 0x%x\n", intr_status1);
	pr_devel("intr_status2 = 0x%x\n", intr_status2);
	pr_devel("hdl->intr_status1 = 0x%x\n", hdl->intr_status1);
	do {
		if (((intr_status1 & INTSTAT1_TIMER_INTR_MASK) ==
					INTSTAT1_TIMER_INTR_MASK) &&
			(intr_status2 == INTSTAT2_ATR_INTR_MASK)) {
			goto BSCD_P_SUCCESS_LABEL;
		} else if ((intr_status1 & INTSTAT1_TIMER_INTR_MASK) ==
						INTSTAT1_TIMER_INTR_MASK) {
			hdl->intr_status1 &= ~INTSTAT1_TIMER_INTR_MASK;
			pr_err("ScardDeviceWaitForatr_start:SC_TIMER_INTR err");
			err = BSCD_STATUS_TIME_OUT;
			goto bscd_p_done_label;
		} else if ((hdl->is_card_removed == true) &&
			((intr_status1 & INTSTAT1_PRES_INTR_MASK) ==
						INTSTAT1_PRES_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			spin_unlock_irqrestore(&hdl->lock, flag);
			hdl->intr_status1 &= ~INTSTAT1_PRES_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RST_CHNL_REQD;
			hdl->is_card_removed = false;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForTDone: SC_CARD_REMOVED err");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if (intr_status2 != INTSTAT2_ATR_INTR_MASK) {
			err = wait_for_event(
#ifdef BSCD_USE_POLLING
					hdl,
#endif
				hdl->chnl_wait_event.atr_start,
				hdl->cur_chs.timeout.value);
			if (err != BERR_SUCCESS) {

				hdl->chnl_status.status1 |= BSCD_RX_TIMEOUT;
				err = BSCD_STATUS_TIME_OUT;
				goto bscd_p_done_label;
			}
		}

		spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
		chnl_p_intr_handler_isr(hdl, 0);
#endif
		intr_status1 = hdl->intr_status1;
		intr_status2 = hdl->intr_status2 & INTSTAT2_ATR_INTR_MASK;
		spin_unlock_irqrestore(&hdl->lock, flag);
	} while (intr_status2 != INTSTAT2_ATR_INTR_MASK);

BSCD_P_SUCCESS_LABEL:
	spin_lock_irqsave(&hdl->lock, flag);
	hdl->intr_status2  &= ~INTSTAT2_ATR_INTR_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);
	pr_devel("scard_atr_start interrupt received\n");

bscd_p_done_label:
	return err;
}

int chnl_p_wait_for_tdone(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int intr_status1, intr_status2;

	pr_devel("Rdy to recv scard_tDone intr,ucSlot:%d\n", hdl->chnl_number);

	spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
	chnl_p_intr_handler_isr(hdl, 0);
#endif
	intr_status1 = hdl->intr_status1;
	intr_status2 = hdl->intr_status2;
	spin_unlock_irqrestore(&hdl->lock, flag);
	do {
		if ((hdl->cur_chs.proto_type == async_proto_e1) &&
				((intr_status1 & INTSTAT1_BGT_INTR_MASK) ==
					INTSTAT1_BGT_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1  &= ~INTSTAT1_BGT_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForTDone: SC_BGT_INTR error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((hdl->cur_chs.proto_type == async_proto_e0) &&
				((intr_status1 & INTSTAT1_RETRY_INTR_MASK) ==
				INTSTAT1_RETRY_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1  &= ~INTSTAT1_RETRY_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForTDone: RETRY_INTR error\n");
			hdl->chnl_status.status1 |= BSCD_TX_PARITY;
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((intr_status2 & INTSTAT2_WAIT_INTR_MASK) ==
						INTSTAT2_WAIT_INTR_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_WAIT_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			hdl->chnl_status.status1 |= BSCD_TX_TIMEOUT;
			pr_err("ScardDeviceWaitForTDone: SC_WAIT_INTR error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((hdl->is_card_removed == true) &&
			((intr_status1 & INTSTAT1_PRES_INTR_MASK) ==
						INTSTAT1_PRES_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1 &= ~INTSTAT1_PRES_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RST_CHNL_REQD;
			hdl->is_card_removed = false;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForTDone:SC_CARD_REMOVED err\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((intr_status1 & INTSTAT1_TDONE_INTR_MASK) !=
						INTSTAT1_TDONE_INTR_MASK) {
			err = BERR_TRACE(wait_for_event(
#ifdef BSCD_USE_POLLING
							hdl,
#endif
						hdl->chnl_wait_event.tdone_wait,
						hdl->cur_chs.timeout.value));
			if (err != BERR_SUCCESS) {
				spin_lock_irqsave(&hdl->lock, flag);
				hdl->chnl_status.status1 |= BSCD_TX_TIMEOUT;
				spin_unlock_irqrestore(&hdl->lock, flag);
				err = BSCD_STATUS_TIME_OUT;
				goto bscd_p_done_label;
			}
		}
		spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
		chnl_p_intr_handler_isr(hdl, 0);
#endif
		intr_status1 = hdl->intr_status1;
		intr_status2 = hdl->intr_status2;
		spin_unlock_irqrestore(&hdl->lock, flag);
	} while ((intr_status1 & INTSTAT1_TDONE_INTR_MASK) !=
					INTSTAT1_TDONE_INTR_MASK);

	spin_lock_irqsave(&hdl->lock, flag);
	hdl->intr_status1  &= ~INTSTAT1_TDONE_INTR_MASK;
	hdl->status1 &= ~STATUS1_TDONE_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);
	pr_devel("tdone_intr interrupt received\n");

bscd_p_done_label:
	return err;
}

int chnl_p_wait_for_rcv(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int intr_status1, intr_status2, status2;


	pr_devel("Ready to receive rcv interrupt\n");
	spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
	chnl_p_intr_handler_isr(hdl, 0);
#endif
	intr_status1 = hdl->intr_status1;
	intr_status2 = hdl->intr_status2;
	status2 =  hdl->status2;
	spin_unlock_irqrestore(&hdl->lock, flag);

	do {
		if ((intr_status1 & INTSTAT1_TIMER_INTR_MASK) ==
					INTSTAT1_TIMER_INTR_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->chnl_status.status1  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			/*
			 * This could be a good error if the caller specify a
			 * length larger than that of  the actual one.
			 */
			pr_devel("ScardDeviceWaitForRcv:SC_TIMER_INTR error\n");
			err = BSCD_STATUS_TIME_OUT;
			goto bscd_p_done_label;
		} else if ((hdl->cur_chs.proto_type == async_proto_e1) &&
				((intr_status1 & INTSTAT1_BGT_INTR_MASK) ==
						INTSTAT1_BGT_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1  &= ~INTSTAT1_BGT_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv: SC_BGT_INTR error\n");
			err = BSCD_STATUS_READ_FAILED;
			goto bscd_p_done_label;
		} else if ((intr_status2 & INTSTAT2_EV2_INTR_MASK) ==
						INTSTAT2_EV2_INTR_MASK)  {

			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_EV2_INTR_MASK;
			hdl->chnl_status.status2  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv: SC_EVENT2_INTR error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((hdl->cur_chs.proto_type == async_proto_e0) &&
				(hdl->cur_chs.sc_std != std_irdeto) &&
				((intr_status1 & INTSTAT1_RETRY_INTR_MASK) ==
						INTSTAT1_RETRY_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1 &= ~INTSTAT1_RETRY_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv: SC_RETRY_INTR error\n");
			hdl->chnl_status.status1 |= BSCD_RX_PARITY;
			err = BSCD_STATUS_PARITY_EDC_ERR;
			goto bscd_p_done_label;
		} else if ((hdl->cur_chs.proto_type == async_proto_e1) &&
			((intr_status2 & INTSTAT2_RLEN_INTR_MASK) ==
						INTSTAT2_RLEN_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_RLEN_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv: SC_RLEN_INTR error\n");
			err = BSCD_STATUS_READ_FAILED;
			goto bscd_p_done_label;
		} else if ((intr_status2 & INTSTAT2_WAIT_INTR_MASK) ==
						INTSTAT2_WAIT_INTR_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_WAIT_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv: SC_WAIT_INTR error\n");
			err = BSCD_STATUS_TIME_OUT;
			goto bscd_p_done_label;
		} else if ((intr_status2 & INTSTAT2_CWT_INTR_MASK) ==
						INTSTAT2_CWT_INTR_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_CWT_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv: SC_CWT_INTR error\n");
			err = BSCD_STATUS_TIME_OUT;
			goto bscd_p_done_label;
		} else if ((status2 & STATUS2_ROVERFLOW_MASK) ==
						STATUS2_ROVERFLOW_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->status2 &= ~STATUS2_ROVERFLOW_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv: SC_ROVERFLOW error\n");
			err = BSCD_STATUS_READ_FAILED;
			goto bscd_p_done_label;
		} else if ((hdl->is_card_removed == true) &&
			((intr_status1 & INTSTAT1_PRES_INTR_MASK) ==
						INTSTAT1_PRES_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1 &= ~INTSTAT1_PRES_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RST_CHNL_REQD;
			hdl->is_card_removed = false;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRcv:SC_CARD_REMOVED error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((intr_status2 & INTSTAT2_RCV_INTR_MASK) !=
						INTSTAT2_RCV_INTR_MASK) {
			err = BERR_TRACE(wait_for_event(
#ifdef BSCD_USE_POLLING
							hdl,
#endif
					hdl->chnl_wait_event.rcv_wait,
					hdl->cur_chs.timeout.value));
			if (err != BERR_SUCCESS) {
				spin_lock_irqsave(&hdl->lock, flag);
				hdl->chnl_status.status1 |= BSCD_RX_TIMEOUT;
				spin_unlock_irqrestore(&hdl->lock, flag);
				err = BSCD_STATUS_TIME_OUT;
				goto bscd_p_done_label;
			}
		}
		spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
		chnl_p_intr_handler_isr(hdl, 0);
#endif
		intr_status1 = hdl->intr_status1;
		intr_status2 = hdl->intr_status2;
		status2 =  hdl->status2;
		spin_unlock_irqrestore(&hdl->lock, flag);
	} while ((intr_status2 & INTSTAT2_RCV_INTR_MASK) !=
					INTSTAT2_RCV_INTR_MASK);

	spin_lock_irqsave(&hdl->lock, flag);
	hdl->intr_status2  &= ~INTSTAT2_RCV_INTR_MASK;
	hdl->status2 |= STATUS2_REMPTY_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);
	pr_devel("rcv interrupt received\n");

bscd_p_done_label:
	pr_devel("LeaveWaitForRcv err = 0x%x\n", err);
	return err;
}

int chnl_p_wait_for_rready(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int intr_status1, intr_status2, status2;
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	unsigned int val;
#endif
	pr_devel("Ready to receive rready interrupt\n");
	spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
	udelay(100);
	chnl_p_intr_handler_isr(hdl, 0);
#endif
	intr_status1 = hdl->intr_status1;
	intr_status2 = hdl->intr_status2;
	status2 =  hdl->status2;
	spin_unlock_irqrestore(&hdl->lock, flag);

	do {
		pr_devel("intr_status1:0x%x\n", intr_status1);
		pr_devel("intr_status2:0x%x\n", intr_status2);
		pr_devel("status2:0x%x\n", status2);
		if ((intr_status1 & INTSTAT1_BGT_INTR_MASK) ==
					INTSTAT1_BGT_INTR_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1  &= ~INTSTAT1_BGT_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady: SC_BGT_INTR error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
#ifdef BSCD_EMV2000_CWT_PLUS_4
		else if (((intr_status2 & INTSTAT2_CWT_INTR_MASK) ==
					INTSTAT2_CWT_INTR_MASK) &&
			((intr_status2 & INTSTAT2_RRDY_INTR_MASK) !=
					INTSTAT2_RRDY_INTR_MASK) &&
			(hdl->cur_chs.sc_std != std_emv2000)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_CWT_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady: SC_CWT_INTR error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
#elif defined(BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR)
		else if (((intr_status1 & INTSTAT1_EV1_INTR_MASK) ==
						INTSTAT1_EV1_INTR_MASK) &&
			((intr_status2 & INTSTAT2_RRDY_INTR_MASK) !=
						INTSTAT2_RRDY_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1 &= ~INTSTAT1_EV1_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady: SC_EVENT1_INTR err");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
#else
		else if (((intr_status2 & INTSTAT2_CWT_INTR_MASK) ==
						INTSTAT2_CWT_INTR_MASK) &&
			((intr_status2 & INTSTAT2_RRDY_INTR_MASK) !=
						INTSTAT2_RRDY_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_CWT_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady: SC_CWT_INTR error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
#endif
		else if ((hdl->cur_chs.proto_type == async_proto_e1) &&
			((intr_status2 & INTSTAT2_RLEN_INTR_MASK) ==
						INTSTAT2_RLEN_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_RLEN_INTR_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady: SC_RLEN_INTR err\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((intr_status2 & INTSTAT2_WAIT_INTR_MASK) ==
						INTSTAT2_WAIT_INTR_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status2 &= ~INTSTAT2_WAIT_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RX_TIMEOUT;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady: SC_WAIT_INTR err\n");
			err = BSCD_STATUS_TIME_OUT;
			goto bscd_p_done_label;
		} else if ((status2 & STATUS2_ROVERFLOW_MASK) ==
						STATUS2_ROVERFLOW_MASK) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->status2 &= ~STATUS2_ROVERFLOW_MASK;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady:SC_ROVERFLOW error\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((hdl->is_card_removed == true) &&
			((intr_status1 & INTSTAT1_PRES_INTR_MASK) ==
						INTSTAT1_PRES_INTR_MASK)) {
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->intr_status1 &= ~INTSTAT1_PRES_INTR_MASK;
			hdl->chnl_status.status1  |= BSCD_RST_CHNL_REQD;
			hdl->is_card_removed = false;
			spin_unlock_irqrestore(&hdl->lock, flag);
			pr_err("ScardDeviceWaitForRReady:SC_REMOVED err\n");
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		} else if ((intr_status2 & INTSTAT2_RRDY_INTR_MASK) !=
						INTSTAT2_RRDY_INTR_MASK) {
			err = BERR_TRACE(wait_for_event(
#ifdef BSCD_USE_POLLING
						hdl,
#endif
				hdl->chnl_wait_event.rcv_wait,
				hdl->cur_chs.timeout.value));
			if (err != BERR_SUCCESS) {
				spin_lock_irqsave(&hdl->lock, flag);
				hdl->chnl_status.status1 |= BSCD_RX_TIMEOUT;
				spin_unlock_irqrestore(&hdl->lock, flag);
				pr_err("wait_for_event timeout error:");
				pr_err("%d\n", hdl->cur_chs.timeout.value);
				err = BSCD_STATUS_TIME_OUT;
				goto bscd_p_done_label;
			}
		}

		spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_USE_POLLING
		chnl_p_intr_handler_isr(hdl, 0);
#endif
		intr_status1 = hdl->intr_status1;
		intr_status2 = hdl->intr_status2;
		status2 =  hdl->status2;
		spin_unlock_irqrestore(&hdl->lock, flag);
	} while ((status2 & STATUS2_RRDY_MASK) != STATUS2_RRDY_MASK);

	spin_lock_irqsave(&hdl->lock, flag);
	hdl->intr_status2  &= ~INTSTAT2_RRDY_INTR_MASK;
	hdl->status2  &= ~STATUS2_RRDY_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);
	pr_devel("rready interrupt received\n");

bscd_p_done_label:
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			/* Disable event1 */
	val = readb(hdl->baddr + BSCD_P_EVENT1_CMD_4);
	val &= ~(EV1CMD4_EV_ENA_MASK);
	writeb(val, hdl->baddr + BSCD_P_EVENT1_CMD_4);
#endif
	pr_devel("%s%x\n", "chnl_p_wait_for_rready err = 0x", err);
#ifdef BSCD_EMV2000_CWT_PLUS_4
	hdl->is_recv = false;
#endif
	return err;
}

int bscd_chnl_p_activating(struct p_chnl_hdl *hdl)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int if_cmd_val, val;
	struct bscd_timer timer = {timer_gpt, {gpt_timer_mode_immediate},
								true, true};
	struct bscd_timer_value time_val = {BSCD_MAX_RESET_IN_CLK_CYCLES,
								unit_clk};

	struct bscd_timer wwt_timer = {timer_wait,
					{gpt_timer_mode_immediate}, true, true};
	struct bscd_timer_value wwttime_val = {
				BSCD_MAX_ETU_PER_ATR_BYTE_EMV2000, unit_etu};


	unsigned char i;
	unsigned int timer_cnt_val1, timer_cnt_val2;
	unsigned int timer_cnt_val;
	unsigned int prev_timer_cnt_val = 0;

	WARN_ON(!hdl);

	if (hdl == NULL) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	/*
	 * We disable interrupt here, since if the card removal interrupt
	 * comes ISR will set the vcc bit
	 */
	spin_lock_irqsave(&hdl->lock, flag);

	/* Now check card presence */
	if (hdl->chnl_status.card_present == false) {
		spin_unlock_irqrestore(&hdl->lock, flag);
		/* this is the only error code which is similar */
		err = BERR_TRACE(BSCD_STATUS_DEACTIVATE);
		goto bscd_p_done_label;
	}

	/* Use Auto Deactivation instead of TDA8004 */
	if (hdl->cur_chs.auto_deactive_req == true) {
		if_cmd_val = readb(hdl->baddr + BSCD_P_IF_CMD_1);
		if_cmd_val |= IFCMD1_AUTO_VCC_MASK;
		writeb(if_cmd_val, hdl->baddr + BSCD_P_IF_CMD_1);
	}

	/* Set SC_RST low = RSTIN low */
	chnl_reset_signal(hdl, 0);

	/* Turn on Vcc (Set SC_VCC low = CMDVCC low) */
	chnl_reset_power_icc(hdl, pwricc_pwrup);

	spin_unlock_irqrestore(&hdl->lock, flag);

	pr_devel("Activating: SC_RST low\n");

	/* wait for 42,000 clk cycles. */
	for (i = 0; i < hdl->cur_chs.external_clk_div; i++) {
		timer.is_timer_intr_ena = true;
		timer.is_timer_ena = true;
		err = chnl_config_timer(hdl, &timer, &time_val);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_wait_for_timer_event(hdl, 1);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		/* Disable timer */
		timer.is_timer_intr_ena = false;
		timer.is_timer_ena = false;
		err = chnl_ena_dis_timer_isr(hdl, &timer);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	/*
	 * Set all required registers before we receive ATR.
	 *
	 * Set this to 0 temporarily during ATR session.  For EMV,
	 * we will set it back in BSCD_Channel_P_EMVATRReceiveAndDecode.
	 * For the rest, the application should set it back
	 */

	writeb(0, hdl->baddr + BSCD_P_UART_CMD_2);

	/* Enable 2 interrupts with callback */
	err = chnl_ena_int_cbisr(hdl, int_atr, chnl_p_atr_cbisr);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	err = chnl_ena_int_cbisr(hdl, int_rcv, chnl_p_rcv_cbisr);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	/*
	 * Enable WWT to ensure the max interval between 2 consecutive ATR
	 * chars of 10080 ETU
	 */
	pr_devel("Activating: Set WWT timer\n");
	if (hdl->cur_chs.sc_std == std_emv2000)
		wwttime_val.value = BSCD_MAX_ETU_PER_ATR_BYTE_EMV2000;
	else /* EMV 96 or the rest */
		wwttime_val.value = BSCD_MAX_ETU_PER_ATR_BYTE;
	wwt_timer.timer_mode.wait_time_mode = wtm_work;

	err = chnl_config_timer(hdl, &wwt_timer, &wwttime_val);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	/* Set BCM to get ATR packet.       */
	pr_devel("Activating: Set BCM to get ATR packet\n");
	val =  readb(hdl->baddr + BSCD_P_UART_CMD_1);
	val |= (UARTCMD1_GET_ATR_MASK | UARTCMD1_IO_ENA_MASK);
	writeb(val, hdl->baddr + BSCD_P_UART_CMD_1);

	/* Set RST */
	/* Use Auto Deactivation instead of TDA8004 */
	if (hdl->cur_chs.auto_deactive_req == true) {
		if_cmd_val = readb(hdl->baddr + BSCD_P_IF_CMD_1);
		if_cmd_val |= IFCMD1_AUTO_RST_MASK;
		writeb(if_cmd_val, hdl->baddr + BSCD_P_IF_CMD_1);
	}
	chnl_reset_signal(hdl, 1);

	/* wait for 40,000 clk cycles for EMV96 and 42000 for EMV2000 */
	for (i = 0; i < hdl->cur_chs.external_clk_div; i++) {
		/* Set Timer */
		timer.is_timer_intr_ena = true;
		timer.is_timer_ena = true;
		timer.timer_type = timer_gpt;
		timer.timer_mode.gpt_timer_mode = gpt_timer_mode_immediate;
		if (hdl->cur_chs.sc_std == std_emv2000)
			time_val.value =
				BSCD_EMV2000_MAX_ATR_START_IN_CLK_CYCLES +
				BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES;
		else
			time_val.value =
				BSCD_MAX_ATR_START_IN_CLK_CYCLES +
				BSCD_ATR_START_BIT_DELAY_IN_CLK_CYCLES;

		time_val.unit  = unit_clk;
		pr_devel("Activating: Set GP timer\n");
		err = chnl_config_timer(hdl, &timer, &time_val);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_p_wait_for_atr_start(hdl);
		if (err != BERR_SUCCESS) {
			pr_devel("Activating: WaitForatr_start failed\n");
			/* Disable timer */
			timer.is_timer_intr_ena = false;
			timer.is_timer_ena = false;
			/* remove the check err since that will set the err */
			chnl_ena_dis_timer_isr(hdl, &timer);
			if (err == BSCD_STATUS_TIME_OUT) {
				if (i == (hdl->cur_chs.external_clk_div - 1)) {
					/*
					 * if this is the last loop and we
					 * still timeout, major error
					 */
					/*
					 * Need to return deactivate for
					 * EMV2000 test 1719 xy=30
					 */
					err = BSCD_STATUS_DEACTIVATE;
					goto bscd_p_done_label;
				} else {
					/* If this is not the last loop,cont */
					prev_timer_cnt_val +=
					BSCD_MAX_ATR_START_IN_CLK_CYCLES;
					continue;
				}
			} else {
				/* If the error is not sctimeout, major error */
				err = BSCD_STATUS_FAILED;
				goto bscd_p_done_label;
			}
		}

		/* Disable timer */
		timer.is_timer_intr_ena = false;
		timer.is_timer_ena = false;
		err = chnl_ena_dis_timer_isr(hdl, &timer);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		pr_devel("Activating: Disable GP timer\n");

		/*
		 * Read timer counter, the ATR shall be received after 400
		 * clock cycles
		 */
		timer_cnt_val2 = readb(hdl->baddr + BSCD_P_TIMER_CNT_2);
		timer_cnt_val1 = readb(hdl->baddr + BSCD_P_TIMER_CNT_1);

		timer_cnt_val = (((timer_cnt_val2) << 8) | timer_cnt_val1) +
							prev_timer_cnt_val;

		if ((timer_cnt_val <
				(unsigned int)(
					BSCD_MIN_ATR_START_IN_CLK_CYCLES *
					hdl->cur_chs.external_clk_div)
						) ||
					(timer_cnt_val > time_val.value)
		) {
			pr_devel("PreATRREceive:");
			pr_devel("timer_cmd_val:%u", timer_cnt_val);
			pr_devel("timerValue.vlValue=%d\n", time_val.value);
			/*
			 * Need to return deactivate for EMV2000 test
			 * 1719 xy=30
			 */
			err = BSCD_STATUS_DEACTIVATE;
			goto bscd_p_done_label;
		}
		/*
		 * Enable WWT to ensure all ATR bytes are received within
		 * certain time
		 */
		if (hdl->cur_chs.sc_std == std_emv2000)
			time_val.value = MAX_EMV_ETU_FOR_ALL_ATR_BYTES_EMV2000;
		else /* EMV 96 or the rest */
			time_val.value = BSCD_MAX_EMV_ETU_FOR_ALL_ATR_BYTES;
		time_val.unit = unit_etu;
		timer.is_timer_intr_ena = true;
		timer.is_timer_ena = true;
		err = chnl_config_timer(hdl, &timer, &time_val);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		pr_devel("timer_cnt_val:%u,", timer_cnt_val);
		pr_devel("min clk cyc:%d\n", BSCD_MIN_ATR_START_IN_CLK_CYCLES);

		if (err == BERR_SUCCESS) {
			hdl->chnl_status.card_activate = true;
			break;
		}
	}

bscd_p_done_label:
	return err;
}

void p_hex_dump(char *inp_title, unsigned char *inp_buf, unsigned int  in_len)
{
	int i;

	pr_devel("\n%s (%u bytes):", inp_title, in_len);
	for (i = 0; i < in_len; i++) {
		if (!(i%20))
			pr_devel("\n");
		pr_devel("%02X  ", *(inp_buf+i));
	}
	pr_devel("\n");
	BSTD_UNUSED(inp_title);
	BSTD_UNUSED(inp_buf);
}

int chnl_p_t0_read_data(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
			unsigned long *rcv_bytes, unsigned long max_read_bytes)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int len = 0;
	unsigned int status2;
#ifndef BSCD_DSS_ICAM
	struct bscd_timer timer = {timer_wait, {wtm_work}, false, false};
	struct bscd_timer_value  time_val = {BSCD_DEFAULT_WORK_WAITING_TIME,
								unit_etu};
	unsigned int ena_here = 0;
#endif

	*rcv_bytes = 0;
	spin_lock_irqsave(&hdl->lock, flag);
	hdl->status2 = status2 = readb(hdl->baddr + BSCD_P_STATUS_2);
	spin_unlock_irqrestore(&hdl->lock, flag);

	pr_devel("%s: max_read_bytes = %lu\n", __func__, max_read_bytes);

	while (len < max_read_bytes) {
#ifndef BSCD_DSS_ICAM
		/*
		 * This is a backup time out for non EMV standard.
		 * Just in case, we do not read all the byte in one shot but
		 * WWT was disable in chnl_rcv
		 */
		if ((hdl->cur_chs.sc_std != std_emv1996) &&
			(hdl->cur_chs.sc_std != std_emv2000)) {
			/*
			 * Cannot enable GT if WWT is disabled, since WWT can be
			 * disabled when reach here, but GT is too small.
			 * we now reenable WWT here, instead of using GT
			 */
			if (!chnl_is_timer_ena(hdl, timer_wait)) {
				time_val.value =
					hdl->cur_chs.work_wait_time.value;
				err = chnl_config_timer(hdl, &timer, &time_val);
				if (err != BERR_SUCCESS)
					goto bscd_p_done_label;
				ena_here = 1;
			}
		}
#endif
		pr_devel("\nSmartCardReadCmd: After SmartCardSetGPTimer\n");

		spin_lock_irqsave(&hdl->lock, flag);
		status2 = hdl->status2;
		spin_unlock_irqrestore(&hdl->lock, flag);
		if ((status2 & STATUS2_REMPTY_MASK) == STATUS2_REMPTY_MASK) {
			err = chnl_p_wait_for_rcv(hdl);
			if (err != BERR_SUCCESS) {
				err = BERR_TRACE(err);
				pr_devel("%s%x\n", "chnl_p_t0_read_data err:0x",
					 err);
#ifndef BSCD_DSS_ICAM
				/* Disable timer */
				if ((hdl->cur_chs.sc_std != std_emv1996) &&
					(hdl->cur_chs.sc_std != std_emv2000)) {
					if (ena_here) {
						timer.is_timer_intr_ena = false;
						timer.is_timer_ena = false;
						chnl_ena_dis_timer_isr(hdl,
									&timer);
					}
				}
#endif
				if (err == BSCD_STATUS_PARITY_EDC_ERR)
					;/* No op in s/w, h/w will retry */
				else if (err == BSCD_STATUS_TIME_OUT)
					break;
				return BSCD_STATUS_READ_FAILED;
			}
		}
		pr_devel("\nSmartCardReadCmd: After ScardDeviceWaitForRcv\n");
#ifndef BSCD_DSS_ICAM
		/* Disable timer */
		if ((hdl->cur_chs.sc_std != std_emv1996) &&
			(hdl->cur_chs.sc_std != std_emv2000)) {
			if (ena_here) {
				timer.is_timer_intr_ena = false;
				timer.is_timer_ena = false;
				err = chnl_ena_dis_timer_isr(hdl, &timer);
				if (err != BERR_SUCCESS)
					goto bscd_p_done_label;
			}
		}
#endif
		while (len < max_read_bytes) {
			pr_devel("In  len < max_read_bytes\n");
			err = chnl_p_byte_read(hdl, &rcv_data[len]);
			if (err == BERR_SUCCESS) {
				hdl->chnl_status.status1 &= ~BSCD_RX_PARITY;
				if ((rcv_data[len] == 0x60) &&
					(hdl->cur_chs.null_filter == true)) {
					continue;
				} else {
					pr_devel("%2X ", rcv_data[len]);
					len++;
				}
			} else if (err == BSCD_STATUS_PARITY_EDC_ERR) {
				pr_devel("err == BSCD_STATUS_PARITY_EDC_ERR\n");
				continue;
			} else {
				break;
			}
		}
	}

#ifndef BSCD_DSS_ICAM
bscd_p_done_label:
#endif
	*rcv_bytes = len;
	pr_devel("%s%x\n", "Leave chnl_p_t0_read_data err = 0x", err);
	return err;
}

int chnl_p_byte_read(struct p_chnl_hdl *hdl, unsigned char *outp_data)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int status2;

	spin_lock_irqsave(&hdl->lock, flag);
	hdl->status2 =  status2 = readb(hdl->baddr + BSCD_P_STATUS_2);
	spin_unlock_irqrestore(&hdl->lock, flag);

	if ((status2 & STATUS2_REMPTY_MASK) != STATUS2_REMPTY_MASK) {
		*outp_data = (unsigned char) readb(hdl->baddr + BSCD_P_RECEIVE);
		spin_lock_irqsave(&hdl->lock, flag);
		hdl->status2 =  status2 = readb(hdl->baddr + BSCD_P_STATUS_2);
		spin_unlock_irqrestore(&hdl->lock, flag);
		if (((status2 & STATUS2_RPAR_ERR_MASK) == STATUS2_RPAR_ERR_MASK)
				&& (hdl->cur_chs.sc_std  != std_irdeto)) {
			pr_devel("Receive a parity error byte\n");
			spin_lock_irqsave(&hdl->lock, flag);
			hdl->chnl_status.status1 |= BSCD_RX_PARITY;
			spin_unlock_irqrestore(&hdl->lock, flag);
			return BSCD_STATUS_PARITY_EDC_ERR;
		}
	} else {
		return BSCD_STATUS_FAILED;
	}
	return err;
}

int chnl_p_t1_read_data(struct p_chnl_hdl *hdl, unsigned char *rcv_data,
			unsigned long *rcv_bytes, unsigned long  max_read_bytes)
{
	unsigned long flag;
	int err = BERR_SUCCESS;
	unsigned int val, len1, len2;
	unsigned int len = 0, i;
	struct bscd_timer timer = {timer_wait, {gpt_timer_mode_immediate},
								false, false};


	BSTD_UNUSED(max_read_bytes);
	*rcv_bytes = 0;

	val = readb(hdl->baddr + BSCD_P_PROTO_CMD);
	val |= PROTOCMD_TBUF_RST_MASK;

	/* This condition added for WHQL card 5 test */
	if ((hdl->cur_chs.sc_std == std_es) || (hdl->cur_chs.tpdu == true)) {
		/*
		 * application computes its own LRC or CRC and appends it as
		 * the last byte
		 */
		val &= ~PROTOCMD_EDC_ENA_MASK;
	} else {
		/* for APDU or other standards, hw checks EDC/LRC */
		val |= PROTOCMD_EDC_ENA_MASK;
	}

	writeb(val, hdl->baddr + BSCD_P_PROTO_CMD);
	err = chnl_p_wait_for_rready(hdl);
	if (err != BERR_SUCCESS)  {
		/* If parity error, continue reading all the bytes */
		err = BERR_TRACE(err);
		return BSCD_STATUS_NO_SC_RESPONSE;
	}

	/* Disable block wait timer */
	timer.timer_type = timer_wait;
	timer.timer_mode.wait_time_mode = wtm_blk;
	err = chnl_ena_dis_timer_isr(hdl, &timer);
	if (err != BERR_SUCCESS) {
		err = BSCD_STATUS_READ_FAILED;
		goto bscd_p_done_label;
	}

	/* Disable cwt since we already receive all the bytes */
	val = readb(hdl->baddr + BSCD_P_TIMER_CMD);
	val &= ~TIMERCMD_CWT_ENA_MASK;
	writeb(val, hdl->baddr + BSCD_P_TIMER_CMD);

	/* Clear cwt_intr so that it won't show up next time */
	spin_lock_irqsave(&hdl->lock, flag);
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	hdl->intr_status1 &= ~INTSTAT1_EV1_INTR_MASK;
#else
	hdl->intr_status2 &= ~INTSTAT2_CWT_INTR_MASK;
	readb(hdl->baddr + BSCD_P_INTR_STAT_2);
#endif
	spin_unlock_irqrestore(&hdl->lock, flag);

	len1 =  readb(hdl->baddr + BSCD_P_RLEN_1);
	len2 =  readb(hdl->baddr + BSCD_P_RLEN_2);

	/* RLEN_9_BIT_MASK = 0x01ff */
	len = ((((unsigned short) len2) << 8) | len1) & BSCD_RLEN_9_BIT_MASK;
	pr_devel("SmartCardBlockRead: rlen = %d\n", len);

	if (len) {
		for (i = 0; i < len; i++) {
			rcv_data[i] =  readb(hdl->baddr + BSCD_P_RECEIVE);
			val =  readb(hdl->baddr + BSCD_P_STATUS_2);
			if ((val & STATUS2_RPAR_ERR_MASK) ==
						STATUS2_RPAR_ERR_MASK) {
				pr_devel("SmartCardBlockRead: parity error\n");
				err = BSCD_STATUS_PARITY_EDC_ERR;
			} else if ((val & STATUS2_EDC_ERR_MASK) ==
						STATUS2_EDC_ERR_MASK) {
				pr_devel("SmartCardBlockRead: EDC error\n");
				err = BSCD_STATUS_PARITY_EDC_ERR;
			}

			if ((i % 16) == 0)
				pr_devel("\n");
			pr_devel("%02x ", rcv_data[i]);
		}
	}

bscd_p_done_label:
	if (err != BERR_SUCCESS)
		len = 0;
	*rcv_bytes = len;
	return err;
}

int chnp_p_rcv_and_decode(struct p_chnl_hdl *hdl)
{

	int err = BERR_SUCCESS;

	if (hdl->cur_chs.rst_card_act == rst_card_act_no_aciton) {
		pr_devel("%s: rst_card_act_no_aciton\n", __func__);
		return BERR_SUCCESS;
	} else if (hdl->cur_chs.rst_card_act == rst_card_act_rcv_decode) {
		pr_devel("%s: rst_card_act_rcv_decode ", __func__);
		pr_devel("standard:%d\n", hdl->cur_chs.sc_std);
		switch (hdl->cur_chs.sc_std) {
		case std_emv1996:
		case std_emv2000:
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		case std_iso:
			err = chnl_p_iso_atr_rcv_and_decode(hdl);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
			break;
		default:
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
	}
bscd_p_done_label:
	return err;
}

/*
 * Interrupt handler bottom half
 * Currently it only handles card insertion/removal bh
 */
void chnl_p_intr_handler_bh(struct p_chnl_hdl *hdl, int in_chnl_number)
{
	unsigned int   sta_reg1 = 0, prev_sta_reg1 = 0;

	/* Store status_1 to determine if hardware failure */
	prev_sta_reg1 = readb(hdl->baddr + BSCD_P_STATUS_1);

#ifdef BSCD_INTERRUPT_DEBUG
	pr_devel("prev_sta_reg1 = 0x%2x\n", prev_sta_reg1);
#endif

	/*
	 * TDA8004 suggests we to wait until debounce stabilizes.
	 * NDS suggests to sleep for 10 milli seconds.  This may hold
	 * the system for 10ms but it is okay since the system should
	 * not continue without the card.
	 *
	 * All customers should use TDA8024 now
	 */

	sta_reg1 = readb(hdl->baddr + BSCD_P_STATUS_1);

	/*
	 * According TDA 8004 Application note, this is how to determine
	 * card presence, card removal and hardware failure.
	 */

	if ((sta_reg1 & STATUS1_CARD_PRES_MASK) &&
			(!(prev_sta_reg1 & STATUS1_CARD_PRES_MASK))) {
#ifdef BSCD_INTERRUPT_DEBUG
		pr_devel("hardware failure, sta_reg1=0x%2x\n", sta_reg1);
#endif
		hdl->chnl_status.card_present = true;
		hdl->chnl_status.status1 |= BSCD_HARDWARE_FAILURE |
						BSCD_RST_CHNL_REQD;
	} else if ((sta_reg1 & STATUS1_CARD_PRES_MASK) &&
			(prev_sta_reg1 & STATUS1_CARD_PRES_MASK)) {
#ifdef BSCD_INTERRUPT_DEBUG
		pr_devel("SC Inserted\n");
#endif
		hdl->chnl_status.card_present  = true;
		/* we have not reset the card yet */
		hdl->chnl_status.card_activate = false;
		hdl->chnl_status.pps_done      = false;
	} else if (!(sta_reg1 & STATUS1_CARD_PRES_MASK) &&
				!(prev_sta_reg1 & STATUS1_CARD_PRES_MASK)) {
#ifdef BSCD_INTERRUPT_DEBUG
		pr_devel("SC Removed\n");
#endif

		/*
		 * Disable all interrupt but pres_intr to support
		 * auto-deactivation. Auto Deactvation will cause a
		 * parity_intr and retry_intr to loop forever
		 */

		writeb(0, hdl->baddr + BSCD_P_INTR_EN_1);
		writeb(0, hdl->baddr + BSCD_P_INTR_EN_2);

		/* 09/20/05,Allen.C, remember Card was removed */
		hdl->is_card_removed = true;

		hdl->chnl_status.card_present  = false;
		hdl->chnl_status.card_activate = false;
		hdl->chnl_status.pps_done      = false;
		hdl->chnl_status.status1 |= BSCD_RST_CHNL_REQD;
	}

	if (hdl->chnl_status.card_present == true)
		pr_devel("SC %d Insertion Interrupt\n", hdl->chnl_number);
	else
		pr_devel("SC %d Removal Interrupt\n", hdl->chnl_number);

	/* re-enable pres intr */
	chnl_ena_int_cbisr(hdl, int_card_insert, chnl_p_card_insert_cbisr);

	/* The bottom half has done */
	set_event(hdl->chnl_wait_event.bh_pres);
	pr_devel("Intr BH done ch:%d\n", hdl->chnl_number);
}

/*
 * ISR handler.
 * Return value: if a bottom half needed or not.
 */
int chnl_p_intr_handler_isr(struct p_chnl_hdl *hdl, int in_chnl_number)
{
	enum bscd_int_type event;
	int err = BERR_SUCCESS;
	int i;
	unsigned int  sta_reg1 = 0, sta_reg2 = 0, proto_cmd_reg = 0;
	unsigned int  intr_en1 = 0, intr_en2 = 0;
	unsigned int intr_sta_reg1 = 0, intr_sta_reg2 = 0;
	unsigned int   val;
	struct bscd_timer timer = {timer_gpt, {gpt_timer_mode_immediate},
								true, true};
	int bh = 0; /* if bottom half needed */

#ifdef BSCD_EMV2000_CWT_PLUS_4
	struct bscd_timer cwttimer = {timer_wait, {wtm_work}, true, true};
	struct bscd_timer_value cwttime_val = {16, unit_etu};
#endif


#ifdef BSCD_USE_POLLING
	BSTD_UNUSED(in_param2);
#endif
	if (hdl == NULL) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	if (hdl->is_open ==  false) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	/* Read Smartcard Interrupt Status & Mask Register */
	proto_cmd_reg = readb(hdl->baddr + BSCD_P_PROTO_CMD);
	sta_reg1 = readb(hdl->baddr + BSCD_P_STATUS_1);
	sta_reg2 = readb(hdl->baddr + BSCD_P_STATUS_2);

	intr_en1 = readb(hdl->baddr + BSCD_P_INTR_EN_1);
	intr_sta_reg1 = readb(hdl->baddr + BSCD_P_INTR_STAT_1);
	intr_en2 = readb(hdl->baddr + BSCD_P_INTR_EN_2);
	intr_sta_reg2 = readb(hdl->baddr + BSCD_P_INTR_STAT_2);

#ifdef BSCD_INTERRUPT_DEBUG
	pr_devel("chnl_number = %d\n", hdl->chnl_number);
	pr_devel("BSCD_P_INTR_EN_1   = 0x%2x\n", intr_en1);
	pr_devel("BSCD_P_INTR_EN_2   = 0x%2x\n", intr_en2);
	pr_devel("BSCD_P_INTR_STAT_1 = 0x%2x\n", intr_sta_reg1);
	pr_devel("BSCD_P_INTR_STAT_2 = 0x%2x\n", intr_sta_reg2);
	pr_devel("BSCD_P_STATUS_1    = 0x%2x\n", sta_reg1);
	pr_devel("BSCD_P_STATUS_2    = 0x%2x\n", sta_reg2);
#endif

	/* Process interrupt */
	if ((intr_en1 & INTEN1_PRES_MASK) &&
			(intr_sta_reg1 & INTSTAT1_PRES_INTR_MASK)) {

		hdl->intr_status1 = intr_sta_reg1;
		hdl->status1  = sta_reg1;

		/* Disable pres intr to debounce the card pres */
		err = chnl_dis_intr_cbisr(hdl, int_card_insert);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
		bh = 1;
		/*
		 * TBC checked...
		 * clear the event so any process waiting would wait for
		 * this bottom half
		 * BKNI_ResetEvent(hdl->chnl_wait_event.bh_pres);
		 */
	}
	if ((intr_en1 & INTEN1_TPAR_MASK) &&
			(intr_sta_reg1 & INTSTAT1_TPAR_INTR_MASK)) {
		hdl->chnl_status.status1 |= BSCD_TX_PARITY;
		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;
		event = int_tparity;
#ifdef BSCD_EMV2000_FIME
		hdl->parity_error_cnt++;
#endif
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.tparity[i] != NULL)
				(*(hdl->callback.tparity[i]))(hdl, &event);
		}
	}
	if ((intr_en1 & INTEN1_TIMER_MASK) &&
			(intr_sta_reg1 & INTSTAT1_TIMER_INTR_MASK)) {
		timer.is_timer_intr_ena = false;
		timer.is_timer_ena = false;
		err = chnl_ena_dis_timer_isr(hdl, &timer);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		/*
		 * We need to signal different events to take care of different
		 * scenarioes
		 */
		event = int_timer;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)  {
			if (hdl->callback.timer[i] != NULL)
				(*(hdl->callback.timer[i]))(hdl, &event);
		}
	}

	if ((intr_en1 & INTEN1_BGT_MASK) &&
		(intr_sta_reg1 & INTSTAT1_BGT_INTR_MASK)) {

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		/*
		 * We need to signal different events to take care of
		 * different scenarioes
		 */
		event = int_bg;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.bgt[i] != NULL)
				(*(hdl->callback.bgt[i]))(hdl, &event);
	}

	if ((intr_en1 & INTEN1_TDONE_MASK) &&
		(intr_sta_reg1 & INTSTAT1_TDONE_INTR_MASK)) {

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		/*
		 * We need to signal different events to take care of
		 * different scenarioes
		 */
		event = int_tdone;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.tdone[i] != NULL)
				(*(hdl->callback.tdone[i]))(hdl, &event);
	}

	if ((intr_en1 & INTEN1_RETRY_MASK) &&
		(intr_sta_reg1 & INTSTAT1_RETRY_INTR_MASK)) {

		/*
		 * If parity tx or rx retrial failes, we should reset uart
		 * and NOT to continue tx any more data
		 */
		val =  readb(hdl->baddr + BSCD_P_UART_CMD_1);
		val |= (UARTCMD1_UART_RST_MASK);
		writeb(val, hdl->baddr + BSCD_P_UART_CMD_1);

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		/*
		 * We need to signal different events to take care of
		 * different scenarioes
		 */
		event = int_retry;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.retry[i] != NULL)
				(*(hdl->callback.retry[i]))(hdl, &event);
	}

	if ((intr_en1 & INTEN1_TEMPTY_MASK) &&
			(intr_sta_reg1 & INTSTAT1_TEMP_INTR_MASK)) {

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		/* Currently we do not need this.  No signal needed */
		event = int_tempty;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.tempty[i] != NULL)
				(*(hdl->callback.tempty[i]))(hdl, &event);
	}

	if ((intr_en2 & INTEN2_RPAR_MASK) &&
			(intr_sta_reg2 & INTSTAT2_RPAR_INTR_MASK)) {

		hdl->chnl_status.status1 |= BSCD_RX_PARITY;
		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		event = int_rparity;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.rparity[i] != NULL)
				(*(hdl->callback.rparity[i]))(hdl, &event);
	}

	if ((intr_en2 & INTEN2_ATRS_MASK) &&
		(intr_sta_reg2 & INTSTAT2_ATR_INTR_MASK)) {

		/*
		 * We need this interrupt to measure the period of time we
		 * received leading edge of the start bit of the first ATR
		 * byte. As soon as we receive this interrupt, we should stop
		 * the timer so that we could get more accurate timing
		 */

		timer.is_timer_intr_ena = false;
		timer.is_timer_ena = false;
		err = chnl_ena_dis_timer_isr(hdl, &timer);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		event = int_atr;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.atr[i] != NULL)
				(*(hdl->callback.atr[i]))(hdl, &event);
	}

	if ((intr_en2 & INTEN2_CWT_MASK) &&
			(intr_sta_reg2 & INTSTAT2_CWT_INTR_MASK)) {

		/*
		 * If cwt_intr comes in after rready_intr,
		 * it is considered normal
		 */
		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		event = int_cw;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.cwt[i] != NULL)
				(*(hdl->callback.cwt[i]))(hdl, &event);
	}

	if ((intr_en2 & INTEN2_RLEN_MASK) &&
			(intr_sta_reg2 & INTSTAT2_RLEN_INTR_MASK)) {

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		event = int_rlen;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.rlen[i] != NULL)
				(*(hdl->callback.rlen[i]))(hdl, &event);
	}

	if ((intr_en2 & INTEN2_WAIT_MASK) &&
		(intr_sta_reg2 & INTSTAT2_WAIT_INTR_MASK)) {

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		/*
		 * We need to signal different events to take care of
		 * different scenarioes
		 */
		event = int_wait;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.wait[i] != NULL)
				(*(hdl->callback.wait[i]))(hdl, &event);
	}

	if ((intr_en2 & INTEN2_RCV_MASK) &&
			(intr_sta_reg2 & INTSTAT2_RCV_INTR_MASK)) {

		/*
		 * Enable RCV_INTR only in T=1,EMV 2000 to resolve CWT+4
		 * issue
		 */
#ifdef BSCD_EMV2000_CWT_PLUS_4
		if ((hdl->cur_chs.sc_std == std_emv2000) &&
			(hdl->cur_chs.proto_type == async_proto_e1) &&
			(hdl->is_recv == true)) {

			/* Disable BWT timer */
			cwttimer.is_timer_intr_ena = false;
			cwttimer.is_timer_ena = false;
			err = chnl_ena_dis_timer_isr(hdl, &cwttimer);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;

			/* Enable WWT in lieu of CWT */
			cwttimer.is_timer_intr_ena = true;
			cwttimer.is_timer_ena = true;
			if (hdl->cur_chs.char_wait_time_integer != 0)
				cwttime_val.value =
				(2 << (hdl->cur_chs.char_wait_time_integer - 1))
				+ 15 + BSCD_CHARACTER_WAIT_TIME_GRACE_PERIOD;
				err = chnl_config_timer(hdl, &cwttimer,
								&cwttime_val);
				if (err != BERR_SUCCESS) {
					err = BSCD_STATUS_READ_FAILED;
					goto bscd_p_done_label;
				}
			pr_devel("RCV_INTR  cwt = %d\n", cwttime_val.value);
			hdl->status2 |= sta_reg2;
			intr_sta_reg2 &= ~INTSTAT2_RCV_INTR_MASK;
		}
#endif
		/*
		 * Enable RCV_INTR only in T=1,EMV 2000 to resolve CWT+4
		 * issue
		 */

		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;
		event = int_rcv;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.rcv[i] != NULL)
				(*(hdl->callback.rcv[i]))(hdl, &event);
	}

	if ((intr_en2 & INTEN2_RRDY_MASK) &&
			(intr_sta_reg2 & INTSTAT2_RRDY_INTR_MASK)) {

#ifdef BSCD_EMV2000_CWT_PLUS_4
		if ((hdl->cur_chs.sc_std == std_emv2000) &&
			(hdl->cur_chs.proto_type == async_proto_e1)) {
			/* Disable WWT timer, which is used as CWT + 4  */
			cwttimer.is_timer_intr_ena = false;
			cwttimer.is_timer_ena = false;
			err = chnl_ena_dis_timer_isr(hdl, &cwttimer);
			if (err != BERR_SUCCESS) {
				err = BSCD_STATUS_READ_FAILED;
				goto bscd_p_done_label;
			}
			pr_devel("RREADY_INTR  cwt disable\n");
		}
#endif
		/*
		 * fix for RReady hang, where it waits for Status2 but Status2
		 * does not contain RcvRdy
		 */
		if ((sta_reg2 & STATUS2_RRDY_MASK) == 0)
			sta_reg2 = readb(hdl->baddr + BSCD_P_STATUS_2);

		hdl->status2 |= sta_reg2;
		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		event = int_rready;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.rrdy[i] != NULL)
				(*(hdl->callback.rrdy[i]))(hdl,  &event);
	}

	if ((proto_cmd_reg & PROTOCMD_EDC_ENA_MASK) &&
			(sta_reg2 & STATUS2_EDC_ERR_MASK)) {

		hdl->chnl_status.status1 |= BSCD_TX_PARITY;
		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;
		event = int_edc;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.edc[i] != NULL)
				(*(hdl->callback.edc[i]))(hdl, &event);
	}

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
	if ((intr_en1 & INTEN1_EV1_MASK) &&
			(intr_sta_reg1 & INTSTAT1_EV1_INTR_MASK)) {

		/*
		 * If cwt_intr comes in after rrdy_intr,it is considered
		 * normal
		 */
		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		event = int_event1;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.event1[i] != NULL)
				(*(hdl->callback.event1[i]))(hdl, &event);
	}

	if ((intr_en2 & BCHP_SCA_SC_INTR_EN_2_event2_ien_MASK) &&
			(intr_sta_reg2 & INTSTAT2_EV2_INTR_MASK)) {

		/*
		 * If cwt_intr comes in after rrdy_intr,it is considered
		 * normal
		 */
		hdl->intr_status1 |= intr_sta_reg1;
		hdl->intr_status2 |= intr_sta_reg2;

		event = int_event2;
		for (i = 0; i < BSCD_MAX_NUM_CALLBACK_FUNC; i++)
			if (hdl->callback.event2[i] != NULL)
				(*(hdl->callback.event2[i]))(hdl, &event);
	}
#endif

bscd_p_done_label:
	return bh;
}

/* For T=0 and T=1 only */
int chnl_p_t0t1_transmit(struct p_chnl_hdl *hdl, unsigned char *xmit_data,
			 unsigned long xmit_bytes)
{
	unsigned long flag;
	unsigned int val;
	unsigned int i;
	int err = BERR_SUCCESS;
	struct bscd_timer timer = {timer_gpt, {gpt_timer_mode_immediate},
								true, true};
	struct bscd_timer_value time_val = {BSCD_MIN_DELAY_BEFORE_TZERO_SEND,
								unit_etu};

	WARN_ON(!hdl);

	if (hdl->magic_number != BSCD_P_CHANNEL_HANDLE_MAGIC_NUMBER) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	/* p_hex_dump("Send",xmit_data, xmit_bytes); */
	spin_lock_irqsave(&hdl->lock, flag);
	hdl->intr_status1 &=
		~INTSTAT1_TPAR_INTR_MASK &
		~INTSTAT1_TIMER_INTR_MASK &
		~INTSTAT1_BGT_INTR_MASK &
		~INTSTAT1_TDONE_INTR_MASK &
		~INTSTAT1_RETRY_INTR_MASK &
		~INTSTAT1_TEMP_INTR_MASK
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
		& ~INTSTAT1_EV1_INTR_MASK
#endif
		;

	hdl->intr_status2 &=
		~INTSTAT2_RPAR_INTR_MASK &
		~INTSTAT2_CWT_INTR_MASK &
		~INTSTAT2_RLEN_INTR_MASK &
		~INTSTAT2_WAIT_INTR_MASK &
		~INTSTAT2_RCV_INTR_MASK &
		~INTSTAT2_RRDY_INTR_MASK &
		~INTSTAT2_EV2_INTR_MASK;
	spin_unlock_irqrestore(&hdl->lock, flag);


	/* Reset the Transmit and Receive buffer */
	val = readb(hdl->baddr + BSCD_P_PROTO_CMD);
	val |= PROTOCMD_TBUF_RST_MASK | PROTOCMD_RBUF_RST_MASK;
	writeb(val, hdl->baddr + BSCD_P_PROTO_CMD);

	/*
	 * Enable cwt here for only T=1. We will disable cwt in
	 * SmartCardTOneReceive() after we receive RREADY_INTR
	 */
	if (hdl->cur_chs.proto_type == async_proto_e1) {

		/* Clear the possible previous cwt_intr */
		hdl->intr_status2 &= ~INTSTAT2_CWT_INTR_MASK;
		readb(hdl->baddr + BSCD_P_INTR_STAT_2);
		val = readb(hdl->baddr + BSCD_P_TIMER_CMD);
		val |= TIMERCMD_CWT_ENA_MASK;
		writeb(val, hdl->baddr + BSCD_P_TIMER_CMD);

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
		/* Clear the possible previous event1 intr */
		hdl->intr_status1 &= ~INTSTAT1_EV1_INTR_MASK;
		if (hdl->cur_chs.sc_std == std_emv2000) {
			/* 4 ETU after CWT */
			writeb(5, hdl->baddr + BSCD_P_EVENT1_CMP);

			/* start event src */
			val = BSCD_P_CWT_INTR_EVENT_SRC;
			writeb(val, hdl->baddr + BSCD_P_EVENT1_CMD_3);

			/* increment event src */
			val = BSCD_P_RX_ETU_TICK_EVENT_SRC;
			writeb(val, hdl->baddr + BSCD_P_EVENT1_CMD_2);

			/* reset event src */
			val = BSCD_P_RX_START_BIT_EVENT_SRC;
			writeb(val, hdl->baddr + BSCD_P_EVENT1_CMD_1);

			/*
			 * event_en, intr_mode, run_after_reset and
			 * run_after_compare
			 */
			val = EV1CMD4_EV_ENA_MASK;
			val |= EV1CMD4_INTR_AFTER_CMP_MASK;
			val |= EV1CMD4_RUN_AFTER_RST_MASK;

			val &= ~(EV1CMD4_INTR_AFTER_RST_MASK |
					EV1CMD4_RUN_AFTER_CMP_MASK);

			writeb(val, hdl->baddr + BSCD_P_EVENT1_CMD_4);
		}
#endif
	}

	/*
	 * For EMV T=0 only, the minimum interval btw the leading
	 * edges of the start bits of 2 consecutive characters sent
	 * in opposite directions shall be 16.  For EMV and ISO T=1,
	 * the minimum interval btw the leading edges of the start bits of 2
	 * consecutive characters sent in opposite directions shall be 22.
	 */

	if (hdl->cur_chs.proto_type == async_proto_e0) {
		err = chnl_config_timer(hdl, &timer, &time_val);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	} else {
		/*
		 * Set Timer :
		 * timer.is_timer_intr_ena = true;
		 * timer.is_timer_ena = true;
		 * timer.timer_type = timer_gpt;
		 * timer.timer_mode.gpt_timer_mode = gpt_timer_mode_immediate;
		 * time_val.value = BSCD_BLOCK_GUARD_TIME;
		 * time_val.unit  = unit_etu;
		 */

		err = chnl_config_timer(hdl, &timer, &time_val);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	err = chnl_p_wait_for_timer_event(hdl, 1);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	/* Disable timer */
	timer.is_timer_intr_ena = false;
	timer.is_timer_ena = false;
	err = chnl_ena_dis_timer_isr(hdl, &timer);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;

	/*
	 * BSYT Issue: RC0 WWT timer could only check the interval
	 * btw the leading edge of 2 consecutive characters sent
	 * by the ICC.  We will use GP timer to check the interval
	 * btw the leading edge of characters in opposite directions
	 */
	if (hdl->cur_chs.proto_type == async_proto_e0) {
		/* Restore the original WWT */
		timer.is_timer_intr_ena = true;
		timer.is_timer_ena = true;
		timer.timer_type = timer_wait;
		timer.timer_mode.wait_time_mode = wtm_work;
		time_val.value = hdl->cur_chs.work_wait_time.value;
		err = chnl_config_timer(hdl, &timer, &time_val);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	} else if (hdl->cur_chs.proto_type == async_proto_e1) {
		timer.is_timer_intr_ena = true;
		timer.is_timer_ena = true;
		timer.timer_type = timer_wait;
		timer.timer_mode.wait_time_mode = wtm_blk;
		if (hdl->cur_chs.blk_wait_time_ext.value == 0) {
			time_val.value =
				hdl->cur_chs.blk_wait_time.value;
		} else {
			time_val.value =
				hdl->cur_chs.blk_wait_time_ext.value;
		}
		err = chnl_config_timer(hdl, &timer, &time_val);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}

	/*
	 * Fill BCM FIFO with the request message.
	 */
	for (i = 0; i < xmit_bytes; i++) {
		writeb(xmit_data[i], hdl->baddr + BSCD_P_TRANSMIT);
		pr_devel("%02x ", xmit_data[i]);
	}
	pr_devel("\n");

	/* Enable EDC */
	val = readb(hdl->baddr + BSCD_P_PROTO_CMD);
	if (hdl->cur_chs.proto_type == async_proto_e0) {
		val |= PROTOCMD_RBUF_RST_MASK;
	} else {
		val |= PROTOCMD_RBUF_RST_MASK;
		if ((hdl->cur_chs.sc_std == std_es) ||
			/* This condition added for WHQL card 5 test */
			(hdl->cur_chs.tpdu == true)) {
			/*
			 * application computes its own LRC or CRC and appends
			 * it as the last byte
			 */
			val &= ~PROTOCMD_EDC_ENA_MASK;
		} else {
			val |= PROTOCMD_EDC_ENA_MASK;
			/* FIXME: this really needs to get from ATR */
			val &= ~PROTOCMD_CRC_LRC_MASK;
		}
	}
	writeb(val, hdl->baddr + BSCD_P_PROTO_CMD);
	/* Set flow cmd */
	val = readb(hdl->baddr + BSCD_P_FLOW_CMD);
	if (hdl->cur_chs.proto_type == async_proto_e0) {

		/*
		 * Only NDS support smartcard flow control. We MUST NOT set
		 * SC_FLOW_EN to 1 for other standards.
		 */
		if (hdl->cur_chs.sc_std == std_nds)
			val |= FLOWCMD_FLOW_ENA_MASK;
		else
			val &= ~FLOWCMD_FLOW_ENA_MASK;
	} else {
		/* No flow control for T=1 protocol or T=14. */
		val &= ~FLOWCMD_FLOW_ENA_MASK;
	}
	writeb(val, hdl->baddr + BSCD_P_FLOW_CMD);

	/* Ready to transmit */
	val = readb(hdl->baddr + BSCD_P_UART_CMD_1);

	/* Always set auto receive */
	val = UARTCMD1_T_R_MASK | UARTCMD1_XMIT_GO_MASK |
			UARTCMD1_IO_ENA_MASK | UARTCMD1_AUTO_RCV_MASK;

	writeb(val, hdl->baddr + BSCD_P_UART_CMD_1);

#ifdef BSCD_EMV2000_CWT_PLUS_4
	hdl->is_recv = true;
#endif
	/*
	 * Wait until the BCM sent all the data.
	 */
	err = chnl_p_wait_for_tdone(hdl);
	if (err != BERR_SUCCESS)
		goto bscd_p_done_label;
bscd_p_done_label:
	return err;
}

int chnl_p_ena_intr_isr(struct p_chnl_hdl *hdl)
{
	int err = BERR_SUCCESS;
	int val = 0;

	if (hdl->is_open == true) {
		/* Update BSCD_P_INTR_EN_1 and BSCD_P_INTR_EN_2 */
		if ((hdl->cur_chs.proto_type == async_proto_e0) &&
				(hdl->cur_chs.sc_std != std_irdeto)) {
			/* Enable parity error re-transmission only in T=0 */
			err = chnl_ena_int_cbisr(hdl, int_retry,
						chnl_p_retry_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;

			/* Enable RCV_INTR only in T=0 */
			err = chnl_ena_int_cbisr(hdl, int_rcv,
							chnl_p_rcv_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;

			/* Enable RPAR_INTR only in T=0 */
			err = chnl_ena_int_cbisr(hdl, int_rparity,
						chnl_p_rparity_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
			/* Enable TPAR_INTR only in T=0 */
			err = chnl_ena_int_cbisr(hdl, int_tparity,
						chnl_p_tparity_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
		/* else for T=1 protocol */
		} else if (hdl->cur_chs.proto_type == async_proto_e1) {

			/* Enable cwt only in T=1 */
#ifdef BSCD_EMV2000_CWT_PLUS_4
			if ((hdl->cur_chs.sc_std != std_emv2000) ||
				(hdl->cur_chs.proto_type != async_proto_e1)) {
#endif
				err = chnl_ena_int_cbisr(hdl, int_cw,
							chnl_p_cwt_cbisr);
				if (err != BERR_SUCCESS)
					goto bscd_p_done_label;
#ifdef BSCD_EMV2000_CWT_PLUS_4
			}
#endif

#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
			/* Enable BGT only in T=1 */
			if (hdl->cur_chs.sc_std == std_emv2000) {
				err = chnl_ena_int_cbisr(hdl, int_event1,
							chnl_p_event1_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
			}
#endif
			/* Enable BGT only in T=1 */
			err = chnl_ena_int_cbisr(hdl, int_bg,
						chnl_p_bgtcb_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
			/* Enable rlen only in T=1 */
			err = chnl_ena_int_cbisr(hdl, int_rlen,
						chnl_p_rlen_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;

			/* Enable rreadyonly in T=1 */
			err = chnl_ena_int_cbisr(hdl, int_rready,
						chnl_p_rready_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
			/* set block guard time for T=1 only */
			/* Update the BSCD_P_BGT */
			val = BGT_R2T_MASK | hdl->cur_chs.blk_guard_time.value;
			writeb(val, hdl->baddr + BSCD_P_BGT);
	/* Enable RCV_INTR only in T=1, EMV 2000 to resolve CWT+4 issue */
#ifdef BSCD_EMV2000_CWT_PLUS_4
			if ((hdl->cur_chs.sc_std == std_emv2000) &&
				(hdl->cur_chs.proto_type == async_proto_e1)) {
				err = chnl_ena_int_cbisr(hdl, int_rcv, NULL);
				if (err != BERR_SUCCESS)
					goto bscd_p_done_label;
			}
#endif
	/* Enable RCV_INTR only in T=1, EMV 2000 to resolve CWT+4 issue */
		/* else part T=14 Irdeto  protocol */
		} else if (hdl->cur_chs.sc_std ==  std_irdeto) {
			/* Enable RCV_INTR only in T=0 */
			err = chnl_ena_int_cbisr(hdl, int_rcv,
							chnl_p_rcv_cbisr);
			if (err != BERR_SUCCESS)
				goto bscd_p_done_label;
		}

		/* Keep the card insertion and removal interrupt */
		err = chnl_ena_int_cbisr(hdl, int_card_insert,
					chnl_p_card_insert_cbisr);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		err = chnl_ena_int_cbisr(hdl, int_card_remove,
						chnl_p_card_remove_cbisr);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;

		/* Enable tdone for T=0 and  T=1 */
		err = chnl_ena_int_cbisr(hdl, int_tdone,
						chnl_p_tdone_cbisr);
		if (err != BERR_SUCCESS)
			goto bscd_p_done_label;
	}
bscd_p_done_label:
	return err;
}

int chnp_p_set_standard(struct p_chnl_hdl *hdl,
			const struct bscd_chnl_settings *chs)
{

	int err = BERR_SUCCESS;

	/* Asynchronous Protocol Types. */
	if ((chs->proto_type <= async_proto_unknown)  ||
			(chs->proto_type > async_proto_e14_irdeto)) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	switch (hdl->cur_chs.sc_std) {
	case std_nds:		/* NDS. T=0 with flow control. */
		if (chs->proto_type != async_proto_e0) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;
	case std_iso:		/* ISO 7816. T_0 or T=1*/
		if ((chs->proto_type != async_proto_e0) &&
			(chs->proto_type != async_proto_e1)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;
	case std_emv1996:		/* EMV. T=0 or T=1 */
	case std_emv2000:		/* EMV. T=0 or T=1 */
		if ((chs->proto_type != async_proto_e0) &&
			(chs->proto_type != async_proto_e1)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;

	case std_arib:		/* ARIB. T=1 */
		if ((chs->proto_type != async_proto_e0) &&
				(chs->proto_type != async_proto_e1)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;

	case std_irdeto: /*
			  * Irdeto. T=14.  Need Major software workarouond
			  * ito support this
			  */
		if ((chs->proto_type != async_proto_e0) &&
			(chs->proto_type != async_proto_e14_irdeto)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;
	case std_es:	/* ES, T=1.  Obsolete. Use ISO */
		if ((chs->proto_type != async_proto_e0) &&
			(chs->proto_type != async_proto_e1)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;

	case std_mt:	/* MT, T=0.  Obsolete. Use ISO */
		if (chs->proto_type != async_proto_e0) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;

	case std_conax:	/* Conax, T=0.  Obsolete. Use ISO */
		if (chs->proto_type != async_proto_e0) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.proto_type = chs->proto_type;
		break;

	default:
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("proto_type = %d\n", hdl->cur_chs.proto_type);

bscd_p_done_label:
	return err;
}

int chnp_p_set_freq(struct p_chnl_hdl *hdl,
		    const struct bscd_chnl_settings *chs)
{
	int err = BERR_SUCCESS;
	int val = 0;

	/* Set F, Clock Rate Conversion Factor */
	if (chs->ffactor == 0) {
		hdl->cur_chs.ffactor = BSCD_DEFAULT_F;
	} else if ((chs->ffactor >= 1) && (chs->ffactor <= 13)) {
		hdl->cur_chs.ffactor = chs->ffactor;
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("ffactor = %d\n", hdl->cur_chs.ffactor);

	/* Set D, Baud Rate Adjustor */
	if (chs->dfactor == 0)
		hdl->cur_chs.dfactor = BSCD_DEFAULT_D;

	if ((chs->dfactor >= 1) && (chs->dfactor <= 9)) {
		if (p_get_iso_baud_rate_adjustor(chs->dfactor) ==
						((unsigned char) -1)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.dfactor = chs->dfactor;
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("dfactor = %d\n", hdl->cur_chs.dfactor);

	/* Set ETU Clock Divider */
	if (chs->etu_clk_div == 0) {
		hdl->cur_chs.etu_clk_div = p_get_etuclk_div(
							hdl->cur_chs.dfactor,
							hdl->cur_chs.ffactor);
	} else if ((chs->etu_clk_div == 1) ||
			(chs->etu_clk_div == 2) ||
			(chs->etu_clk_div == 3) ||
			(chs->etu_clk_div == 4) ||
			(chs->etu_clk_div == 5) ||
			(chs->etu_clk_div == 6) ||
			(chs->etu_clk_div == 7) ||
			(chs->etu_clk_div == 8)) {

		hdl->cur_chs.etu_clk_div = chs->etu_clk_div;
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("etu_clk_div = %d\n", hdl->cur_chs.etu_clk_div);

	/* Set SC Clock Divider */
	if (chs->sc_clk_div == 0) {
		hdl->cur_chs.sc_clk_div = p_get_clk_div(hdl->cur_chs.dfactor,
							hdl->cur_chs.ffactor);
	} else if ((chs->sc_clk_div == 1) ||
			(chs->sc_clk_div == 2) ||
			(chs->sc_clk_div == 3) ||
			(chs->sc_clk_div == 4) ||
			(chs->sc_clk_div == 5) ||
			(chs->sc_clk_div == 8) ||
			(chs->sc_clk_div == 10) ||
			(chs->sc_clk_div == 16)) {

		hdl->cur_chs.sc_clk_div = chs->sc_clk_div;
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("sc_clk_div = %d\n", hdl->cur_chs.sc_clk_div);

	/* Set external Clock Divisor.For TDA only 1,2,4,8 are valid value. */
	if (chs->external_clk_div == 0)
		hdl->cur_chs.external_clk_div =
				BSCD_DEFAULT_EXTERNAL_CLOCK_DIVISOR;
	else
		hdl->cur_chs.external_clk_div = chs->external_clk_div;

	pr_devel("external_clk_div = %d\n", hdl->cur_chs.external_clk_div);

	/* Set Prescale */
	if (chs->prescale == 0) {
		hdl->cur_chs.prescale = p_get_prescale(hdl->cur_chs.dfactor,
						hdl->cur_chs.ffactor) *
					hdl->cur_chs.external_clk_div +
					(hdl->cur_chs.external_clk_div - 1);
	} else if (chs->prescale <= BSCD_MAX_PRESCALE) {
		hdl->cur_chs.prescale = chs->prescale *
			hdl->cur_chs.external_clk_div +
			(hdl->cur_chs.external_clk_div - 1);
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("prescale = %lu\n", hdl->cur_chs.prescale);

	/* Set baud divisor */
	if (chs->baud_div == 0) {
		hdl->cur_chs.baud_div = BSCD_DEFAULT_BAUD_DIV;
	} else if ((chs->baud_div == 31) ||
			(chs->baud_div == 32) ||
			(chs->baud_div == 25)) {
		hdl->cur_chs.baud_div = chs->baud_div;
	} else {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	pr_devel("baud_div = %d\n", hdl->cur_chs.baud_div);

	/* Set ICC CLK Freq */
	hdl->cur_chs.curr_icc_clk_freq =
			hdl->mod_hdl->cur_settings.mod_clk_freq.clk_freq  /
					hdl->cur_chs.sc_clk_div /
					hdl->cur_chs.etu_clk_div/
					hdl->cur_chs.external_clk_div;
	pr_devel("curr_icc_clk_freq = %lu\n", hdl->cur_chs.curr_icc_clk_freq);

	hdl->cur_chs.curr_baud_rate =
			hdl->mod_hdl->cur_settings.mod_clk_freq.clk_freq /
					hdl->cur_chs.etu_clk_div/
					(hdl->cur_chs.prescale+1)/
					hdl->cur_chs.baud_div;
	pr_devel("curr_baud_rate = %lu\n", hdl->cur_chs.curr_baud_rate);

	if (hdl->cur_chs.sc_std != std_irdeto) {
		val = hdl->cur_chs.curr_icc_clk_freq *
			p_get_iso_baud_rate_adjustor(hdl->cur_chs.dfactor) /
			p_get_iso_clkrate_conversion_factor(
							hdl->cur_chs.ffactor);
		pr_devel("ISO curr_baud_rate = %d\n", val);

		val = p_get_iso_baud_rate_adjustor(hdl->cur_chs.dfactor);
		pr_devel("ISOBaudRateAdjustor = %d\n", val);

		val = p_get_iso_clkrate_conversion_factor(hdl->cur_chs.ffactor);
		pr_devel("ISOClockRateConversionFactor = %d\n", val);


		/*
		 * If the final ISO baudrate is not equal to the final BRCM
		 * baudrate, there is a potential mismatch
		 */
		if (hdl->cur_chs.curr_baud_rate !=
			(hdl->cur_chs.curr_icc_clk_freq *
			p_get_iso_baud_rate_adjustor(hdl->cur_chs.dfactor) /
			p_get_iso_clkrate_conversion_factor(
						hdl->cur_chs.ffactor))) {
			err = BSCD_STATUS_FAILED;
			pr_err("SCI: baudrate mismatch\n");
			goto bscd_p_done_label;
		}
	} else {
		/* For T=14 Irdeto */
		val =  hdl->cur_chs.curr_icc_clk_freq /
			BSCD_T14_IRDETO_CONSTANT_CLOCK_RATE_CONV_FACTOR;
		pr_devel("ISO curr_baud_rate = %d\n", val);

		/*
		 * If the final ISO baudrate is not equal to the final BRCM
		 * baudrate, there is a potential mismatch
		 */
#ifndef A582_HAWK /* Temporary(?) workaround to get T=14 card to work */
		if (hdl->cur_chs.curr_baud_rate !=
			(hdl->cur_chs.curr_icc_clk_freq  /
			BSCD_T14_IRDETO_CONSTANT_CLOCK_RATE_CONV_FACTOR)) {
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
#endif
	}
bscd_p_done_label:
	return err;
}

int chnp_p_set_wait_time(struct p_chnl_hdl *hdl,
			 const struct bscd_chnl_settings *chs)
{

	int err = BERR_SUCCESS;
	int val = 0;
	/* Set work waiting time */
	if (chs->work_wait_time.value == 0) {
		hdl->cur_chs.work_wait_time.value =
				BSCD_DEFAULT_WORK_WAITING_TIME;
		hdl->cur_chs.work_wait_time.unit = unit_etu;
	} else {
		switch (chs->work_wait_time.unit) {
		case unit_etu:
			hdl->cur_chs.work_wait_time.value =
					chs->work_wait_time.value;
			break;
		case unit_clk:
			hdl->cur_chs.work_wait_time.value =
				chs->work_wait_time.value *
				hdl->cur_chs.curr_baud_rate /
				hdl->cur_chs.curr_icc_clk_freq;
			break;
		case unit_ms:
			hdl->cur_chs.work_wait_time.value =
				chs->work_wait_time.value *
				hdl->cur_chs.curr_baud_rate/1000;
			break;
		default:
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.work_wait_time.unit =  unit_etu;
	}
	val = hdl->cur_chs.work_wait_time.value;
	pr_devel("work_wait_time.value in ETU = %d\n", val);
	val = hdl->cur_chs.work_wait_time.unit;
	pr_devel("work_wait_time.unit = %d\n", val);

	/* Set block Wait time */
	if (chs->blk_wait_time.value == 0) {
		hdl->cur_chs.blk_wait_time.value =
			BSCD_DEFAULT_BLOCK_WAITING_TIME;
		hdl->cur_chs.blk_wait_time.unit = unit_etu;
	} else {
		switch (chs->blk_wait_time.unit) {
		case unit_etu:
			hdl->cur_chs.blk_wait_time.value =
					chs->blk_wait_time.value;
			break;
		case unit_clk:
			hdl->cur_chs.blk_wait_time.value =
					chs->blk_wait_time.value *
					hdl->cur_chs.curr_baud_rate /
					hdl->cur_chs.curr_icc_clk_freq;
			break;
		case unit_ms:
			hdl->cur_chs.blk_wait_time.value =
					chs->blk_wait_time.value *
					hdl->cur_chs.curr_baud_rate /
					1000;
			break;
		default:
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.blk_wait_time.unit =  unit_etu;
	}
	val = hdl->cur_chs.blk_wait_time.value;
	pr_devel("blk_wait_time.value in ETU = %d\n", val);
	val = hdl->cur_chs.blk_wait_time.unit;
	pr_devel("blk_wait_time.unit = %d\n", val);

	/* Set block Wait time extension */
	hdl->cur_chs.blk_wait_time_ext.value = chs->blk_wait_time_ext.value;
	hdl->cur_chs.blk_wait_time_ext.unit =  unit_etu;

	/* Set Character Waiting Time Integer */
	if (chs->char_wait_time_integer >
				BSCD_MAX_CHARACTER_WAIT_TIME_INTEGER) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	hdl->cur_chs.char_wait_time_integer = chs->char_wait_time_integer;
	val = hdl->cur_chs.char_wait_time_integer;
	pr_devel("char_wait_time_integer = %d\n", val);

bscd_p_done_label:
	return err;
}

int chnp_p_set_guard_time(struct p_chnl_hdl *hdl,
			  const struct bscd_chnl_settings *chs)
{
	int err = BERR_SUCCESS;
	int val = 0;


	/* Set Extra Guard Time  */
	switch (chs->extra_guard_time.unit) {
	case unit_etu:
		hdl->cur_chs.extra_guard_time.value =
					chs->extra_guard_time.value;
		break;
	case unit_clk:
		hdl->cur_chs.extra_guard_time.value =
					chs->extra_guard_time.value *
					hdl->cur_chs.curr_baud_rate /
					hdl->cur_chs.curr_icc_clk_freq;
		break;
	case unit_ms:
		hdl->cur_chs.extra_guard_time.value =
			chs->extra_guard_time.value *
			hdl->cur_chs.curr_baud_rate /
			1000;
		break;
	default:
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	hdl->cur_chs.extra_guard_time.unit =  unit_etu;
	val = hdl->cur_chs.extra_guard_time.value;
	pr_devel("extra_guard_time.value in ETU = %d\n", val);
	val = hdl->cur_chs.extra_guard_time.unit;
	pr_devel("extra_guard_time.unit = %d\n", val);

	/* Set block Guard time */
	if (chs->blk_guard_time.value == 0) {
		hdl->cur_chs.blk_guard_time.value =
				BSCD_DEFAULT_BLOCK_GUARD_TIME;
		hdl->cur_chs.blk_guard_time.unit = unit_etu;
	} else {
		switch (chs->blk_guard_time.unit) {
		case unit_etu:
			hdl->cur_chs.blk_guard_time.value =
					chs->blk_guard_time.value;
			break;
		case unit_clk:
			hdl->cur_chs.blk_guard_time.value =
					chs->blk_guard_time.value *
					hdl->cur_chs.curr_baud_rate /
					hdl->cur_chs.curr_icc_clk_freq;
			break;
		case unit_ms:
			hdl->cur_chs.blk_guard_time.value =
					chs->blk_guard_time.value *
					hdl->cur_chs.curr_baud_rate /
					1000;
			break;
		default:
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.blk_guard_time.unit = unit_etu;
	}
	if ((hdl->cur_chs.blk_guard_time.value > BSCD_MAX_BLOCK_GUARD_TIME) ||
			(hdl->cur_chs.blk_guard_time.value <
						BSCD_MIN_BLOCK_GUARD_TIME)) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	val = hdl->cur_chs.blk_guard_time.value;
	pr_devel("blk_guard_time.value in ETU = %d\n", val);

	val = hdl->cur_chs.blk_guard_time.unit;
	pr_devel("blk_guard_time.unit = %d\n", val);
bscd_p_done_label:
	return err;
}

int chnp_p_set_transmission_time(struct p_chnl_hdl  *hdl,
				 const struct bscd_chnl_settings *chs)
{
	int err = BERR_SUCCESS;


	if (chs->timeout.value == 0) {
		hdl->cur_chs.timeout.value = BSCD_DEFAULT_TIME_OUT;
		hdl->cur_chs.timeout.unit = unit_ms;
	} else {
		switch (chs->timeout.unit) {
		case unit_etu:
			hdl->cur_chs.timeout.value =  chs->timeout.value *
					1000000 / hdl->cur_chs.curr_baud_rate;
			break;
		case unit_clk:
			hdl->cur_chs.timeout.value =
					chs->timeout.value * 1000000 /
					hdl->cur_chs.curr_icc_clk_freq;
			break;
		case unit_ms:
			hdl->cur_chs.timeout.value =  chs->timeout.value;
			break;
		default:
			err = BSCD_STATUS_FAILED;
			goto bscd_p_done_label;
		}
		hdl->cur_chs.timeout.unit =  unit_ms;
	}
	pr_devel("timeout.value in ms= %d\n", hdl->cur_chs.timeout.value);
	pr_devel("timeout.unit = %d\n", hdl->cur_chs.timeout.unit);

bscd_p_done_label:
	return err;
}

int chnp_p_set_edc_parity(struct p_chnl_hdl *hdl,
			  const struct bscd_chnl_settings  *chs)
{

	int err = BERR_SUCCESS;

	/* Set Number of transmit parity retries */
	if (chs->tx_retries > BSCD_MAX_TX_PARITY_RETRIES) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}

	hdl->cur_chs.tx_retries =  chs->tx_retries;
	pr_devel("tx_retries = %d\n", hdl->cur_chs.tx_retries);

	/* Set Number of receive parity retries */
	if (chs->rx_retries > BSCD_MAX_TX_PARITY_RETRIES) {
		err = BSCD_STATUS_FAILED;
		goto bscd_p_done_label;
	}
	hdl->cur_chs.rx_retries = chs->rx_retries;
	pr_devel("rx_retries = %d\n", hdl->cur_chs.rx_retries);

	/* Set EDC encoding */
	hdl->cur_chs.edc_setting.is_enabled =  chs->edc_setting.is_enabled;
	hdl->cur_chs.edc_setting.edc_encode =  chs->edc_setting.edc_encode;

	pr_devel("edc_setting.is_ena:%d", hdl->cur_chs.edc_setting.is_enabled);
	pr_devel("edc_encode = %d\n", hdl->cur_chs.edc_setting.edc_encode);
bscd_p_done_label:
	return err;
}
