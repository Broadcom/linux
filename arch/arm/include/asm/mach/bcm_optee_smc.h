/*
 * Copyright (C) 2017 Broadcom
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

#ifndef BCM_OPTEE_SMC_H
#define BCM_OPTEE_SMC_H

#include <linux/arm-smccc.h>
#include <linux/types.h>

#define FAST_SMC_CALL		(ARM_SMCCC_FAST_CALL << ARM_SMCCC_TYPE_SHIFT)
#define FAST_SMC(call)		(FAST_SMC_CALL | call)
#define SMC_CALL_OK		0x0

#define SSAPI_ENABLE_L2_CACHE	0x01000002
#define SSAPI_DISABLE_L2_CACHE	0x01000003
#define SSAPI_SLEEP_DORMANT	0x01004000
#define SSAPI_SLEEP_DEEP	0x01004001

#endif /* BCM_OPTEE_SMC_H */
