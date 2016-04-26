/*
 * Copyright 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 *
 * ip_fast_csum() loosely derived from original arch/arm implementation
 */

#ifndef __ASM_ARM64_CHECKSUM_H
#define __ASM_ARM64_CHECKSUM_H

/*
 * This is a version of ip_compute_csum() optimized for IP headers,
 * which always checksum on 4 octet boundaries.
 */
static inline __sum16
ip_fast_csum(const void *iph, unsigned int ihl)
{
	u64 tmp, sum;

	__asm__ __volatile__ (
"	ldp	%0, %3, [%1], #16\n"
"	sub	%2, %2, #4\n"
"	adds	%0, %0, %3\n"
"1:	ldr	%w3, [%1], #4\n"
"	sub	%2, %2, #1\n"
"	adcs	%0, %0, %3\n"
"	tst	%2, #15\n"
"	bne	1b\n"
"	adc	%0, %0, xzr\n"
"	ror	%3, %0, #32\n"
"	add     %0, %0, %3\n"
"	lsr	%0, %0, #32\n"
"	ror	%w3, %w0, #16\n"
"	add     %w0, %w0, %w3\n"
"	lsr	%0, %0, #16\n"
	: "=r" (sum), "=r" (iph), "=r" (ihl), "=r" (tmp)
	: "1" (iph), "2" (ihl)
	: "cc", "memory");

	return (__force __sum16)(~sum);
}

#define ip_fast_csum ip_fast_csum
#include <asm-generic/checksum.h>

#endif	/* __ASM_ARM64_CHECKSUM_H */
