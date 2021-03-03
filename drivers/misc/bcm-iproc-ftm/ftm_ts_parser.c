/*
* Copyright (C) 2016 Broadcom
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

#include <linux/module.h>
#include <linux/printk.h>

static const unsigned char track23cset[32] = {
0x00,  '1',  '2',  0x00,  '4',  0x00,  0x00,  '7',
'8',  0x00,  0x00,  ';',  0x00,  '=',  '>',  0x00,
'0',  0x00,  0x00,  '3',  0x00,  '5',  '6',  0x00,
0x00,  '9',  ':',  0x00,  '<',  0x00,  0x00,  '?',
};

static const unsigned char track1cset[128] = {
0x00,  '!',  '"',  0x00,  '$',  0x00,  0x00,  '\'',
'(',  0x00,  0x00,  '+',  0x00,  '-',  '.',  0x00,
'0',  0x00,  0x00,  '3',  0x00,  '5',  '6',  0x00,
0x00,  '9',  ':',  0x00,  '<',  0x00,  0x00,  '?',
'@',  0x00,  0x00,  'C',  0x00,  'E',  'F',  0x00,
0x00,  'I',  'J',  0x00,  'L',  0x00,  0x00,  'O',
0x00,  'Q',  'R',  0x00,  'T',  0x00,  0x00,  'W',
'X',  0x00,  0x00,  '[',  0x00,  ']',  '^',  0x00,
' ',  0x00,  0x00,  '#',  0x00,  '%',  '&',  0x00,
0x00,  ')',  '*',  0x00,  ',',  0x00,  0x00,  '/',
0x00,  '1',  '2',  0x00,  '4',  0x00,  0x00,  '7',
'8',  0x00,  0x00,  ';',  0x00,  '=',  '>',  0x00,
0x00,  'A',  'B',  0x00,  'D',  0x00,  0x00,  'G',
'H',  0x00,  0x00,  'K',  0x00,  'M',  'N',  0x00,
'P',  0x00,  0x00,  'S',  0x00,  'U',  'V',  0x00,
'X',   'Y',  'Z',  0x00,  '\\',  0x00,  0x00,  '-',
};

static int ftm_convert_bs(unsigned short *p, int len)
{
	int bitlen = p[0];
	bool skip = false;
	int i, j = 0;

	for (i = 0; i < len - 1; i++) {
		if (skip) {
			skip = false;
			continue;
		}
		if ((p[i] < (bitlen * 70 / 100))
				&& (p[i + 1] < (bitlen * 70 / 100))
				&& ((p[i] + p[i + 1]) < (bitlen * 140 / 100))
		   ) {
			bitlen = bitlen * 80 / 100 +
				(p[i] + p[i + 1]) * 20 / 100;
			p[j++] = 1;
			skip = true;
		} else {
			if (p[i] < bitlen * 80 / 100)
				p[i + 1] -= (bitlen - p[i]) * 80 / 100;

			bitlen = bitlen * 80 / 100 + p[i] * 20 / 100;
			p[j++] = 0;
		}
	}
	return j;
}

static void ftm_convert_ts(unsigned short *p, int len)
{
	int i;

	for (i = 0; i < len - 1; i++) {
		if (p[i + 1] > p[i])
			p[i] = p[i + 1] - p[i];
		else
			p[i] = 0xffff + p[i + 1] - p[i];
	}
}

static int ftm_convert_char(unsigned short *p, int len, int ch,
		unsigned char *out)
{
	int i = 0, j = 0;
	int start = 0;
	unsigned char *pout = (unsigned char *)out;
	unsigned char c;

	while (i < len) {
		if (p[i]) {
			start = i;
			break;
		}
		i++;
	}

	if (ch == 0) {
		for (i = start; i < len - 7; i += 7) {
			c = (p[i + 6] << 6) + (p[i + 5] << 5) +
				(p[i + 4] << 4) + (p[i + 3] << 3) +
				(p[i + 2] << 2) + (p[i + 1] << 1) + (p[i]);

			c &= 0x7f;

			if (track1cset[c]) {
				pout[j] = track1cset[c];
				p[j++] = c;
			}
		}
	} else {
		for (i = start; i < len - 5; i += 5) {
			c = (p[i + 4] << 4) + (p[i + 3] << 3) +
				(p[i + 2] << 2) + (p[i + 1] << 1) + (p[i]);
			c &= 0x1f;

			if (track23cset[c]) {
				pout[j] = track23cset[c];
				p[j++] = c;
			}
		}
	}
	pout[j-1] = 0;

	return j-1;
}

static int ftm_check_lrc(unsigned short *p, int size, int ch)
{
	unsigned char startb;
	unsigned char endb;
	unsigned char lrc_p = 0;
	unsigned char lrc = 0;
	int bits;
	int i, j;

	if (ch == 0) {
		startb = 0x45;
		endb = 0x1f;
		bits = 6;
	} else {
		startb = 0xb;
		endb = 0x1f;
		bits = 4;
	}

	if (p[0] != startb) {
		pr_err("Failed to get begin mark 0x45/0xb, 0x%2x\n", p[0]);
		return -EINVAL;
	}

	for (i = 0; i < (size - 1); i++) {
		lrc = lrc ^ p[i];
		if (p[i] == endb)
			break;
	}

	if (p[i] != endb) {
		pr_err("Failed to get end mark 0x1f(?)\n");
		return -EINVAL;
	}

	if (i == (size - 1)) {
		pr_err("lrc not found\n");
		return -EINVAL;
	}

	lrc_p = 0;
	for (j = 0; j < bits; j++)
		lrc_p ^= (lrc & (1 << j)) >> j;

	lrc_p ^= 1;

	lrc &= (1 << bits) - 1;
	lrc = lrc | (lrc_p << bits);

	if (p[i + 1] != lrc) {
		pr_err("lrc not match %x\n", p[i + 1]);
		pr_err("calculated: 0x%x, P: 0x%x\n", lrc, lrc_p);
		return -EINVAL;
	}
	return 0;
}

int ftm_decode(unsigned short *p, unsigned int size, int chan,
				unsigned char *out, unsigned int *outlen)
{
	int len;
	int checkr;

	ftm_convert_ts(p, size);
	len = ftm_convert_bs(p, size - 1);
	len = ftm_convert_char(p, len, chan, out);
	checkr =  ftm_check_lrc(p, len, chan);
	out[len] = 0;
	*outlen = len;

	return checkr;
}
