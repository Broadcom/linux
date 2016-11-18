#!/bin/bash
#*****************************************************************************
#  Copyright 2014-2015 Broadcom Corporation.  All rights reserved.
#
#  Unless you and Broadcom execute a separate written software license
#  agreement governing use of this software, this software is licensed to you
#  under the terms of the GNU General Public License version 2, available at
#  http://www.gnu.org/licenses/old-license/gpl-2.0.html (the "GPL").
#
#  Notwithstanding the above, under no circumstances may you combine this
#  software in any way with any other Broadcom software provided under a
#  license other than the GPL, without Broadcom's express prior written
#  consent.
#
#*****************************************************************************

#*****************************************************************************
##
#  @file    update_defconfig.sh
#
#  @brief   Script to update the defconfig files based on the config bits
#           located in this directory.
#
#  @note    Usage: ./brcm-genconfigs/update_defconfig.sh
#
#*****************************************************************************

# ---- Private Variables -----------------------------------------------------

#*****************************************************************************
###
#  @brief   Verbose flag
#
#  @note    Call with V=1 to debug this script.
#
#*****************************************************************************
V=${V-0}

#*****************************************************************************
###
#  @brief   Linux Architecture name
#
#  default to arm64
#
#*****************************************************************************
ARCH=arm64

#*****************************************************************************
###
#  @brief   SEARCH_PATH
#
#  @note    Where to search for config files (in order of precedence)
#
#*****************************************************************************
declare -a SEARCH_PATH=(
	brcm-genconfigs
)

# ==== Private Functions ============================================

#*****************************************************************************
###
#  @brief   Entry point for the script.
#
#  @return  None
#
#  @note    The entry point is called at the end of the file, because we want
#           Bash to parse the entire file before executing anything.
#
#*****************************************************************************
main() {
	local iproc_base
	local iproc_extra

	# Show all commands (for debugging).
	if [ $V -ne 0 ]; then
		set -x
	fi

	# iproc_defconfig, for both NS2 and Stingray
	# for uses when NS2/Stingray run standalone without NB host
	iproc_base="base-arm64 clocksource dbg"
	iproc_extra="blk net dma fs i2c iomux mailbox md misc mmc mtd mtd-spi rng spi virtio watchdog pwm usb"
	do_update iproc $iproc_base $iproc_extra

	# sr_nitro_lite_defconfig. for PAXC/Nitro bring up on Stingray palladium
	do_update sr_nitro_lite $iproc_base blk fs net

	#Cleanup
	make ARCH=${ARCH} mrproper > /dev/null
	rm defconfig
	echo "Done."
}

#*****************************************************************************
###
#  @brief   Search config bits, call merge-config and save the defconfig.
#
#  @param   $1     Defconfig file name (without path or _defconfig)
#  @param   $[2-*] Config bits file names (without path or .cfg)
#
#  @return  0 on success, 1 on error.
#
#*****************************************************************************
do_update() {
	local defcfg	# Destination file
	local arg		# Argument iterator
	local path		# Search path iterator
	local tst		# Test path
	local found	# File found flag
	local -a bits	# Array of configuration files found.

	if [ $# -lt 1 ]; then
		echo "Error in $FUNCNAME: missing arguments!" >&2
		return 1
	fi

	defcfg=arch/${ARCH}/configs/${1}_defconfig
	shift

	for arg in $*; do
		found=0
		for path in ${SEARCH_PATH[@]}; do
			tst=${path}/${arg}.cfg
			if [ -r $tst ]; then
				bits[${#bits[*]}]=$tst # Append to bits array
				found=1
				break
			fi
		done
		if [ $found -ne 1 ]; then
			echo "Error: cannot find config bits for $arg in \"$*\"" >&2
			return 1
		fi
	done

	if ARCH=${ARCH} scripts/kconfig/merge_config.sh -m ${bits[@]}; then
		rm defconfig
		echo "Creating minimal defconfig from .config"
		make ARCH=${ARCH} savedefconfig
		cp -v defconfig $defcfg # Save defconfig
	else
		return 1
	fi
	return 0
}

#*****************************************************************************
##
#  @brief   Entry (and exit) point for the script.
#
#*****************************************************************************
main
