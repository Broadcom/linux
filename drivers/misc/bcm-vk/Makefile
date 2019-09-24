# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Broadcom VK driver
#

#
# Note: define BCM_VK_LEGACY_API for older version of kernels,
#       eg, adding "ccflags-y :=-DBCM_VK_LEGACY_API=1"
#
obj-$(CONFIG_BCM_VK) += bcm_vk.o
bcm_vk-objs := bcm_vk_dev.o bcm_vk_msg.o bcm_vk_pci.o bcm_vk_sg.o bcm_vk_tty.o
