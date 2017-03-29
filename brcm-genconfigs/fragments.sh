#!/bin/sh

# All supported defconfigs. Add new defconfigs to list for the proper
# architecture.
arm_defconfigs="bcm_cygnus_defconfig"
arm64_defconfigs="iproc_defconfig sr_nitro_lite_defconfig"

# Defines the fragments that apply to each defconfig. Variable name must be the
# same as the defconfig file.
bcm_cygnus_defconfig="base-arm32 blk dbg dma fs i2c mailbox misc mmc mtd \
    net pwm rng spi watchdog bt wifi regulator cygnus lcd sound trace stmpe \
    usb usbd input"
iproc_defconfig="base-arm64 clocksource dbg mmu blk net dma fs i2c iomux \
    mailbox md misc mmc mtd mtd-spi rng spi virtio watchdog pwm usb usbd perf \
    regulator rtc infiniband nvme uio crypto hba bpf vfio profiling input ata"
sr_nitro_lite_defconfig="base-arm64 clocksource dbg mmu blk fs net"
