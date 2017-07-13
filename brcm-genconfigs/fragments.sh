#!/bin/sh

# All supported defconfigs. Add new defconfigs to list for the proper
# architecture.
arm_defconfigs="bcm_cygnus_defconfig"
arm64_defconfigs="iproc_defconfig"
arm64_defconfigs+="bcm_omega_defconfig"

# Defines the fragments that apply to each defconfig. Variable name must be the
# same as the defconfig file.
bcm_cygnus_defconfig="base-arm32 cygnus  \
    blk bt                               \
    dbg dma dte                          \
    fs i2c                               \
    iio                                  \
    input                                \
    lcd mailbox misc mmc mtd             \
    net                                  \
    optee                                \
    pwm                                  \
    regulator rng                        \
    sound spi stmpe trace                \
    usb usbd                             \
    watchdog wifi                        \
    media"

bcm_omega_defconfig="base-arm64          \
    blk bt                               \
    dbg dma dte                          \
    fs i2c                               \
    iio                                  \
    input                                \
    lcd mailbox media misc mmc mtd       \
    net                                  \
    optee                                \
    pwm                                  \
    regulator rng                        \
    sound spi stmpe trace                \
    usb usbd                             \
    watchdog wifi"

iproc_defconfig="base-arm64 ns2 stingray \
    ata blk bpf                          \
    clocksource crypto crashdump         \
    dbg dma                              \
    efi fs hba                           \
    i2c infiniband iomux                 \
    mailbox md misc mmc mmu mtd mtd-spi  \
    net nvme                             \
    pci perf profiling pwm               \
    regulator rng rtc                    \
    spi                                  \
    uio usb usbd                         \
    vfio watchdog"
