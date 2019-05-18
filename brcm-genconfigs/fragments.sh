#!/bin/sh

# All supported defconfigs. Add new defconfigs to list for the proper
# architecture.
arm_defconfigs="bcm_cygnus_defconfig"
arm64_defconfigs="iproc_defconfig bcm_omega_defconfig"

# Defines the fragments that apply to each defconfig. Variable name must be the
# same as the defconfig file.
bcm_cygnus_defconfig="base-arm32 cygnus  \
    blk bt camera                        \
    dbg dma drm dte                      \
    fs i2c                               \
    iio                                  \
    input                                \
    lcd mailbox media misc mmc mtd       \
    net                                  \
    optee                                \
    pci pwm                              \
    regulator rng                        \
    sound spi stmpe trace                \
    usb usbd                             \
    watchdog wifi                        \
    "

bcm_omega_defconfig="base-arm64 omega    \
    blk bt                               \
    clocksource                          \
    dbg dma dte                          \
    fs i2c                               \
    iio                                  \
    input iomux                          \
    lcd mailbox media misc mmc mtd       \
    net                                  \
    optee                                \
    pci perf pwm                         \
    regulator rng                        \
    sound spi stmpe switch trace         \
    usb usbd                             \
    watchdog wifi"

iproc_defconfig="base-arm64 ns2 stingray \
    ata blk bpf                          \
    clocksource crypto crashdump         \
    dbg dma                              \
    efi fs hba hwmon                     \
    i2c i2c-slave infiniband iomux       \
    iscsi                                \
    mailbox md misc mmc mtd mtd-spi      \
    net nvme                             \
    optee                                \
    pci perf profiling pwm               \
    regulator rng rtc                    \
    spi                                  \
    tc                                   \
    tmon                                 \
    uio usb usbd                         \
    vfio vhost virtio watchdog"
