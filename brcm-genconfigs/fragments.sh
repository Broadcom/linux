#!/bin/sh

# All supported defconfigs. Add new defconfigs to list for the proper
# architecture.
arm64_defconfigs="iproc_defconfig"

# Defines the fragments that apply to each defconfig. Variable name must be the
# same as the defconfig file.
iproc_defconfig="base-arm64 stingray \
    ata blk bpf                          \
    clocksource crypto crashdump         \
    dbg dma                              \
    edac efi fs hba hwmon                \
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

vk_defconfig="base-arm64 valkyrie \
    blk                                  \
    clocksource                          \
    dbg dma                              \
    efi                                  \
    mailbox                              \
    perf profiling                       \
    uio                                  \
    vfio vhost watchdog"
