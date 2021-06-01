#!/bin/sh

# All supported defconfigs. Add new defconfigs to list for the proper
# architecture.
arm64_defconfigs="vk_defconfig"

# Defines the fragments that apply to each defconfig. Variable name must be the
# same as the defconfig file.
vk_defconfig="base-arm64 valkyrie \
    blk                                  \
    clocksource                          \
    dbg dma                              \
    hungtask                             \
    panic perf ppp profiling             \
    softlockup                           \
    uio                                  \
    vfio watchdog"
