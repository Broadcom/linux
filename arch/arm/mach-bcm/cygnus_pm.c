/*
 * Copyright (C) 2016-2017 Broadcom
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

#include <asm/cacheflush.h>
#include <asm/mach/bcm_optee_smc.h>
#include <asm/outercache.h>
#include <asm/suspend.h>
#include <linux/clocksource.h>
#include <linux/io.h>
#include <linux/suspend.h>
#include "cygnus_pm.h"

#define SCU_CONTROL               0x0
#define SCU_CONFIG                0x4
#define SCU_POWER_STATUS          0x8
#define SCU_FILTER_START          0x40
#define SCU_FILTER_END            0x44
#define SCU_ACCESS_CONTROL        0x50
#define SCU_SECURE_ACCESS         0x54

#define CRU_STATUS                0x20
#define CRU_STATUS_DETECTED_WFI   BIT(2)

#define CRMU_POWER_CONFIG         0x14

enum cygnus_power_status {
	CYGNUS_PM_STATE_RUN = 0,
	CYGNUS_PM_STATE_SLEEP,
	CYGNUS_PM_STATE_DEEPSLEEP,
	CYGNUS_PM_STATE_END
};

/* Values for CRMU_POWER_CONFIG register. */
enum cygnus_crmu_power_cfg {
	CRMU_RUN        = 0x0,
	CRMU_SLEEP      = 0x2,
	CRMU_DEEPSLEEP  = 0x3
};

/* Values for SCU_POWER_STATUS register. */
enum cygnus_scu_power_status {
	SCU_POWER_STATUS_RUN   = 0xfffffffc,
	SCU_POWER_STATUS_SLEEP = 0xffffffff,
};

/* Managed by brcm,cygnus-pm device. */
static struct cygnus_pm *cygnus_pm;

/* Note: the register order is important! */
static struct {
	u32 scu_control;
	u32 scu_config;
	u32 scu_filter_start;
	u32 scu_filter_end;
	u32 scu_access_control;
	u32 scu_secure_access;
} saved_common_regs;

static const char * const cygnus_pm_states_str[] = {
	[CYGNUS_PM_STATE_RUN] = "RUN",
	[CYGNUS_PM_STATE_SLEEP] = "SLEEP",
	[CYGNUS_PM_STATE_DEEPSLEEP] = "DEEPSLEEP",
	[CYGNUS_PM_STATE_END] = "UNKNOWN",
};

/*
 * Save registers on suspend that must be restored on resume.
 */
static void cygnus_pm_save_common_regs(void)
{
	saved_common_regs.scu_control = ioread32(cygnus_pm->scu + SCU_CONTROL);
	saved_common_regs.scu_config = ioread32(cygnus_pm->scu + SCU_CONFIG);
	saved_common_regs.scu_filter_start =
		ioread32(cygnus_pm->scu + SCU_FILTER_START);
	saved_common_regs.scu_filter_end =
		ioread32(cygnus_pm->scu + SCU_FILTER_END);
	saved_common_regs.scu_access_control =
		ioread32(cygnus_pm->scu + SCU_ACCESS_CONTROL);
	saved_common_regs.scu_secure_access =
		ioread32(cygnus_pm->scu + SCU_SECURE_ACCESS);
}

static void cygnus_pm_restore_common_regs(void)
{
	iowrite32(saved_common_regs.scu_control, cygnus_pm->scu + SCU_CONTROL);
	iowrite32(saved_common_regs.scu_config, cygnus_pm->scu + SCU_CONFIG);
	iowrite32(saved_common_regs.scu_filter_start,
		cygnus_pm->scu + SCU_FILTER_START);
	iowrite32(saved_common_regs.scu_filter_end,
		cygnus_pm->scu + SCU_FILTER_END);
	iowrite32(saved_common_regs.scu_access_control,
		cygnus_pm->scu + SCU_ACCESS_CONTROL);
	iowrite32(saved_common_regs.scu_secure_access,
		cygnus_pm->scu + SCU_SECURE_ACCESS);
}

/*
 * On a successful sleep & wake, the CPU will eventually continue from after the
 * cpu_suspend() call.
 */
static int cygnus_pm_finish_switch(unsigned long sleep_state)
{
	struct arm_smccc_res res;
	phys_addr_t resume_addr = virt_to_phys(cygnus_pm_cpu_resume);
	unsigned int cmd;

	if (sleep_state == CYGNUS_PM_STATE_DEEPSLEEP)
		cmd = FAST_SMC(SSAPI_SLEEP_DEEP);
	else
		cmd = FAST_SMC(SSAPI_SLEEP_DORMANT);

	arm_smccc_smc(cmd, resume_addr, 0, 0, 0, 0, 0, 0, &res);

	/*
	 * If the CPU gets to this point via the normal execution path,
	 * we have a wfi drop-through situation.
	 */
	WARN(1, "A9 exited WFI on sleep/deepsleep entry path\n");

	return -EINTR;
}

static const char *cygnus_pm_get_state_str_by_id(
	enum cygnus_power_status stateid)
{
	if (stateid <= CYGNUS_PM_STATE_END)
		return cygnus_pm_states_str[stateid];
	else
		return NULL;
}

static void cygnus_pm_set_pwrctrl_state(enum cygnus_power_status state)
{
	u32 pm_val;

	switch (state) {
	case CYGNUS_PM_STATE_RUN:
		pm_val = CRMU_RUN;
		break;
	case CYGNUS_PM_STATE_SLEEP:
		pm_val = CRMU_SLEEP;
		break;
	case CYGNUS_PM_STATE_DEEPSLEEP:
		pm_val = CRMU_DEEPSLEEP;
		break;
	default:
		pr_err("Invalid power control state\n");
		return;
	}

	pr_info("Set PWRCTRLO to %d(%s)\n", pm_val,
		cygnus_pm_get_state_str_by_id(state));

	iowrite32(pm_val, cygnus_pm->crmu + CRMU_POWER_CONFIG);
}

static void cygnus_pm_set_scu_status(enum cygnus_power_status state)
{
	u32 pm_val;

	if (state == CYGNUS_PM_STATE_RUN)
		pm_val = SCU_POWER_STATUS_RUN;
	else
		pm_val = SCU_POWER_STATUS_SLEEP;

	pr_info("Set SCU power status to 0x%08x\n", pm_val);
	iowrite32(pm_val, cygnus_pm->scu + SCU_POWER_STATUS);
}

/* This is the last step to enter new power mode */
static void cygnus_pm_prepare_enter_pm_mode(unsigned long state)
{
	u32 status;

	cygnus_pm_set_pwrctrl_state(state);
	cygnus_pm_set_scu_status(CYGNUS_PM_STATE_SLEEP);

	status = ioread32(cygnus_pm->cru + CRU_STATUS);
	iowrite32(status & ~CRU_STATUS_DETECTED_WFI,
		cygnus_pm->cru + CRU_STATUS);

	dsb();
	isb();
}

static void cygnus_pm_dump_resume_entry(void)
{
	uint32_t reg_phy_addr = (uint32_t) cygnus_pm_cpu_resume;

	pr_debug("cygnus_pm_cpu_resume() vaddr = 0x%08x\n",
		reg_phy_addr);
	pr_debug("cygnus_pm_cpu_resume() paddr = 0x%08x\n",
		  virt_to_phys((void *)reg_phy_addr));
}

/* Enable or disable the L2C. */
static void cygnus_pm_l2c_enable(bool enable)
{
	struct arm_smccc_res res;
	unsigned int cmd;

	if (enable) {
		cmd = FAST_SMC(SSAPI_ENABLE_L2_CACHE);
		arm_smccc_smc(cmd, 0, ~0, 0, 0, 0, 0, 0, &res);
	} else {
		cmd = FAST_SMC(SSAPI_DISABLE_L2_CACHE);
		arm_smccc_smc(cmd, 0, 0, 0, 0, 0, 0, 0, &res);
	}
}

/* Suspend to Ram or standby */
static void cygnus_pm_soc_enter_sleep(enum cygnus_power_status state)
{
	unsigned long irq_flags;

	pr_info("Entering %s\n", cygnus_pm_get_state_str_by_id(state));

	cygnus_pm_dump_resume_entry();
	cygnus_pm_save_common_regs();

	local_irq_save(irq_flags);

	cygnus_pm_prepare_enter_pm_mode(state);

	pr_info("Flushing L1 cache, L2 cache, then suspending...\n");

	/* Flush D-cache and I-cache. */
	flush_cache_all();

	/* Clean, invalidate, disable L2C. */
	cygnus_pm_l2c_enable(false);

	cpu_suspend(state, cygnus_pm_finish_switch);

	/*
	 * Enable clocksource early so that the uart which requires it is
	 * functional. This is required when 'no_console_suspend' is true.
	 */
	clocksource_resume();

	pr_info("... resuming\n");

	/* Restore L2C configuration and re-enable. */
	cygnus_pm_l2c_enable(true);

	local_irq_restore(irq_flags);

	cygnus_pm_set_scu_status(CYGNUS_PM_STATE_RUN);
	cygnus_pm_restore_common_regs();
	cygnus_pm_set_pwrctrl_state(CYGNUS_PM_STATE_RUN);
}

static int cygnus_pm_valid(suspend_state_t pm_state)
{
	return (pm_state == PM_SUSPEND_STANDBY) || (pm_state == PM_SUSPEND_MEM);
}

static int cygnus_pm_enter(suspend_state_t state)
{
	int ret = 0;

	if (state == PM_SUSPEND_STANDBY)
		cygnus_pm_soc_enter_sleep(CYGNUS_PM_STATE_SLEEP);
	else if (state == PM_SUSPEND_MEM)
		cygnus_pm_soc_enter_sleep(CYGNUS_PM_STATE_DEEPSLEEP);
	else {
		pr_err("PM state %d not supported\n", state);
		ret = -EINVAL;
	}

	return ret;
}

static const struct platform_suspend_ops cygnus_pm_suspend_ops = {
	.enter = cygnus_pm_enter,
	.valid = cygnus_pm_valid,
};

void __init cygnus_pm_init(void)
{
	cygnus_pm = cygnus_pm_device_init();
	if (!cygnus_pm) {
		pr_err("PM device not registered\n");
		return;
	}

	suspend_set_ops(&cygnus_pm_suspend_ops);
}
