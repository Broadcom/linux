/*
 * Copyright 2017 Broadcom
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
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/rtc.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/arm-smccc.h>

#define ARM_SMCCC_OWNER_TRUSTED_OS_SECIPS (ARM_SMCCC_OWNER_TRUSTED_OS + 5)

#define FN_ID(fun_num)	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL,\
					   ARM_SMCCC_SMC_32,\
					   ARM_SMCCC_OWNER_TRUSTED_OS_SECIPS, \
					   fun_num)

#define SMCID_BBL_POWER_GOOD	FN_ID(0x4)
#define SMCID_RTC_GET_TIME	FN_ID(0x5)
#define SMCID_RTC_SET_TIME	FN_ID(0x6)
#define SMCID_RTC_ALARM_ENA	FN_ID(0x7)
#define SMCID_RTC_GET_ALARM	FN_ID(0x8)
#define SMCID_RTC_SET_ALARM	FN_ID(0x9)

struct bcm_iproc_rtc {
	struct device *dev;
	struct rtc_device  *rtc;
	int periodic_irq;
};

static irqreturn_t rtc_sec_smc_irq(int irq, void *pdev_data)
{
	struct bcm_iproc_rtc *rtc = pdev_data;

	rtc_update_irq(rtc->rtc, 1, (RTC_IRQF | RTC_PF));

	return IRQ_HANDLED;
}

static int rtc_read_time_smc(struct device *dev, struct rtc_time *tm)
{
	u32 seconds = 0;
	int ret;
	struct arm_smccc_res res;

	arm_smccc_smc(SMCID_RTC_GET_TIME, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(dev, "rtc get time err :%lx\n", res.a0);
		return res.a0;
	}

	seconds = res.a1;
	rtc_time_to_tm(seconds, tm);
	ret = rtc_valid_tm(tm);

	return ret;
}

static int rtc_set_time_smc(struct device *dev, struct rtc_time *tm)
{
	unsigned long t;
	struct arm_smccc_res res;

	rtc_tm_to_time(tm, &t);
	arm_smccc_smc(SMCID_RTC_SET_TIME, t, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(dev, "%s failed(%lu) for t:%lx\n", __func__, res.a0, t);

	return res.a0;
}

static int rtc_alarm_irq_enable_smc(struct device *dev, u32 enabled)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMCID_RTC_ALARM_ENA, enabled, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(dev, "%s failed ret:%lx", __func__, res.a0);

	return res.a0;
}

static int rtc_read_alarm_smc(struct device *dev, struct rtc_wkalrm *alm)
{
	u32 seconds = 0;
	struct arm_smccc_res res;
	struct bcm_iproc_rtc *rtc = dev_get_drvdata(dev);

	arm_smccc_smc(SMCID_RTC_GET_ALARM, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0) {
		dev_err(rtc->dev, "%s failed(%lx)", __func__, res.a0);
		goto err;
	}

	seconds = res.a1;
	alm->pending = res.a2;
	rtc_time_to_tm(seconds, &alm->time);
	alm->enabled = alm->pending && device_may_wakeup(dev);
err:
	return res.a0;
}

static int rtc_set_alarm_smc(struct device *dev,
						struct rtc_wkalrm *alm)
{
	unsigned long seconds;
	struct arm_smccc_res res;
	struct bcm_iproc_rtc *rtc = dev_get_drvdata(dev);
	/*
	 * Setting RTC match to the time
	 * for alarm interrupt
	 */
	rtc_tm_to_time(&alm->time, &seconds);
	arm_smccc_smc(SMCID_RTC_SET_ALARM, seconds, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		dev_err(rtc->dev, "%s failed(%lx)", __func__, res.a0);

	return res.a0;
}

static struct rtc_class_ops rtc_ops = {
	.read_time		= rtc_read_time_smc,
	.set_time		= rtc_set_time_smc,
	.alarm_irq_enable	= rtc_alarm_irq_enable_smc,
	.read_alarm		= rtc_read_alarm_smc,
	.set_alarm		= rtc_set_alarm_smc,
};

static int rtc_check_power(void)
{
	struct arm_smccc_res res;

	arm_smccc_smc(SMCID_BBL_POWER_GOOD, 0, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0)
		return res.a0;

	return res.a1;
}

static int rtc_probe(struct platform_device *pdev)
{
	struct bcm_iproc_rtc *rtc;
	int ret;

	ret = rtc_check_power();
	if (ret) {
		dev_err(&pdev->dev, "bbl is not powered up:%d", ret);
		return -ENODEV;
	}

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;
	platform_set_drvdata(pdev, rtc);
	rtc->dev = &pdev->dev;

	rtc->periodic_irq = platform_get_irq(pdev, 0);
	if (rtc->periodic_irq < 0) {
		dev_err(rtc->dev, "RTC interrupt not defined\n");
		return rtc->periodic_irq;
	}

	ret = devm_request_irq(rtc->dev, rtc->periodic_irq, rtc_sec_smc_irq, 0,
			       "iproc_periodic_rtc", rtc);
	if (ret)
		return ret;

	rtc->rtc = devm_rtc_device_register(rtc->dev, pdev->name,
					    &rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc))
		return PTR_ERR(rtc->rtc);

	dev_info(rtc->dev, "rtc_probe done\n");
	return 0;
}

static const struct of_device_id rtc_of_match[] = {
	{.compatible = "brcm,iproc-rtc",},
	{ }
};

static struct platform_driver rtc_driver = {
	.probe		= rtc_probe,
	.driver		= {
		.name = "iproc-rtc",
		.of_match_table = rtc_of_match
	},
};

module_platform_driver(rtc_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom iProc RTC Driver");
MODULE_LICENSE("GPL v2");

