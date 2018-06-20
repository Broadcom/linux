// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>

#define OMEGA_ETIMER_MCTRL0_OFFSET			0x00
#define OMEGA_ETIMER_MCTRL0_RUN_BIT			0
#define OMEGA_ETIMER_MCTRL1_OFFSET			0x04
#define OMEGA_ETIMER_MCTRL1_EN_OUTPUT_BIT		6
#define OMEGA_ETIMER_MCTRL1_INV_POLARITY_BIT		5
#define OMEGA_ETIMER_MCTRL1_PWM_ENABLE_BIT		4
#define OMEGA_ETIMER_PERIOD_OFFSET			0x08
#define OMEGA_ETIMER_CONTINUOUS_MODE_BIT		8
#define OMEGA_ETIMER_VAL0_OFFSET			0x0c
#define OMEGA_ETIMER_VAL1_OFFSET			0x10
#define OMEGA_ETIMER_VAL2_OFFSET			0x14
#define OMEGA_ETIMER_VAL3_OFFSET			0x18
#define OMEGA_ETIMER_INITCNT_OFFSET			0x1c


#define OMEGA_PWM_PERIOD_MAX				0xffff
#define OMEGA_PWM_DUTY_CYCLE_MAX			0xffff

struct omega_pwm_chip {
	struct pwm_chip chip;
	void __iomem *base;
	struct clk *clk;
};

static inline struct omega_pwm_chip *to_omega_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct omega_pwm_chip, chip);
}

static inline u32 omega_pwm_readl(struct omega_pwm_chip *opc,
				  unsigned int hwpwm, unsigned int offset)
{
	return readl(opc->base + hwpwm * 0x400 + offset);
}

static inline void omega_pwm_writel(struct omega_pwm_chip *opc,
				    unsigned int hwpwm, unsigned int offset,
				    u32 value)
{
	writel(value, opc->base + hwpwm * 0x400 + offset);
}

static void omega_pwm_set_bit(struct omega_pwm_chip *opc, unsigned int hwpwm,
			      unsigned int offset, u32 bit, bool set)
{
	u32 data;

	data = omega_pwm_readl(opc, hwpwm, offset);
	if (set)
		data |= BIT(bit);
	else
		data &= ~BIT(bit);

	omega_pwm_writel(opc, hwpwm, offset, data);
}

static void omega_pwm_enable(struct omega_pwm_chip *opc, unsigned int channel,
			     bool enable)
{
	omega_pwm_set_bit(opc, channel, OMEGA_ETIMER_MCTRL0_OFFSET,
			  OMEGA_ETIMER_MCTRL0_RUN_BIT, enable);
}

static void omega_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				struct pwm_state *state)
{
	struct omega_pwm_chip *opc = to_omega_pwm_chip(chip);
	u64 tmp, rate;
	u32 value;

	rate = clk_get_rate(opc->clk);
	if (!rate) {
		dev_err(chip->dev, "Failed to get clock rate");
		return;
	}
	rate /= 4;

	value = omega_pwm_readl(opc, pwm->hwpwm, OMEGA_ETIMER_MCTRL0_OFFSET);

	if (value & BIT(OMEGA_ETIMER_MCTRL0_RUN_BIT))
		state->enabled = true;
	else
		state->enabled = false;

	value = omega_pwm_readl(opc, pwm->hwpwm, OMEGA_ETIMER_MCTRL1_OFFSET);

	if (value & BIT(OMEGA_ETIMER_MCTRL1_INV_POLARITY_BIT))
		state->polarity = PWM_POLARITY_INVERSED;
	else
		state->polarity = PWM_POLARITY_NORMAL;

	value = omega_pwm_readl(opc, pwm->hwpwm, OMEGA_ETIMER_VAL1_OFFSET);
	tmp = value & OMEGA_PWM_PERIOD_MAX;
	state->period = div64_u64(tmp, rate) * NSEC_PER_SEC;

	value = omega_pwm_readl(opc, pwm->hwpwm, OMEGA_ETIMER_VAL2_OFFSET);
	tmp = value & OMEGA_PWM_DUTY_CYCLE_MAX;
	state->duty_cycle = div64_u64(tmp, rate) * NSEC_PER_SEC;
}

static int omega_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			   struct pwm_state *state)
{
	struct omega_pwm_chip *opc = to_omega_pwm_chip(chip);
	struct pwm_state cur_state;
	u64 rate, tmp;

	pwm_get_state(pwm, &cur_state);

	rate = clk_get_rate(opc->clk);
	if (!rate) {
		dev_err(chip->dev, "Failed to get clock rate");
		return -EINVAL;
	}
	/* Internal fixed divider within eTimer block */
	rate /= 4;

	omega_pwm_enable(opc, pwm->hwpwm, false);

	if (state->polarity != cur_state.polarity)
		omega_pwm_set_bit(opc, pwm->hwpwm, OMEGA_ETIMER_MCTRL1_OFFSET,
				  OMEGA_ETIMER_MCTRL1_INV_POLARITY_BIT,
				  state->polarity);

	if (state->period != cur_state.period) {
		tmp = state->period * rate;
		do_div(tmp, NSEC_PER_SEC);
		omega_pwm_writel(opc, pwm->hwpwm, OMEGA_ETIMER_VAL1_OFFSET,
				 tmp & OMEGA_PWM_PERIOD_MAX);
	}

	if (state->duty_cycle != cur_state.duty_cycle) {
		tmp = state->duty_cycle * rate;
		do_div(tmp, NSEC_PER_SEC);
		omega_pwm_writel(opc, pwm->hwpwm, OMEGA_ETIMER_VAL2_OFFSET,
				 tmp & OMEGA_PWM_DUTY_CYCLE_MAX);
	}

	if (state->enabled)
		omega_pwm_enable(opc, pwm->hwpwm, true);

	return 0;
}

static const struct pwm_ops omega_pwm_ops = {
	.apply = omega_pwm_apply,
	.get_state = omega_pwm_get_state,
};

static int omega_pwm_probe(struct platform_device *pdev)
{
	struct omega_pwm_chip *opc;
	struct resource *res;
	unsigned int i;
	int ret;

	opc = devm_kzalloc(&pdev->dev, sizeof(*opc), GFP_KERNEL);
	if (!opc)
		return -ENOMEM;

	platform_set_drvdata(pdev, opc);

	opc->chip.dev = &pdev->dev;
	opc->chip.ops = &omega_pwm_ops;
	opc->chip.base = -1;
	opc->chip.npwm = 4;
	opc->chip.of_xlate = of_pwm_xlate_with_flags;
	opc->chip.of_pwm_n_cells = 3;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	opc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(opc->base))
		return PTR_ERR(opc->base);

	opc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(opc->clk)) {
		dev_err(&pdev->dev, "failed to get clock: %ld\n",
			PTR_ERR(opc->clk));
		return PTR_ERR(opc->clk);
	}

	ret = clk_prepare_enable(opc->clk);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to enable clock: %d\n", ret);
		return ret;
	}

	for (i = 0; i < opc->chip.npwm; i++) {
		/* Set normal polarity for all channels */
		omega_pwm_set_bit(opc, i, OMEGA_ETIMER_MCTRL1_OFFSET,
				  OMEGA_ETIMER_MCTRL1_INV_POLARITY_BIT,
				  PWM_POLARITY_NORMAL);
		/* Enable output */
		omega_pwm_set_bit(opc, i, OMEGA_ETIMER_MCTRL1_OFFSET,
				  OMEGA_ETIMER_MCTRL1_EN_OUTPUT_BIT, 1);
		/* Enable PWM */
		omega_pwm_set_bit(opc, i, OMEGA_ETIMER_MCTRL1_OFFSET,
				  OMEGA_ETIMER_MCTRL1_PWM_ENABLE_BIT, 1);
		/* Set PWM continuous mode */
		omega_pwm_set_bit(opc, i, OMEGA_ETIMER_PERIOD_OFFSET,
				  OMEGA_ETIMER_CONTINUOUS_MODE_BIT, 1);
	}

	ret = pwmchip_add(&opc->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		clk_disable_unprepare(opc->clk);
	}

	return ret;
}

static int omega_pwm_remove(struct platform_device *pdev)
{
	struct omega_pwm_chip *opc = platform_get_drvdata(pdev);
	int ret, i;

	ret = pwmchip_remove(&opc->chip);
	if (ret)
		return ret;

	for (i = 0; i < opc->chip.npwm; i++)
		omega_pwm_enable(opc, i, false);
	clk_disable_unprepare(opc->clk);

	return 0;
}

static const struct of_device_id bcm_omega_pwm_dt[] = {
	{ .compatible = "brcm,omega-pwm" },
	{ },
};
MODULE_DEVICE_TABLE(of, bcm_omega_pwm_dt);

static struct platform_driver omega_pwm_driver = {
	.driver = {
		.name = "bcm-omega-pwm",
		.of_match_table = bcm_omega_pwm_dt,
	},
	.probe = omega_pwm_probe,
	.remove = omega_pwm_remove,
};
module_platform_driver(omega_pwm_driver);

MODULE_AUTHOR("Li Jin <li.jin@broadcom.com>");
MODULE_DESCRIPTION("Broadcom Omega PWM driver");
MODULE_LICENSE("GPL v2");
