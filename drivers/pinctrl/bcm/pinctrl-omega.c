/*
 * Copyright (C) 2018 Broadcom
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

/*
 * This file contains the Broadcom omega pinctrl driver
 * The following pinconf options are supported:
 * pull up/down, drive strength, slew rate, input enable/disable,
 * input schmitt-trigger enable/disable.
 */

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/slab.h>

#include "../pinctrl-utils.h"

#define PIN(id) \
	{					\
		.name = "mfio" #id,		\
		.io_reg = 0x10 * id,		\
		.intr_reg = 0x4 + 0x10 * id,	\
		.clr_reg = 0x8 + 0x10 * id,	\
		.pad_reg = 0xc + 0x10 * id,	\
		.oeb_bit = 2,			\
		.in_bit = 1,			\
		.out_bit = 0,			\
		.mstat_bit = 5,			\
		.stat_bit = 4,			\
		.msk_bit = 3,			\
		.edge_bit = 2,			\
		.de_bit = 1,			\
		.type_bit = 0,			\
		.clr_bit = 0,			\
		.pull_bit = 6,			\
		.drv_bit = 3,			\
		.src_bit = 2,			\
		.ind_bit = 1,			\
		.hys_bit = 0,			\
	}
/*
 * struct omega_pin - Omega pin definition
 * @name:		Name of the pin
 * @io_reg:		Offset of the register holding input/output bits
 * @intr_reg:		Offset of the register holding interrupt configuration
 *			and status bits
 * @clr_reg:		Offset of the register holding interrupt clear bits
 * @pad_reg:		Offset of the register holding pad config bits
 * @oeb_bit:		Offset in @io_reg to control output enable(active_low)
 * @in_bit:		Offset in @io_reg for the input bit value
 * @out_bit:		Offset in @io_reg for the output bit value
 * @mstat_bit:		Offset in @intr_reg for interrupt masked status
 * @stat_bit:		Offset in @intr_reg for interrupt status
 * @msk_bit:		Offset in @intr_reg for interrupt mask
 * @edge_bit:		Offset in @intr_reg for edge/level selection
 * @de_bit:		Offset in @intr_reg for dual edge selection
 * @type_bit:		Offset in @intr_reg for interrupt type:
 *			1 - level; 0 - edge
 * @clr_bit:		Offset in @clr_reg for interrupt clear
 * @pull_bit:		Offset in @pad_reg for bias configuration
 * @drv_bit:		Offset in @pad_reg for drive strength configuration
 * @src_bit:		Offset in @pad_reg for slew rate configuration
 * @ind_bit:		Offset in @pad_reg for input disable/enable
 * @hys_bit:		Offset in @pad_reg for input schmitt enable

*/
struct omega_pin {
	const char *name;
	u32 io_reg;
	u32 intr_reg;
	u32 clr_reg;
	u32 pad_reg;

	unsigned int oeb_bit:5;
	unsigned int in_bit:5;
	unsigned int out_bit:5;

	unsigned int mstat_bit:5;
	unsigned int stat_bit:5;
	unsigned int msk_bit:5;
	unsigned int edge_bit:5;
	unsigned int de_bit:5;
	unsigned int type_bit:5;

	unsigned int clr_bit:5;

	unsigned int pull_bit:5;
	unsigned int drv_bit:5;
	unsigned int src_bit:5;
	unsigned int ind_bit:5;
	unsigned int hys_bit:5;
};

/**
 * struct omega_pinctrl - state for an omega pinctrl device
 * @dev:            device handle.
 * @pctrl:          pinctrl handle.
 * @pctrldesc:      pinctrl descriptor.
 * @chip:           gpiochip handle.
 * @irq:            parent irq for the irq_chip.
 * @lock:           spinlock to protect register resources as well
 *                  as omega_pinctrl data structures.
 * @regs:           base address for the pinctrl register map.
 * @ngpio:         number of gpios in the controller
 */
struct omega_pinctrl {
	struct device *dev;
	struct pinctrl_dev *pctrl;
	struct pinctrl_desc pctrldesc;
	struct gpio_chip chip;
	int irq;
	raw_spinlock_t lock;
	void __iomem *regs;
	unsigned int ngpio;
};

/**
 *  omega_set_bit - set or clear one bit in pinctrl register
 *
 *  @pctrl: pinctrl handle
 *  @reg_offset: register offset
 *  @bit: bit offset to set/clear
 *  @set: set or clear
 */
static inline void omega_set_bit(struct omega_pinctrl *pctrl,
				 u32 reg_offset,
				 unsigned int bit, bool set)
{
	u32 val;

	val = readl(pctrl->regs + reg_offset);
	if (set)
		val |= BIT(bit);
	else
		val &= ~BIT(bit);
	writel(val, pctrl->regs + reg_offset);
}

static void omega_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	unsigned int gpio;
	u32 val;

	chained_irq_enter(irq_chip, desc);

	/* go through all GPIOs and handle all interrupts */
	for (gpio = 0; gpio < pctrl->ngpio; gpio++) {
		struct omega_pin p = PIN(gpio);

		val = readl(pctrl->regs + p.intr_reg);
		if (val & BIT(p.mstat_bit)) {
			int child_irq = irq_find_mapping(gc->irq.domain, gpio);

			/*
			 * Clear the interrupt before invoking the
			 * handler, so we do not leave any window
			 * Writing a 1 acknowledges the interrupt (interrupt
			 * status is cleared), but the bit has to be set back to
			 * 0 to allow another interrupt to be triggered.
			 */
			omega_set_bit(pctrl, p.clr_reg, p.clr_bit, true);
			omega_set_bit(pctrl, p.clr_reg, p.clr_bit, false);

			generic_handle_irq(child_irq);
		}
	}

	chained_irq_exit(irq_chip, desc);
}


static void omega_gpio_irq_ack(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	unsigned int gpio = d->hwirq;
	struct omega_pin p = PIN(gpio);

	/*
	 * Writing a 1 acknowledges the interrupt (interrupt status is cleared)
	 * but the bit has to be set back to 0 to allow another interrupt to be
	 * triggered.
	 */
	omega_set_bit(pctrl, p.clr_reg, p.clr_bit, true);
	omega_set_bit(pctrl, p.clr_reg, p.clr_bit, false);
}

/**
 *  omega_gpio_irq_mask - mask a GPIO interrupt
 *
 *  @d: IRQ chip data
 */
static void omega_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	unsigned int gpio = d->hwirq;
	struct omega_pin p = PIN(gpio);
	unsigned long flags;

	raw_spin_lock_irqsave(&pctrl->lock, flags);
	omega_set_bit(pctrl, p.intr_reg, p.msk_bit, false);
	raw_spin_unlock_irqrestore(&pctrl->lock, flags);
}

/**
 *  omega_gpio_irq_unmask - unmask a GPIO interrupt
 *
 *  @d: IRQ chip data
 */
static void omega_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	unsigned int gpio = d->hwirq;
	struct omega_pin p = PIN(gpio);
	unsigned long flags;

	raw_spin_lock_irqsave(&pctrl->lock, flags);
	omega_set_bit(pctrl, p.intr_reg, p.msk_bit, true);
	raw_spin_unlock_irqrestore(&pctrl->lock, flags);
}

static int omega_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	unsigned int gpio = d->hwirq;
	struct omega_pin p = PIN(gpio);
	unsigned long flags;
	bool level_triggered = false;
	bool dual_edge = false;
	bool rising_or_high = false;

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		rising_or_high = true;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		break;

	case IRQ_TYPE_EDGE_BOTH:
		dual_edge = true;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		level_triggered = true;
		rising_or_high = true;
		break;

	case IRQ_TYPE_LEVEL_LOW:
		level_triggered = true;
		break;

	default:
		dev_err(pctrl->dev, "invalid GPIO IRQ type 0x%x\n",
			type);
		return -EINVAL;
	}

	raw_spin_lock_irqsave(&pctrl->lock, flags);
	omega_set_bit(pctrl, p.intr_reg, p.type_bit, level_triggered);
	omega_set_bit(pctrl, p.intr_reg, p.de_bit, dual_edge);
	omega_set_bit(pctrl, p.intr_reg, p.edge_bit, rising_or_high);
	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	dev_dbg(pctrl->dev,
		"gpio:%u level_triggered:%d dual_edge:%d rising_or_high:%d\n",
		gpio, level_triggered, dual_edge, rising_or_high);

	return 0;
}

static struct irq_chip omega_gpio_irq_chip = {
	.name = "bcm-omega-gpio",
	.irq_ack = omega_gpio_irq_ack,
	.irq_mask = omega_gpio_irq_mask,
	.irq_unmask = omega_gpio_irq_unmask,
	.irq_set_type = omega_gpio_irq_set_type,
	.irq_enable = omega_gpio_irq_unmask,
	.irq_disable = omega_gpio_irq_mask,
};

static int omega_gpio_direction_input(struct gpio_chip *gc, unsigned int gpio)
{
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	struct omega_pin p = PIN(gpio);
	unsigned long flags;

	raw_spin_lock_irqsave(&pctrl->lock, flags);
	omega_set_bit(pctrl, p.io_reg, p.oeb_bit, true);
	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	dev_dbg(pctrl->dev, "gpio:%u set input\n", gpio);

	return 0;
}

static int omega_gpio_direction_output(struct gpio_chip *gc,
				       unsigned int gpio,
				       int val)
{
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	struct omega_pin p = PIN(gpio);
	unsigned long flags;

	raw_spin_lock_irqsave(&pctrl->lock, flags);
	omega_set_bit(pctrl, p.io_reg, p.oeb_bit, false);
	omega_set_bit(pctrl, p.io_reg, p.out_bit, !!(val));
	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	dev_dbg(pctrl->dev, "gpio:%u set output, value:%d\n", gpio, val);

	return 0;
}

static void omega_gpio_set(struct gpio_chip *gc, unsigned int gpio, int val)
{
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	struct omega_pin p = PIN(gpio);
	unsigned long flags;

	raw_spin_lock_irqsave(&pctrl->lock, flags);
	omega_set_bit(pctrl, p.io_reg, p.out_bit, !!(val));
	raw_spin_unlock_irqrestore(&pctrl->lock, flags);

	dev_dbg(pctrl->dev, "gpio:%u set, value:%d\n", gpio, val);
}

static int omega_gpio_get(struct gpio_chip *gc, unsigned int gpio)
{
	struct omega_pinctrl *pctrl = gpiochip_get_data(gc);
	struct omega_pin p = PIN(gpio);

	return !!(readl(pctrl->regs + p.io_reg) & BIT(p.in_bit));
}

static const struct gpio_chip omega_gpio = {
	.direction_input  = omega_gpio_direction_input,
	.direction_output = omega_gpio_direction_output,
	.get              = omega_gpio_get,
	.set              = omega_gpio_set,
	.request          = gpiochip_generic_request,
	.free             = gpiochip_generic_free,
};

static int omega_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 1;
}

static const char *omega_get_group_name(struct pinctrl_dev *pctldev,
					 unsigned int selector)
{
	return "gpio_grp";
}

static const struct pinctrl_ops omega_pinctrl_ops = {
	.get_groups_count = omega_get_groups_count,
	.get_group_name = omega_get_group_name,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

#define MSK_BIAS	0x3
#define MSK_DRV		0x7
#define MSK_SRC		0x1
#define MSK_IND		0x1
#define MSK_HYS		0x1
static int omega_config_reg(struct omega_pinctrl *pctrl,
			    struct omega_pin p,
			    unsigned int param,
			    unsigned int *mask,
			    unsigned int *bit)
{
	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
		*bit = p.pull_bit;
		*mask = MSK_BIAS;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		*bit = p.drv_bit;
		*mask = MSK_DRV;
		break;
	case PIN_CONFIG_SLEW_RATE:
		*bit = p.src_bit;
		*mask = MSK_SRC;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		*bit = p.ind_bit;
		*mask = MSK_IND;
		break;
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		*bit = p.hys_bit;
		*mask = MSK_HYS;
		break;
	default:
		return -ENOTSUPP;
	}

	return 0;
}

static unsigned int omega_regval_to_drive(u32 val)
{
	return (val + 1) * 2;
}

#define OMEGA_PULL_DOWN	2
#define OMEGA_PULL_UP	1
#define OMEGA_NO_PULL	0
#define OMEGA_INPUT_DISABLE 1
#define OMEGA_SCHMITT_ENABLE 1
static int omega_pin_config_get(struct pinctrl_dev *pctldev,
				unsigned int pin,
				unsigned long *config)
{
	struct omega_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg, mask, val;
	unsigned int bit;
	int ret = -1;

	struct omega_pin p = PIN(pin);

	ret = omega_config_reg(pctrl, p, param, &mask, &bit);
	if (ret < 0)
		return ret;

	val = readl(pctrl->regs + p.pad_reg);
	arg = (val >> bit) & mask;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		return (arg == OMEGA_NO_PULL) ? 0 : -EINVAL;

	case PIN_CONFIG_BIAS_PULL_DOWN:
		return (arg == OMEGA_PULL_DOWN) ? 0 : -EINVAL;

	case PIN_CONFIG_BIAS_PULL_UP:
		return (arg == OMEGA_PULL_UP) ? 0 : -EINVAL;

	case PIN_CONFIG_DRIVE_STRENGTH:
		arg = omega_regval_to_drive(arg);
		break;

	case PIN_CONFIG_INPUT_ENABLE:
		/* 1 means input disable */
		return (arg == OMEGA_INPUT_DISABLE) ? -EINVAL : 0;

	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		return (arg == OMEGA_SCHMITT_ENABLE) ? 0 : -EINVAL;

	case PIN_CONFIG_SLEW_RATE:
		break;

	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int omega_pin_config_set(struct pinctrl_dev *pctldev,
				unsigned int pin,
				unsigned long *configs,
				unsigned int num_configs)
{
	struct omega_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	unsigned long flags;
	u32 arg, mask, val;
	unsigned int i;
	int ret = -ENOTSUPP;
	unsigned int bit;

	struct omega_pin p = PIN(pin);

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);

		ret = omega_config_reg(pctrl, p, param, &mask, &bit);
		if (ret < 0)
			return ret;

		switch (param) {
		case PIN_CONFIG_BIAS_DISABLE:
			arg = OMEGA_NO_PULL;
		break;

		case PIN_CONFIG_BIAS_PULL_DOWN:
			arg = OMEGA_PULL_DOWN;
			break;

		case PIN_CONFIG_BIAS_PULL_UP:
			arg = OMEGA_PULL_UP;
			break;

		case PIN_CONFIG_DRIVE_STRENGTH:
			/* Check for invalid values */
			if (arg > 16 || arg < 2 || (arg % 2) != 0)
				arg = -1;
			else
				arg = (arg / 2) - 1;
			break;

		case PIN_CONFIG_INPUT_ENABLE:
			arg = arg ^ 1;
			break;

		case PIN_CONFIG_SLEW_RATE:
		case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
			break;

		default:
			dev_err(pctrl->dev, "invalid configuration\n");
			return -ENOTSUPP;
		}

		/* Range-check user-supplied value */
		if (arg & ~mask) {
			dev_err(pctrl->dev, "config %x: %x is invalid\n",
				param, arg);
			return -EINVAL;
		}

		raw_spin_lock_irqsave(&pctrl->lock, flags);
		val = readl(pctrl->regs + p.pad_reg);
		val &= ~(mask << bit);
		val |= arg << bit;
		writel(val, pctrl->regs + p.pad_reg);
		raw_spin_unlock_irqrestore(&pctrl->lock, flags);
	} /* for each config */

	return 0;
}

static const struct pinconf_ops omega_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = omega_pin_config_get,
	.pin_config_set = omega_pin_config_set,
};

/*
 *
 * Here a local pinctrl device is created with simple 1-to-1 pin mapping to the
 * local GPIO pins
 *
 * for west bank pin controller: pin 0 is mapped to mfio0
 *				pin 50 is mapped to mfio50
 * for east bank pin controller: pin 0 is mapped to mfio51
 *				pin 61 is ampped to mfio112
 */
static int omega_gpio_register_pinctrl(struct omega_pinctrl *pctrl)
{
	struct pinctrl_pin_desc *pins;
	struct pinctrl_desc *pctrldesc = &pctrl->pctrldesc;
	int i;

	pins = devm_kcalloc(pctrl->dev, pctrl->ngpio, sizeof(*pins),
			    GFP_KERNEL);
	if (!pins)
		return -ENOMEM;

	for (i = 0; i < pctrl->ngpio; i++) {
		pins[i].number = i;
		pins[i].name = devm_kasprintf(pctrl->dev, GFP_KERNEL,
					      "mfio-%d", i);
		if (!pins[i].name)
			return -ENOMEM;
	}

	pctrldesc->name = dev_name(pctrl->dev);
	pctrldesc->pins = pins;
	pctrldesc->npins = pctrl->ngpio;
	pctrldesc->pctlops = &omega_pinctrl_ops;
	pctrldesc->confops = &omega_pinconf_ops;

	pctrl->pctrl = devm_pinctrl_register(pctrl->dev, pctrldesc, pctrl);
	if (IS_ERR(pctrl->pctrl)) {
		dev_err(pctrl->dev, "unable to register pinctrl device\n");
		return PTR_ERR(pctrl->pctrl);
	}

	return 0;
}

static const struct of_device_id omega_gpio_of_match[] = {
	{ .compatible = "brcm,omega-pinctrl" },
};

static int omega_gpio_init(struct omega_pinctrl *pctrl)
{
	struct gpio_chip *chip;
	int ret;

	chip = &pctrl->chip;
	chip->base = -1;
	chip->ngpio = pctrl->ngpio;
	chip->label = dev_name(pctrl->dev);
	chip->parent = pctrl->dev;
	chip->owner = THIS_MODULE;
	chip->of_node = pctrl->dev->of_node;

	ret = gpiochip_add_data(&pctrl->chip, pctrl);
	if (ret) {
		dev_err(pctrl->dev, "Failed register gpiochip\n");
		return ret;
	}

	ret = gpiochip_add_pin_range(&pctrl->chip, dev_name(pctrl->dev),
				     0, 0, chip->ngpio);
	if (ret) {
		dev_err(pctrl->dev, "Failed to add pin range\n");
		goto err_rm_gpiochip;
	}

	ret = gpiochip_irqchip_add(chip,
				   &omega_gpio_irq_chip,
				   0,
				   handle_edge_irq,
				   IRQ_TYPE_NONE);
	if (ret) {
		dev_err(pctrl->dev, "Failed to add irqchip to gpiochip\n");
		goto err_rm_gpiochip;
	}

	gpiochip_set_chained_irqchip(chip, &omega_gpio_irq_chip, pctrl->irq,
				     omega_gpio_irq_handler);

	return 0;

err_rm_gpiochip:
	gpiochip_remove(&pctrl->chip);

	return ret;
}

static int omega_pinctrl_probe(struct platform_device *pdev)
{
	struct omega_pinctrl *pctrl;
	struct resource *res;
	int ret;

	pctrl = devm_kzalloc(&pdev->dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->dev = &pdev->dev;
	pctrl->chip = omega_gpio;

	/* get irq from dts */
	pctrl->irq = platform_get_irq(pdev, 0);
	if (pctrl->irq < 0) {
		dev_err(&pdev->dev, "No interrupt defined for omega gpio\n");
		return pctrl->irq;
	}

	raw_spin_lock_init(&pctrl->lock);

	/* get reg, ngpio, base from dts*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pctrl->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(pctrl->regs)) {
		dev_err(&pdev->dev, "unable to map I/O memory\n");
		return PTR_ERR(pctrl->regs);
	}

	if (of_property_read_u32(pdev->dev.of_node, "ngpio", &(pctrl->ngpio))) {
		dev_err(&pdev->dev, "missing ngpio DT property\n");
		return -ENODEV;
	}

	ret = omega_gpio_register_pinctrl(pctrl);
	if (ret) {
		dev_err(&pdev->dev, "unable to register pinconf\n");
		return ret;
	}

	ret = omega_gpio_init(pctrl);
	if (ret) {
		dev_err(&pdev->dev, "unable to init gpio\n");
		return ret;
	}

	platform_set_drvdata(pdev, pctrl);

	return 0;
}

static struct platform_driver omega_pinctrl_driver = {
	.driver = {
		.name = "omega-gpio",
		.of_match_table = omega_gpio_of_match,
	},
	.probe = omega_pinctrl_probe,
};

static int __init omega_pinctrl_init(void)
{
	return platform_driver_register(&omega_pinctrl_driver);
}
arch_initcall_sync(omega_pinctrl_init);
