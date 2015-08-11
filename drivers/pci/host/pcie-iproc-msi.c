/*
 * Copyright (C) 2015 Broadcom Corporation
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

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

#include "pcie-iproc.h"

#define SYS_MSI_INTS_EN_OFFSET       0x208
#define SYS_MSI_INTR_EN_SHIFT        11
#define SYS_MSI_INTR_EN              BIT(SYS_MSI_INTR_EN_SHIFT)
#define SYS_MSI_INT_N_EVENT_SHIFT    1
#define SYS_MSI_INT_N_EVENT          BIT(SYS_MSI_INT_N_EVENT_SHIFT)
#define SYS_MSI_EQ_EN_SHIFT          0
#define SYS_MSI_EQ_EN                BIT(SYS_MSI_EQ_EN_SHIFT)

#define SYS_EQ_MASK                  0x3f

/* number of queues in each event queue */
#define SYS_EQ_LEN                   64

/* size of each event queue memory region */
#define EQ_MEM_REGION_SIZE           SZ_4K

/* size of each MSI message memory region */
#define MSI_MSG_MEM_REGION_SIZE      SZ_4K

/**
 * iProc event queue based MSI
 *
 * Only meant to be used on legacy platforms without MSI support integrated
 * into the GIC
 *
 * @pcie: pointer to iProc PCIe data
 * @chip: MSI controller
 * @irqs: pointer to an array that contains the interrupt IDs
 * @nirqs: number of total interrupts
 * @has_inten_reg: indicates the MSI interrupt enable register needs to be
 * set explicitly (required for some legacy platforms)
 * @used: bitmap to track usage of MSI
 * @domain: IRQ domain
 * @bitmap_lock: lock to protect access to the IRQ bitmap
 * @n_eq_region: required number of 4K aligned memory region for MSI event
 * queue
 * @n_msi_msg_region: required number of 4K aligned memory region for MSI
 * posted writes
 * @eq_base: pointer to allocated memory region for MSI event queue
 * @msi_base: pointer to allocated memory region for MSI posted writes
 */
struct iproc_msi {
	struct iproc_pcie *pcie;
	struct msi_controller chip;
	int *irqs;
	int nirqs;
	bool has_inten_reg;
	DECLARE_BITMAP(used, IPROC_PCIE_MAX_NUM_IRQS);
	struct irq_domain *domain;
	struct mutex bitmap_lock;
	unsigned int n_eq_region;
	unsigned int n_msi_msg_region;
	void *eq_base;
	void *msi_base;
};

/**
 * Required to accommodate different register layout between PAXB and PAXC
 */
struct iproc_msi_reg {
	u32 eq_page_offset;
	u32 eq_page_upper_offset;
	u32 msi_page_offset;
	u32 msi_page_upper_offset;
	u32 msi_ctrl_offset;
	u32 eq_head_offset;
	u32 eq_tail_offset;
};

static const struct iproc_msi_reg iproc_msi_reg[][IPROC_PCIE_MAX_NUM_IRQS] = {
	[IPROC_PCIE_PAXB] = {
		{ 0x200, 0x2c0, 0x204, 0x2c4, 0x210, 0x250, 0x254 },
		{ 0x200, 0x2c0, 0x204, 0x2c4, 0x214, 0x258, 0x25c },
		{ 0x200, 0x2c0, 0x204, 0x2c4, 0x218, 0x260, 0x264 },
		{ 0x200, 0x2c0, 0x204, 0x2c4, 0x21c, 0x268, 0x26c },
		{ 0x200, 0x2c0, 0x204, 0x2c4, 0x220, 0x270, 0x274 },
		{ 0x200, 0x2c0, 0x204, 0x2c4, 0x224, 0x278, 0x27c },
	},
	[IPROC_PCIE_PAXC] = {
		{ 0xc00, 0xc04, 0xc08, 0xc0c, 0xc40, 0xc50, 0xc60 },
		{ 0xc10, 0xc14, 0xc18, 0xc1c, 0xc44, 0xc54, 0xc64 },
		{ 0xc20, 0xc24, 0xc28, 0xc2c, 0xc48, 0xc58, 0xc68 },
		{ 0xc30, 0xc34, 0xc38, 0xc3c, 0xc4c, 0xc5c, 0xc6c },
	},
};

#define EQ_PAGE(type, eq) (iproc_msi_reg[type][eq].eq_page_offset)
#define EQ_PAGE_UPPER(type, eq) (iproc_msi_reg[type][eq].eq_page_upper_offset)
#define MSI_PAGE(type, eq) (iproc_msi_reg[type][eq].msi_page_offset)
#define MSI_PAGE_UPPER(type, eq) (iproc_msi_reg[type][eq].msi_page_upper_offset)
#define MSI_CTRL(type, eq) (iproc_msi_reg[type][eq].msi_ctrl_offset)
#define EQ_HEAD(type, eq) (iproc_msi_reg[type][eq].eq_head_offset)
#define EQ_TAIL(type, eq) (iproc_msi_reg[type][eq].eq_tail_offset)

static inline struct iproc_msi *to_iproc_msi(struct msi_controller *chip)
{
	return container_of(chip, struct iproc_msi, chip);
}

static int iproc_msi_alloc(struct iproc_msi *msi)
{
	int irq;

	mutex_lock(&msi->bitmap_lock);

	irq = find_first_zero_bit(msi->used, msi->nirqs);
	if (irq < msi->nirqs)
		set_bit(irq, msi->used);
	else
		irq = -ENOSPC;

	mutex_unlock(&msi->bitmap_lock);

	return irq;
}

static void iproc_msi_free(struct iproc_msi *msi, unsigned long irq)
{
	struct device *dev = msi->chip.dev;

	mutex_lock(&msi->bitmap_lock);

	if (!test_bit(irq, msi->used))
		dev_warn(dev, "trying to free unused MSI IRQ#%lu\n", irq);
	else
		clear_bit(irq, msi->used);

	mutex_unlock(&msi->bitmap_lock);
}

static int iproc_msi_setup_irq(struct msi_controller *chip,
			       struct pci_dev *pdev, struct msi_desc *desc)
{
	struct iproc_msi *msi = to_iproc_msi(chip);
	struct msi_msg msg;
	unsigned int irq;
	int hwirq;
	phys_addr_t addr;

	hwirq = iproc_msi_alloc(msi);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(msi->domain, hwirq);
	if (!irq) {
		iproc_msi_free(msi, hwirq);
		return -EINVAL;
	}

	irq_set_msi_desc(irq, desc);

	addr = virt_to_phys(msi->msi_base) | (hwirq * 4);
	msg.address_lo = lower_32_bits(addr);
	if (IS_ENABLED(CONFIG_PHYS_ADDR_T_64BIT))
		msg.address_hi = upper_32_bits(addr);
	else
		msg.address_hi = 0;
	msg.data = hwirq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

static void iproc_msi_teardown_irq(struct msi_controller *chip,
				   unsigned int irq)
{
	struct iproc_msi *msi = to_iproc_msi(chip);
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	irq_dispose_mapping(irq);
	iproc_msi_free(msi, hwirq);
}

static struct irq_chip iproc_msi_irq_chip = {
	.name = "iProc PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

static int iproc_msi_map(struct irq_domain *domain, unsigned int irq,
			 irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &iproc_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	set_irq_flags(irq, IRQF_VALID);

	return 0;
}

static const struct irq_domain_ops msi_domain_ops = {
	.map = iproc_msi_map,
};

static void iproc_msi_enable(struct iproc_msi *msi)
{
	struct iproc_pcie *pcie = msi->pcie;
	int i, eq;
	u32 val;

	/* program memory region for each event queue */
	for (i = 0; i < msi->n_eq_region; i++) {
		phys_addr_t addr =
			virt_to_phys(msi->eq_base + (i * EQ_MEM_REGION_SIZE));

		writel(lower_32_bits(addr),
		       pcie->base + EQ_PAGE(pcie->type, i));

		if (IS_ENABLED(CONFIG_PHYS_ADDR_T_64BIT))
			val = upper_32_bits(addr);
		else
			val = 0;
		writel(val, pcie->base + EQ_PAGE_UPPER(pcie->type, i));
	}

	/* program memory region for MSI posted writes */
	for (i = 0; i < msi->n_msi_msg_region; i++) {
		phys_addr_t addr =
			virt_to_phys(msi->msi_base +
				     (i * MSI_MSG_MEM_REGION_SIZE));

		writel(lower_32_bits(addr),
		       pcie->base + MSI_PAGE(pcie->type, i));

		if (IS_ENABLED(CONFIG_PHYS_ADDR_T_64BIT))
			val = upper_32_bits(addr);
		else
			val = 0;
		writel(val, pcie->base + MSI_PAGE_UPPER(pcie->type, i));
	}

	for (eq = 0; eq < msi->nirqs; eq++) {
		/* enable MSI event queue */
		val = SYS_MSI_INTR_EN | SYS_MSI_INT_N_EVENT | SYS_MSI_EQ_EN;
		writel(val, pcie->base + MSI_CTRL(pcie->type, eq));

		/*
		 * Some legacy platforms require the MSI interrupt enable
		 * register to be set explicitly
		 */
		if (msi->has_inten_reg) {
			val = readl(pcie->base + SYS_MSI_INTS_EN_OFFSET);
			val |= BIT(eq);
			writel(val, pcie->base + SYS_MSI_INTS_EN_OFFSET);
		}
	}
}

static void iproc_msi_handler(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	struct iproc_msi *msi;
	struct iproc_pcie *pcie;
	u32 eq, head, tail, num_events;
	int virq;

	chained_irq_enter(irq_chip, desc);

	msi = irq_get_handler_data(irq);
	pcie = msi->pcie;

	eq = irq - msi->irqs[0];
	virq = irq_find_mapping(msi->domain, eq);
	head = readl(pcie->base + EQ_HEAD(pcie->type, eq)) & SYS_EQ_MASK;
	do {
		tail = readl(pcie->base + EQ_TAIL(pcie->type, eq)) &
			SYS_EQ_MASK;

		num_events = (tail < head) ?
			(SYS_EQ_LEN - (head - tail)) : (tail - head);
		if (!num_events)
			break;

		generic_handle_irq(virq);

		head++;
		head %= SYS_EQ_LEN;
		writel(head, pcie->base + EQ_HEAD(pcie->type, eq));
	} while (true);

	chained_irq_exit(irq_chip, desc);
}

struct msi_controller *iproc_pcie_msi_init(struct iproc_pcie *pcie,
					   struct device_node *node)
{
	struct iproc_msi *msi;
	struct msi_controller *chip;
	int i, ret;

	if (!of_find_property(node, "msi-controller", NULL))
		return ERR_PTR(-ENODEV);

	msi = devm_kzalloc(pcie->dev, sizeof(*msi), GFP_KERNEL);
	if (!msi)
		return ERR_PTR(-ENOMEM);
	msi->pcie = pcie;

	mutex_init(&msi->bitmap_lock);

	ret = of_property_read_u32(node, "brcm,num-eq-region",
				   &msi->n_eq_region);
	if (ret || msi->n_eq_region == 0) {
		dev_err(pcie->dev,
			"invalid property 'brcm,num-eq-region' %u\n",
			msi->n_eq_region);
		return ERR_PTR(-ENODEV);
	}

	ret = of_property_read_u32(node, "brcm,num-msi-msg-region",
				   &msi->n_msi_msg_region);
	if (ret || msi->n_msi_msg_region == 0) {
		dev_err(pcie->dev,
			"invalid property 'brcm,num-msi-msg-region' %u\n",
			msi->n_msi_msg_region);
		return ERR_PTR(-ENODEV);
	}

	/* reserve memory for MSI event queue */
	msi->eq_base = devm_kcalloc(pcie->dev, msi->n_eq_region + 1,
				    EQ_MEM_REGION_SIZE, GFP_KERNEL);
	if (!msi->eq_base)
		return ERR_PTR(-ENOMEM);
	msi->eq_base = PTR_ALIGN(msi->eq_base, EQ_MEM_REGION_SIZE);

	/* reserve memory for MSI posted writes */
	msi->msi_base = devm_kcalloc(pcie->dev, msi->n_msi_msg_region + 1,
				     MSI_MSG_MEM_REGION_SIZE, GFP_KERNEL);
	if (!msi->msi_base)
		return ERR_PTR(-ENOMEM);
	msi->msi_base = PTR_ALIGN(msi->msi_base, MSI_MSG_MEM_REGION_SIZE);

	if (of_find_property(node, "brcm,pcie-msi-inten", NULL))
		msi->has_inten_reg = true;

	msi->nirqs = of_irq_count(node);
	if (!msi->nirqs) {
		dev_err(pcie->dev, "found no MSI interrupt in DT\n");
		return ERR_PTR(-ENODEV);
	}
	if (msi->nirqs > IPROC_PCIE_MAX_NUM_IRQS) {
		dev_warn(pcie->dev, "too many MSI interrupts defined %d\n",
			 msi->nirqs);
		msi->nirqs = IPROC_PCIE_MAX_NUM_IRQS;
	}
	msi->irqs = devm_kcalloc(pcie->dev, msi->nirqs, sizeof(*msi->irqs),
				 GFP_KERNEL);
	if (!msi->irqs)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < msi->nirqs; i++) {
		msi->irqs[i] = irq_of_parse_and_map(node, i);
		if (!msi->irqs[i]) {
			dev_err(pcie->dev, "unable to parse/map interrupt\n");
			return ERR_PTR(-ENODEV);
		}
	}

	chip = &msi->chip;
	chip->dev = pcie->dev;
	chip->setup_irq = iproc_msi_setup_irq;
	chip->teardown_irq = iproc_msi_teardown_irq;
	msi->domain = irq_domain_add_linear(pcie->dev->of_node, msi->nirqs,
					    &msi_domain_ops, chip);
	if (!msi->domain) {
		dev_err(pcie->dev, "failed to create IRQ domain\n");
		return ERR_PTR(-ENXIO);
	}

	for (i = 0; i < msi->nirqs; i++) {
		irq_set_handler_data(msi->irqs[i], msi);
		irq_set_chained_handler(msi->irqs[i], iproc_msi_handler);
	}

	iproc_msi_enable(msi);

	return chip;
}
EXPORT_SYMBOL(iproc_pcie_msi_init);
