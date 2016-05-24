/*
 * Copyright (C) 2016 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Note this driver is meant for initial Nitro bring up on Stingray.
 * After issues with GICv3 ITS are sorted out, we will switch to GICv3 ITS for
 * PAXC MSIX support and this driver will be removed
 *
 * DO NOT UPSTREAM THIS DRIVER
 */

#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>

#include "pcie-iproc.h"

#define IPROC_MSI_EQ_MASK              0x7ff

#define EQ_TOGGLE_BIT                  11

/* Number of entries in the event queue */
#define EQ_LEN                         2048

/* Size of each event queue memory region */
#define EQ_MEM_REGION_SIZE             SZ_16K

#define EQ_MEM_ALIGN                   SZ_16K

/**
 * iProc event queue based MSI
 *
 * @pcie: pointer to iProc PCIe data
 * @reg_offsets: MSI register offsets
 * @bitmap: MSI vector bitmap
 * @bitmap_lock: lock to protect access to the MSI bitmap
 * @nr_msi_vecs: total number of MSI vectors
 * @inner_domain: inner IRQ domain
 * @msi_domain: MSI IRQ domain
 * @eq_cpu: pointer to allocated memory region for MSI event queues
 * @eq_dma: DMA address of MSI event queues
 * @msi_addr: MSI address
 */
struct iproc_msi {
	struct iproc_pcie *pcie;
	const u16 *reg_offsets;
	int irq;
	unsigned long *bitmap;
	struct mutex bitmap_lock;
	unsigned int nr_msi_vecs;
	struct irq_domain *inner_domain;
	struct irq_domain *msi_domain;
	void *eq_raw_cpu;
	void *eq_cpu;
	dma_addr_t eq_raw_dma;
	dma_addr_t eq_dma;
	phys_addr_t msi_addr;
};

enum iproc_msi_reg {
	IPROC_MSI_GICV3_CFG = 0,
	IPROC_MSI_BASE_ADDR,
	IPROC_MSI_WINDOW_SIZE,
	IPROC_MSI_EQ_PAGE,
	IPROC_MSI_EQ_PAGE_UPPER,
	IPROC_MSI_EQ_RD,
	IPROC_MSI_EQ_WR_STATUS,
	IPROC_MSI_CTRL,
	IPROC_MSI_INTR_EN,
	IPROC_MSI_ENABLE_CFG,
	IPROC_MSI_REG_SIZE,
};

static const u16 iproc_msi_reg_paxc_v2[IPROC_MSI_REG_SIZE] = {
	0x50, 0x74, 0x78, 0x7c, 0x80, 0x84, 0x88, 0x90, 0x98, 0x9c
};

static inline u32 iproc_msi_read_reg(struct iproc_msi *msi,
				     enum iproc_msi_reg reg)
{
	struct iproc_pcie *pcie = msi->pcie;

	return readl_relaxed(pcie->base + msi->reg_offsets[reg]);
}

static inline void iproc_msi_write_reg(struct iproc_msi *msi,
				       enum iproc_msi_reg reg, u32 val)
{
	struct iproc_pcie *pcie = msi->pcie;

	writel_relaxed(val, pcie->base + msi->reg_offsets[reg]);
}

static struct irq_chip iproc_msi_irq_chip = {
	.name = "iProc-MSI",
};

static struct msi_domain_info iproc_msi_domain_info = {
	.flags = MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		MSI_FLAG_PCI_MSIX,
	.chip = &iproc_msi_irq_chip,
};

static int iproc_msi_irq_set_affinity(struct irq_data *data,
				      const struct cpumask *mask, bool force)
{
	/*
	 * Unable to support MSI IRQ affinity properly with only one event
	 * queue. Need this callback here to return an error for the IRQ chip
	 * to be registered successfully
	 */
	return -EINVAL;
}

static void iproc_msi_irq_compose_msi_msg(struct irq_data *data,
					  struct msi_msg *msg)
{
	struct iproc_msi *msi = irq_data_get_irq_chip_data(data);

	msg->address_lo = lower_32_bits(msi->msi_addr);
	msg->address_hi = upper_32_bits(msi->msi_addr);
	msg->data = data->hwirq;
}

static struct irq_chip iproc_msi_bottom_irq_chip = {
	.name = "MSI",
	.irq_set_affinity = iproc_msi_irq_set_affinity,
	.irq_compose_msi_msg = iproc_msi_irq_compose_msi_msg,
};

static int iproc_msi_irq_domain_alloc(struct irq_domain *domain,
				      unsigned int virq, unsigned int nr_irqs,
				      void *args)
{
	struct iproc_msi *msi = domain->host_data;
	unsigned long hwirq;

	mutex_lock(&msi->bitmap_lock);

	hwirq = find_first_zero_bit(msi->bitmap, msi->nr_msi_vecs);
	if (hwirq < msi->nr_msi_vecs) {
		set_bit(hwirq, msi->bitmap);
	} else {
		mutex_unlock(&msi->bitmap_lock);
		return -ENOSPC;
	}

	mutex_unlock(&msi->bitmap_lock);

	irq_domain_set_info(domain, virq, hwirq, &iproc_msi_bottom_irq_chip,
			    domain->host_data, handle_simple_irq, NULL, NULL);

	return 0;
}

static void iproc_msi_irq_domain_free(struct irq_domain *domain,
				      unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	struct iproc_msi *msi = irq_data_get_irq_chip_data(data);

	mutex_lock(&msi->bitmap_lock);
	__clear_bit(data->hwirq, msi->bitmap);
	mutex_unlock(&msi->bitmap_lock);

	irq_domain_free_irqs_parent(domain, virq, nr_irqs);
}

static const struct irq_domain_ops msi_domain_ops = {
	.alloc = iproc_msi_irq_domain_alloc,
	.free = iproc_msi_irq_domain_free,
};

static inline u32 decode_msi_hwirq(struct iproc_msi *msi, u32 head)
{
	u64 *msg;
	u32 hwirq;
	unsigned int offs;

	/* each MSI message in the event queue is 8 bytes */
	offs = head * sizeof(u64);
	msg = (u64 *)(msi->eq_cpu + offs);
	hwirq = *msg & 0xffffffff;

	return hwirq;
}

static void iproc_msi_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct iproc_msi *msi;
	struct iproc_pcie *pcie;
	u32 head, tail, nr_events;
	unsigned long hwirq;
	int virq;
	bool toggle;
	u32 toggle_bit;

	chained_irq_enter(chip, desc);

	msi = irq_desc_get_handler_data(desc);
	pcie = msi->pcie;

	/*
	 * iProc MSI event queue is tracked by head and tail pointers.  Head
	 * pointer indicates the next entry (MSI data) to be consumed by SW in
	 * the queue and needs to be updated by SW.  iProc MSI core uses the
	 * tail pointer as the next data insertion point.
	 *
	 * Entries between head and tail pointers contain valid MSI data.  MSI
	 * data is guaranteed to be in the event queue memory before the tail
	 * pointer is updated by the iProc MSI core.
	 */
	head = iproc_msi_read_reg(msi, IPROC_MSI_EQ_RD);

	/* save the toggle bit */
	toggle_bit = (head >> EQ_TOGGLE_BIT) & 1;

	/* mask to get the head index */
	head &= IPROC_MSI_EQ_MASK;
	do {
		/* now get the tail index */
		tail = iproc_msi_read_reg(msi, IPROC_MSI_EQ_WR_STATUS)
			& IPROC_MSI_EQ_MASK;

		/*
		 * Figure out total number of events (MSI data) to be
		 * processed.
		 */
		nr_events = (tail < head) ?
			(EQ_LEN - (head - tail)) : (tail - head);
		if (!nr_events)
			break;

		/*
		 * In the case of wrap around, toggle bit needs to be flipped
		 * to allow the HW to properly distinguish between queue full
		 * and queue empty
		 */
		if (tail < head)
			toggle = true;
		else
			toggle = false;

		/* now let's process all outstanding events */
		while (nr_events--) {
			hwirq = decode_msi_hwirq(msi, head);
			virq = irq_find_mapping(msi->inner_domain, hwirq);
			generic_handle_irq(virq);

			head++;
			head %= EQ_LEN;
		}

		if (toggle) {
			/* flip the toggle bit of the head pointer */
			if (toggle_bit)
				head &= ~BIT(EQ_TOGGLE_BIT);
			else
				head |= BIT(EQ_TOGGLE_BIT);
		}

		/*
		 * Now all outstanding events have been processed.  Update the
		 * head pointer.
		 */
		iproc_msi_write_reg(msi, IPROC_MSI_EQ_RD, head);

		/*
		 * Now go read the tail pointer again to see if there are new
		 * oustanding events that came in during the above window.
		 */
	} while (true);

	chained_irq_exit(chip, desc);
}

static void iproc_msi_enable(struct iproc_msi *msi)
{
	dma_addr_t addr;

	/* choose event queue instead of GICv3 ITS */
	iproc_msi_write_reg(msi, IPROC_MSI_GICV3_CFG, 0);

	/*
	 * Program bits [43:13] of MSI address into bits [30:0] of the MSI
	 * base address register
	 */
	addr = msi->msi_addr;
	iproc_msi_write_reg(msi, IPROC_MSI_BASE_ADDR,
			     (u32)(addr >> 13));

	/* use a default 8K window size for MSI base address */
	iproc_msi_write_reg(msi, IPROC_MSI_WINDOW_SIZE, 0);

	/*
	 * Program bits [43:2] of address of event queue into the event queue
	 * address registers
	 */
	addr = msi->eq_dma >> 2;
	iproc_msi_write_reg(msi, IPROC_MSI_EQ_PAGE, lower_32_bits(addr));
	iproc_msi_write_reg(msi, IPROC_MSI_EQ_PAGE_UPPER, upper_32_bits(addr));

	/*
	 * Set interrupt threshold to zero, so interrupt is fired per MSI
	 * message write
	 */
	iproc_msi_write_reg(msi, IPROC_MSI_CTRL, 0x0);

	/* enable interrupt to GIC */
	iproc_msi_write_reg(msi, IPROC_MSI_INTR_EN, 0x1);

	/* enable MSI write detection */
	iproc_msi_write_reg(msi, IPROC_MSI_ENABLE_CFG, 0x1);
}

static void iproc_msi_disable(struct iproc_msi *msi)
{
	iproc_msi_write_reg(msi, IPROC_MSI_INTR_EN, 0);
	iproc_msi_write_reg(msi, IPROC_MSI_ENABLE_CFG, 0);
}

static int iproc_msi_alloc_domains(struct device_node *node,
				   struct iproc_msi *msi)
{
	msi->inner_domain = irq_domain_add_linear(NULL, msi->nr_msi_vecs,
						  &msi_domain_ops, msi);
	if (!msi->inner_domain)
		return -ENOMEM;

	msi->msi_domain = pci_msi_create_irq_domain(of_node_to_fwnode(node),
						    &iproc_msi_domain_info,
						    msi->inner_domain);
	if (!msi->msi_domain) {
		irq_domain_remove(msi->inner_domain);
		return -ENOMEM;
	}

	return 0;
}

static void iproc_msi_free_domains(struct iproc_msi *msi)
{
	if (msi->msi_domain)
		irq_domain_remove(msi->msi_domain);

	if (msi->inner_domain)
		irq_domain_remove(msi->inner_domain);
}

static void iproc_msi_irq_free(struct iproc_msi *msi)
{
	irq_set_chained_handler_and_data(msi->irq, NULL, NULL);
}

static int iproc_msi_irq_setup(struct iproc_msi *msi)
{
	irq_set_chained_handler_and_data(msi->irq, iproc_msi_handler, msi);
	return 0;
}

int iproc_msi_paxc_v2_init(struct iproc_pcie *pcie, struct device_node *node)
{
	struct iproc_msi *msi;
	int ret;
	void *eq_raw_cpu;
	dma_addr_t eq_raw_dma;

	if (!of_device_is_compatible(node, "brcm,iproc-msi-paxc-v2"))
		return -ENODEV;

	if (!of_find_property(node, "msi-controller", NULL))
		return -ENODEV;

	if (pcie->msi)
		return -EBUSY;

	msi = devm_kzalloc(pcie->dev, sizeof(*msi), GFP_KERNEL);
	if (!msi)
		return -ENOMEM;

	msi->pcie = pcie;
	pcie->msi = msi;
	msi->msi_addr = pcie->base_addr;
	mutex_init(&msi->bitmap_lock);

	msi->reg_offsets = iproc_msi_reg_paxc_v2;
	msi->nr_msi_vecs = EQ_LEN;
	msi->bitmap = devm_kcalloc(pcie->dev, BITS_TO_LONGS(msi->nr_msi_vecs),
				   sizeof(*msi->bitmap), GFP_KERNEL);
	if (!msi->bitmap)
		return -ENOMEM;

	msi->irq = irq_of_parse_and_map(node, 0);
	if (!msi->irq) {
		dev_err(pcie->dev, "unable to parse/map interrupt\n");
		return -ENODEV;
	}

	/* Reserve memory for event queue and make sure memories are zeroed */
	eq_raw_cpu = dma_zalloc_coherent(pcie->dev,
					 EQ_MEM_REGION_SIZE + EQ_MEM_ALIGN,
					 &eq_raw_dma, GFP_KERNEL);
	if (!eq_raw_cpu) {
		ret = -ENOMEM;
		goto free_irqs;
	}

	msi->eq_raw_cpu = eq_raw_cpu;
	msi->eq_raw_dma = eq_raw_dma;
	msi->eq_dma = ALIGN(eq_raw_dma, EQ_MEM_ALIGN);
	msi->eq_cpu = eq_raw_cpu + (msi->eq_dma - eq_raw_dma);

	ret = iproc_msi_alloc_domains(node, msi);
	if (ret) {
		dev_err(pcie->dev, "failed to create MSI domains\n");
		goto free_eq_dma;
	}

	ret = iproc_msi_irq_setup(msi);
	if (ret)
		goto free_msi_irq;

	iproc_msi_enable(msi);

	dev_info(pcie->dev, "PAXC v2 event queue MSI initialized\n");

	return 0;

free_msi_irq:
	iproc_msi_irq_free(msi);
	iproc_msi_free_domains(msi);

free_eq_dma:
	dma_free_coherent(pcie->dev, EQ_MEM_REGION_SIZE, eq_raw_cpu,
			  eq_raw_dma);

free_irqs:
	irq_dispose_mapping(msi->irq);
	pcie->msi = NULL;
	return ret;
}
EXPORT_SYMBOL(iproc_msi_paxc_v2_init);

void iproc_msi_paxc_v2_exit(struct iproc_pcie *pcie)
{
	struct iproc_msi *msi = pcie->msi;

	if (!msi)
		return;

	iproc_msi_disable(msi);
	iproc_msi_irq_free(msi);
	iproc_msi_free_domains(msi);
	dma_free_coherent(pcie->dev, EQ_MEM_REGION_SIZE, msi->eq_raw_cpu,
			  msi->eq_raw_dma);
	irq_dispose_mapping(msi->irq);
}
EXPORT_SYMBOL(iproc_msi_paxc_v2_exit);
