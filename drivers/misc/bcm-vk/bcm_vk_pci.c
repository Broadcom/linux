// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018-2019 Broadcom.
 */

#include <linux/pci.h>
#include "bcm_vk.h"

/**
 * Following two functions are needed only for backward compatibility.
 * It is further assumed that this file will not be used on kernels
 * that are "antique" in a sense that that data structure of msix
 * or the api pci_enable_msix_range() is missing.
 */
#if BCM_VK_LEGACY_API

int pci_irq_vector(struct pci_dev *pdev, unsigned int nr)
{
	struct bcm_vk *vk = pci_get_drvdata(pdev);

	return vk->msix[nr].vector;
}

int pci_alloc_irq_vectors(struct pci_dev *pdev, unsigned int min_vecs,
			  unsigned int max_vecs, unsigned int flags)
{
	struct device *dev = &pdev->dev;
	struct bcm_vk *vk = pci_get_drvdata(pdev);
	uint32_t i;
	int num_vecs = 0;

	dev_info(dev, "Request min %d max %d flags 0x%x\n",
		 min_vecs, max_vecs, flags);

	for (i = 0; i < ARRAY_SIZE(vk->msix); i++) {
		vk->msix[i].entry = i;
		vk->msix[i].vector = 0;
	}

	if (flags & PCI_IRQ_MSIX)
		num_vecs = pci_enable_msix_range(pdev, vk->msix,
						 min_vecs, max_vecs);

	/* if MSI-x fails and MSI is chosen */
	if ((num_vecs == 0) && (flags & PCI_IRQ_MSI)) {
		num_vecs = pci_enable_msi_block(pdev, max_vecs);
		if (num_vecs == 0) {

			for (num_vecs = 0; num_vecs < max_vecs; num_vecs++)
				vk->msix[num_vecs].vector =
					num_vecs + pdev->irq;

			dev_info(dev, "Fallback to MSI with %d irqs\n",
				 num_vecs);
		}
	}

	return num_vecs;
}

#endif
