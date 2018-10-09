// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright(c) 2018 Broadcom
 */

#include <linux/pci.h>
#include "bcm_vk.h"

/**
 * Following two functions are needed only for backward compatibility.
 * It is further assumed that this file will not be used on kernels
 * that are "antique" in a sense that that data structure of msix
 * or the api pci_enable_msix_range() is missing.
 */
#if BCM_VK_MISC_API

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

	dev_info(dev, "Request min %d max %d flags 0x%x\n",
		 min_vecs, max_vecs, flags);

	for (i = 0; i < ARRAY_SIZE(vk->msix); i++) {
		vk->msix[i].entry = i;
		vk->msix[i].vector = 0;
	}

	return pci_enable_msix_range(pdev, vk->msix, min_vecs, max_vecs);
}

#endif
