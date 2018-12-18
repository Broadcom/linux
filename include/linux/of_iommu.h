/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __OF_IOMMU_H
#define __OF_IOMMU_H

#include <linux/device.h>
#include <linux/iommu.h>
#include <linux/of.h>

struct of_iommu_resv_region {
	u32 busno;
	u32 prot;
	u64 bus_addr;
	u64 size;
};

#ifdef CONFIG_OF_IOMMU

int of_get_resv_region(struct device_node *dn, const char *region, int *index,
		      struct of_iommu_resv_region *of_region);

extern const struct iommu_ops *of_iommu_configure(struct device *dev,
					struct device_node *master_np);

void of_iommu_resv_dma_regions(struct device_node *np, struct list_head *list);

#else

static inline int of_get_resv_region(struct device_node *dn, const char *region,
			int *index, struct of_iommu_resv_region *of_region)
{
	return -EINVAL;
}

static inline const struct iommu_ops *of_iommu_configure(struct device *dev,
					 struct device_node *master_np)
{
	return NULL;
}

static inline int of_iommu_resv_dma_regions(struct device_node *np,
					    struct list_head *list)
{
	return -EINVAL;
}

#endif	/* CONFIG_OF_IOMMU */

#endif /* __OF_IOMMU_H */
