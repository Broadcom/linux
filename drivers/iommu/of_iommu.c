/*
 * OF helpers for IOMMU
 *
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/export.h>
#include <linux/iommu.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/of_iommu.h>
#include <linux/of_pci.h>
#include <linux/slab.h>

#define NO_IOMMU	1

/**
 * of_get_dma_window - Parse *dma-windows property and returns 0 if found.
 *
 * @dn: device node
 * @prefix: prefix for property name if any
 * @index: index to start to parse
 * @dma_window: returns read of_iommu_dma_window <busno prot bus_addr size>
 *
 * This supports different formats using "prefix" configured if any.
 */
int of_get_dma_window(struct device_node *dn, const char *prefix, int *index,
		      struct of_iommu_dma_window *dma_window)
{
	char propname[NAME_MAX];
	int na, ns, len, pos;
	const __be32 *prop;

	if (!dn || !dma_window || !index)
		return -EINVAL;

	if (!prefix)
		prefix = "";

	prop = of_get_property(dn, "#dma-address-cells", NULL);
	na = prop ? be32_to_cpup(prop) : of_n_addr_cells(dn);
	prop = of_get_property(dn, "#dma-size-cells", NULL);
	ns = prop ? be32_to_cpup(prop) : of_n_size_cells(dn);

	snprintf(propname, sizeof(propname), "%sdma-windows", prefix);
	prop = of_get_property(dn, propname, &len);
	if (!prop)
		return -ENOENT;

	len /= sizeof(*prop);
	pos = *index;
	/* prot and busno takes one cell each */
	if (pos >= len || (len - pos) % (na + ns + 2))
		return -EINVAL;

	memset(dma_window, 0, sizeof(*dma_window));
	dma_window->busno = be32_to_cpup(prop + pos++);
	dma_window->prot = be32_to_cpup(prop + pos++);
	if (dma_window->prot && !(dma_window->prot & IOMMU_PROT_FLAGS))
		return -EINVAL;

	dma_window->bus_addr = of_read_number(prop + pos, na);
	pos += na;
	dma_window->size = of_read_number(prop + pos, ns);
	pos += ns;
	*index = pos;
	return 0;

}
EXPORT_SYMBOL_GPL(of_get_dma_window);

void of_iommu_resv_dma_regions(struct device_node *np, struct list_head *list)
{
	struct of_iommu_dma_window dma_window;
	struct iommu_resv_region *region;
	int index = 0;

	while (!of_get_dma_window(np, "reserved-", &index, &dma_window)) {
		region = iommu_alloc_resv_region(dma_window.bus_addr,
						 dma_window.size,
						 dma_window.prot,
						 IOMMU_RESV_RESERVED);
		if (!region)
			break;

		list_add_tail(&region->list, list);
	}
}
EXPORT_SYMBOL_GPL(of_iommu_resv_dma_regions);

static int of_iommu_xlate(struct device *dev,
			  struct of_phandle_args *iommu_spec)
{
	const struct iommu_ops *ops;
	struct fwnode_handle *fwnode = &iommu_spec->np->fwnode;
	int err;

	ops = iommu_ops_from_fwnode(fwnode);
	if ((ops && !ops->of_xlate) ||
	    !of_device_is_available(iommu_spec->np))
		return NO_IOMMU;

	err = iommu_fwspec_init(dev, &iommu_spec->np->fwnode, ops);
	if (err)
		return err;
	/*
	 * The otherwise-empty fwspec handily serves to indicate the specific
	 * IOMMU device we're waiting for, which will be useful if we ever get
	 * a proper probe-ordering dependency mechanism in future.
	 */
	if (!ops)
		return driver_deferred_probe_check_state(dev);

	return ops->of_xlate(dev, iommu_spec);
}

struct of_pci_iommu_alias_info {
	struct device *dev;
	struct device_node *np;
};

static int of_pci_iommu_init(struct pci_dev *pdev, u16 alias, void *data)
{
	struct of_pci_iommu_alias_info *info = data;
	struct of_phandle_args iommu_spec = { .args_count = 1 };
	int err;

	err = of_pci_map_rid(info->np, alias, "iommu-map",
			     "iommu-map-mask", "iommu-map-drop-mask",
			     &iommu_spec.np, iommu_spec.args);
	if (err)
		return err == -ENODEV ? NO_IOMMU : err;

	err = of_iommu_xlate(info->dev, &iommu_spec);
	of_node_put(iommu_spec.np);
	return err;
}

const struct iommu_ops *of_iommu_configure(struct device *dev,
					   struct device_node *master_np)
{
	const struct iommu_ops *ops = NULL;
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;
	int err = NO_IOMMU;

	if (!master_np)
		return NULL;

	if (fwspec) {
		if (fwspec->ops)
			return fwspec->ops;

		/* In the deferred case, start again from scratch */
		iommu_fwspec_free(dev);
	}

	/*
	 * We don't currently walk up the tree looking for a parent IOMMU.
	 * See the `Notes:' section of
	 * Documentation/devicetree/bindings/iommu/iommu.txt
	 */
	if (dev_is_pci(dev)) {
		struct of_pci_iommu_alias_info info = {
			.dev = dev,
			.np = master_np,
		};

		err = pci_for_each_dma_alias(to_pci_dev(dev),
					     of_pci_iommu_init, &info);
	} else {
		struct of_phandle_args iommu_spec;
		int idx = 0;

		while (!of_parse_phandle_with_args(master_np, "iommus",
						   "#iommu-cells",
						   idx, &iommu_spec)) {
			err = of_iommu_xlate(dev, &iommu_spec);
			of_node_put(iommu_spec.np);
			idx++;
			if (err)
				break;
		}
	}

	/*
	 * Two success conditions can be represented by non-negative err here:
	 * >0 : there is no IOMMU, or one was unavailable for non-fatal reasons
	 *  0 : we found an IOMMU, and dev->fwspec is initialised appropriately
	 * <0 : any actual error
	 */
	if (!err)
		ops = dev->iommu_fwspec->ops;
	/*
	 * If we have reason to believe the IOMMU driver missed the initial
	 * add_device callback for dev, replay it to get things in order.
	 */
	if (ops && ops->add_device && dev->bus && !dev->iommu_group)
		err = ops->add_device(dev);

	/* Ignore all other errors apart from EPROBE_DEFER */
	if (err == -EPROBE_DEFER) {
		ops = ERR_PTR(err);
	} else if (err < 0) {
		dev_dbg(dev, "Adding to IOMMU failed: %d\n", err);
		ops = NULL;
	}

	return ops;
}
