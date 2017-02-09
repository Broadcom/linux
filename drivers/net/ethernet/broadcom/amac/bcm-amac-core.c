/*
 * Copyright (C) 2015 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kfifo.h>
#include <linux/types.h>

#include "bcm-amac-core.h"

#define GMAC_RESET_DELAY 2
#define DMA_DESC_ALIGN 0x2000 /* 8K aligned */

#define DMA_LOW_ADDR(addr) cpu_to_le32(lower_32_bits(addr))
#define DMA_HIGH_ADDR(addr) cpu_to_le32(upper_32_bits(addr))

#define SPINWAIT(exp, ms, err) { \
	u32 countdown = ms; \
	err = 0; \
	while ((exp) && (countdown)) {\
		usleep_range(900, 1100); \
		countdown--; \
	} \
	if (!countdown) \
		err = -EBUSY; \
}

void bcm_amac_enet_set_speed(struct bcm_amac_priv *privp, u32 speed, u32 duplex)
{
	u32 cmd;

	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);

	cmd &= ~CC_ES_MASK;

	if (speed == AMAC_PORT_SPEED_1G)
		cmd |= (AMAC_SPEED_1000 << CC_ES_SHIFT);
	else if (speed == AMAC_PORT_SPEED_100M)
		cmd |= (AMAC_SPEED_100 << CC_ES_SHIFT);

	if (duplex == DUPLEX_HALF)
		cmd |= CC_HD;
	else
		cmd &= ~(CC_HD);

	writel(cmd, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
}

/* amac_alloc_rx_skb() - Allocate RX SKB
 * @privp: driver info pointer
 * @len: length of skb
 * @node: skb node pointer
 *
 * Returns: error or 0
 */
static int amac_alloc_rx_skb(struct bcm_amac_priv *privp,
			     int len,
			     struct skb_list_node *node)
{
	int offset;
	struct sk_buff *skb;
	struct net_device *ndev = privp->ndev;
	dma_addr_t dma_addr;
	void *bounce_rx_buf;
	struct amac_dma_priv *dma_p;

	if (privp->dma.sr_dma.enable_bounce)
		dma_p = &privp->dma;

	skb = __netdev_alloc_skb(ndev, (len + (2 * AMAC_DMA_RXALIGN)),
				 GFP_ATOMIC | GFP_DMA);
	if (!skb)
		return -ENOMEM;

	/* Align buffer for DMA requirements */
	/* Desc has to be 16-bit aligned */
	offset = PTR_ALIGN(skb->data, 16) - skb->data;
	skb_reserve(skb, offset);

	if (privp->dma.sr_dma.enable_bounce) {
		/* set Rx bounce buffers. */
		dma_p->sr_dma.rx_bounce_data.alloc_size = len;
		bounce_rx_buf =
			 dma_alloc_coherent(&privp->pdev->dev,
					    dma_p->sr_dma.rx_bounce_data.
					    alloc_size,
					    &dma_p->sr_dma.rx_bounce_data.
					    raw_addr,
					    GFP_KERNEL | GFP_DMA);
		if (!bounce_rx_buf)
			return -ENOMEM;
	} else {
		/* Set buffer ownership of the new skb pointer */
		dma_addr = dma_map_single(&privp->pdev->dev, skb->data,
					  len,
					  DMA_FROM_DEVICE);
		if (dma_mapping_error(&privp->pdev->dev, dma_addr)) {
			netdev_err(privp->ndev, "RX: SKB DMA mapping error\n");

			dev_kfree_skb_any(skb);
			return -EFAULT;
		}
	}

	node->skb = skb;
	node->len = len;
	node->dma_addr = dma_addr;

	if (privp->dma.sr_dma.enable_bounce) {
		node->skb_bounce = bounce_rx_buf;
		node->dma_addr = dma_p->sr_dma.rx_bounce_data.raw_addr;
	}

	return 0;
}

/* amac_free_rx_skb() - Allocate RX SKB
 * @privp: driver info pointer
 * @len: length of skb
 * @node: skb node pointer
 */
static void amac_free_rx_skb(struct bcm_amac_priv *privp,
			     int len,
			     struct skb_list_node *node)
{
	struct amac_dma_priv *dma_p = &privp->dma;

	if (dma_p->sr_dma.enable_bounce) {
		if (node->skb_bounce)
			dma_free_coherent(&privp->pdev->dev,
					  dma_p->sr_dma.rx_bounce_data.
					  alloc_size,
					  node->skb_bounce,
					  dma_p->sr_dma.rx_bounce_data.
					  raw_addr);
	} else
		dma_unmap_single(&privp->pdev->dev,
				 node->dma_addr,
				 len,
				 DMA_FROM_DEVICE);

	dev_kfree_skb_any(node->skb);

	node->skb = NULL;
	node->dma_addr = 0;
	node->len = 0;
}

/* amac_dma_check_rx_done() - Check to see if we have a packet to be read.
 * @privp: driver info pointer
 *
 * Returns: '0' if no data or else the offset value
 */
static unsigned int amac_dma_check_rx_done(struct bcm_amac_priv *privp)
{
	struct amac_dma_priv	*dmap = &privp->dma;
	u32 stat0 = 0, stat1 = 0;
	u32 offset;
	u32 control;
	int index, curr, active;

	index = dmap->rx.index;
	offset = (u32)(dmap->rx.base_addr & 0xffffffff);

	stat0 = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS0_REG)
		& D64_RS0_CD_MASK;

	stat1 = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_STATUS1_REG)
		& D64_RS0_CD_MASK;

	curr = ((stat0 - offset) & D64_RS0_CD_MASK) /
			sizeof(struct amac_dma64_desc);
	active = ((stat1 - offset) & D64_RS0_CD_MASK) /
			sizeof(struct amac_dma64_desc);

	if (index == curr)
		return 0; /* No Data */

	/* Payload contains hw data hence offset to be used */
	control = readl(
		privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_REG);
	offset = (control & D64_RC_RO_MASK) >> D64_RC_RO_SHIFT;

	return offset;
}

/* bcm_amac_clear_intr() - Clear Interrupt status
 * @privp: driver info pointer
 * @is_rx: TX or RX direction to clear
 *
 * Returns: none
 */
void bcm_amac_clear_intr(struct bcm_amac_priv *privp, bool is_rx)
{
	u32 intr_status;

	intr_status = readl(privp->hw.reg.amac_core +
			    GMAC_INT_STATUS_REG);

	if (is_rx)
		intr_status &= I_RI; /* Clear only RX interrupt */
	else
		intr_status &= I_XI_ALL; /* Clear only TX interrupt(s) */

	writel(intr_status,
	       (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));
}

/* amac_enable_tx_intr() - Enable TX interrupt
 * @privp: driver info pointer
 * @is_rx: rx or tx
 * @enable: enable/disable the interrupt
 *
 * Returns: none
 */
void bcm_amac_enable_intr(struct bcm_amac_priv *privp, bool is_rx, bool enable)
{
	u32 intr_mask;

	intr_mask = readl(privp->hw.reg.amac_core +
			  GMAC_INT_MASK_REG);

	if (enable) {
		if (is_rx)
			intr_mask |= I_RI;
		else
			intr_mask |= I_XI_ALL;

	} else {
		if (is_rx)
			intr_mask &= ~I_RI;
		else
			intr_mask &= ~I_XI_ALL;
	}

	writel(intr_mask,
	       (privp->hw.reg.amac_core + GMAC_INT_MASK_REG));

	/* Clear the interrupt status if disabling */
	if (!enable)
		bcm_amac_clear_intr(privp, is_rx);
}

static void amac_dma_rx_init_chnl(struct bcm_amac_priv *privp)
{
	struct amac_dma_priv *dma_p = &privp->dma;

	/* initailize the DMA channel */
	writel(DMA_LOW_ADDR(dma_p->rx.base_addr),
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_LO_REG));

	writel(DMA_HIGH_ADDR(dma_p->rx.base_addr),
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_ADDR_HI_REG));

	/* now update the dma last descriptor */
	writel(dma_p->rx.base_addr,
	       (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));
}

/* amac_dma_rx_init() - RX DMA initialization routine
 * @privp: driver info pointer
 *
 * Descriptors and skb are allocated and initlialized, the various rx registers
 * are updated with the descriptor information
 *
 * Returns: '0' or error
 */
static int amac_dma_rx_init(struct bcm_amac_priv *privp)
{
	struct net_device *ndev = privp->ndev;
	struct amac_dma_priv *dma_p = &privp->dma;
	struct amac_dma64_desc *descp;
	unsigned int size;
	u32 ctrl, i;
	int rc = 0;
	u32 offset;

	dma_p->rx.ring_len = privp->dma.dma_rx_desc_count;

	/* Allocate rx descriptors */
	dma_p->rx.index = 0;
	dma_p->rx.alloc_size = (privp->dma.dma_rx_desc_count *
			       sizeof(struct amac_dma64_desc)) + DMA_DESC_ALIGN;
	dma_p->rx.raw_descp = dma_alloc_coherent(&privp->pdev->dev,
					     dma_p->rx.alloc_size,
					     &dma_p->rx.raw_addr,
					     GFP_KERNEL | GFP_DMA);
	if (!dma_p->rx.raw_descp) {
		netdev_err(ndev, "Failed to alloc rx dma desc\n");
		return -ENOMEM;
	}

	dma_p->rx.base_addr = ALIGN(dma_p->rx.raw_addr, DMA_DESC_ALIGN);
	offset = dma_p->rx.base_addr - dma_p->rx.raw_addr;
	dma_p->rx.descp = dma_p->rx.raw_descp + offset;

	/* Allocate rx skb list */
	size = dma_p->rx.ring_len * sizeof(*dma_p->rx_skb_list);
	dma_p->rx_skb_list = kzalloc(size, GFP_KERNEL | GFP_DMA);
	if (!dma_p->rx_skb_list) {
		netdev_err(ndev, "Failed to alloc rx skb list size=%u\n", size);
		rc = -ENOMEM;
		goto rx_init_err;
	}

	/* Setup rx descriptor ring */
	for (i = 0; i < privp->dma.dma_rx_desc_count; i++) {
		descp = (struct amac_dma64_desc *)(dma_p->rx.descp) + i;

		rc = amac_alloc_rx_skb(privp, AMAC_DMA_RX_BUF_LEN,
				       &dma_p->rx_skb_list[i]);
		if (rc)
			goto rx_init_skb_err;

		ctrl = 0;

		/* if last descr set endOfTable */
		if (i == (privp->dma.dma_rx_desc_count - 1))
			ctrl = D64_CTRL1_EOT;

		descp->ctrl1 = cpu_to_le32(ctrl);
		descp->ctrl2 = cpu_to_le32(AMAC_DMA_RX_BUF_LEN);
		descp->addrlow =
			DMA_LOW_ADDR(dma_p->rx_skb_list[i].dma_addr);
		descp->addrhigh =
			DMA_HIGH_ADDR(dma_p->rx_skb_list[i].dma_addr);
	}

	amac_dma_rx_init_chnl(privp);

	return 0;

rx_init_skb_err:

	for (i = 0; i < privp->dma.dma_rx_desc_count; i++) {
		if (dma_p->rx_skb_list[i].skb)
			amac_free_rx_skb(privp, AMAC_DMA_RX_BUF_LEN,
					 &dma_p->rx_skb_list[i]);
		else
			break;
	}

	kfree(privp->dma.rx_skb_list);
	privp->dma.rx_skb_list = NULL;

rx_init_err:
	if (privp->dma.rx.descp) {
		dma_free_coherent(&privp->pdev->dev,
				  privp->dma.rx.alloc_size,
				  privp->dma.rx.descp,
				  privp->dma.rx.base_addr);
		privp->dma.rx.descp = NULL;
	}

	return rc;
}

/* amac_dma_tx_init() - TX DMA initialization routine
 * @privp: driver info pointer
 *
 * Descriptors and skb list are allocated.
 *
 * Returns: 0 or error
 */
static int amac_dma_tx_init(struct bcm_amac_priv *privp)
{
	struct net_device *ndev = privp->ndev;
	struct amac_dma_priv *dma_p = &privp->dma;
	u32 size;
	u32 offset;

	dma_p->tx.ring_len = privp->dma.dma_tx_desc_count;

	/* Allocate tx descriptors */
	dma_p->tx.index = 0;
	dma_p->tx.alloc_size = (dma_p->tx.ring_len *
				sizeof(struct amac_dma64_desc)) +
				DMA_DESC_ALIGN;
	dma_p->tx.raw_descp = dma_alloc_coherent(&privp->pdev->dev,
					     dma_p->tx.alloc_size,
					     &dma_p->tx.raw_addr,
					     GFP_KERNEL | GFP_DMA);
	if (!dma_p->tx.raw_descp) {
		netdev_err(ndev, "Cannot allocate tx dma descriptors.\n");
		return -ENOMEM;
	}

	dma_p->tx.base_addr = ALIGN(dma_p->tx.raw_addr, DMA_DESC_ALIGN);
	offset = dma_p->tx.base_addr - dma_p->tx.raw_addr;
	dma_p->tx.descp = dma_p->tx.raw_descp + offset;

	/* Allocate tx skb list */
	size = dma_p->tx_max_pkts * sizeof(*dma_p->tx_skb_list);
	dma_p->tx_skb_list = kzalloc(size, GFP_KERNEL | GFP_DMA);
	if (!dma_p->tx_skb_list) {
		netdev_err(ndev, "Failed to alloc tx skb list size=%u\n", size);
		goto tx_init_err;
	}

	return 0;

tx_init_err:
	dma_free_coherent(&privp->pdev->dev,
			  dma_p->tx.alloc_size,
			  privp->dma.tx.descp,
			  privp->dma.tx.base_addr);
	dma_p->tx.alloc_size = 0;

	return -ENOMEM;
}

static inline void amac_core_init_reset(struct bcm_amac_priv *privp)
{
	u32 tmp;

	tmp = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	tmp |= CC_SR;
	writel(tmp, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	udelay(GMAC_RESET_DELAY);
}

static inline void amac_core_clear_reset(struct bcm_amac_priv *privp)
{
	u32 tmp;

	tmp = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	tmp &= ~(CC_SR);
	writel(tmp, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	udelay(GMAC_RESET_DELAY);
}

/* amac_set_prom() - Enable / Disable promiscuous mode
 * @privp: device data pointer
 * @enable: '0' disables promiscuous mode, >0 enables promiscuous mode
 */
static void amac_set_prom(struct bcm_amac_priv *privp, bool enable)
{
	u32 reg;

	reg = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);

	if ((enable) && (!(reg & CC_PROM)))
		reg |= CC_PROM;
	else if ((!enable) && (reg & CC_PROM))
		reg &= ~CC_PROM;
	else
		return;

	amac_core_init_reset(privp);
	writel(reg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
	amac_core_clear_reset(privp);
}

/* bcm_amac_core_init() - Initialize the gmac core
 * @privp: driver info pointer
 *
 * Initialize the gmac core, phy, setup clock, initialize swith mode or
 * switch bypass mode.
 *
 * Returns: '0' for success or '-1'
 */
int bcm_amac_core_init(struct bcm_amac_priv *privp)
{
	u32 tmp;
	u32 cmd;

	/* Reset AMAC core */
	writel(0, privp->hw.reg.amac_idm_base + AMAC_IDM_RST_CTRL_REG);

	/* Set clock */
	tmp = readl(privp->hw.reg.amac_idm_base + AMAC_IDM0_IO_CTRL_REG);
	tmp &= ~BIT(AMAC_IDM0_IO_CTRL_CLK_250_SEL_BIT);
	tmp |= BIT(AMAC_IDM0_IO_CTRL_GMII_MODE_BIT);
	tmp &= ~BIT(AMAC_IDM0_IO_CTRL_DEST_SYNC_MODE_EN_BIT);
	writel(tmp, (privp->hw.reg.amac_idm_base + AMAC_IDM0_IO_CTRL_REG));

	/* reset GMAC core */
	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	cmd &= ~(CC_TE | CC_RE | CC_RPI | CC_TAI | CC_HD | CC_ML |
		CC_CFE | CC_RL | CC_RED | CC_PE | CC_TPI |
		CC_PAD_EN | CC_PF);
	cmd |= (CC_NLC | CC_CFE); /* keep promiscuous mode disabled */

	amac_core_init_reset(privp);
	writel(cmd, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
	amac_core_clear_reset(privp);

	/* Enable clear MIB on read */
	tmp = readl(privp->hw.reg.amac_core);
	tmp |= DC_MROR;
	writel(tmp, privp->hw.reg.amac_core);

	/* PHY set smi_master to driver mdc_clk */
	tmp = readl(privp->hw.reg.amac_core + GMAC_PHY_CTRL_REG);
	tmp |= PC_MTE;
	writel(tmp, (privp->hw.reg.amac_core + GMAC_PHY_CTRL_REG));

	/* Clear persistent sw intstatus */
	writel(0, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));

	if (privp->hw.reg.switch_global_cfg) {
		/* Required to access the PHY's in
		 * switch-by-pass mode in some SoC's
		 * when the PHY needs to be connected
		 * directly to the mac.
		 */
		dev_info(&privp->pdev->dev,
			 "%s: Switch bypass mode\n", __func__);
		/* Configure Switch */
		tmp = readl(privp->hw.reg.switch_global_cfg);
		tmp |= BIT(CDRU_SWITCH_CFG_BYPASS_SWITCH);
		writel(tmp, (privp->hw.reg.switch_global_cfg));
	}

	if (privp->hw.reg.crmu_io_pad_ctrl) {
		/* Setup IO PAD CTRL */
		tmp = readl(privp->hw.reg.crmu_io_pad_ctrl);
		tmp &= ~BIT(CRMU_CHIP_IO_PAD_CONTROL__CDRU_IOMUX_FORCE_PAD_IN);
		writel(tmp, (privp->hw.reg.crmu_io_pad_ctrl));
	}

	/* Configure GMAC0 */
	/* enable one rx interrupt per received frame */
	writel(BIT(GMAC0_IRL_FRAMECOUNT_SHIFT),
	       (privp->hw.reg.amac_core + GMAC_INTR_RX_LAZY_REG));

	cmd = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);

	/* enable 802.3x tx flow control (honor received PAUSE frames) */
	cmd &= ~CC_RPI;
	/* Disbale loopback mode */
	cmd &= ~CC_ML;
	/* set the speed */
	bcm_amac_enet_set_speed(privp, privp->port.imp_port_speed, DUPLEX_FULL);

	amac_core_init_reset(privp);
	writel(cmd, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));
	amac_core_clear_reset(privp);

	return 0;
}

/* bcm_amac_core_enable() - Enable GMAC core
 * @privp: driver info pointer
 * @enable: Enable or disable
 *
 * Returns: none
 */
void bcm_amac_core_enable(struct bcm_amac_priv *privp, bool enable)
{
	u32 cmdcfg;

	amac_core_init_reset(privp);

	cmdcfg = readl(privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG);
	cmdcfg |= CC_SR;
	/* first deassert rx_ena and tx_ena while in reset */
	cmdcfg &= ~(CC_RE | CC_TE);
	/* write command config reg */
	writel(cmdcfg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	amac_core_clear_reset(privp);

	/* if not enable exit now */
	if (!enable)
		return;

	/* enable the mac transmit and receive paths now */
	udelay(2);
	cmdcfg &= ~CC_SR;
	cmdcfg |= (CC_RE | CC_TE);

	/* assert rx_ena and tx_ena when out of reset to enable the mac */
	writel(cmdcfg, (privp->hw.reg.amac_core + UNIMAC_CMD_CFG_REG));

	/* Clear Interrupts */
	writel(I_INTMASK, (privp->hw.reg.amac_core + GMAC_INT_STATUS_REG));
}

/* bcm_amac_tx_clean() - Prepare for transmission
 * @privp: driver info pointer
 *
 * Returns: none
 */
void bcm_amac_tx_clean(struct bcm_amac_priv *privp)
{
	struct amac_dma_priv *dmap = &privp->dma;
	const struct net_device *ndev = privp->ndev;
	int i;

	for (i = 0; i < dmap->tx_curr; i++) {
		if (privp->dma.sr_dma.enable_bounce) {
			if (dmap->tx_skb_list[i].skb_bounce) {
				dma_free_coherent(&privp->pdev->dev,
						  dmap->sr_dma.tx_bounce_data.
						  alloc_size,
						  dmap->tx_skb_list[i].
						  skb_bounce,
						  dmap->sr_dma.tx_bounce_data.
						  raw_addr);
			}
		} else {
			if (dmap->tx_skb_list[i].skb) {
				dma_unmap_single(&privp->pdev->dev,
						 dmap->tx_skb_list[i].dma_addr,
						 dmap->tx_skb_list[i].len,
						 DMA_TO_DEVICE);

				dev_kfree_skb_any(dmap->tx_skb_list[i].skb);
			} else {
				netdev_err(ndev, "invalid skb?\n");
			}
		}

		dmap->tx_skb_list[i].skb = NULL;
		dmap->tx_skb_list[i].len = 0;
		dmap->tx_skb_list[i].dma_addr = 0;
		if (privp->dma.sr_dma.enable_bounce)
			dmap->tx_skb_list[i].skb_bounce = NULL;
	}

	dmap->tx.index = 0;
	dmap->tx_curr = 0;
}

/* bcm_amac_set_rx_mode() - Set the rx mode callback
 * @ndev: net device pointer
 *
 * The API enables multicast or promiscuous mode as required. Otherwise
 * it disables multicast and promiscuous mode and adds ARL entries.
 */
void bcm_amac_set_rx_mode(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!netif_running(ndev))
		return;

	if (ndev->flags & (IFF_PROMISC | IFF_ALLMULTI)) {
		/* Enable promiscuous mode in switch bypass mode */
		amac_set_prom(privp, 1);
		return;
	}

		/* Disable promiscuous in switch bypass mode */
		amac_set_prom(privp, 0);

	/* In switch by pass mode we can only enable promiscuous mode
	 * to allow all packets.
	 */
	if ((ndev->flags & IFF_MULTICAST) && netdev_mc_count(ndev))
		/* In switch bypass mode there is no mac filtering
		 * so enable promiscuous mode to pass all packets up.
		 */
		amac_set_prom(privp, true);
}

/* bcm_amac_enable_tx_dma() - Enable the DMA
 * @privp: driver info pointer
 * @dir: TX or RX direction to enable
 *
 * Returns: 0 or error
 */
int bcm_amac_enable_tx_dma(struct bcm_amac_priv *privp, bool enable)
{
	u32 control;
	int status = 0;

	if (enable) {
		/* tx dma flags:
		 *  burst len (2**(N+4)): N=3
		 *  parity check: disabled
		 *  tx enable: enabled
		 */
		control = (3 << D64_XC_BL_SHIFT) | D64_XC_PD  | D64_XC_XE;

		writel(control,
		       (privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_REG));

		SPINWAIT(((status =
			  (readl(privp->hw.reg.amac_core +
				 GMAC_DMA_TX_STATUS0_REG) &
			   D64_XS0_XS_MASK)) != D64_XS0_XS_IDLE),
			  10, status);
	} else {
		/* Disable TX DMA */

		/* suspend tx DMA first, if active */
		status = readl(privp->hw.reg.amac_core +
			       GMAC_DMA_TX_STATUS0_REG)
				& D64_XS0_XS_MASK;

		if (status == D64_XS0_XS_ACTIVE) {
			status = readl(privp->hw.reg.amac_core +
				       GMAC_DMA_TX_CTRL_REG);
			status |= D64_XC_SE;

			writel(status, (privp->hw.reg.amac_core +
			       GMAC_DMA_TX_CTRL_REG));

			/* DMA engines are not disabled */
			/* until transfer finishes */
			SPINWAIT(
				 ((readl(privp->hw.reg.amac_core +
					GMAC_DMA_TX_STATUS0_REG)
				  & D64_XS0_XS_MASK) == D64_XS0_XS_ACTIVE),
				 10, status);
		}

		writel(0, (privp->hw.reg.amac_core + GMAC_DMA_TX_CTRL_REG));
		SPINWAIT(((status =
			  (readl(privp->hw.reg.amac_core +
				 GMAC_DMA_TX_STATUS0_REG)
			   & D64_XS0_XS_MASK)) != D64_XS0_XS_DISABLED),
			 10, status);
	}

	return status;
}

/* bcm_amac_enable_rx_dma() - Enable/Disable the RX DMA
 * @privp: driver data structure pointer
 * @enable: enable/disable rx dma
 *
 * Returns: 0 or error
 */
int bcm_amac_enable_rx_dma(struct bcm_amac_priv *privp, bool enable)
{
	u32 control;
	int status = 0;

	if (enable) {
		/* rx dma flags:
		 *  dma prefetch control: upto 4
		 *  burst len(2**(N+4)): default (N=1)
		 *  parity check: disabled
		 *  overflow continue: enabled
		 *  hw status size: 30
		 *  rx enable: enabled
		 */
		control = (D64_RC_PC_4_DESC << D64_RC_PC_SHIFT) |
			BIT(D64_XC_BL_SHIFT) |
			D64_XC_PD |
			D64_RC_OC |
			(HWRXOFF << D64_RC_RO_SHIFT) |
			D64_RC_RE;

		writel(control,
		       privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_REG);

		/* the rx descriptor ring should have
		 * the addresses set properly
		 * set the lastdscr for the rx ring
		 */
		writel((unsigned long)((privp->dma.rx.descp) +
			(privp->dma.dma_rx_desc_count - 1) * AMAC_RX_BUF_SIZE) &
			 D64_XP_LD_MASK,
			(privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));
	} else {
		/* Disable RX DMA */

		/* PR2414 WAR: DMA engines are not disabled until
		 * transfer completes
		 */
		writel(0, (privp->hw.reg.amac_core + GMAC_DMA_RX_CTRL_REG));
		SPINWAIT(((status =
			(readl(privp->hw.reg.amac_core +
				GMAC_DMA_RX_STATUS0_REG)
			& D64_RS0_RS_MASK)) != D64_RS0_RS_DISABLED),
			 10, status);
	}

	return status;
}

void bcm_amac_dma_stop(struct bcm_amac_priv *privp)
{
	u32 i;

	/* Stop the RX DMA */
	bcm_amac_enable_rx_dma(privp, false);

	/* Disable RX Interrupt */
	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, false);

	/* Free Rx buffers */
	for (i = 0; i < privp->dma.dma_rx_desc_count; i++)
		if (privp->dma.rx_skb_list[i].skb)
			amac_free_rx_skb(privp, AMAC_DMA_RX_BUF_LEN,
					 &privp->dma.rx_skb_list[i]);

	kfree(privp->dma.rx_skb_list);
	privp->dma.rx_skb_list = NULL;

	if (privp->dma.rx.raw_descp) {
		dma_free_coherent(&privp->pdev->dev,
				  privp->dma.rx.alloc_size,
				  privp->dma.rx.raw_descp,
				  privp->dma.rx.raw_addr);
		privp->dma.rx.descp = NULL;
		privp->dma.rx.raw_descp = NULL;
		privp->dma.rx.base_addr = 0;
		privp->dma.rx.raw_addr = 0;
	}

	/* Stop the TX DMA */
	bcm_amac_enable_tx_dma(privp, false);

	/* Disable TX Interrupt */
	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_TX, false);

	/* Free Tx buffers */
	bcm_amac_tx_clean(privp);

	if (privp->dma.tx.raw_descp) {
		dma_free_coherent(&privp->pdev->dev,
				  privp->dma.tx.alloc_size,
				  privp->dma.tx.raw_descp,
				  privp->dma.tx.raw_addr);
		privp->dma.tx.descp = NULL;
		privp->dma.tx.raw_descp = NULL;
		privp->dma.tx.raw_addr = 0;
		privp->dma.tx.base_addr = 0;
	}

	kfree(privp->dma.tx_skb_list);
	privp->dma.tx_skb_list = NULL;
}

/* bcm_amac_dma_start() - Initialize and start the DMA.
 * @privp: driver info pointer
 *
 * Both RX and TX are initialized.
 * Only RX is enabled, TX is enabled as required.
 *
 * Returns: '0' or error
 */
int bcm_amac_dma_start(struct bcm_amac_priv *privp)
{
	int rc;

	if (!privp)
		return -EINVAL;

	rc = amac_dma_rx_init(privp); /* Initialize RX DMA */
	if (rc) {
		netdev_err(privp->ndev, "Failed!! DMA RX Init\n");
		goto dma_start_err;
	}

	rc = amac_dma_tx_init(privp); /* Initialize TX DMA */
	if (rc != 0) {
		netdev_err(privp->ndev, "Failed!! DMA TX Init\n");
		goto dma_start_err;
	}

	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_TX, true);
	bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, true);

	/* Enable RX DMA */
	rc = bcm_amac_enable_rx_dma(privp, true);
	if (rc) {
		netdev_err(privp->ndev, "Rx DMA enable failed!\n");
		goto dma_start_err;
	}

	atomic_set(&privp->dma.tx_dma_busy, BCM_AMAC_DMA_FREE);

	return 0;

dma_start_err:
	/* Stop DMA */
	bcm_amac_dma_stop(privp);

	return rc;
}

/* bcm_amac_dma_get_rx_data() - Retrieves RX data if available.
 * @privp: driver info pointer
 * @skb: skb pointer
 *
 * If data is available, the function updates the rx pointer register.
 * It also strips out the hw specific status from the data.
 *
 * Returns: '0' if no frames, 'length' of packet or error
 */
int bcm_amac_dma_get_rx_data(struct bcm_amac_priv *privp,
			     struct sk_buff **skbp)
{
	u32 rd_offset;
	int len;
	unsigned char *bufp;
	u32 rx_ptr;
	struct amac_dma64_desc *descp;
	struct amac_dma64_desc *rx_ptr_desc;
	struct amac_dma_priv *dmap = &privp->dma;
	struct skb_list_node  read_skb_node;
	struct skb_list_node *const node =
		&dmap->rx_skb_list[dmap->rx.index];

	/* Check to see if there is new data */
	rd_offset = amac_dma_check_rx_done(privp);
	if (!rd_offset)
		return 0;

	/* Get frame descriptor */
	descp = (&((struct amac_dma64_desc *)(dmap->rx.descp))[dmap->rx.index]);
	descp->ctrl2 = cpu_to_le32(AMAC_DMA_RX_BUF_LEN);

	rx_ptr_desc = (struct amac_dma64_desc *)dmap->rx.base_addr;
	rx_ptr = ((unsigned long)(&rx_ptr_desc[dmap->rx.index]) & 0xFFFFFFFF);

	/* Save current skb, we will allocate a new one in place */
	memcpy(&read_skb_node, node, sizeof(struct skb_list_node));

	/* Allocate and re-arm the descriptor with new skb */
	len = amac_alloc_rx_skb(privp,
				AMAC_DMA_RX_BUF_LEN,
				node);
	if (len) {
		netdev_err(privp->ndev, "SKB alloc error\n");
		/* skb allocation error, no new skb available.
		 * So leave the existing skb in place for re-use.
		 * Will not process the frame further, dropping it.
		 * That's the best we can do at this point.
		 */
		goto rx_dma_data_done;
	}

	/* Re-arm descriptor with new skb */
	descp->addrlow = DMA_LOW_ADDR(node->dma_addr);
	descp->addrhigh = DMA_HIGH_ADDR(node->dma_addr);

	dma_sync_single_for_cpu(&privp->pdev->dev,
				read_skb_node.dma_addr,
				AMAC_DMA_RX_BUF_LEN,
				DMA_FROM_DEVICE);

	*skbp = read_skb_node.skb;
	/* Process the SKB with data */

	if (privp->dma.sr_dma.enable_bounce)
		bufp = (unsigned char *)read_skb_node.skb_bounce;
	else
		bufp = (*skbp)->data;

	len = cpu_to_le16(*((u16 *)bufp));

	/* Received an invalid frame length */
	if (len > AMAC_DMA_RX_BUF_LEN) {
		netdev_err(privp->ndev, "Invalid frame length, len=%d\n", len);

		/* Dropping frame, free the saved skb node */
		amac_free_rx_skb(privp, AMAC_DMA_RX_BUF_LEN, &read_skb_node);
		*skbp = NULL;

		len = -EBADMSG;
		goto rx_dma_data_done;
	}

	bufp += rd_offset; /* Point to real data */
	/* Realign the data in SKB */
	memmove((void *)(*skbp)->data, (void *)bufp, len);

	/* we are done with bounce buffer, give it back. */
	if (privp->dma.sr_dma.enable_bounce)
		dma_free_coherent(&privp->pdev->dev,
				  AMAC_DMA_RX_BUF_LEN,
				  bufp,
				  read_skb_node.dma_addr);

rx_dma_data_done:
	/* Update RX pointer */
	writel(rx_ptr, (privp->hw.reg.amac_core + GMAC_DMA_RX_PTR_REG));

	/* Increment descp index */
	if (++dmap->rx.index >= privp->dma.dma_rx_desc_count)
		dmap->rx.index = 0;

	return len;
}

/* bcm_amac_tx_send_packet() - Ethernet TX routine.
 * @privp: driver info pointer
 *
 * Called within the TX tasklet with packets in fifo.
 * Gets data from the queue, formats the descriptor ,
 * setup and starts the TX DMA.
 *
 * Returns: none
 */
void bcm_amac_tx_send_packet(struct bcm_amac_priv *privp)
{
	struct amac_dma_priv *dmap = &privp->dma;
	int curr = 0;
	int desc_idx = 0;
	struct amac_dma64_desc *descp = NULL;
	u32 last_desc;
	struct sk_buff *skb;
	char *tx_skb_bounce;
	void *p_skb;
	u32 len;
	dma_addr_t buf_dma;

	/* Build descriptor chain */
	while ((len = kfifo_out_spinlocked(&dmap->txfifo,
					   (unsigned char *)&p_skb,
					   sizeof(struct sk_buff *),
					   &privp->lock)) ==
		   sizeof(unsigned char *)) {
		if (privp->dma.sr_dma.enable_bounce)
			tx_skb_bounce = (unsigned char *)p_skb;
		else
			skb = (struct sk_buff *)p_skb;
		/* Indicate we are busy sending a packet */
		if (!atomic_read(&privp->dma.tx_dma_busy))
			atomic_set(&privp->dma.tx_dma_busy, BCM_AMAC_DMA_BUSY);

		if (privp->dma.sr_dma.enable_bounce) {
			len = privp->dma.sr_dma.bounce_tx_len;
			buf_dma = dmap->sr_dma.tx_bounce_data.raw_addr;
		} else {
			len = skb->len;
		}

		if (!privp->dma.sr_dma.enable_bounce) {
			/* Timestamp the packet */
			skb_tx_timestamp(skb);

			buf_dma = dma_map_single(&privp->pdev->dev, skb->data,
						 len, DMA_TO_DEVICE);
			if (dma_mapping_error(&privp->pdev->dev, buf_dma)) {
				netdev_err(privp->ndev, "TX: DMA mapping Failed !!\n");

				dev_kfree_skb_any(skb);

				privp->ndev->stats.tx_bytes -= len;
				privp->ndev->stats.tx_packets--;
				privp->ndev->stats.tx_fifo_errors++;
				privp->ndev->stats.tx_dropped++;
				continue;
			}
		}

		descp = (&((struct amac_dma64_desc *)
					(dmap->tx.descp))[(desc_idx)]);

		descp->addrhigh = DMA_HIGH_ADDR(buf_dma);
		descp->addrlow = DMA_LOW_ADDR(buf_dma);
		descp->ctrl1 = cpu_to_le32((D64_CTRL1_SOF | D64_CTRL1_EOF));
		descp->ctrl2 = cpu_to_le32((len & D64_CTRL2_BC_MASK));

		/* Add skb to list */
		dmap->tx_skb_list[curr].skb = skb;
		dmap->tx_skb_list[curr].len = len;
		dmap->tx_skb_list[curr].dma_addr = buf_dma;
		if (privp->dma.sr_dma.enable_bounce)
			dmap->tx_skb_list[curr].skb_bounce =
						 (unsigned char *)tx_skb_bounce;
		desc_idx++;
		curr++;
	}

	if (descp) {
		dmap->tx_curr = curr;
		dmap->tx.index = desc_idx;

		/* Interrupt after the last one*/
		descp->ctrl1 |= cpu_to_le32(D64_CTRL1_IOC);

		descp = (&((struct amac_dma64_desc *)
					(dmap->tx.descp))[(desc_idx)]);

		/* Mark last descriptor as EOT */
		descp->ctrl1 = cpu_to_le32(D64_CTRL1_EOT);

		last_desc = ((unsigned long)(&((struct amac_dma64_desc *)
			(dmap->tx.base_addr))[desc_idx]));
		last_desc &= D64_XP_LD_MASK;

		/* Disable TX DMA */
		bcm_amac_enable_tx_dma(privp, false);

		/* initailize the DMA channel */
		writel(DMA_LOW_ADDR((dma_addr_t)
			&((struct amac_dma64_desc *)(dmap->tx.base_addr))[0]),
		       (privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_LO_REG));
		writel(DMA_HIGH_ADDR((dma_addr_t)&
			((struct amac_dma64_desc *)(dmap->tx.base_addr))[0]),
		       (privp->hw.reg.amac_core + GMAC_DMA_TX_ADDR_HI_REG));

		/* Ensure the DMA descriptors are setup before progressing */
		wmb();

		/* Enable TX DMA Interrupts */
		bcm_amac_enable_intr(privp, BCM_AMAC_DIR_TX, true);

		/* Enable TX DMA */
		bcm_amac_enable_tx_dma(privp, true);

		/* update the dma last descriptor */
		writel(last_desc,
		       (privp->hw.reg.amac_core + GMAC_DMA_TX_PTR_REG));
	}
}

