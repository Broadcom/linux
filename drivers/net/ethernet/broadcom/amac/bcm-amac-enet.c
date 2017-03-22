/*
 * Copyright (C) 2015-2017 Broadcom
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

#include <linux/etherdevice.h>
#include <linux/module.h>
#include <linux/of_net.h>
#include <linux/of_reserved_mem.h>

#include "bcm-amac-core.h"
#include "bcm-amac-enet.h"

#define TX_TIMEOUT (5 * HZ)

/* Netlink */
#define NETLINK_MAX_PAYLOAD 256
#define KERNEL_PID 0
#define DST_GROUP 1 /* to mcast group 1<<0 */

struct bcm_amac_cmd_line_param {
	char mac_addr[20];
};

static struct bcm_amac_cmd_line_param cmdline_params = {
	"00:00:00:00:00:00"
};

static int bcm_amac_enet_remove(struct platform_device *pdev);
static int amac_enet_start(struct bcm_amac_priv *privp);
static int bcm_amac_enet_probe(struct platform_device *pdev);
static int bcm_amac_enet_do_ioctl(struct net_device *dev,
				  struct ifreq *ifr, int cmd);
static int bcm_amac_get_dt_data(struct bcm_amac_priv *privp);
static int bcm_amac_enet_open(struct net_device *dev);
static int amac_enet_set_mac(struct net_device *dev, void *addr);
static void amac_tx_task(unsigned long data);
static int bcm_amac_enet_close(struct net_device *dev);
static int bcm_amac_enet_hard_xmit(struct sk_buff *skb, struct net_device *dev);
static void bcm_amac_enet_tx_timeout(struct net_device *dev);
static int bcm_amac_enet_rx_poll(struct napi_struct *napi, int quota);
static struct net_device_stats *bcm_amac_enet_get_stats(struct net_device *dev);

static const struct net_device_ops bcm_amac_enet_ops = {
	.ndo_open = bcm_amac_enet_open,
	.ndo_stop = bcm_amac_enet_close,
	.ndo_start_xmit = bcm_amac_enet_hard_xmit,
	.ndo_tx_timeout = bcm_amac_enet_tx_timeout,
	.ndo_get_stats = bcm_amac_enet_get_stats,
	.ndo_set_rx_mode = bcm_amac_set_rx_mode,
	.ndo_do_ioctl = bcm_amac_enet_do_ioctl,
	.ndo_set_mac_address = amac_enet_set_mac,
	.ndo_validate_addr = eth_validate_addr,
	.ndo_change_mtu = eth_change_mtu
};

/* bcm_amac_enet_get_stats() - Get device status
 * @dev: net device pointer
 *
 * Returns: network statistics
 */
static struct net_device_stats *bcm_amac_enet_get_stats(struct net_device *dev)
{
	return &dev->stats;
}

/* bcm_amac_isr() - GMAC ISR Routine. Handles both RX and TX interrupts.
 * @irq: intr number
 * @userdata: driver info data pointer
 *
 * RX interrupt: the interrupt is disabled, cleared and the
 *   NAPI routine is invoked.
 *
 * TX interrupt: interrupt and DMA are both disabled and the tasklet is
 *   scheduled, in case there is more data in the queue.
 *
 * Returns: interrupt handler status
 */
irqreturn_t bcm_amac_isr(int irq, void *userdata)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)userdata;
	u32 intr;
	irqreturn_t rc = IRQ_NONE;

	intr = readl(privp->hw.reg.amac_core + GMAC_INT_STATUS_REG);

	if (intr & I_RI) {
		/* RX DMA complete interrupt */
		if (likely(netif_running(privp->ndev))) {
			/* Disable RX DMA Interrupt*/
			bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, false);

			napi_schedule(&privp->napi);
		}
		rc = IRQ_HANDLED;
	}

	if (intr & I_XI_ALL) {
		/* TX DMA complete interrupt */
		if (likely(netif_running(privp->ndev))) {
			/* Disable TX DMA */
			bcm_amac_enable_tx_dma(privp, false);

			bcm_amac_clear_intr(privp, BCM_AMAC_DIR_TX);

			atomic_set(&privp->dma.tx_dma_busy, BCM_AMAC_DMA_FREE);

			/* trigger tx processing in case packets are waiting */
			tasklet_schedule(&privp->tx_tasklet);

			if (unlikely(netif_queue_stopped(privp->ndev)))
				netif_wake_queue(privp->ndev);
		}
		rc = IRQ_HANDLED;
	}

	return rc;
}

/* amac_enet_start() - Initialize core, phy,mac address etc.
 * @privp: device info pointer
 *
 * @Returns: 0 or error
 */
static int amac_enet_start(struct bcm_amac_priv *privp)
{
	int rc;
	const void *mac_addr;
	char parsed_cmd_mac[14];

	/* Look for mac addr passed via cmdline or dt */
	if (is_valid_ether_addr(cmdline_params.mac_addr)) {
		mac_pton(cmdline_params.mac_addr, parsed_cmd_mac);
		mac_addr = (const void *)parsed_cmd_mac;
	} else {
		/* get mac_addr from DT */
		mac_addr = of_get_mac_address(privp->pdev->dev.of_node);
	}

	if (mac_addr) {
		ether_addr_copy(privp->ndev->dev_addr, mac_addr);
		amac_write_mac_address(privp, (u8 *)mac_addr);
	} else {
		dev_err(&privp->pdev->dev,
			"Error: no mac address specified\n");
		return -EFAULT;
	}

	rc = bcm_amac_core_init(privp);
	if (rc != 0) {
		dev_err(&privp->pdev->dev, "core init failed!\n");
		return rc;
	}

	/* Disable RX and TX DMA just in case it was enabled earlier */
	rc = bcm_amac_enable_rx_dma(privp, false);
	if (rc) {
		dev_err(&privp->pdev->dev, "Rx DMA config failed!\n");
		return rc;
	}

	rc = bcm_amac_enable_tx_dma(privp, false);
	if (rc) {
		dev_err(&privp->pdev->dev, "Tx DMA config failed!\n");
		return rc;
	}

	/* Initialize the PHY's */
	rc = bcm_amac_gphy_init(privp);
	if (rc) {
		dev_err(&privp->pdev->dev, "PHY Init failed\n");
		return rc;
	}

	/* Register GMAC Interrupt */
	rc = devm_request_irq(&privp->pdev->dev, privp->hw.intr_num,
			      bcm_amac_isr, IRQF_SHARED, "amac_enet", privp);
	if (rc) {
		netdev_err(privp->ndev,
			   "IRQ request failed, irq=%i, err=%i\n",
			   privp->hw.intr_num, rc);
		goto err_amac_enet_start;
	}

	return 0;

err_amac_enet_start:
	bcm_amac_gphy_exit(privp);

	return rc;
}

static int bcm_amac_enet_stop(struct bcm_amac_priv *privp)
{
	devm_free_irq(&privp->pdev->dev, privp->hw.intr_num, privp);
	bcm_amac_gphy_exit(privp);

	return 0;
}

/* amac_tx_task() - Packet transmission routing.
 * @data: device info pointer
 *
 * This api is registered with the tx tasklet. The task performs the
 * transmission of packets if the tx is free
 *
 * Returns: none
 */
static void amac_tx_task(unsigned long data)
{
	struct bcm_amac_priv *privp = (struct bcm_amac_priv *)data;

	/* If TX DMA is busy return */
	if (atomic_read(&privp->dma.tx_dma_busy))
		return;

	bcm_amac_tx_clean(privp);
	bcm_amac_tx_send_packet(privp);

	if (privp->dma.sr_dma.enable_bounce)
		if (kfifo_avail(&privp->dma.txfifo) > 0)
			netif_wake_queue(privp->ndev);
}

/* bcm_amac_enet_rx_poll() - Packet reception routine
 * @napi: napi structure pointer
 * @quota: quota info
 *
 * This NAPI RX routine gets the received packets and sends it to the upper
 * layers. IT strips off any Broadcom Tags found in the packet. It also
 * updates the packet stats.
 * It enables the RX interrupt based on the quota.
 *
 * Returns: quota used
 */
static int bcm_amac_enet_rx_poll(struct napi_struct *napi, int quota)
{
	struct bcm_amac_priv *privp =
		container_of(napi, struct bcm_amac_priv, napi);
	int used = 0;
	struct sk_buff *skb;
	int len;
	void *bufp;
	static int last_used;
	static int prev_used[8] = {0};
	static int prev_idx;

	if (!privp)
		return -EINVAL;

	while (used < quota) {
		/* Check and retrieve rx packets */
		len = bcm_amac_dma_get_rx_data(privp, &skb);
		if (len > 0) {
			/* Check frame length and discard if invalid*/
			if (unlikely(len < ETH_ZLEN)) {
				netdev_warn(privp->ndev,
					    "bad frame: len=%i\n", len);

				dev_kfree_skb_any(skb);

				/* Update error stats */
				privp->ndev->stats.rx_dropped++;
				continue;
			}
			/* Process the packet */
			bufp = skb->data;

			/* Update remainder of socket buffer information */
			skb_put(skb, len);

			skb->dev = privp->ndev;
			skb->protocol = eth_type_trans(skb, privp->ndev);
			skb->ip_summed = CHECKSUM_NONE;

			/* Update Stats */
			privp->ndev->stats.rx_bytes += len;
			privp->ndev->stats.rx_packets++;
			if (is_multicast_ether_addr((char *)bufp))
				privp->ndev->stats.multicast++;

			/* Pass the packet up for processing */
			netif_receive_skb(skb);

			used++;
		} else if (len == 0) {
			break; /* no frames to process */
		} else if (len == -EBADMSG) {
			/* Update error stats */
			privp->ndev->stats.rx_dropped++;
			privp->ndev->stats.rx_length_errors++;

			if (netif_msg_rx_err(privp) && net_ratelimit())
				netdev_err(privp->ndev,
					   "rx frame length err, used=%d\n",
					   used);

			continue;
		} else {
			/* Error retriving frame */

			/* Update error stats */
			privp->ndev->stats.rx_dropped++;
			privp->ndev->stats.rx_fifo_errors++;

			if (netif_msg_rx_err(privp) && net_ratelimit())
				netdev_err(privp->ndev,
					   "rx skb alloc err, used=%d\n", used);

			/* Don't try to read any more frames
			 * just drop out of the loop
			 */
			break;
		}
	}

	/* If quota not fully consumed, exit polling mode */
	if (likely(used < quota)) {
		napi_complete(napi);

		/* Enable RX Interrupt */
		bcm_amac_enable_intr(privp, BCM_AMAC_DIR_RX, true);
	}

	last_used = used;
	prev_used[prev_idx++ % 8] = used;

	return used;
}

/* bcm_amac_enet_open() - Ethernet interface open routine
 * @ndev: network device pointer
 *
 * The routine is called when the Ethernet interface is opened.
 * This stats the DMA's, enables the RX (NAPI), powers up the PHY,
 * starts up the TX queue etc.
 *
 * Returns: '0' for success or the error number
 */
static int bcm_amac_enet_open(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc;

	/* Allocate a TX fifo to stash to hold skb pointers */
	if (privp->dma.sr_dma.enable_bounce)
		rc = kfifo_alloc(&privp->dma.txfifo,
				 AMAC_SRDMA_TX_MAX_QUEUE_LEN * sizeof(void *),
				 GFP_KERNEL | GFP_DMA);
	else
		rc = kfifo_alloc(&privp->dma.txfifo,
				 AMAC_DMA_TX_MAX_QUEUE_LEN * sizeof(void *),
				 GFP_KERNEL | GFP_DMA);

	if (rc) {
		netdev_err(ndev,
			   "cannot alloc tx fifo, err=%i\n", rc);
		return rc;
	}

	/* Enable napi before rx interrupts */
	napi_enable(&privp->napi);

	bcm_amac_core_enable(privp, true);

	/* Start DMA */
	rc = bcm_amac_dma_start(privp);
	if (rc) {
		netdev_err(ndev, "Failed to start DMA\n");
		goto err_free_kfifo;
	}

	/* Power-up the PHY(s) */
	rc = bcm_amac_gphy_powerup(privp, true);
	if (rc) {
		netdev_err(ndev, "Failed to powerup PHY\n");
		goto err_phy_powerup;
	}

	netif_wake_queue(ndev);

	return rc;

err_phy_powerup:
	bcm_amac_dma_stop(privp);

err_free_kfifo:
	bcm_amac_core_enable(privp, false);
	napi_disable(&privp->napi);
	kfifo_free(&privp->dma.txfifo);
	netdev_err(ndev, "%s, open failed!\n", __func__);

	return rc;
}

/* bcm_amac_enet_close() - Ethernet interface close routine
 * @ndev: network device pointer
 *
 * The routine is called when the Ethernet interface is closed or disabled.
 * This stops the DMA, disables interrupts, switches off the PHY, disables
 * NAPI routine etc.
 *
 * Returns: '0'
 */
static int bcm_amac_enet_close(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	netif_stop_queue(ndev);

	/* Shutdown PHY(s) */
	bcm_amac_gphy_powerup(privp, false);

	/* Disable RX NAPI */
	napi_disable(&privp->napi);

	/* Stop DMA */
	bcm_amac_dma_stop(privp);

	bcm_amac_core_enable(privp, false);

	kfifo_free(&privp->dma.txfifo);
	return 0;
}

/* amac_enet_set_mac() - Assigns the mac address to the interface.
 * @dev: network device pointer
 * @addr: mac address
 *
 * Returns: '0' or error
 */
static int amac_enet_set_mac(struct net_device *ndev, void *addr)
{
	int rc;
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	rc = eth_mac_addr(ndev, addr);
	if (rc) {
		netdev_err(ndev, "cannot setup MAC, err=%i\n", rc);
		return rc;
	}

	amac_write_mac_address(privp, ndev->dev_addr);
	memcpy(privp->cur_etheraddr.sa_data, ndev->dev_addr, 6);

	return 0;
}

/* bcm_amac_enet_hard_xmit() - hard transmit routine
 * @skb: skb buffer pointer with data to be transmitted
 * @ndev: network device pointer.
 *
 * The hard transmit routine is called by the upper layers to transmit data.
 * The data is part of the skb pointer. The interface adds broadcom tags if
 * enabled, inserts the skb into the internal transmit queue and schedules
 * the transmit task to run.
 *
 * Returns: NETDEV_TX_OK
 */
static int bcm_amac_enet_hard_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct bcm_amac_priv *privp;
	int len;
	int rc;
	void *bounce_skb;
	struct amac_dma_priv *dma_p = NULL;

	rc = NETDEV_TX_OK;
	privp = netdev_priv(ndev);
	if (privp->dma.sr_dma.enable_bounce)
		dma_p = &privp->dma;

	if (unlikely(skb->len < ETH_ZLEN)) {
		/* Clear the padded memory to avoid 'etherleak'
		 * vulnerability
		 */
		memset(skb->data + skb->len, 0, (ETH_ZLEN - skb->len));
		skb->len = ETH_ZLEN;
	}

	if (privp->dma.sr_dma.enable_bounce) {
		dma_p->sr_dma.tx_bounce_data.alloc_size = skb->len;
		bounce_skb = dma_alloc_coherent(&privp->pdev->dev,
						dma_p->sr_dma.tx_bounce_data.
						alloc_size,
						&dma_p->sr_dma.tx_bounce_data.
						raw_addr,
						GFP_KERNEL | GFP_DMA);
		if (!bounce_skb) {
			rc = -ENOMEM;
			goto err_enet_bounce_alloc;
		}

		privp->dma.sr_dma.bounce_tx_len = skb->len;
		memcpy(bounce_skb, (void *)(skb)->data, skb->len);

		len = kfifo_in_locked(&privp->dma.txfifo,
				      (unsigned char *)&bounce_skb,
				      sizeof(bounce_skb), &privp->lock);

		if (kfifo_is_full(&privp->dma.txfifo)) {
			netif_stop_queue(ndev);
			rc = NETDEV_TX_OK;
		}
	} else {
		/* Insert skb pointer into fifo */
		len = kfifo_in_locked(&privp->dma.txfifo, (unsigned char *)&skb,
				      sizeof(skb), &privp->lock);

		if (unlikely(len != sizeof(skb))) {
			/* Not enough space, which shouldn't
			 * happen since the queue
			 * should have been stopped already.
			 */
			netif_stop_queue(ndev);
			netdev_info(privp->ndev,
				    "xmit called with no tx desc avail!");

			ndev->stats.tx_fifo_errors++;

			rc = NETDEV_TX_OK;
			goto err_enet_hard_xmit;
		}
	}

	tasklet_schedule(&privp->tx_tasklet);

	/* Update stats */
	if (is_multicast_ether_addr((char *)skb->data))
		ndev->stats.multicast++;

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;

	if (privp->dma.sr_dma.enable_bounce)
		kfree_skb(skb);
	return rc;

err_enet_hard_xmit:
	dma_free_coherent(&privp->pdev->dev,
			  dma_p->sr_dma.tx_bounce_data.alloc_size,
			  bounce_skb,
			  dma_p->sr_dma.tx_bounce_data.raw_addr);

err_enet_bounce_alloc:
	kfree_skb(skb);
	/* Update stats */
	ndev->stats.tx_dropped++;
	ndev->stats.tx_fifo_errors++;

	return rc;
}

/* bcm_amac_enet_tx_timeout() - Transmit timeout routine
 * @ndev - network device pointer
 */
static void bcm_amac_enet_tx_timeout(struct net_device *ndev)
{
	struct bcm_amac_priv *privp;

	privp = netdev_priv(ndev);

	netdev_warn(ndev, "tx timeout\n");

	ndev->stats.tx_errors++;

	netif_wake_queue(ndev);
}

/* bcm_amac_enet_do_ioctl() - ioctl support in the driver
 * @ndev: network device
 * @ifr: ioctl data pointer
 * @cmd: ioctl cmd
 *
 * Returns: '0' or error
 */
static int bcm_amac_enet_do_ioctl(struct net_device *ndev,
				  struct ifreq *ifr, int cmd)
{
	int rc;
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!netif_running(ndev))
		return -EINVAL;

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		if (privp->port.ext_port.phydev)
			rc = phy_mii_ioctl(privp->port.ext_port.phydev,
					   ifr, cmd);
		else
			rc = -ENODEV;
		break;
	break;

	default:
		rc = -EOPNOTSUPP;
	}

	return rc;
}

/* bcm_amac_get_dt_data() - Retrieve data from the device tree
 * @pdev: platform device data structure
 * @privp: driver privagte data structure
 *
 * Returns: '0' or error
 */
static int bcm_amac_get_dt_data(struct bcm_amac_priv *privp)
{
	struct platform_device *pdev = privp->pdev;
	struct device_node *np = pdev->dev.of_node;
	struct resource *iomem;
	struct device_node *phy_node;
	int rc;

	/* GMAC Core register */
	iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "core_base");
	privp->hw.reg.amac_core = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.amac_core)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of amac_core failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.amac_core);
	}

	/* optional RGMII base register */
	iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					     "rgmii_base");
	if (iomem) {
		privp->hw.reg.rgmii_regs =
				devm_ioremap_resource(&pdev->dev, iomem);
		if (IS_ERR(privp->hw.reg.rgmii_regs)) {
			dev_err(&privp->pdev->dev,
				"%s: ioremap of rgmii failed\n",
				__func__);
			return PTR_ERR(privp->hw.reg.rgmii_regs);
		}
	}

	/* AMAC IDM Base register */
	iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					     "amac_idm_base");
	privp->hw.reg.amac_idm_base = devm_ioremap_resource(&pdev->dev, iomem);
	if (IS_ERR(privp->hw.reg.amac_idm_base)) {
		dev_err(&privp->pdev->dev,
			"%s: ioremap of amac_idm_base failed\n",
			__func__);
		return PTR_ERR(privp->hw.reg.amac_idm_base);
	}

	privp->switch_mode = of_property_read_bool(np, "brcm,enet-switch-mode");

	if (!privp->switch_mode) {
		/* optional SWITCH GLOBAL CONFIG register
		 * This is only required for switch-by-pass
		 * mode to connect IMP port to the PHY
		 */
		iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "switch_global_base");
		if (iomem) {
			privp->hw.reg.switch_global_cfg =
				devm_ioremap_resource(&pdev->dev, iomem);
			if (IS_ERR(privp->hw.reg.switch_global_cfg)) {
				dev_err(&privp->pdev->dev,
					"%s: ioremap of switch_global_cfg failed\n",
					__func__);
				return PTR_ERR(privp->hw.reg.switch_global_cfg);
			}
		}
	}

	/* optional CRMU IO PAD CTRL register */
	iomem = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					     "crmu_io_pad_ctrl");
	if (iomem) {
		privp->hw.reg.crmu_io_pad_ctrl =
			devm_ioremap_resource(&pdev->dev, iomem);
		if (IS_ERR(privp->hw.reg.crmu_io_pad_ctrl)) {
			dev_err(&privp->pdev->dev,
				"%s: ioremap of crmu_io_pad_ctrl failed\n",
				__func__);
			return PTR_ERR(privp->hw.reg.crmu_io_pad_ctrl);
		}
	}

	/* Read Interrupt */
	privp->hw.intr_num = platform_get_irq(pdev, 0);
	if (privp->hw.intr_num == 0) {
		dev_err(&privp->pdev->dev,
			"%s: gmac0 interrupt not specified\n",
			__func__);
		return -EINVAL;
	}

	/* AMAC handles the PHY for SoC's without an
	 * internal switch or 'switch-by-pass' mode in
	 * case of SoC's with switch.
	 *
	 * In both cases above, the PHY connects directly
	 * to the IMP port.
	 *
	 * With an internal switch involved all PHY
	 * handling will be done by the switch.
	 */
	phy_node = of_parse_phandle(np, "phy-handle", 0);
	if (!phy_node) {
		dev_err(&privp->pdev->dev,
			"%s: phy-handle not specified\n",
			__func__);
		return -EINVAL;
	}

	/* max-speed setting for the IMP port */
	rc = of_property_read_u32(np, "max-speed",
			&privp->port.imp_port_speed);
	if (rc)
		privp->port.imp_port_speed = AMAC_PORT_DEFAULT_SPEED;

	/* Get internal / external PHY info */

	/* NOTE: 'External' only refers to the port type
	 * The PHY that is connected can be an internal PHY
	 * or an external PHY.
	 * IMP is considered 'internal' port.
	 */
	privp->port.ext_port.phy_node = phy_node;

	privp->port.ext_port.phy_mode = of_get_phy_mode(np);
	if (privp->port.ext_port.phy_mode < 0) {
		dev_err(&privp->pdev->dev,
			"Invalid phy interface specified\n");
		return -EINVAL;
	}

	privp->port.ext_port.lswap =
		of_property_read_bool(np, "brcm,enet-phy-lswap");

	privp->port.ext_port.pause_disable =
		of_property_read_bool(np, "brcm,enet-pause-disable");

	privp->port.ext_port.phy54810_rgmii_sync =
		of_property_read_bool(np, "brcm,enet-phy54810-rgmii-sync");

	return 0;
}

/* bcm_amac_enet_probe() - driver probe function
 * @pdev: platform device pointer
 *
 * Returns: '0' or error
 */
static int bcm_amac_enet_probe(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct bcm_amac_priv *privp;
	int rc;

	/* Initialize driver resource */
	ndev = alloc_etherdev(sizeof(struct bcm_amac_priv));
	if (!ndev) {
		dev_err(&pdev->dev,
			"%s: Failed to allocate device\n", __func__);
		return -ENOMEM;
	}

	privp = netdev_priv(ndev);
	memset(privp, 0, sizeof(struct bcm_amac_priv));
	privp->pdev = pdev;
	privp->ndev = ndev;

	privp->dma.dma_rx_desc_count = AMAC_DMA_RX_DESC_CNT;
	privp->dma.dma_tx_desc_count = AMAC_DMA_TX_DESC_CNT;
	privp->dma.tx_max_pkts = AMAC_DMA_TX_MAX_QUEUE_LEN;
	privp->dma.sr_dma.enable_bounce = false;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "brcm,amac-enet-extsram")) {
		privp->dma.dma_rx_desc_count = AMAC_SRDMA_RX_DESC_CNT;
		privp->dma.dma_tx_desc_count = AMAC_SRDMA_TX_DESC_CNT;
		privp->dma.tx_max_pkts = AMAC_SRDMA_TX_MAX_QUEUE_LEN;
		privp->dma.sr_dma.enable_bounce = true;
		of_reserved_mem_device_init(&pdev->dev);
		dev_info(&pdev->dev, "amac: enabled bounce\n");
	}

	/* Read DT data */
	rc = bcm_amac_get_dt_data(privp);
	if (rc != 0) {
		dev_err(&pdev->dev,
			"%s: Failed to get platform data\n", __func__);
		goto amac_err_plat_data;
	}

	spin_lock_init(&privp->lock);

	platform_set_drvdata(pdev, ndev);
	SET_NETDEV_DEV(ndev, &pdev->dev);
	bcm_amac_set_ethtool_ops(ndev);

	ndev->netdev_ops = &bcm_amac_enet_ops;
	ndev->watchdog_timeo = TX_TIMEOUT;

	if (of_device_is_compatible(pdev->dev.of_node,
				    "brcm,amac-enet-extsram")) {
		netif_napi_add(ndev, &privp->napi, bcm_amac_enet_rx_poll,
			       NAPI_POLL_SRAM_WEIGHT);
		ndev->tx_queue_len = AMAC_SRDMA_TX_MAX_QUEUE_LEN;
	} else {
		netif_napi_add(ndev, &privp->napi, bcm_amac_enet_rx_poll,
			       NAPI_POLL_WEIGHT);
		ndev->tx_queue_len = AMAC_DMA_TX_MAX_QUEUE_LEN;
	}

	ndev->features &= ~(NETIF_F_SG | NETIF_F_FRAGLIST);

	/* Clear stats */
	memset(&ndev->stats, 0, sizeof(ndev->stats));

	/* Start ethernet block */
	rc = amac_enet_start(privp);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: Failed to start ethernet block\n", __func__);
		goto amac_err_plat_data;
	}

	tasklet_init(&privp->tx_tasklet, amac_tx_task, (unsigned long)privp);

	rc = register_netdev(ndev);
	if (rc) {
		dev_err(&pdev->dev,
			"%s: netdev register failed\n", __func__);
		goto amac_err_stop_eth;
	}

	pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
	/* Keep dma ops of both ndev & pdev in sync */
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	ndev->dev.coherent_dma_mask = pdev->dev.coherent_dma_mask;
	/* FIX ME */
	/* set_dma_ops(&ndev->dev, pdev->dev.archdata.dma_ops); */

	/* create netlink socket to send link notifications */
	privp->nl_sk = netlink_kernel_create(&init_net,
		NETLINK_USERSOCK, (struct netlink_kernel_cfg *)NULL);

	netdev_info(ndev, "NETLINK_USERSOCK create: %s!\n",
		    privp->nl_sk ? "ok" : "failed");

	return 0;

amac_err_stop_eth:
	tasklet_kill(&privp->tx_tasklet);
	bcm_amac_enet_stop(privp);
	netif_napi_del(&privp->napi);

amac_err_plat_data:
	/* unregister_netdevice(ndev); */
	free_netdev(ndev);

	return rc;
}

/* bcm_amac_enet_remove() - interface remove callback
 * @pdev: platform data structure pointer
 *
 * Returns: 0
 */
static int bcm_amac_enet_remove(struct platform_device *pdev)
{
	struct bcm_amac_priv *privp;

	privp = netdev_priv((struct net_device *)&pdev->dev);

	if (privp->nl_sk) {
		netlink_kernel_release(privp->nl_sk);
		privp->nl_sk = NULL;
		netdev_info(privp->ndev, "netlink released\n");
	}

	netif_napi_del(&privp->napi);

	tasklet_kill(&privp->tx_tasklet);

	if (privp->ndev) {
		unregister_netdev(privp->ndev);
		free_netdev(privp->ndev);
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
/* bcm_amac_enet_suspend() - interface suspend callback
 * @dev: device data structure pointer
 *
 * Suspends the Ethernet interface by disabling dma, interrupt etc.
 *
 * Returns: 0
 */
static int bcm_amac_enet_suspend(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	netdev_info(ndev, "Suspending AMAC driver\n");

	if (netif_running(ndev)) {
		/* Stop TX queues */
		netif_stop_queue(ndev);

		/* Wait for TX FIFO to drain */
		while (kfifo_len(&privp->dma.txfifo) != 0)
			;

		/* Stop the DMA */
		bcm_amac_dma_stop(privp);

		/* Disable RX NAPI */
		napi_disable(&privp->napi);

		netif_tx_lock(ndev);
		netif_device_detach(ndev);
		netif_tx_unlock(ndev);
	}

	/* Stop PHY's */
	bcm_amac_gphy_start(privp, false);

	return 0;
}

/* bcm_amac_enet_resume() - interface resume callback
 * @dev: device data structure pointer
 *
 * Resumes the Ethernet interface from sleep or deepsleep. Restores device
 * settings, dma, interrupts.
 *
 * Returns: 0 or error number
 */
static int bcm_amac_enet_resume(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	int rc;

	netdev_info(ndev, "Resuming AMAC driver\n");

	/* Since we dont have h/w support for register retention,
	 * initialize everything.
	 */
	rc = bcm_amac_core_init(privp);
	if (rc != 0) {
		netdev_err(ndev, "core init failed!\n");
		return rc;
	}

	bcm_amac_gphy_start(privp, true);

	if (netif_running(ndev)) {
		/* Start DMA */
		rc = bcm_amac_dma_start(privp);
		if (rc) {
			netdev_err(ndev, "DMA start failed!\n");
			goto err_res_dma_start;
		}

		bcm_amac_core_enable(privp, true);

		/* Power up the PHY */
		rc = bcm_amac_gphy_powerup(privp, true);
		if (rc) {
			netdev_err(ndev, "PHY powerup failed!\n");
			goto err_res_phy_powerup;
		}

		napi_enable(&privp->napi);

		netif_tx_lock(ndev);
		netif_device_attach(ndev);
		netif_tx_unlock(ndev);

		netif_start_queue(ndev);
	}

	return 0;

err_res_phy_powerup:
	bcm_amac_core_enable(privp, false);
	bcm_amac_dma_stop(privp);

err_res_dma_start:
	/* Stop PHY's */
	bcm_amac_gphy_start(privp, false);

	return rc;
}

static const struct dev_pm_ops amac_enet_pm_ops = {
	.suspend = bcm_amac_enet_suspend,
	.resume = bcm_amac_enet_resume
};
#endif /* CONFIG_PM_SLEEP */

static int __init bcm_amac_setup_ethaddr(char *s)
{
	bool rc;

	if ((!s) || (!strlen(s))) {
		pr_err("bcm-amac: No ethaddr specified\n");
		return 0;
	}

	rc = is_valid_ether_addr(s);
	if (rc) {
		pr_info("bcm-amac: setting ethaddr: %s\n", s);
		strcpy(cmdline_params.mac_addr, s);
	} else {
		pr_err("bcm-amac: Invalid ethaddr - %s\n", s);
		return 0;
	}

	return 1;
}
__setup("ethaddr=", bcm_amac_setup_ethaddr);

static const struct of_device_id bcm_amac_of_enet_match[] = {
	{.compatible = "brcm,amac-enet",},
	{.compatible = "brcm,amac-enet-v2",},
	{.compatible = "brcm,amac-enet-extsram",},
	{},
};

MODULE_DEVICE_TABLE(of, bcm_amac_of_enet_match);

static struct platform_driver bcm_amac_enet_driver = {
	.driver = {
		.name  = "amac-enet",
		.of_match_table = bcm_amac_of_enet_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &amac_enet_pm_ops,
#endif
	},
	.probe    = bcm_amac_enet_probe,
	.remove   = bcm_amac_enet_remove,
};

module_platform_driver(bcm_amac_enet_driver)

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("Broadcom AMAC Ethernet Driver");
MODULE_LICENSE("GPL v2");
