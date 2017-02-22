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

#ifndef __BCM_AMAC_ENET_H__
#define __BCM_AMAC_ENET_H__

#include <linux/kfifo.h>
#include <linux/netdevice.h>
#include <linux/platform_device.h>

/* SRAM specific DMA configuration. */
#define NAPI_POLL_SRAM_WEIGHT  16

#define AMAC_SRDMA_RX_DESC_CNT 4

#define AMAC_SRDMA_TX_CHAIN_LEN_MAX   1 /* Limit TX DMA chain len */
/* Must be power of two because of the use of kfifo */
#define AMAC_SRDMA_TX_MAX_QUEUE_LEN (AMAC_SRDMA_TX_CHAIN_LEN_MAX * 8)

#define AMAC_SRDMA_TX_DESC_CNT  (AMAC_SRDMA_TX_MAX_QUEUE_LEN * 1)

#define AMAC_RX_BUF_SIZE      2048 /* MAX RX buffer size */
#define AMAC_DMA_RX_DESC_CNT  512 /* Number of rx dma descriptors */
#define AMAC_DMA_RXALIGN      16 /* Alignment for SKB */
/* 802.3as defines max packet size to be 2000 bytes, size is rounded up
 * to be multiple of 32 to be cache aligned
 */
#define AMAC_DMA_RX_BUF_LEN   2016

#define AMAC_DMA_TX_CHAIN_LEN_MAX   128 /* Limit TX DMA chain len */
/* Must be power of two because of the use of kfifo */
#define AMAC_DMA_TX_MAX_QUEUE_LEN (AMAC_DMA_TX_CHAIN_LEN_MAX * 4)
/* Two descriptors per packet, one each for: config data and payload */
#define AMAC_DMA_TX_DESC_CNT  (AMAC_DMA_TX_MAX_QUEUE_LEN * 2)

/* PORT Settings */
#define AMAC_PORT_SPEED_1G    SPEED_1000
#define AMAC_PORT_SPEED_100M  SPEED_100
#define AMAC_PORT_SPEED_10M   SPEED_10

#define AMAC_PORT_DEFAULT_SPEED   AMAC_PORT_SPEED_1G

/* DMA Descriptor */
struct amac_dma64_desc {
	u32 ctrl1;    /* misc control bits */
	u32 ctrl2;    /* buffer count and address extension */
	u32 addrlow;  /* mem addr of data buffer, bits 31:0 */
	u32 addrhigh; /* mem addr of data buffer, bits 63:32 */
};

/* DMA configuration data structure
 * These are used to configure the DMA block
 */
struct amac_dma_cfg {
	void *raw_descp;
	dma_addr_t raw_addr;
	void *descp;    /* Aligned Descriptor pointer */
	dma_addr_t base_addr;/* Aligned bus base address */
	u32 ring_len;   /* Total number of descriptors */
	u32 alloc_size; /* Total memory alloc in bytes */
	u32 index;      /* Current descriptor index */
};

/* SKB node data structure */
struct skb_list_node {
	struct sk_buff *skb;
	void *skb_bounce;
	dma_addr_t dma_addr;/* bus base address of region */
	int len;
};

/* SRAM DMA private data for both RX and TX. */
struct amac_srdma_priv {
	struct amac_dma_cfg rx_bounce_data;
	struct amac_dma_cfg tx_bounce_data;
	int bounce_tx_len;
	bool enable_bounce;
};

/* DMA private data for both RX and TX */
struct amac_dma_priv {
	struct amac_dma_cfg rx;
	struct amac_dma_cfg tx;
	struct amac_srdma_priv sr_dma;
	int dma_rx_desc_count;
	int dma_tx_desc_count;
	struct kfifo txfifo;
	u32 tx_max_pkts; /* max number of packets */
	u32 tx_curr;     /* current packet index */
	struct skb_list_node *tx_skb_list; /* list of skb given to hw for tx */
	struct skb_list_node *rx_skb_list; /* list of skb given to hw for rx */
	atomic_t tx_dma_busy; /* keep track of DMA status */
};

struct port_status {
	u32 link; /* link status */
	u32 speed; /* port speed */
	u32 duplex; /* port duplex */
	u32 pause; /* pause frames */
	u32 aneg; /* auto negotiation */
};

struct port_info {
	/* Current port status for link updates */
	struct port_status stat;

	/* In case of switch-by-pass mode */
	struct device_node *phy_node;
	struct phy_device *phydev; /* Connected PHY dev */
	int phy_mode; /* phy mode */
	bool lswap; /* lane swapping */
	bool phy54810_rgmii_sync; /* PHY54810 fix up */
	bool pause_disable;
};

/* Ethernet Port data structure */
struct port_data {
	u32 imp_port_speed; /* IMP Port (Port8) max speed */
	/* external port (with internal or ext PHY) */
	struct port_info ext_port;
};

/* AMAC registers */
struct bcm_amac_reg_base {
	void __iomem *amac_core;
	void __iomem *amac_idm_base;
	void __iomem *rgmii_regs;
	void __iomem *switch_global_cfg;
	void __iomem *crmu_io_pad_ctrl;
};

/* Structure contains all the hardware info */
struct bcm_amac_hw {
	struct bcm_amac_reg_base reg; /* iomapped register addresses */
	u32 intr_num; /* Interrupt number */
};

/* AMAC Driver's private data structure.
 * Contains data for the driver instance. This structure is used
 * through-out the driver to derrive status of various blocks,
 * settings, stats, hw registers etc.
 */
struct bcm_amac_priv {
	struct napi_struct napi ____cacheline_aligned;
	struct tasklet_struct tx_tasklet;

	struct net_device *ndev; /* net device reference */
	struct platform_device *pdev; /* platform device reference */

	struct port_data port; /* Port and PHY Info */

	struct amac_dma_priv dma; /* DMA info */

	/* netlink socket for link change notifications */
	struct sock *nl_sk;
	u32 nl_seq; /* link notification sequence number */

	struct bcm_amac_hw hw; /* Hardware info */

	struct sockaddr cur_etheraddr; /* local ethernet address */

	bool switch_mode; /* internal switch availability */

	spinlock_t lock; /* used by netdev api's */
	u32 msg_enable; /* message filter bit mask */
};

void bcm_amac_set_ethtool_ops(struct net_device *netdev);
#endif /*__BCM_AMAC_ENET_H__*/

