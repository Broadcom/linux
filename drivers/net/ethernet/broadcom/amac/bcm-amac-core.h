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
#ifndef __BCM_AMAC_CORE_H__
#define __BCM_AMAC_CORE_H__

#include "bcm-amac-enet.h"
#include "bcm-amac-regs.h"

#define BCM_AMAC_CORE_INTR_UNKNOWN 0
#define BCM_AMAC_CORE_INTR_TX      1
#define BCM_AMAC_CORE_INTR_RX      2

#define BCM_AMAC_DMA_BUSY 1
#define BCM_AMAC_DMA_FREE 0

#define BCM_AMAC_DIR_TX false
#define BCM_AMAC_DIR_RX true

#define AMAC_SPEED_1000 2
#define AMAC_SPEED_100  1

int bcm_amac_gphy_init(struct bcm_amac_priv *privp);
void bcm_amac_gphy_exit(struct bcm_amac_priv *privp);
int bcm_amac_gphy_powerup(struct bcm_amac_priv *privp, bool powerup);
void bcm_amac_gphy_start(struct bcm_amac_priv *privp, bool start);

int bcm_amac_core_init(struct bcm_amac_priv *privp);
int bcm_amac_dma_start(struct bcm_amac_priv *privp);
void bcm_amac_dma_stop(struct bcm_amac_priv *privp);
void bcm_amac_core_enable(struct bcm_amac_priv *privp, bool enable);
void bcm_amac_tx_send_packet(struct bcm_amac_priv *privp);
void bcm_amac_tx_clean(struct bcm_amac_priv *privp);
int bcm_amac_enable_tx_dma(struct bcm_amac_priv *privp, bool enable);
int bcm_amac_enable_rx_dma(struct bcm_amac_priv *privp, bool enable);
void bcm_amac_enable_rx_intr(struct bcm_amac_priv *privp, bool enable);

int bcm_amac_get_tx_flag(void);
int bcm_amac_dma_get_rx_data(struct bcm_amac_priv *privp,
			     struct sk_buff **skbp);
void bcm_amac_set_rx_mode(struct net_device *ndev);

void bcm_amac_clear_intr(struct bcm_amac_priv *privp, bool dir);
void bcm_amac_enable_intr(struct bcm_amac_priv *privp, bool dir, bool enable);

void amac_write_mac_address(struct bcm_amac_priv *privp, u8 *addr);
void bcm_amac_enet_set_speed(struct bcm_amac_priv *priv, u32 speed, u32 duplex);
#endif /*__BCM_AMAC_CORE_H__*/
