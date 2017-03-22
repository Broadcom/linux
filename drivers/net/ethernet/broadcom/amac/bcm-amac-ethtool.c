/*
 * Copyright 2016-2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2, as
 * published by the Free Software Foundation (the "GPL").
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License version 2 (GPLv2) for more details.
 *
 * You should have received a copy of the GNU General Public License
 * version 2 (GPLv2) along with this source code.
 */

#include <linux/ethtool.h>
#include <linux/phy.h>

#include "bcm-amac-enet.h"
#include "bcm-amac-regs.h"

#define MOD_VERSION "0.9.0"

struct amac_ethtool_stats {
	u8 size;
	u32 offset;
	const char name[ETH_GSTRING_LEN];
};

static const struct amac_ethtool_stats amac_get_strings_stats[] = {
	{ 8, AMAC_TX_GOOD_OCTETS, "tx_good_octets" },
	{ 4, AMAC_TX_GOOD_PKTS, "tx_good" },
	{ 8, AMAC_TX_OCTETS, "tx_octets" },
	{ 4, AMAC_TX_PKTS, "tx_pkts" },
	{ 4, AMAC_TX_BROADCAST_PKTS, "tx_broadcast" },
	{ 4, AMAC_TX_MULTICAST_PKTS, "tx_multicast" },
	{ 4, AMAC_TX_LEN_64, "tx_64" },
	{ 4, AMAC_TX_LEN_65_TO_127, "tx_65_127" },
	{ 4, AMAC_TX_LEN_128_TO_255, "tx_128_255" },
	{ 4, AMAC_TX_LEN_256_TO_511, "tx_256_511" },
	{ 4, AMAC_TX_LEN_512_TO_1023, "tx_512_1023" },
	{ 4, AMAC_TX_LEN_1024_TO_1522, "tx_1024_1522" },
	{ 4, AMAC_TX_LEN_1523_TO_2047, "tx_1523_2047" },
	{ 4, AMAC_TX_LEN_2048_TO_4095, "tx_2048_4095" },
	{ 4, AMAC_TX_LEN_4096_TO_8191, "tx_4096_8191" },
	{ 4, AMAC_TX_LEN_8192_TO_MAX, "tx_8192_max" },
	{ 4, AMAC_TX_JABBER_PKTS, "tx_jabber" },
	{ 4, AMAC_TX_OVERSIZE_PKTS, "tx_oversize" },
	{ 4, AMAC_TX_FRAGMENT_PKTS, "tx_fragment" },
	{ 4, AMAC_TX_UNDERRUNS, "tx_underruns" },
	{ 4, AMAC_TX_TOTAL_COLS, "tx_total_cols" },
	{ 4, AMAC_TX_SINGLE_COLS, "tx_single_cols" },
	{ 4, AMAC_TX_MULTIPLE_COLS, "tx_multiple_cols" },
	{ 4, AMAC_TX_EXCESSIVE_COLS, "tx_excessive_cols" },
	{ 4, AMAC_TX_LATE_COLS, "tx_late_cols" },
	{ 4, AMAC_TX_DEFERED, "tx_defered" },
	{ 4, AMAC_TX_CARRIER_LOST, "tx_carrier_lost" },
	{ 4, AMAC_TX_PAUSE_PKTS, "tx_pause" },
	{ 4, AMAC_TX_UNI_PKTS, "tx_unicast" },
	{ 4, AMAC_TX_Q0_PKTS, "tx_q0" },
	{ 8, AMAC_TX_Q0_OCTETS, "tx_q0_octets" },
	{ 4, AMAC_TX_Q1_PKTS, "tx_q1" },
	{ 8, AMAC_TX_Q1_OCTETS, "tx_q1_octets" },
	{ 4, AMAC_TX_Q2_PKTS, "tx_q2" },
	{ 8, AMAC_TX_Q2_OCTETS, "tx_q2_octets" },
	{ 4, AMAC_TX_Q3_PKTS, "tx_q3" },
	{ 8, AMAC_TX_Q3_OCTETS, "tx_q3_octets" },
	{ 8, AMAC_RX_GOOD_OCTETS, "rx_good_octets" },
	{ 4, AMAC_RX_GOOD_PKTS, "rx_good" },
	{ 8, AMAC_RX_OCTETS, "rx_octets" },
	{ 4, AMAC_RX_PKTS, "rx_pkts" },
	{ 4, AMAC_RX_BROADCAST_PKTS, "rx_broadcast" },
	{ 4, AMAC_RX_MULTICAST_PKTS, "rx_multicast" },
	{ 4, AMAC_RX_LEN_64, "rx_64" },
	{ 4, AMAC_RX_LEN_65_TO_127, "rx_65_127" },
	{ 4, AMAC_RX_LEN_128_TO_255, "rx_128_255" },
	{ 4, AMAC_RX_LEN_256_TO_511, "rx_256_511" },
	{ 4, AMAC_RX_LEN_512_TO_1023, "rx_512_1023" },
	{ 4, AMAC_RX_LEN_1024_TO_1522, "rx_1024_1522" },
	{ 4, AMAC_RX_LEN_1523_TO_2047, "rx_1523_2047" },
	{ 4, AMAC_RX_LEN_2048_TO_4095, "rx_2048_4095" },
	{ 4, AMAC_RX_LEN_4096_TO_8191, "rx_4096_8191" },
	{ 4, AMAC_RX_LEN_8192_TO_MAX, "rx_8192_max" },
	{ 4, AMAC_RX_JABBER_PKTS, "rx_jabber" },
	{ 4, AMAC_RX_OVERSIZE_PKTS, "rx_oversize" },
	{ 4, AMAC_RX_FRAGMENT_PKTS, "rx_fragment" },
	{ 4, AMAC_RX_MISSED_PKTS, "rx_missed" },
	{ 4, AMAC_RX_CRC_ALIGN_ERRS, "rx_crc_align" },
	{ 4, AMAC_RX_UNDERSIZE, "rx_undersize" },
	{ 4, AMAC_RX_CRC_ERRS, "rx_crc" },
	{ 4, AMAC_RX_ALIGN_ERRS, "rx_align" },
	{ 4, AMAC_RX_SYMBOL_ERRS, "rx_symbol" },
	{ 4, AMAC_RX_PAUSE_PKTS, "rx_pause" },
	{ 4, AMAC_RX_NONPAUSE_PKTS, "rx_nonpause" },
	{ 4, AMAC_RX_SACHANGES, "rx_sa_changes" },
	{ 4, AMAC_RX_UNI_PKTS, "rx_unicast" },
};

#define AMAC_STATS_LEN ARRAY_SIZE(amac_get_strings_stats)

static int amac_ethtool_get_settings(struct net_device *ndev,
				     struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	return phy_ethtool_gset(privp->port.ext_port.phydev, cmd);
}

int amac_ethtool_set_settings(struct net_device *ndev,
			      struct ethtool_cmd *cmd)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	return phy_ethtool_sset(privp->port.ext_port.phydev, cmd);
}

static void amac_ethtool_get_drvinfo(struct net_device *ndev,
				     struct ethtool_drvinfo *info)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	strcpy(info->driver, "amac-enet");
	strcpy(info->version, MOD_VERSION);
	strcpy(info->fw_version, "N/A");

	if (privp->port.ext_port.phydev) {
		if (privp->port.ext_port.phydev->mdio.bus)
			strcpy(info->bus_info,
			       privp->port.ext_port.phydev->mdio.bus->name);
	} else {
		strcpy(info->bus_info, "N/A");
	}
}

static int amac_ethtool_nway_reset(struct net_device *ndev)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	if (!privp->port.ext_port.phydev)
		return -ENODEV;

	if (!netif_running(ndev))
		return -EAGAIN;

	return phy_start_aneg(privp->port.ext_port.phydev);
}

static void amac_ethtool_get_strings(struct net_device *ndev,
				     u32 stringset, u8 *buf)
{
	int i;

	if (stringset != ETH_SS_STATS)
		return;

	for (i = 0; i < AMAC_STATS_LEN; i++)
		strlcpy(buf + i * ETH_GSTRING_LEN,
			amac_get_strings_stats[i].name, ETH_GSTRING_LEN);
}

static int amac_ethtool_get_sset_count(struct net_device *ndev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return AMAC_STATS_LEN;
}

static void amac_ethtool_get_stats(struct net_device *ndev,
				   struct ethtool_stats *estats, u64 *data)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);
	const struct amac_ethtool_stats *stats;
	unsigned int i;
	u64 val;

	for (i = 0; i < AMAC_STATS_LEN; i++) {
		stats = &amac_get_strings_stats[i];
		if (stats->size == 8)
			val = (u64)readl(privp->hw.reg.amac_core +
					 stats->offset + 4) << 32;
		val = readl(privp->hw.reg.amac_core + stats->offset);
		data[i] = val;
	}
}

static void amac_ethtool_get_pauseparam(struct net_device *ndev,
					struct ethtool_pauseparam *epause)
{
	struct bcm_amac_priv *privp = netdev_priv(ndev);

	epause->autoneg = 0;
	epause->rx_pause = 0;
	epause->tx_pause = 0;

	if (privp->port.ext_port.phydev) {
		epause->autoneg = privp->port.ext_port.phydev->autoneg;

		if (privp->port.ext_port.phydev->pause & FLOW_CTRL_RX)
			epause->rx_pause = 1;

		if (privp->port.ext_port.phydev->pause & FLOW_CTRL_TX)
			epause->tx_pause = 1;
	}
}

static const struct ethtool_ops amac_ethtool_ops = {
	.get_settings = amac_ethtool_get_settings,
	.set_settings = amac_ethtool_set_settings,
	.get_drvinfo = amac_ethtool_get_drvinfo,
	.nway_reset = amac_ethtool_nway_reset,
	.get_link = ethtool_op_get_link,
	.get_pauseparam = amac_ethtool_get_pauseparam,
	.get_strings = amac_ethtool_get_strings,
	.get_ethtool_stats = amac_ethtool_get_stats,
	.get_sset_count = amac_ethtool_get_sset_count,
};

void bcm_amac_set_ethtool_ops(struct net_device *ndev)
{
	ndev->ethtool_ops = &amac_ethtool_ops;
}
