/*
 * Copyright (C) 2017 Broadcom
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
#ifndef SCI_REGS_H__
#define SCI_REGS_H__

#define SCI_SCIRQ1_SCIRQEN              0x00
#define SCIRQ1_SCIRQEN_SCB_IRQEN        1
#define SCIRQ1_SCIRQEN_SCA_IRQEN        0
#define SCI_SCIRQ1_SCIRQSTAT            0x04
#define SCIRQ1_SCIRQSTAT_SCBIRQ         1
#define SCIRQ1_SCIRQSTAT_SCAIRQ         0
#define SCI_SCIRQ0_SCIRQEN              0x10
#define SCIRQ0_SCIRQEN_SCB_IRQEN        1
#define SCIRQ0_SCIRQEN_SCA_IRQEN        0
#define SCI_SCIRQ0_SCIRQSTAT            0x14
#define SCIRQ0_SCIRQSTAT_SCBIRQ         1
#define SCIRQ0_SCIRQSTAT_SCAIRQ         0
#define SCI_SCIRQ_SCPU_SCIRQEN          0x20
#define SCIRQ_SCPU_SCIRQEN_SCB_IRQEN    1
#define SCIRQ_SCPU_SCIRQEN_SCA_IRQEN    0
#define SCI_SCIRQ_SCPU_SCIRQSTAT        0x24
#define SCIRQ_SCPU_SCIRQSTAT_SCBIRQ     1
#define SCIRQ_SCPU_SCIRQSTAT_SCAIRQ     0



#define BSCD_P_UART_CMD_1      0x00  /* UART COMMAND REGISTER */
#define BSCD_P_UART_CMD_2      0x40  /* UART COMMAND REGISTER */
#define BSCD_P_PROTO_CMD       0x0c  /* PROTOCOL COMMAND REGISTER */
#define BSCD_P_FLOW_CMD        0x28  /* FLOW CONTROL COMMAND REGISTER */
#define BSCD_P_IF_CMD_1        0x04  /* INTERFACE COMMAND REGISTER */
#define BSCD_P_IF_CMD_2        0x4c  /* INTERFACE COMMAND REGISTER */
#define BSCD_P_INTR_STAT_1     0x58  /* INTERRUPT STATUS REGISTER */
#define BSCD_P_INTR_STAT_2     0x5c  /* INTERRUPT STATUS REGISTER */
#define BSCD_P_INTR_EN_1       0x50  /* INTERRUPT ENABLE REGISTER */
#define BSCD_P_INTR_EN_2       0x54  /* INTERRUPT ENABLE REGISTER */
#define BSCD_P_CLK_CMD         0x08  /* CLOCK COMMAND */
#define BSCD_P_PRESCALE        0x10  /* CLOCK PRESCALE */
#define BSCD_P_TIMER_CMD       0x48  /* TIMER COMMAND REGISTER */
#define BSCD_P_BGT             0x44  /* BLOCK GUARD TIME REGISTER */
#define BSCD_P_TIMER_CNT_1     0x68  /* GENERAL PURPOSE TIMER COUNT REGISTER */
#define BSCD_P_TIMER_CNT_2     0x6c  /* GENERAL PURPOSE TIMER COUNT REGISTER */
#define BSCD_P_TIMER_CMP_1     0x60  /* GENERAL PURPOSE TIMER COMPARE REG */
#define BSCD_P_TIMER_CMP_2     0x64  /* GENERAL PURPOSE TIMER COMPARE REG */
#define BSCD_P_WAIT_1          0x70  /* WAITING TIMER REGISTER */
#define BSCD_P_WAIT_2          0x74  /* WAITING TIMER REGISTER */
#define BSCD_P_WAIT_3          0x78  /* WAITING TIMER REGISTER */
#define BSCD_P_TGUARD          0x14  /* TRANSMIT GUARD TIME REGISTER */
#define BSCD_P_TRANSMIT        0x18  /* TRANSMIT REGISTER */
#define BSCD_P_RECEIVE         0x1c  /* RECEIVE REGISTER */
#define BSCD_P_STATUS_1        0x34  /* STATUS 1 REGISTER */
#define BSCD_P_STATUS_2        0x38  /* STATUS 2 REGISTER */
#define BSCD_P_TLEN_2          0x20  /* TRANSMIT LENGTH REGISTER */
#define BSCD_P_TLEN_1          0x24  /* TRANSMIT LENGTH REGISTER */
#define BSCD_P_RLEN_2          0x2c  /* RECEIVE LENGTH REGISTER */
#define BSCD_P_RLEN_1          0x30  /* RECEIVE LENGTH REGISTER */
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
#define BSCD_P_EVENT1_CNT      0x80 /* EVENT 1 COUNT REGISTER */
#define BSCD_P_EVENT1_CMP      0x88 /* EVENT 1 COMPARE REGISTER  */
#define BSCD_P_EVENT1_CMD_1    0x90 /* EVENT 1 COMMAND 1 REGISTER */
#define BSCD_P_EVENT1_CMD_2    0x94 /* EVENT 1 COMMAND 2 REGISTER  */
#define BSCD_P_EVENT1_CMD_3    0x98 /* SMART CARD EVENT 1 COMMAND 3 REGISTER*/
#define BSCD_P_EVENT1_CMD_4    0x9c /* SMART CARD EVENT 1 COMMAND 4 REGISTER */
#define BSCD_P_EVENT2_CMP      0xa0 /* SMART CARD EVENT 2 COMPARE REGISTER  */
#define BSCD_P_EVENT2_CNT      0xa8 /* SMART CARD EVENT 2 COUNT REGISTER  */
#define BSCD_P_EVENT2_CMD_1    0xb0 /* SMART CARD EVENT 2 COMMAND 1 REGISTER*/
#define BSCD_P_EVENT2_CMD_2    0xb4 /* SMART CARD EVENT 2 COMMAND 2 REGISTER */
#define BSCD_P_EVENT2_CMD_3    0xb8 /* SMART CARD EVENT 2 COMMAND 3 REGISTER */
#define BSCD_P_EVENT2_CMD_4    0xbc /* SMART CARD EVENT 2 COMMAND 4 REGISTER */
#endif
/* BCM5880 specific registers */
#define BSCD_P_DMA_RX_INTF              (0x030 << 2)
#define BSCD_P_DMA_TX_INTF              (0x031 << 2)
#define BSCD_P_MODE_REGISTER            (0x032 << 2)
#define BSCD_P_DB_WIDTH                 (0x033 << 2)
#define BSCD_P_SYNCCLK_DIVIDER1         (0x034 << 2)
#define BSCD_P_SYNCCLK_DIVIDER2         (0x035 << 2)
#define BSCD_P_SYNC_XMIT                (0x036 << 2)
#define BSCD_P_SYNC_RCV                 (0x037 << 2)
#define BSCD_P_LDO_PARAMETERS           (0x038 << 2)



/* Smart Card Module Event source for Event interrupt */
#ifdef BSCD_EMV2000_CWT_PLUS_4_EVENT_INTR
#define  BSCD_P_EVENT1_INTR_EVENT_SRC             0x00
#define  BSCD_P_TEMPTY_INTR_EVENT_SRC             0x01
#define  BSCD_P_RETRY_INTR_EVENT_SRC              0x02
#define  BSCD_P_TDONE_INTR_EVENT_SRC              0x03
#define  BSCD_P_BGT_INTR_EVENT_SRC                0x04
#define  BSCD_P_PRES_INTR_EVENT_SRC               0x05
#define  BSCD_P_TIMER_INTR_EVENT_SRC              0x06
#define  BSCD_P_TPAR_INTR_EVENT_SRC               0x07
#define  BSCD_P_RREADY_INTR_EVENT_SRC             0x08
#define  BSCD_P_RCV_INTR_EVENT_SRC                0x09
#define  BSCD_P_EVENT2_INTR_EVENT_SRC             0x0a
#define  BSCD_P_WAIT_INTR_EVENT_SRC               0x0b
#define  BSCD_P_RLEN_INTR_EVENT_SRC               0x0c
#define  BSCD_P_CWT_INTR_EVENT_SRC                0x0d
#define  BSCD_P_ATRS_INTR_EVENT_SRC               0x0e
#define  BSCD_P_RPAR_INTR_EVENT_SRC               0x0f
#define  BSCD_P_RX_ETU_TICK_EVENT_SRC             0x10
#define  BSCD_P_TX_ETU_TICK_EVENT_SRC             0x11
#define  BSCD_P_HALF_TX_ETU_TICK_EVENT_SRC        0x12
#define  BSCD_P_RX_START_BIT_EVENT_SRC            0x13
#define  BSCD_P_TX_START_BIT_EVENT_SRC            0x14
#define  BSCD_P_LAST_TX_START_BIT_BLOCK_EVENT_SRC 0x15
#define  BSCD_P_ICC_FLOW_CTRL_ASSERTED_EVENT_SRC  0x16
#define  BSCD_P_ICC_FLOW_CTRL_DONE_EVENT_SRC      0x17
#define  BSCD_P_START_IMMEDIATE_EVENT_SRC         0x1f
#define  BSCD_P_DISABLE_COUNTING_EVENT_SRC        0x1f
#define  BSCD_P_NO_EVENT_EVENT_SRC                0x1f
#endif

/* SC_UART_CMD_1 - SMART CARD UART COMMAND REGISTER */

/* SCA :: SC_UART_CMD_1 :: inv_par [07:07] */
#define UARTCMD1_INV_PAR_MASK                        0x00000080
#define UARTCMD1_INV_PAR_SHIFT                       7

/* SCA :: SC_UART_CMD_1 :: get_atr [06:06] */
#define UARTCMD1_GET_ATR_MASK                        0x00000040
#define UARTCMD1_GET_ATR_SHIFT                       6

/* SCA :: SC_UART_CMD_1 :: io_en [05:05] */
#define UARTCMD1_IO_ENA_MASK                          0x00000020
#define UARTCMD1_IO_ENA_SHIFT                         5

/* SCA :: SC_UART_CMD_1 :: auto_rcv [04:04] */
#define UARTCMD1_AUTO_RCV_MASK                       0x00000010
#define UARTCMD1_AUTO_RCV_SHIFT                      4

/* SCA :: SC_UART_CMD_1 :: RESERVED [03:03] */
#define UARTCMD1_RSVD_MASK                           0x00000008
#define UARTCMD1_RSVD_SHIFT                          3

/* SCA :: SC_UART_CMD_1 :: t_r [02:02] */
#define UARTCMD1_T_R_MASK                            0x00000004
#define UARTCMD1_T_R_SHIFT                           2

/* SCA :: SC_UART_CMD_1 :: xmit_go [01:01] */
#define UARTCMD1_XMIT_GO_MASK                        0x00000002
#define UARTCMD1_XMIT_GO_SHIFT                       1

/* SCA :: SC_UART_CMD_1 :: uart_rst [00:00] */
#define UARTCMD1_UART_RST_MASK                       0x00000001
#define UARTCMD1_UART_RST_SHIFT                      0

/* SC_UART_CMD_2 - SMART CARD UART COMMAND REGISTER */
/* SCA :: SC_UART_CMD_2 :: RESERVED [07:07] */
#define UARTCMD2_RSVD_MASK                       0x00000080
#define UARTCMD2_RSVD_SHIFT                      7

/* SCA :: SC_UART_CMD_2 :: convention [06:06] */
#define UARTCMD2_CONVENTION_MASK                     0x00000040
#define UARTCMD2_CONVENTION_SHIFT                    6

/* SCA :: SC_UART_CMD_2 :: rpar_retry [05:03] */
#define UARTCMD2_RPAR_RETRY_MASK                     0x00000038
#define UARTCMD2_RPAR_RETRY_SHIFT                    3

/* SCA :: SC_UART_CMD_2 :: tpar_retry [02:00] */
#define UARTCMD2_TPAR_RETRY_MASK                     0x00000007
#define UARTCMD2_TPAR_RETRY_SHIFT                    0

/* SC_PROTO_CMD - SMART CARD PROTOCOL COMMAND REGISTER */

/* SCA :: SC_PROTO_CMD :: crc_lrc [07:07] */
#define PROTOCMD_CRC_LRC_MASK                         0x00000080
#define PROTOCMD_CRC_LRC_SHIFT                        7

/* SCA :: SC_PROTO_CMD :: edc_en [06:06] */
#define PROTOCMD_EDC_ENA_MASK                          0x00000040
#define PROTOCMD_EDC_ENA_SHIFT                         6

/* SCA :: SC_PROTO_CMD :: tbuf_rst [05:05] */
#define PROTOCMD_TBUF_RST_MASK                        0x00000020
#define PROTOCMD_TBUF_RST_SHIFT                       5

/* SCA :: SC_PROTO_CMD :: rbuf_rst [04:04] */
#define PROTOCMD_RBUF_RST_MASK                        0x00000010
#define PROTOCMD_RBUF_RST_SHIFT                       4

/* SCA :: SC_PROTO_CMD :: cwi [03:00] */
#define PROTOCMD_CWI_MASK                             0x0000000f
#define PROTOCMD_CWI_SHIFT                            0

/* SC_FLOW_CMD - SMART CARD INTERFACE COMMAND REGISTER */

/* SCA :: SC_FLOW_CMD :: rflow [01:01] */
#define FLOWCMD_RFLOW_MASK                            0x00000002
#define FLOWCMD_RFLOW_SHIFT                           1

/* SCA :: SC_FLOW_CMD :: flow_en [00:00] */
#define FLOWCMD_FLOW_ENA_MASK                          0x00000001
#define FLOWCMD_FLOW_ENA_SHIFT                         0

/* SC_IF_CMD_1 - SMART CARD INTERFACE COMMAND REGISTER */

/* SCA :: SC_IF_CMD_1 :: auto_clk [07:07] */
#define IFCMD1_AUTO_CLK_MASK                         0x00000080
#define IFCMD1_AUTO_CLK_SHIFT                        7

/* SCA :: SC_IF_CMD_1 :: auto_io [06:06] */
#define IFCMD1_AUTO_IO_MASK                          0x00000040
#define IFCMD1_AUTO_IO_SHIFT                         6

/* SCA :: SC_IF_CMD_1 :: auto_rst [05:05] */
#define IFCMD1_AUTO_RST_MASK                         0x00000020
#define IFCMD1_AUTO_RST_SHIFT                        5

/* SCA :: SC_IF_CMD_1 :: auto_vcc [04:04] */
#define IFCMD1_AUTO_VCC_MASK                         0x00000010
#define IFCMD1_AUTO_VCC_SHIFT                        4

/* SCA :: SC_IF_CMD_1 :: io [03:03] */
#define IFCMD1_IO_MASK                               0x00000008
#define IFCMD1_IO_SHIFT                              3

/* SCA :: SC_IF_CMD_1 :: pres_pol [02:02] */
#define IFCMD1_PRES_POL_MASK                         0x00000004
#define IFCMD1_PRES_POL_SHIFT                        2

/* SCA :: SC_IF_CMD_1 :: rst [01:01] */
#define IFCMD1_RST_MASK                              0x00000002
#define IFCMD1_RST_SHIFT                             1

/* SCA :: SC_IF_CMD_1 :: vcc [00:00] */
#define IFCMD1_VCC_MASK                              0x00000001
#define IFCMD1_VCC_SHIFT                             0

/* SC_IF_CMD_2 - SMART CARD INTERFACE COMMAND REGISTER */

/* SCA :: SC_IF_CMD_2 :: db_en [07:07] */
#define IFCMD2_DB_EN_MASK                            0x00000080
#define IFCMD2_DB_EN_SHIFT                           7

/* SCA :: SC_IF_CMD_2 :: db_mask [06:06] */
#define IFCMD2_DB_MASK_MASK                          0x00000040
#define IFCMD2_DB_MASK_SHIFT                         6

/* SCA :: SC_IF_CMD_2 :: db_width [05:00] */
#define IFCMD2_DB_WIDTH_MASK                         0x0000003f
#define IFCMD2_DB_WIDTH_SHIFT                        0

/* SC_INTR_STAT_1 - SMART CARD INTERRUPT STATUS REGISTER */

/* SCA :: SC_INTR_STAT_1 :: tpar_intr [07:07] */
#define INTSTAT1_TPAR_INTR_MASK                     0x00000080
#define INTSTAT1_TPAR_INTR_SHIFT                    7

/* SCA :: SC_INTR_STAT_1 :: timer_intr [06:06] */
#define INTSTAT1_TIMER_INTR_MASK                    0x00000040
#define INTSTAT1_TIMER_INTR_SHIFT                   6

/* SCA :: SC_INTR_STAT_1 :: pres_intr [05:05] */
#define INTSTAT1_PRES_INTR_MASK                     0x00000020
#define INTSTAT1_PRES_INTR_SHIFT                    5

/* SCA :: SC_INTR_STAT_1 :: bgt_intr [04:04] */
#define INTSTAT1_BGT_INTR_MASK                      0x00000010
#define INTSTAT1_BGT_INTR_SHIFT                     4

/* SCA :: SC_INTR_STAT_1 :: tdone_intr [03:03] */
#define INTSTAT1_TDONE_INTR_MASK                    0x00000008
#define INTSTAT1_TDONE_INTR_SHIFT                   3

/* SCA :: SC_INTR_STAT_1 :: retry_intr [02:02] */
#define INTSTAT1_RETRY_INTR_MASK                    0x00000004
#define INTSTAT1_RETRY_INTR_SHIFT                   2

/* SCA :: SC_INTR_STAT_1 :: tempty_intr [01:01] */
#define INTSTAT1_TEMP_INTR_MASK                   0x00000002
#define INTSTAT1_TEMP_INTR_SHIFT                  1

/* SCA :: SC_INTR_STAT_1 :: event1_intr [00:00] */
#define INTSTAT1_EV1_INTR_MASK                   0x00000001
#define INTSTAT1_EV1_INTR_SHIFT                  0

/* SC_INTR_STAT_2 - SMART CARD INTERRUPT STATUS REGISTER */

/* SCA :: SC_INTR_STAT_2 :: rpar_intr [07:07] */
#define INTSTAT2_RPAR_INTR_MASK                     0x00000080
#define INTSTAT2_RPAR_INTR_SHIFT                    7

/* SCA :: SC_INTR_STAT_2 :: atrs_intr [06:06] */
#define INTSTAT2_ATR_INTR_MASK                     0x00000040
#define INTSTAT2_ATR_INTR_SHIFT                    6

/* SCA :: SC_INTR_STAT_2 :: cwt_intr [05:05] */
#define INTSTAT2_CWT_INTR_MASK                      0x00000020
#define INTSTAT2_CWT_INTR_SHIFT                     5

/* SCA :: SC_INTR_STAT_2 :: rlen_intr [04:04] */
#define INTSTAT2_RLEN_INTR_MASK                     0x00000010
#define INTSTAT2_RLEN_INTR_SHIFT                    4

/* SCA :: SC_INTR_STAT_2 :: wait_intr [03:03] */
#define INTSTAT2_WAIT_INTR_MASK                     0x00000008
#define INTSTAT2_WAIT_INTR_SHIFT                    3

/* SCA :: SC_INTR_STAT_2 :: event1_intr [02:02] */
#define INTSTAT2_EV2_INTR_MASK                   0x00000004
#define INTSTAT2_EV2_INTR_SHIFT                  2

/* SCA :: SC_INTR_STAT_2 :: rcv_intr [01:01] */
#define INTSTAT2_RCV_INTR_MASK                      0x00000002
#define INTSTAT2_RCV_INTR_SHIFT                     1

/* SCA :: SC_INTR_STAT_2 :: rready_intr [00:00] */
#define INTSTAT2_RRDY_INTR_MASK                   0x00000001
#define INTSTAT2_RRDY_INTR_SHIFT                  0

/* SC_INTR_EN_1 - SMART CARD INTERRUPT ENABLE REGISTER */

/* SCA :: SC_INTR_EN_1 :: tpar_ien [07:07] */
#define INTEN1_TPAR_MASK                        0x00000080
#define INTEN1_TPAR_SHIFT                       7

/* SCA :: SC_INTR_EN_1 :: timer_ien [06:06] */
#define INTEN1_TIMER_MASK                       0x00000040
#define INTEN1_TIMER_SHIFT                      6

/* SCA :: SC_INTR_EN_1 :: pres_ien [05:05] */
#define INTEN1_PRES_MASK                        0x00000020
#define INTEN1_PRES_SHIFT                       5

/* SCA :: SC_INTR_EN_1 :: bgt_ien7 [04:04] */
#define INTEN1_BGT_MASK                        0x00000010
#define INTEN1_BGT_SHIFT                       4

/* SCA :: SC_INTR_EN_1 :: tdone_ien [03:03] */
#define INTEN1_TDONE_MASK                       0x00000008
#define INTEN1_TDONE_SHIFT                      3

/* SCA :: SC_INTR_EN_1 :: retry_ien [02:02] */
#define INTEN1_RETRY_MASK                       0x00000004
#define INTEN1_RETRY_SHIFT                      2

/* SCA :: SC_INTR_EN_1 :: tempty_ien [01:01] */
#define INTEN1_TEMPTY_MASK                      0x00000002
#define INTEN1_TEMPTY_SHIFT                     1

/* SCA :: SC_INTR_EN_1 :: event1_ien [00:00] */
#define INTEN1_EV1_MASK                      0x00000001
#define INTEN1_EV1_SHIFT                     0

/* SC_INTR_EN_2 - SMART CARD INTERRUPT ENABLE REGISTER */

/* SCA :: SC_INTR_EN_2 :: rpar_ien [07:07] */
#define INTEN2_RPAR_MASK                        0x00000080
#define INTEN2_RPAR_SHIFT                       7

/* SCA :: SC_INTR_EN_2 :: atrs_ien [06:06] */
#define INTEN2_ATRS_MASK                        0x00000040
#define INTEN2_ATRS_SHIFT                       6

/* SCA :: SC_INTR_EN_2 :: cwt_ien [05:05] */
#define INTEN2_CWT_MASK                         0x00000020
#define INTEN2_CWT_SHIFT                        5

/* SCA :: SC_INTR_EN_2 :: rlen_ien [04:04] */
#define INTEN2_RLEN_MASK                        0x00000010
#define INTEN2_RLEN_SHIFT                       4

/* SCA :: SC_INTR_EN_2 :: wait_ien [03:03] */
#define INTEN2_WAIT_MASK                        0x00000008
#define INTEN2_WAIT_SHIFT                       3

/* SCA :: SC_INTR_EN_2 :: event1_ien [02:02] */
#define INTEN2_EV1_MASK                      0x00000004
#define INTEN2_EV1_SHIFT                     2

/* SCA :: SC_INTR_EN_2 :: rcv_ien [01:01] */
#define INTEN2_RCV_MASK                         0x00000002
#define INTEN2_RCV_SHIFT                        1

/* SCA :: SC_INTR_EN_2 :: rready_ien [00:00] */
#define INTEN2_RRDY_MASK                      0x00000001
#define INTEN2_RRDY_SHIFT                     0

/* SC_CLK_CMD - SMART CARD CLOCK COMMAND */

/* SCA :: SC_CLK_CMD :: clk_en [07:07] */
#define CLKCMD_CLK_ENA_MASK                            0x00000080
#define CLKCMD_CLK_ENA_SHIFT                           7

/* SCA :: SC_CLK_CMD :: sc_clk_div [06:04] */
#define CLKCMD_SCCLK_DIV_MASK                         0x00000070
#define CLKCMD_SCCLK_DIV_SHIFT                        4

/* SCA :: SC_CLK_CMD :: etu_clk_div [03:01] */
#define CLKCMD_ETUCLK_DIV_MASK                        0x0000000e
#define CLKCMD_ETUCLK_DIV_SHIFT                       1

/* SCA :: SC_CLK_CMD :: baud_div [00:00] */
#define CLKCMD_BAUD_DIV_MASK                           0x00000001
#define CLKCMD_BAUD_DIV_SHIFT                          0

/* SC_PRESCALE - SMART CARD CLOCK PRESCALE */

/* SCA :: SC_PRESCALE :: sc_prescale [07:00] */
#define PRESCALE_SC_PRESCALE_MASK                      0x000000ff
#define PRESCALE_SC_PRESCALE_SHIFT                     0

/* SC_TIMER_CMD - SMART CARD TIMER COMMAND REGISTER */

/* SCA :: SC_TIMER_CMD :: timer_src [07:07] */
#define TIMERCMD_TIMER_SRC_MASK                       0x00000080
#define TIMERCMD_TIMER_SRC_SHIFT                      7

/* SCA :: SC_TIMER_CMD :: timer_mode [06:06] */
#define TIMERCMD_TIMER_MODE_MASK                      0x00000040
#define TIMERCMD_TIMER_MODE_SHIFT                     6

/* SCA :: SC_TIMER_CMD :: timer_en [05:05] */
#define TIMERCMD_TIMER_ENA_MASK                        0x00000020
#define TIMERCMD_TIMER_ENA_SHIFT                       5

/* SCA :: SC_TIMER_CMD :: cwt_en [04:04] */
#define TIMERCMD_CWT_ENA_MASK                          0x00000010
#define TIMERCMD_CWT_ENA_SHIFT                         4

/* SCA :: SC_TIMER_CMD :: RESERVED [03:02] */
#define TIMERCMD_RSVD_MASK                        0x0000000c
#define TIMERCMD_RSVD_SHIFT                       2

/* SCA :: SC_TIMER_CMD :: wait_mode [01:01] */
#define TIMERCMD_WAIT_MODE_MASK                       0x00000002
#define TIMERCMD_WAIT_MODE_SHIFT                      1

/* SCA :: SC_TIMER_CMD :: wait_en [00:00] */
#define TIMERCMD_WAIT_ENA_MASK                         0x00000001
#define TIMERCMD_WAIT_ENA_SHIFT                        0

/* SC_BGT - SMART CARD BLOCK GUARD TIME REGISTER */
/* SCA :: SC_BGT :: r2t [07:07] */
#define BGT_R2T_MASK                                   0x00000080
#define BGT_R2T_SHIFT                                  7

/* SCA :: SC_BGT :: t2r [06:06] */
#define BGT_T2R_MASK                                   0x00000040
#define BGT_T2R_SHIFT                                  6

/* SCA :: SC_BGT :: bgt [05:00] */
#define BGT_BGT_MASK                                   0x0000003f
#define BGT_BGT_SHIFT                                  0

/* SC_TIMER_CNT_1 - SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER */
/* SCA :: SC_TIMER_CNT_1 :: sc_timer_cnt [07:00] */
#define TIMERCNT1_SCTIMER_CNT_MASK                  0x000000ff
#define TIMERCNT1_SCTIMER_CNT_SHIFT                 0

/* SC_TIMER_CNT_2 - SMART CARD GENERAL PURPOSE TIMER COUNT REGISTER */
/* SCA :: SC_TIMER_CNT_2 :: sc_timer_cnt [07:00] */
#define TIMERCNT2_SCTIMER_CNT_MASK                  0x000000ff
#define TIMERCNT2_SCTIMER_CNT_SHIFT                 0

/* SC_TIMER_CMP_1 - SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER */
/* SCA :: SC_TIMER_CMP_1 :: sc_timer_cmp [07:00] */
#define TIMERCMP1_SCTIMER_CMP_MASK                  0x000000ff
#define TIMERCMP1_SCTIMER_CMP_SHIFT                 0

/* SC_TIMER_CMP_2 - SMART CARD GENERAL PURPOSE TIMER COMPARE REGISTER */
/* SCA :: SC_TIMER_CMP_2 :: sc_timer_cmp [07:00] */
#define TIMERCMP2_SCTIMER_CMP_MASK                  0x000000ff
#define TIMERCMP2_SCTIMER_CMP_SHIFT                 0

/* SC_WAIT_1 - SMART CARD WAITING TIMER REGISTER */
/* SCA :: SC_WAIT_1 :: sc_wait [07:00] */
#define WAIT1_SC_WAIT_MASK                            0x000000ff
#define WAIT1_SC_WAIT_SHIFT                           0

/* SC_WAIT_2 - SMART CARD WAITING TIMER REGISTER */
/* SCA :: SC_WAIT_2 :: sc_wait [07:00] */
#define WAIT2_SC_WAIT_MASK                            0x000000ff
#define WAIT2_SC_WAIT_SHIFT                           0

/* SC_WAIT_3 - SMART CARD WAITING TIMER REGISTER */
/* SCA :: SC_WAIT_3 :: sc_wait [07:00] */
#define WAIT3_SC_WAIT_MASK                            0x000000ff
#define WAIT3_SC_WAIT_SHIFT                           0

/* SC_TGUARD - SMART CARD TRANSMIT GUARD TIME REGISTER */
/* SCA :: SC_TGUARD :: sc_tguard [07:00] */
#define TGUARD_SC_TGUARD_MASK                          0x000000ff
#define TGUARD_SC_TGUARD_SHIFT                         0

/* SC_TRANSMIT - SMART CARD TRANSMIT REGISTER */
/* SCA :: SC_TRANSMIT :: sc_transmit [07:00] */
#define TRANSMIT_SC_TRANSMIT_MASK                      0x000000ff
#define TRANSMIT_SC_TRANSMIT_SHIFT                     0

/* SC_RECEIVE - SMART CARD RECEIVE REGISTER */
/* SCA :: SC_RECEIVE :: sc_receive [07:00] */
#define RECEIVE_SC_RECEIVE_MASK                        0x000000ff
#define RECEIVE_SC_RECEIVE_SHIFT                       0

/* SC_STATUS_1 - SMART CARD STATUS 1 REGISTER */
/* SCA :: SC_STATUS_1 :: card_pres [06:06] */
#define STATUS1_CARD_PRES_MASK                        0x00000040
#define STATUS1_CARD_PRES_SHIFT                       6

/* SCA :: SC_STATUS_1 :: sc_io [02:02] */
#define STATUS1_SC_IO_MASK                            0x00000004
#define STATUS1_SC_IO_SHIFT                           2

/* SCA :: SC_STATUS_1 :: tempty [01:01] */
#define STATUS1_TEMPTY_MASK                           0x00000002
#define STATUS1_TEMPTY_SHIFT                          1

/* SCA :: SC_STATUS_1 :: tdone [00:00] */
#define STATUS1_TDONE_MASK                            0x00000001
#define STATUS1_TDONE_SHIFT                           0

/* SC_STATUS_2 - SMART CARD STATUS 2 REGISTER */
/* SCA :: SC_STATUS_2 :: rpar_err [07:07] */
#define STATUS2_RPAR_ERR_MASK                         0x00000080
#define STATUS2_RPAR_ERR_SHIFT                        7

/* SCA :: SC_STATUS_2 :: roverflow [03:03] */
#define STATUS2_ROVERFLOW_MASK                        0x00000008
#define STATUS2_ROVERFLOW_SHIFT                       3

/* SCA :: SC_STATUS_2 :: edc_err [02:02] */
#define STATUS2_EDC_ERR_MASK                          0x00000004
#define STATUS2_EDC_ERR_SHIFT                         2

/* SCA :: SC_STATUS_2 :: rempty [01:01] */
#define STATUS2_REMPTY_MASK                           0x00000002
#define STATUS2_REMPTY_SHIFT                          1

/* SCA :: SC_STATUS_2 :: rready [00:00] */
#define STATUS2_RRDY_MASK                           0x00000001
#define STATUS2_RRDY_SHIFT                          0

/* SC_TLEN_2 - SMART CARD TRANSMIT LENGTH REGISTER */
/* SCA :: SC_TLEN_2 :: sc_tlen [00:00] */
#define TLEN2_SC_TLEN_MASK                            0x00000001
#define TLEN2_SC_TLEN_SHIFT                           0

/* SC_TLEN_1 - SMART CARD TRANSMIT LENGTH REGISTER */
/* SCA :: SC_TLEN_1 :: sc_tlen [07:00] */
#define TLEN1_SC_TLEN_MASK                            0x000000ff
#define TLEN1_SC_TLEN_SHIFT                           0

/* SC_RLEN_2 - SMART CARD RECEIVE LENGTH REGISTER */
/* SCA :: SC_RLEN_2 :: sc_rlen [00:00] */
#define RLEN2_SC_RLEN_MASK                            0x00000001
#define RLEN2_SC_RLEN_SHIFT                           0

/* SC_RLEN_1 - SMART CARD RECEIVE LENGTH REGISTER */
/* SCA :: SC_RLEN_1 :: sc_rlen [07:00] */
#define RLEN1_SC_RLEN_MASK                            0x000000ff
#define RLEN1_SC_RLEN_SHIFT                           0

/* SC_EVENT1_CNT - SMART CARD EVENT 1 COUNT REGISTER */
/* SCA :: SC_EVENT1_CNT :: sc_event1_cnt [07:00] */
#define EV1CNT_SC_EV1_CNT_MASK                  0x000000ff
#define EV1CNT_SC_EV1_CNT_SHIFT                 0

/* SC_EVENT1_CMP - SMART CARD EVENT 1 COMPARE REGISTER */
/* SCA :: SC_EVENT1_CMP :: sc_event1_cmp [07:00] */
#define EV1CMP_SC_EV1_CMP_MASK                  0x000000ff
#define EV1CMP_SC_EV1_CMP_SHIFT                 0

/* SC_EVENT1_CMD_1 - SMART CARD EVENT 1 COMMAND 1 REGISTER */
/* SCA :: SC_EVENT1_CMD_1 :: RESERVED [07:05] */
#define EV1CMD1_RSVD_MASK                     0x000000e0
#define EV1CMD1_RSVD_SHIFT                    5

/* SCA :: SC_EVENT1_CMD_1 :: increment_event_src [04:00] */
#define EV1CMD1_INC_EV_SRC_MASK          0x0000001f
#define EV1CMD1_INC_EV_SRC_SHIFT         0

/* SC_EVENT1_CMD_2 - SMART CARD EVENT 1 COMMAND 2 REGISTER */
/* SCA :: SC_EVENT1_CMD_2 :: RESERVED [07:05] */
#define EV1CMD2_RSVD_MASK                     0x000000e0
#define EV1CMD2_RSVD_SHIFT                    5

/* SCA :: SC_EVENT1_CMD_2 :: increment_event_src [04:00] */
#define EV1CMD2_INC_EV_SRC_MASK          0x0000001f
#define EV1CMD2_INC_EV_SRC_SHIFT         0

/* SC_EVENT1_CMD_3 - SMART CARD EVENT 1 COMMAND 3 REGISTER */
/* SCA :: SC_EVENT1_CMD_3 :: RESERVED [07:05] */
#define EV1CMD3_RSVD_MASK                     0x000000e0
#define EV1CMD3_RSVD_SHIFT                    5

/* SCA :: SC_EVENT1_CMD_3 :: start_event_src [04:00] */
#define EV1CMD3_START_EV_SRC_MASK              0x0000001f
#define EV1CMD3_START_EV_SRC_SHIFT             0

/* SC_EVENT1_CMD_4 - SMART CARD EVENT 1 COMMAND 4 REGISTER */
/* SCA :: SC_EVENT1_CMD_4 :: event_en [07:07] */
#define EV1CMD4_EV_ENA_MASK                     0x00000080
#define EV1CMD4_EV_ENA_SHIFT                    7

/* SCA :: SC_EVENT1_CMD_4 :: RESERVED [06:04] */
#define EV1CMD4_EV_RSVD_MASK                     0x00000070
#define EV1CMD4_EV_RSVD_SHIFT                    4

/* SCA :: SC_EVENT1_CMD_4 :: intr_after_reset [03:03] */
#define EV1CMD4_INTR_AFTER_RST_MASK             0x00000008
#define EV1CMD4_INTR_AFTER_RST_SHIFT            3

/* SCA :: SC_EVENT1_CMD_4 :: intr_after_compare [02:02] */
#define EV1CMD4_INTR_AFTER_CMP_MASK           0x00000004
#define EV1CMD4_INTR_AFTER_CMP_SHIFT          2

/* SCA :: SC_EVENT1_CMD_4 :: run_after_reset [01:01] */
#define EV1CMD4_RUN_AFTER_RST_MASK              0x00000002
#define EV1CMD4_RUN_AFTER_RST_SHIFT             1

/* SCA :: SC_EVENT1_CMD_4 :: run_after_compare [00:00] */
#define EV1CMD4_RUN_AFTER_CMP_MASK            0x00000001
#define EV1CMD4_RUN_AFTER_CMP_SHIFT           0

/* SC_event1_CNT - SMART CARD EVENT 2 COUNT REGISTER */
/* SCA :: SC_event1_CNT :: sc_event1_cnt [07:00] */
#define EV1CNT_SC_EV1_CNT_MASK                  0x000000ff
#define EV1CNT_SC_EV1_CNT_SHIFT                 0

/* SC_event1_CMP - SMART CARD EVENT 2 COMPARE REGISTER */
/* SCA :: SC_event1_CMP :: sc_event1_cmp [07:00] */
#define EV1CMP_SC_EV1_CMP_MASK                  0x000000ff
#define EV1CMP_SC_EV1_CMP_SHIFT                 0

/* SC_event1_CMD_1 - SMART CARD EVENT 2 COMMAND 1 REGISTER */
/* SCA :: SC_event1_CMD_1 :: RESERVED [07:05] */
#define EV1CMD1_RSVD_MASK                     0x000000e0
#define EV1CMD1_RSVD_SHIFT                    5

/* SCA :: SC_event1_CMD_1 :: increment_event_src [04:00] */
#define EV1CMD1_INC_EV_SRC_MASK          0x0000001f
#define EV1CMD1_INC_EV_SRC_SHIFT         0

/* SC_event1_CMD_2 - SMART CARD EVENT 2 COMMAND 2 REGISTER */
/* SCA :: SC_event1_CMD_2 :: RESERVED [07:05] */
#define EV1CMD2_RSVD_MASK                     0x000000e0
#define EV1CMD2_RSVD_SHIFT                    5

/* SCA :: SC_event1_CMD_2 :: increment_event_src [04:00] */
#define EV1CMD2_INC_EV_SRC_MASK          0x0000001f
#define EV1CMD2_INC_EV_SRC_SHIFT         0

/* SC_event1_CMD_3 - SMART CARD EVENT 2 COMMAND 3 REGISTER */
/* SCA :: SC_event1_CMD_3 :: RESERVED [07:05] */
#define EV1CMD3_RSVD_MASK                     0x000000e0
#define EV1CMD3_RSVD_SHIFT                    5

/* SCA :: SC_event1_CMD_3 :: start_event_src [04:00] */
#define EV1CMD3_START_EV_SRC_MASK              0x0000001f
#define EV1CMD3_START_EV_SRC_SHIFT             0

/* SC_event1_CMD_4 - SMART CARD EVENT 2 COMMAND 4 REGISTER */
/* SCA :: SC_event1_CMD_4 :: event_en [07:07] */
#define EV1CMD4_EV_ENA_MASK                     0x00000080
#define EV1CMD4_EV_ENA_SHIFT                    7

/* SCA :: SC_event1_CMD_4 :: RESERVED [06:04] */
#define EV1CMD4_EV_RSVD_MASK                     0x00000070
#define EV1CMD4_EV_RSVD_SHIFT                    4

/* SCA :: SC_event1_CMD_4 :: intr_after_reset [03:03] */
#define EV1CMD4_INTR_AFTER_RST_MASK             0x00000008
#define EV1CMD4_INTR_AFTER_RST_SHIFT            3

/* SCA :: SC_event1_CMD_4 :: intr_after_compare [02:02] */
#define EV1CMD4_INTR_AFTER_CMP_MASK           0x00000004
#define EV1CMD4_INTR_AFTER_CMP_SHIFT          2

/* SCA :: SC_event1_CMD_4 :: run_after_reset [01:01] */
#define EV1CMD4_RUN_AFTER_RST_MASK              0x00000002
#define EV1CMD4_RUN_AFTER_RST_SHIFT             1

/* SCA :: SC_event1_CMD_4 :: run_after_compare [00:00] */
#define EV1CMD4_RUN_AFTER_CMP_MASK            0x00000001
#define EV1CMD4_RUN_AFTER_CMP_SHIFT           0


#endif /* #ifndef SCI_REGS_H__ */

/* End of File */
