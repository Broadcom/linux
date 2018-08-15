/*
 * Copyright (C) 2017 Broadcom
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>

#include "bcm_iproc_axipv.h"
#include "bcm_iproc_dsi_ctrl.h"
#include "bcm_iproc_pv.h"

#define DSI_1MHZ			(1000 * 1000)
#define SEMA_TIMEOUT_MS			1000

#define MAX_TXM_SIZE	(DSI_HW_CMND_FIFO_SIZE_B + DSI_HW_PIXEL_FIFO_SIZE_B)
#define TICKS_IN_MILLISECONDS(x)	msecs_to_jiffies(x)
#define TICKS_FOREVER			MAX_SCHEDULE_TIMEOUT

struct dsi_clk {
	unsigned int clk_in_mhz;	/* input clock */
	unsigned int clk_in_div;	/* input clock divider value */
};

struct dsi_cfg {
	unsigned int dlcount;	/* Number of Data Lines */
	struct dsi_counter *phy_timing;	/* DSI Timing parameters */
	struct dsi_clk esc_clk;	/* ESC Clk Configuration */
	struct dsi_clk hsbit_clk;	/* HS  Clk Configuration */
	unsigned int lpbitrate_mbps;	/* LP Data Bit Rate, MAX=10[Mbps] */
	bool ena_st_end;	/* enable ST_END */
	bool vmode;		/* 1 = Video Mode, 0 = Command Mode */
	uint8_t hs, hbp, hfp, hbllp;
	uint8_t vs, vbp, vfp;
};

struct dsi_cmd {
	unsigned int vc;	/* Destination Virtual Channel */
	bool is_lp;		/* LP(Low Power) | HS(High Speed) */
	bool is_long;		/* LONG | SHORT */
	unsigned int dsi_cmnd;	/* DSI command to send */
	uint8_t *msg;		/* TX msg payload */
	unsigned int msg_len;	/* TX Msg Size  [bytes] */
	bool end_with_bta;	/* End command with BTA */
};

enum dsi_lcd_cm_out {
	LCD_IF_CM_O_RGB565,		/* RGB565 */
	LCD_IF_CM_O_RGB565_DSI_VM,	/* RGB565 */
	LCD_IF_CM_O_RGB666,		/* RGB666 Packed */
	LCD_IF_CM_O_xRGB8888,		/* xRGB8888 */
	LCD_IF_CM_O_xBGR8888,		/* xRGB8888 */
	LCD_IF_CM_O_RGB666U,		/* RGB666 unpacked */
	LCD_IF_CM_O_INV,
};

enum dsi_lcd_cm_in {
	LCD_IF_CM_I_RGB565P,	/* 2 565 pixels per 32-bit word */
	LCD_IF_CM_I_xRGB8888,
	LCD_IF_CM_I_xBGR8888,
	LCD_IF_CM_I_INV,
};

struct dsi_cm_vc {
	unsigned int vc;	/* Virtual Channel */
	unsigned int dsi_cmnd;	/* DSI Command DT */
	unsigned int dcs_cmd_start;	/* Display MEMWR Start    Command */
	unsigned int dcs_cmd_cont;	/* Display MEMWR Continue Command */
	bool is_lp;	/* LP(Low Power) / HS(High Speed) */
	enum dsi_lcd_cm_in cm_in;	/* Color Mode In  (frame buffer) */
	enum dsi_lcd_cm_out cm_out;	/* Color Mode Out (display) */
};

struct dsi_lcd_upd_req {
	dma_addr_t buff;		/* frame buffer */
	unsigned int buff_bpp;	/* frame buffer BytesPerPixel */
	unsigned int line_len_p;	/* HOR length in pixels */
	unsigned int line_count;	/* VER length in lines */
	unsigned int xstride_b;	/* stride in bytes */
};

struct dsi_clk_cfg {
	unsigned int hs_pllreq_mhz;	/* in:  PLL freq requested */
	unsigned int esc_clkin_mhz;	/* in:  ESC clk in requested */
	unsigned int hs_pllset_mhz;	/* out: PLL freq set */
	unsigned int hsbit_clk_mhz;	/* out: end HS bit clock */
	unsigned int esc_clk_mhz;	/* out: ESC clk after req inDIV */
	unsigned int hsclkdiv;	/* out: HS  CLK Div Reg value */
	unsigned int esc_clkdiv;	/* out: ESC CLK Div Reg value */
	unsigned int hspll_p1;	/* out: PLL setting */
	unsigned int hspll_n_int;	/* out: PLL setting */
	unsigned int hspll_n_frac;	/* out: PLL setting */
	enum dsi_clk_sel coreclksel;	/* out: core_clk_sel */
};

struct dsi_client {
	void *lcd_handle;
	bool open;
};

struct dsi_cm_handle {
	struct dsi_client *client;
	bool configured;
	uint8_t dcs_cmd_start;
	uint8_t dcs_cmd_cont;
	enum dsi_de1_col_mod cm;
	unsigned int vc;
	unsigned int dsi_cmnd;
	bool is_lp;
	unsigned int vm_when;
	unsigned int bpp_wire;
	unsigned int bpp_dma;
	unsigned int wc_rshift;
};

struct dsi_handle {
	void *dsi_hw_handle;
	unsigned int init;
	unsigned int initonce;
	unsigned int initpv;
	unsigned int initaxipv;
	void __iomem *dsicore_regaddr;
	unsigned int clients;
	struct dsi_clk_cfg clkcfg;	/* HS & ESC Clk configuration */
	void *sema_dsi;
	void *sema_int;
	void *sema_axipv;
	void *sema_pv;
	irq_handler_t lisr;
	void (*hisr)(void);
	void *ihisr;
	unsigned int interruptid;
	struct dsi_client client;
	struct dsi_cm_handle chcm;
	struct axipv_config *axipv_cfg;
	struct pv_config *pv_cfg;

	bool vmode;		/* 1 = Video Mode, 0 = Command Mode */
	unsigned int dlcount;	/* Number of data lanes*/
	bool st_end;		/* control transfer of end SYNC events */
};

struct dsi_upd_req_msg {
	struct dsi_handle *dsic_handle;		/* DSI CNTRL handle */
	struct dsi_client *client_handle;	/* DSI CNTRL Client handle */
	struct dsi_lcd_upd_req upd_req;		/* update Request */
};

struct platform_device *dsi_pdev;
static struct dsi_handle dsi_bus;

/* DSI Command Mode VC Configuration */
struct dsi_cm_vc dispdrv_vccm_cfg = {
	.vc = 0,
	.dsi_cmnd = DSI_DT_LG_DCS_WR,
	.dcs_cmd_start = MIPI_DCS_WRITE_MEMORY_START,
	.dcs_cmd_cont = MIPI_DCS_WRITE_MEMORY_CONTINUE,
};

/* DSI BUS CONFIGURATION */
struct dsi_cfg dispdrv_dsicfg;

static inline void *dsic_sem_create(unsigned int count, unsigned int mode)
{
	void **sem_ptr = devm_kzalloc(&dsi_pdev->dev,
					sizeof(struct semaphore), GFP_KERNEL);

	if (sem_ptr == NULL)
		return sem_ptr;

	sema_init((struct semaphore *)sem_ptr, count);
	return sem_ptr;
}

static inline void dsic_sem_free(void *sem_ptr)
{
	if (sem_ptr != NULL)
		devm_kfree(&dsi_pdev->dev, sem_ptr);
}

static inline unsigned int dsic_sem_obtain(void *s, long timeout)
{
	int status = down_timeout((struct semaphore *)s, timeout);

	if (status)
		dev_err(&dsi_pdev->dev, "sem_obtain error:%d\n", status);

	return status;
}

static inline unsigned int dsic_sem_release(void *s)
{
	up((struct semaphore *)s);
	return 0;
}

static void dsic_hw_reset(bool on)
{
	struct dsi_platform_data *dsip_data;

	dsip_data = platform_get_drvdata(dsi_pdev);

	if (dsip_data->reset_base) {
		dsi_reset(dsip_data->reset_base, 1, on);
	} else if (dsip_data->reset_gpio) {
		dev_info(&dsi_pdev->dev, "Panel status is %d, gpio reset", on);
		if (!on) {
			/* Panel's off and we are in panel-on sequence */
			gpiod_set_value_cansleep(dsip_data->reset_gpio, 0);
			usleep_range(700, 701);
			gpiod_set_value_cansleep(dsip_data->reset_gpio, 1);
			usleep_range(1000, 1001);
			gpiod_set_value_cansleep(dsip_data->reset_gpio, 0);
			msleep(100);
		} else {
			/* Panel is on and we are in panel-off sequence */
			gpiod_set_value_cansleep(dsip_data->reset_gpio, 0);
		}
	}
}

int dsic_init(struct dispdrv_info *info, void **handle, int id)
{
	int res = 0;
	struct dispdrv_panel *panel_t;

	panel_t = &panel;

	if (panel_t->drvstate != DRV_STATE_OFF) {
		dev_err(&dsi_pdev->dev, "Not in OFF state\n");
		return -EINVAL;
	}
	panel_t->id = id;

	panel_t->cmnd_mode = &dispdrv_vccm_cfg;
	dispdrv_vccm_cfg.is_lp = false;

	switch (info->in_fmt) {
	case DISPDRV_FB_FORMAT_RGB565:
		dispdrv_vccm_cfg.cm_in = LCD_IF_CM_I_RGB565P;
		dispdrv_vccm_cfg.cm_out = LCD_IF_CM_O_RGB565;
		break;
	case DISPDRV_FB_FORMAT_xRGB8888:
		dispdrv_vccm_cfg.cm_in = LCD_IF_CM_I_xRGB8888;
		dispdrv_vccm_cfg.cm_out = LCD_IF_CM_O_xRGB8888;
		break;
	case DISPDRV_FB_FORMAT_xBGR8888:
		dispdrv_vccm_cfg.cm_in = LCD_IF_CM_I_xBGR8888;
		dispdrv_vccm_cfg.cm_out = LCD_IF_CM_O_xRGB8888;
		break;
	default:
		dispdrv_vccm_cfg.cm_in = LCD_IF_CM_I_xBGR8888;
		dispdrv_vccm_cfg.cm_out = LCD_IF_CM_O_xRGB8888;
		dev_err(&dsi_pdev->dev, "Unknown format %d\n", info->in_fmt);
		break;
	}

	panel_t->dsi_cfg = (struct dsi_cfg *)&dispdrv_dsicfg;
	dispdrv_dsicfg.dlcount = info->lanes;
	dispdrv_dsicfg.phy_timing = info->phy_timing;

	dispdrv_dsicfg.esc_clk.clk_in_mhz = 500;
	dispdrv_dsicfg.esc_clk.clk_in_div = 5;
	dispdrv_dsicfg.hsbit_clk.clk_in_mhz = info->hs_bps / DSI_1MHZ;
	while (dispdrv_dsicfg.hsbit_clk.clk_in_mhz < 600)
		dispdrv_dsicfg.hsbit_clk.clk_in_mhz *= 2;
	dispdrv_dsicfg.hsbit_clk.clk_in_div =
		(dispdrv_dsicfg.hsbit_clk.clk_in_mhz * DSI_1MHZ) / info->hs_bps;
	dispdrv_dsicfg.lpbitrate_mbps = info->lp_bps / DSI_1MHZ;

	dispdrv_dsicfg.vmode = info->vmode;
	dispdrv_dsicfg.vs = info->vs;
	dispdrv_dsicfg.vbp = info->vbp;
	dispdrv_dsicfg.vfp = info->vfp;
	dispdrv_dsicfg.hs = info->hs;
	dispdrv_dsicfg.hbp = info->hbp;
	dispdrv_dsicfg.hfp = info->hfp;
	dispdrv_dsicfg.hbllp = info->hbllp;
	dispdrv_dsicfg.ena_st_end = info->sync_pulses;

	panel_t->disp_info = info;
	panel_t->maxretpktsize = 0;

	panel_t->drvstate = DRV_STATE_INIT;
	if (!panel_t->display_enabled)
		dev_dbg(&dsi_pdev->dev, "Display not Enabled\n");
	else
		panel_t->pwrstate = STATE_SCREEN_ON;

	*handle	= (void *)panel_t;

	return res;
}

int dsic_exit(void *drv_h)
{
	struct dispdrv_panel *panel_t;

	panel_t = (struct dispdrv_panel *)drv_h;
	panel_t->drvstate = DRV_STATE_OFF;
	return 0;
}

DEFINE_MUTEX(cmnd_mutex);

int dsic_start(void)
{
	int ret;
	struct dsi_platform_data *dsip_data;

	dsip_data = platform_get_drvdata(dsi_pdev);

	ret = clk_prepare_enable(dsip_data->clk);
	if (ret) {
		dev_err(&dsi_pdev->dev, "Failed to enable clock\n");
		return -EINVAL;
	}
	return 0;
}

int dsic_stop(void)
{
	struct dsi_platform_data *dsip_data;

	dsip_data = platform_get_drvdata(dsi_pdev);
	clk_disable_unprepare(dsip_data->clk);
	return 0;
}

static void dsic_clearall_fifo(struct dsi_handle *dsic_handle)
{
	unsigned int fifomask;

	fifomask = DSI_HW_CTRL_CLR_LANED_FIFO
	    | DSI_HW_CTRL_CLR_RXPKT_FIFO
	    | DSI_HW_CTRL_CLR_PIX_DATA_FIFO | DSI_HW_CTRL_CLR_CMD_DATA_FIFO;
	dsi_hw_clr_fifo(dsic_handle->dsi_hw_handle, fifomask);
}

static int dsic_axipv_start(struct dsi_upd_req_msg *updmsg)
{
	struct axipv_config *axipv_cfg = updmsg->dsic_handle->axipv_cfg;
	struct pv_config *pv_cfg = updmsg->dsic_handle->pv_cfg;

	if (!axipv_cfg || !pv_cfg) {
		dev_err(&dsi_pdev->dev, "Invalid configuration\n");
		return -EINVAL;
	}
	axipv_cfg->width =
		(updmsg->upd_req.line_len_p + updmsg->upd_req.xstride_b)
				* updmsg->upd_req.buff_bpp;
	axipv_cfg->height = updmsg->upd_req.line_count;

	axipv_cfg->buff.sync.addr = updmsg->upd_req.buff;
	axipv_cfg->buff.sync.xlen = updmsg->upd_req.line_len_p *
					updmsg->upd_req.buff_bpp;
	axipv_cfg->buff.sync.ylen = updmsg->upd_req.line_count;

	axipv_change_state(AXIPV_CONFIG, axipv_cfg);
	pv_cfg->hact = updmsg->upd_req.line_len_p;
	pv_cfg->vact = updmsg->upd_req.line_count;
	pv_vid_config(pv_cfg);
	axipv_change_state(AXIPV_START, axipv_cfg);
	if (dsic_sem_obtain(updmsg->dsic_handle->sema_axipv,
			TICKS_IN_MILLISECONDS(SEMA_TIMEOUT_MS))) {
		dev_err(&dsi_pdev->dev,	"Timed out waiting for PV_START\n");
		return -ETIMEDOUT;
	}
	dsi_hw_de0_enable(updmsg->dsic_handle->dsi_hw_handle, true);
	pv_start(pv_cfg);

	return 0;
}

static void axipv_irq_cb(int stat)
{
	struct dsi_handle *dsic_handle = &dsi_bus;
	struct dsi_platform_data *dsip_data;

	dsip_data = platform_get_drvdata(dsi_pdev);

	if (stat & (AXIPV_DISABLED_INT | PV_START_THRESH_INT))
		dsic_sem_release(dsic_handle->sema_axipv);
	else if (stat & WATER_LVL2_INT)
		dev_dbg(&dsi_pdev->dev, "AXIPV hit LVL_2 threshold\n");
	if (stat & TE_INT)
		complete(&dsip_data->vsync_event);
}

static void pv_eof_cb(void)
{
	struct dsi_handle *dsic_handle = &dsi_bus;

	dsic_sem_release(dsic_handle->sema_pv);
}

static DEFINE_SPINLOCK(lock);

static irqreturn_t dsi0stat_lisr(int i, void *j)
{
	unsigned int  int_status;
	struct dsi_handle *dsic_handle = &dsi_bus;
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);
	int_status = dsi_hw_get_int(dsic_handle->dsi_hw_handle);
	if (!int_status) {
		dev_dbg(&dsi_pdev->dev, "interrupt status is NULL, ignoring\n");
		spin_unlock_irqrestore(&lock, flags);
		return IRQ_HANDLED;
	}
	if (int_status & BIT(FIFO_ERR_STATUS))
		dev_dbg(&dsi_pdev->dev, "fifo error %x ignoring\n", int_status);

	dsi_hw_ena_int(dsic_handle->dsi_hw_handle, 0);
	dsi_hw_clr_int(dsic_handle->dsi_hw_handle, int_status);

	dsic_sem_release(dsic_handle->sema_int);
	spin_unlock_irqrestore(&lock, flags);

	return IRQ_HANDLED;
}

bool dsi_sem_init(struct dsi_handle *dsic_handle)
{
	bool res = true;
	int ret;

	/* DSI Interface Semaphore */
	dsic_handle->sema_dsi = dsic_sem_create(1, SUSPEND_PRIORITY);
	if (!dsic_handle->sema_dsi) {
		dev_err(&dsi_pdev->dev, "sema_dsi creation error\n");
		res = false;
	}

	/* DSI Interrupt Event Semaphore */
	dsic_handle->sema_int = dsic_sem_create(0, SUSPEND_PRIORITY);
	if (!dsic_handle->sema_int) {
		dev_err(&dsi_pdev->dev, "sema_int creation error\n");
		res = false;
	}

	/* Axipv Semaphore */
	dsic_handle->sema_axipv = dsic_sem_create(0, SUSPEND_PRIORITY);
	if (!dsic_handle->sema_axipv) {
		dev_err(&dsi_pdev->dev, "sema_axipv creation error\n");
		res = false;
	}

	/* PV Semaphore */
	dsic_handle->sema_pv = dsic_sem_create(0, SUSPEND_PRIORITY);
	if (!dsic_handle->sema_pv) {
		dev_err(&dsi_pdev->dev, "sema_pv creation error\n");
		res = false;
	}

	ret = devm_request_irq(&dsi_pdev->dev, dsic_handle->interruptid,
		dsic_handle->lisr, IRQF_NO_SUSPEND, "BRCM DSI CTRL", NULL);
	if (ret < 0) {
		dev_err(&dsi_pdev->dev, "request_irq failed irq %d\n",
						dsic_handle->interruptid);
		res = false;
	}

	return res;
}

void dsi_sem_deinit(struct dsi_handle *dsic_handle)
{
	dsic_sem_free(dsic_handle->sema_pv);
	dsic_sem_free(dsic_handle->sema_axipv);
	dsic_sem_free(dsic_handle->sema_int);
	dsic_sem_free(dsic_handle->sema_dsi);
}

static int dsi_waitforstat_poll(struct dsi_handle *dsic_handle,
				unsigned int statmask, unsigned int *int_stat,
				unsigned int msec)
{
	int res = 0;
	unsigned int stat = 0;
	unsigned int  counter = 1000 * msec;

	stat = dsi_hw_get_status(dsic_handle->dsi_hw_handle);

	while ((stat & statmask) == 0 && --counter)
		stat = dsi_hw_get_status(dsic_handle->dsi_hw_handle);

	if (!counter)
		dev_dbg(&dsi_pdev->dev, "dsi hw not responding\n");

	if (int_stat != NULL)
		*int_stat = stat;

	return res;
}

static void dsic_disint(struct dsi_handle *dsic_handle)
{
	dsi_hw_ena_int(dsic_handle->dsi_hw_handle, 0);
	dsi_hw_clr_int(dsic_handle->dsi_hw_handle, 0xFFFFFFFF);
}

static inline int dsic_checkcompletion(struct dsi_handle *dsic_handle)
{
	int ret = -1;
	unsigned int  int_status, enabled_int;
	unsigned long flags = 0;

	spin_lock_irqsave(&lock, flags);

	int_status = dsi_hw_get_int(dsic_handle->dsi_hw_handle);
	if (!int_status) {
		dev_err(&dsi_pdev->dev, "Intr stat is NULL\n");
		ret = -EINVAL;
		goto done;
	}

	enabled_int = dsi_hw_get_ena_int(dsic_handle->dsi_hw_handle);
	if (int_status & enabled_int) {
		dev_dbg(&dsi_pdev->dev, "DSI ISR was pending\n");
		dsi_hw_ena_int(dsic_handle->dsi_hw_handle, 0);
		dsi_hw_clr_int(dsic_handle->dsi_hw_handle, int_status);
		ret = 0;
	}
done:
	spin_unlock_irqrestore(&lock, flags);

	return ret;
}

static int dsi_waitforint(struct dsi_handle *dsic_handle,
							unsigned int tout_msec)
{
	int res = 0;

	res = dsic_sem_obtain(
		dsic_handle->sema_int, TICKS_IN_MILLISECONDS(tout_msec));

	if (res) {
		res = -ETIMEDOUT;
		dev_err(&dsi_pdev->dev, "dsi wait for intr timed Out\n");
		dsic_disint(dsic_handle);
	}
	return res;
}

int dsic_sendtrigger(void *client, uint8_t trig)
{
	struct dsi_handle *dsic_handle;
	struct dsi_client *client_handle;
	int res;

	client_handle = (struct dsi_client *)client;
	dsic_handle = (struct dsi_handle *)client_handle->lcd_handle;

	dsi_hw_clr_status(dsic_handle->dsi_hw_handle, 0xFFFFFFFF);

	dsi_hw_tx_trig(dsic_handle->dsi_hw_handle, TX_PKT_ENG_1, trig);

	res = dsi_waitforstat_poll(dsic_handle,
				DSI_HW_STAT_TXPKT1_DONE, NULL, 100);

	return res;
}

static int dsic_sendpacket(void *client, struct dsi_cmd *command)
{
	struct dsi_handle *dsic_handle;
	struct dsi_client *client_handle;
	struct dsi_tx_cfg tx_pkt;
	enum dsi_hw_res dsihw_res;
	unsigned int stat, event, pfifo_len = 0;
	int res = 0;
	unsigned int dsi_stat, dsi_i_stat;

	client_handle = (struct dsi_client *)client;
	dsic_handle = (struct dsi_handle *)client_handle->lcd_handle;

	if (command->msg_len > DSI_HW_TX_MSG_MAX) {
		dev_err(&dsi_pdev->dev, "error, tx pkt size too big %d\n",
								command->vc);
		return -EINVAL;
	}

	res = dsic_sem_obtain(dsic_handle->sema_dsi,
				TICKS_IN_MILLISECONDS(1000));
	if (res) {
		dev_err(&dsi_pdev->dev, "sema_dsi timeout\n");
		return -ETIMEDOUT;
	}

	if (dsic_handle->init != DSI_INITIALIZED) {
		dsic_sem_release(dsic_handle->sema_dsi);
		return -EINVAL;
	}

	tx_pkt.dsi_cmnd = command->dsi_cmnd;
	tx_pkt.msg = command->msg;
	tx_pkt.msg_len = command->msg_len;
	tx_pkt.vc = command->vc;
	tx_pkt.is_lp = command->is_lp;
	tx_pkt.end_with_bta = command->end_with_bta;
	tx_pkt.vm_when = DSI_HW_CMND_WHEN_BEST_EFFORT;
	tx_pkt.repeat = 1;
	/*
	 * Don't start here if PV is enabled,
	 * instead wait for video stream to stop
	 */
	tx_pkt.start = !panel.video_enabled;
	tx_pkt.disp_engine = 1;

	dsi_hw_clr_status(dsic_handle->dsi_hw_handle, 0xFFFFFFFF);

	event = DSI_HW_ISTAT_TXPKT1_DONE;
	dsi_stat = dsi_hw_get_status(dsic_handle->dsi_hw_handle);
	dsi_i_stat = dsi_hw_get_int(dsic_handle->dsi_hw_handle);
	if (tx_pkt.start)
		dsi_hw_ena_int(dsic_handle->dsi_hw_handle, event);

	if (tx_pkt.msg_len <= 2) {
		dev_dbg(&dsi_pdev->dev, "short, msg length %d\n",
							tx_pkt.msg_len);
		tx_pkt.msg_len_cfifo = 0;	/* NA to short */
		dsihw_res = dsi_hw_tx_short
			(dsic_handle->dsi_hw_handle, TX_PKT_ENG_1, &tx_pkt);
		if (dsihw_res != DSI_HW_OK) {
			res = -EINVAL;
			goto exit_err;
		}
	} else {
		if (tx_pkt.msg_len <= DSI_HW_CMND_FIFO_SIZE_B) {
			tx_pkt.msg_len_cfifo = tx_pkt.msg_len;

			dsihw_res = dsi_hw_wr_cfifo(dsic_handle->dsi_hw_handle,
					tx_pkt.msg, tx_pkt.msg_len_cfifo);
			if (dsihw_res != DSI_HW_OK) {
				res = -EINVAL;
				goto exit_err;
			}

			dsihw_res = dsi_hw_tx_long(dsic_handle->dsi_hw_handle,
							TX_PKT_ENG_1, &tx_pkt);
			if (dsihw_res != DSI_HW_OK) {
				res = -EINVAL;
				goto exit_err;
			}
		} else {
			if (tx_pkt.msg_len > DSI_HW_PIXEL_FIFO_SIZE_B)
				tx_pkt.msg_len_cfifo =
				    tx_pkt.msg_len - DSI_HW_PIXEL_FIFO_SIZE_B;
			else
				tx_pkt.msg_len_cfifo = tx_pkt.msg_len % 4;

			pfifo_len = tx_pkt.msg_len - tx_pkt.msg_len_cfifo;

			dsihw_res = dsi_hw_wr_cfifo(dsic_handle->dsi_hw_handle,
					tx_pkt.msg, tx_pkt.msg_len_cfifo);

			if (dsihw_res != DSI_HW_OK) {
				res = -EINVAL;
				goto exit_err;
			}

			if (pfifo_len > DE1_DEF_THRESHOLD_B) {
				dsi_hw_de1_set_dma_thresh
				(dsic_handle->dsi_hw_handle, pfifo_len >> 2);
			}

			dsi_hw_de1_set_cm(dsic_handle->dsi_hw_handle,
								DE1_CM_BE);
			dsi_hw_de1_enable(dsic_handle->dsi_hw_handle, true);

			dsihw_res = dsi_hw_wr_pfifo_be
				(dsic_handle->dsi_hw_handle, tx_pkt.msg +
					tx_pkt.msg_len_cfifo, tx_pkt.msg_len -
						       tx_pkt.msg_len_cfifo);

			if (dsihw_res != DSI_HW_OK) {
				res = -EINVAL;
				goto exit_err;
			}

			dsihw_res = dsi_hw_tx_long(dsic_handle->dsi_hw_handle,
							TX_PKT_ENG_1, &tx_pkt);
			if (dsihw_res != DSI_HW_OK) {
				res = -EINVAL;
				goto exit_err;
			}
		}
	}

	res = dsi_waitforint(dsic_handle, 100);
	stat = dsi_hw_get_status(dsic_handle->dsi_hw_handle);

	if (!res)
		goto exit_ok;

exit_err:
	dsic_disint(dsic_handle);
	dsic_clearall_fifo(dsic_handle);
exit_ok:
	dsi_hw_de1_enable(dsic_handle->dsi_hw_handle, false);
	dsi_hw_tx_start(dsic_handle->dsi_hw_handle, TX_PKT_ENG_1, false);

	if (pfifo_len > DE1_DEF_THRESHOLD_B)
		dsi_hw_de1_set_dma_thresh
			(dsic_handle->dsi_hw_handle, DE1_DEF_THRESHOLD_W);

	dsic_sem_release(dsic_handle->sema_dsi);
	return res;
}

static void dsic_exec_cmdlist(struct dispdrv_panel *panel_t, char *buff)
{
	struct dsi_cmd msg;
	int res = 0;
	bool generic;

	mutex_lock(&cmnd_mutex);
	msg.vc = panel_t->cmnd_mode->vc;
	msg.is_lp = panel_t->disp_info->cmnd_lp;
	msg.end_with_bta = false;

	while (*buff) {
		uint8_t len = *buff++;

		if (len == DISPCTRL_TAG_SLEEP) {
			msleep(*buff++);
			continue;
		}
		generic = (len == DISPCTRL_TAG_GEN_WR) ? true : false;

		if (generic)
			len = *buff++;

		if (len == 1) {
			msg.dsi_cmnd = generic ? DSI_DT_SH_GEN_WR_P1 :
							DSI_DT_SH_DCS_WR_P0;
		} else if (len == 2) {
			msg.dsi_cmnd = generic ? DSI_DT_SH_GEN_WR_P2 :
							DSI_DT_SH_DCS_WR_P1;
		} else if (len > 2 && len <= MAX_TXM_SIZE) {
			msg.dsi_cmnd = generic ? DSI_DT_LG_GEN_WR :
							DSI_DT_LG_DCS_WR;
			msg.is_long = true;
		} else {
			dev_err(&dsi_pdev->dev, "Packet size err %d\n", res);
			res = -EINVAL;
			goto err;
		}
		msg.msg = buff;
		msg.msg_len = len;

		if (panel.drvstate != DRV_STATE_OPEN) {
			dev_err(&dsi_pdev->dev, "driver not in OPEN state\n");
			goto err;
		}

		res = dsic_sendpacket(panel_t->client_handle, &msg);
		buff += len;
	}
err:
	mutex_unlock(&cmnd_mutex);
}

int dsic_panel_on(struct dispdrv_panel *panel_t, bool on)
{
	struct dsi_cmd msg;
	int res = 0;

	msg.dsi_cmnd = (on == true) ? DSI_DT_SH_TURN_ON : DSI_DT_SH_SHUT_DOWN;
	msg.msg = NULL;
	msg.msg_len = 0;
	msg.vc = panel_t->cmnd_mode->vc;
	msg.is_lp = panel_t->disp_info->cmnd_lp;
	msg.is_long = false;
	msg.end_with_bta = false;

	res = dsic_sendpacket(panel_t->client_handle, &msg);
	return res;
}

static int dsic_open_cmvc(void *client,
		struct dsi_cm_vc *dsi_cmvc_cfg, void **dsi_cmvc_h)
{
	struct dsi_handle *dsic_handle;
	struct dsi_client *client_handle;
	struct dsi_cm_handle *cm_vc_h;
	struct axipv_config *apv_cfg;
	int res = 0;

	client_handle = (struct dsi_client *)client;
	dsic_handle = (struct dsi_handle *)client_handle->lcd_handle;
	apv_cfg = dsic_handle->axipv_cfg;

	dsic_sem_obtain(dsic_handle->sema_dsi, TICKS_FOREVER);

	cm_vc_h = &dsic_handle->chcm;
	cm_vc_h->vc = dsi_cmvc_cfg->vc;
	cm_vc_h->dsi_cmnd = dsi_cmvc_cfg->dsi_cmnd;
	cm_vc_h->is_lp = dsi_cmvc_cfg->is_lp;
	cm_vc_h->vm_when = DSI_HW_CMND_WHEN_BEST_EFFORT;

	cm_vc_h->dcs_cmd_start = dsi_cmvc_cfg->dcs_cmd_start;
	cm_vc_h->dcs_cmd_cont = dsi_cmvc_cfg->dcs_cmd_cont;

	switch (dsi_cmvc_cfg->cm_in) {
	case LCD_IF_CM_I_xBGR8888:
	case LCD_IF_CM_I_xRGB8888:
		switch (dsi_cmvc_cfg->cm_out) {
		case LCD_IF_CM_O_xRGB8888:
			cm_vc_h->bpp_dma = 4;
			cm_vc_h->bpp_wire = 3;
			cm_vc_h->wc_rshift = 0;
			cm_vc_h->cm = DE0_CM_888U;
			if (dsi_cmvc_cfg->cm_in == LCD_IF_CM_I_xBGR8888)
				apv_cfg->pix_fmt = AXIPV_PIXEL_FORMAT_24BPP_BGR;
			else
				apv_cfg->pix_fmt = AXIPV_PIXEL_FORMAT_24BPP_RGB;
			dsic_handle->pv_cfg->pix_fmt = DSI_VIDEO_CMD_18_24BPP;
			break;
		default:
			res = -EINVAL;
			goto exit_err;
		}
		break;
	case LCD_IF_CM_I_RGB565P:
		switch (dsi_cmvc_cfg->cm_out) {
		case LCD_IF_CM_O_RGB565:
			cm_vc_h->bpp_dma = 2;
			cm_vc_h->bpp_wire = 2;
			cm_vc_h->wc_rshift = 1;
			apv_cfg->pix_fmt = AXIPV_PIXEL_FORMAT_16BPP_PACKED;
			dsic_handle->pv_cfg->pix_fmt = DSI_VIDEO_16BPP;
			cm_vc_h->cm = DE0_CM_565P;
			break;
		default:
			res = -EINVAL;
			goto exit_err;
		}
		break;

	default:
		res = -EINVAL;
		goto exit_err;
	}

	cm_vc_h->configured = true;
	cm_vc_h->client = client_handle;

	*dsi_cmvc_h = (void *)cm_vc_h;
	dsic_sem_release(dsic_handle->sema_dsi);

	return res;

exit_err:
	dev_err(&dsi_pdev->dev, "invalid color mode\n");
	*dsi_cmvc_h = NULL;
	dsic_sem_release(dsic_handle->sema_dsi);
	return res;
}

static int dsic_close_cmvc(void *vc_handle)
{
	struct dsi_cm_handle *dsic_ch_handle;
	struct dsi_handle *dsic_handle;
	struct dsi_client *client_handle;

	dsic_ch_handle = (struct dsi_cm_handle *)vc_handle;
	client_handle = (struct dsi_client *)dsic_ch_handle->client;
	dsic_handle = (struct dsi_handle *)client_handle->lcd_handle;

	dsic_sem_obtain(dsic_handle->sema_dsi, TICKS_FOREVER);
	dsic_ch_handle->configured = false;

	dsic_sem_release(dsic_handle->sema_dsi);

	return 0;
}

static int dsic_update_vmvc(void *vc_handle, struct dsi_lcd_upd_req *req)
{
	int res = 0;
	struct dsi_handle *dsic_handle;
	struct dsi_client *client_handle;
	struct dsi_cm_handle *dsic_ch_handle;
	struct dsi_upd_req_msg updmsg;
	struct axipv_config *axipv_cfg;
	struct pv_config *pv_cfg;

	dsic_ch_handle = (struct dsi_cm_handle *) vc_handle;
	client_handle = (struct dsi_client *) dsic_ch_handle->client;
	updmsg.client_handle = client_handle;
	dsic_handle = (struct dsi_handle *)client_handle->lcd_handle;
	updmsg.upd_req = *req;
	updmsg.dsic_handle = dsic_handle;
	updmsg.upd_req.buff_bpp = dsic_ch_handle->bpp_dma;
	axipv_cfg = dsic_handle->axipv_cfg;
	pv_cfg = dsic_handle->pv_cfg;

	if (!axipv_cfg || !pv_cfg) {
		dev_err(&dsi_pdev->dev, "Configuration is NULL\n");
		return -EINVAL;
	}
	dsic_sem_obtain(dsic_handle->sema_dsi, TICKS_FOREVER);
	if (!panel.video_enabled) {
		dsic_clearall_fifo(dsic_handle);
		dsi_hw_clr_status(dsic_handle->dsi_hw_handle, 0xffffffff);
		dsi_hw_de0_set_cm(dsic_handle->dsi_hw_handle,
							dsic_ch_handle->cm);
		dsi_hw_de0_set_mode(dsic_handle->dsi_hw_handle, DE0_MODE_VID);
		dsi_hw_de0_st_end(dsic_handle->dsi_hw_handle,
							dsic_handle->st_end);
		/* Set pix clk divider to bits per pixel for non-burst mode */
		dsi_hw_de0_set_pix_clk_div(dsic_handle->dsi_hw_handle,
			(dsic_ch_handle->bpp_wire << 3) / dsic_handle->dlcount);
		res = dsic_axipv_start(&updmsg);
		if (res) {
			dev_err(&dsi_pdev->dev, "Failed To Start DMA\n");
			goto done;
		}
		panel.video_enabled = 1;
	} else {
		axipv_cfg->buff.async = updmsg.upd_req.buff;
		axipv_post(axipv_cfg);
	}
done:
	dsic_sem_release(dsic_handle->sema_dsi);
	return res;
}

static int dsic_suspend(void *vc_handle)
{
	struct dsi_handle *dsic_handle;
	struct dsi_client *client_handle;
	struct dsi_cm_handle *dsic_ch_handle;
	struct axipv_config *axipv_cfg;
	struct pv_config *pv_cfg;
	int res;

	dsic_ch_handle = (struct dsi_cm_handle *)vc_handle;
	client_handle = (struct dsi_client *)dsic_ch_handle->client;
	dsic_handle = (struct dsi_handle *)client_handle->lcd_handle;
	axipv_cfg = dsic_handle->axipv_cfg;
	pv_cfg = dsic_handle->pv_cfg;

	axipv_change_state(AXIPV_STOP_EOF, axipv_cfg);
	res = dsic_sem_obtain(dsic_handle->sema_axipv,
					msecs_to_jiffies(SEMA_TIMEOUT_MS));
	if (res) {
		dev_err(&dsi_pdev->dev, "couldn't stop AXIPV at EOF\n");
		return -ETIMEDOUT;
	}
	pv_send_event(PV_STOP_EOF_ASYNC, pv_cfg);
	res = dsic_sem_obtain(dsic_handle->sema_pv,
					msecs_to_jiffies(SEMA_TIMEOUT_MS));
	if (res) {
		dev_dbg(&dsi_pdev->dev, "couldn't stop pv, using force stop");
		pv_send_event(PV_STOP_IMM, pv_cfg);
	}
	dsi_hw_de0_enable(dsic_handle->dsi_hw_handle, false);
	panel.video_enabled = 0;

	return 0;
}

static int dsic_closeclient(void *client)
{
	struct dsi_handle *dsic_handle;
	struct dsi_client *client_handle;

	client_handle = (struct dsi_client *)client;
	dsic_handle = (struct dsi_handle *)client_handle->lcd_handle;

	dsic_sem_obtain(dsic_handle->sema_dsi, TICKS_FOREVER);

	client_handle->open = false;
	dsic_handle->clients--;
	dsic_sem_release(dsic_handle->sema_dsi);

	return 0;
}

static int dsic_openclient(void **client_handle)
{
	struct dsi_handle *dsic_handle;
	int res = 0;

	dsic_handle = (struct dsi_handle *)&dsi_bus;
	dsic_sem_obtain(dsic_handle->sema_dsi, TICKS_FOREVER);

	if (dsic_handle->init != DSI_INITIALIZED) {
		*client_handle = (void *)NULL;
		res = -EINVAL;
		goto fail;
	}

	dsic_handle->client.lcd_handle = &dsi_bus;
	dsic_handle->client.open = true;
	*client_handle = (void *)&dsic_handle->client;
	dsic_handle->clients++;

fail:
	dsic_sem_release(dsic_handle->sema_dsi);
	return res;
}

static int dsic_mod_close(void)
{
	struct dsi_handle *dsic_handle;
	int res = 0;

	dsic_handle = (struct dsi_handle *)&dsi_bus;
	dsic_sem_obtain(dsic_handle->sema_dsi, TICKS_FOREVER);

	if (dsic_handle->init != DSI_INITIALIZED || !dsic_handle->pv_cfg ||
					!dsic_handle->axipv_cfg) {
		dev_err(&dsi_pdev->dev, "failed to close dsic\n");
		res = -EINVAL;
		goto fail;
	}

	dsi_hw_off(dsic_handle->dsi_hw_handle);
	dsi_hw_phy_afe_off(dsic_handle->dsi_hw_handle);
	dsic_handle->init = ~DSI_INITIALIZED;
	dsic_handle->initonce = ~DSI_INITIALIZED;
	dsic_handle->initpv = ~DSI_INITIALIZED;
	dsic_handle->initaxipv = ~DSI_INITIALIZED;

	dsi_sem_deinit(dsic_handle);
	dev_dbg(&dsi_pdev->dev, "dsi close is ok\n");
	return res;

fail:
	dsic_sem_release(dsic_handle->sema_dsi);
	return res;
}

static void dsi_hw_set_api_clks(struct dsi_handle *dsic_handle,
				      const struct dsi_cfg *dsi_cfg) {

	dsic_handle->clkcfg.esc_clk_mhz = dsi_cfg->esc_clk.clk_in_mhz
	    / dsi_cfg->esc_clk.clk_in_div;
	dsic_handle->clkcfg.hsbit_clk_mhz = dsi_cfg->hsbit_clk.clk_in_mhz
	    / dsi_cfg->hsbit_clk.clk_in_div;

	if (dsic_handle->clkcfg.hsbit_clk_mhz > 200)
		dsic_handle->clkcfg.coreclksel = DSI_HW_BIT_CLK_DIV_BY_8;
	else
		dsic_handle->clkcfg.coreclksel = DSI_HW_BIT_CLK_DIV_BY_2;
}

static int dsic_mod_init(const struct dsi_cfg *dsi_cfg,
				struct dsi_platform_data *dsic_pvt_data)
{
	int ret;
	int res = 0;
	struct dsi_handle *dsic_handle;

	struct dsi_hw_mode dsi_hw_mode;
	struct dsi_hw_init dsihw_init;
	unsigned long timeout_val;
	struct dsi_clk_cfg *clk_cfg;

	struct axipv_init axipv_init_data = {
		.dsi_dev = &dsi_pdev->dev,
		.irq = dsic_pvt_data->axipv_irq,
		.base_addr = dsic_pvt_data->axipv_base,
		.irq_cb = axipv_irq_cb,
	};
	struct pv_init pv_init_data = {
		.dsi_dev = &dsi_pdev->dev,
		.irq = dsic_pvt_data->pv_irq,
		.base_addr = dsic_pvt_data->pv_base,
		.eof_cb = pv_eof_cb,
	};

	dsic_handle = (struct dsi_handle *)&dsi_bus;

	if (dsic_handle->init == DSI_INITIALIZED) {
		dev_err(&dsi_pdev->dev, "DSI init already done\n");
		return -EINVAL;
	}

	if (dsic_handle->initonce != DSI_INITIALIZED) {

		memset(dsic_handle, 0, sizeof(struct dsi_handle));

		dsic_handle->dsicore_regaddr = dsic_pvt_data->dsi_base;
		dsic_handle->interruptid = dsic_pvt_data->dsi_irq;
		dsic_handle->lisr = dsi0stat_lisr;
		dsic_handle->hisr = NULL;

		if (!dsi_sem_init(dsic_handle)) {
			dev_err(&dsi_pdev->dev, "error in dsi init\n");
			return -EINVAL;
		}
		dsic_handle->initonce = DSI_INITIALIZED;
	}

	dsic_handle->dlcount = dsi_cfg->dlcount;
	dsic_handle->st_end = dsi_cfg->ena_st_end;
	dsic_handle->vmode = dsi_cfg->vmode;

	if (dsic_handle->initpv != DSI_INITIALIZED) {
		dev_info(&dsi_pdev->dev, "Initialising PV\n");
		ret = pv_init(&pv_init_data, &dsic_handle->pv_cfg);
		if ((ret < 0) || !dsic_handle->pv_cfg) {
			dev_err(&dsi_pdev->dev, "pv_init failed %d\n", ret);
			return ret;
		}
		dsic_handle->initpv = DSI_INITIALIZED;
	}

	dsic_handle->pv_cfg->pclk_sel = DISP_CTRL_DSI;
	dsic_handle->pv_cfg->vs = dsi_cfg->vs;
	dsic_handle->pv_cfg->vbp = dsi_cfg->vbp;
	dsic_handle->pv_cfg->vfp = dsi_cfg->vfp;
	dsic_handle->pv_cfg->hs = dsi_cfg->hs;
	dsic_handle->pv_cfg->hbp = dsi_cfg->hbp;
	dsic_handle->pv_cfg->hfp = dsi_cfg->hfp;
	dsic_handle->pv_cfg->hbllp = dsi_cfg->hbllp;
	dsic_handle->pv_cfg->vsyncd = 0;
	dsic_handle->pv_cfg->pix_stretch = 0;

	if (dsic_handle->initaxipv != DSI_INITIALIZED) {
		dev_info(&dsi_pdev->dev, "Initialising AXIPV\n");
		ret = axipv_init(&axipv_init_data,
				&dsic_handle->axipv_cfg, panel.display_enabled);
		if ((ret < 0) || !dsic_handle->axipv_cfg) {
			dev_err(&dsi_pdev->dev,	"axipv_init failed %d\n", ret);
			return ret;
		}
		dsic_handle->initaxipv = DSI_INITIALIZED;
	}

	/* Init User Controlled Values */
	dsihw_init.dlcount = dsi_cfg->dlcount;

	dsi_hw_set_api_clks(dsic_handle, dsi_cfg);
	panel.video_enabled = (panel.display_enabled && dsic_handle->vmode);

	dsic_handle->dsi_hw_handle = (void *)dsi_hw_init
				(dsic_handle->dsicore_regaddr, &dsihw_init);
	dsi_errata_init(dsic_pvt_data->dsi_genpll_base,
				dsic_pvt_data->mipi_errata_size,
					dsic_pvt_data->has_mipi_errata);

	if (dsic_handle->dsi_hw_handle == NULL) {
		dev_err(&dsi_pdev->dev, "dsi_hw_init failed\n");
		res = -EINVAL;
		goto err;
	}

	clk_cfg = &dsic_handle->clkcfg;
	dsi_hw_mode.clksel = clk_cfg->coreclksel;

	if (panel.display_enabled)
		goto init_done;

	if (!dsi_hw_set_timing(dsic_handle->dsi_hw_handle, dsi_cfg->phy_timing,
			clk_cfg->coreclksel, clk_cfg->esc_clk_mhz,
			clk_cfg->hsbit_clk_mhz,	dsi_cfg->lpbitrate_mbps)) {
		dev_err(&dsi_pdev->dev, "timing calculation error\n");
		res = -EINVAL;
		goto err;
	}
	dsi_hw_de1_set_dma_thresh(dsic_handle->dsi_hw_handle,
							DE1_DEF_THRESHOLD_W);
	dsic_clearall_fifo(dsic_handle);
	/* wait for STOP state */
	timeout_val = TICKS_IN_MILLISECONDS(1);
	while (timeout_val)
		timeout_val = schedule_timeout_uninterruptible(timeout_val);

init_done:
	if (!res) {
		dev_info(&dsi_pdev->dev, "dsi init is ok\n");
		dsic_handle->init = DSI_INITIALIZED;
		return res;
	}

err:
	dsic_handle->init = 0;
	return res;
}

int dsic_open(void *drv_h)
{
	int res = 0;
	struct dispdrv_panel *panel_t;
	struct dsi_platform_data *dsic_pvt_data;
	struct dsi_handle *dsic_handle;

	panel_t = (struct dispdrv_panel *)drv_h;
	dsic_pvt_data = platform_get_drvdata(dsi_pdev);

	if (panel_t->drvstate != DRV_STATE_INIT) {
		dev_err(&dsi_pdev->dev, "panel init not done\n");
		return -EINVAL;
	}

	res = dsic_mod_init(panel_t->dsi_cfg, dsic_pvt_data);
	if (res) {
		dev_err(&dsi_pdev->dev, "DSI CTRL Init Failed\n");
		return res;
	}

	res = dsic_openclient(&panel_t->client_handle);
	if (res) {
		dev_err(&dsi_pdev->dev, "dsic_openclient Failed\n");
		goto err_open_cl;
	}

	res = dsic_open_cmvc(panel_t->client_handle,
				panel_t->cmnd_mode, &panel_t->dsicmvc_handle);
	if (res) {
		dev_err(&dsi_pdev->dev, "dsic_open_cmvc Failed\n");
		goto err_open_cm;
	}

	dsic_handle = &dsi_bus;

	dsi_hw_on(dsic_handle->dsi_hw_handle, dsic_pvt_data->dsi_genpll_base);

	/* do dsic_hw_reset in power off state only */
	if (panel_t->pwrstate == STATE_PWR_OFF)
		dsic_hw_reset(false);

	panel_t->win_dim.l = 0;
	panel_t->win_dim.r = panel_t->disp_info->width - 1;
	panel_t->win_dim.t = 0;
	panel_t->win_dim.b = panel_t->disp_info->height - 1;
	panel_t->win_dim.w = panel_t->disp_info->width;
	panel_t->win_dim.h = panel_t->disp_info->height;
	panel_t->drvstate = DRV_STATE_OPEN;

	return res;

err_open_cm:
	dsic_closeclient(panel_t->client_handle);
err_open_cl:
	dsic_mod_close();
	return res;
}

int dsic_close(void *drv_h)
{
	int res = 0;
	struct dispdrv_panel *panel_t = (struct dispdrv_panel *)drv_h;

	dsic_exec_cmdlist(panel_t, panel_t->disp_info->slp_in_seq);
	panel_t->drvstate = DRV_STATE_INIT;
	dsic_suspend(panel_t->dsicmvc_handle);

	res = dsic_close_cmvc(panel_t->dsicmvc_handle);
	if (res) {
		dev_err(&dsi_pdev->dev, "error closing dsi cmvc\n");
		return res;
	}

	res = dsic_closeclient(panel_t->client_handle);
	if (res) {
		dev_err(&dsi_pdev->dev, "error closing dsi client\n");
		return res;
	}

	res = dsic_mod_close();
	if (res) {
		dev_err(&dsi_pdev->dev, "error closing dsi controller\n");
		return res;
	}

	panel_t->drvstate = DRV_STATE_INIT;
	dev_info(&dsi_pdev->dev, "dsi close is ok\n");

	return res;
}

int dsic_powercontrol(void *drv_h, enum disp_pwr_state state)
{
	int  res = 0;
	struct dispdrv_panel *panel_t = (struct dispdrv_panel *)drv_h;
	struct dispdrv_info *info = panel_t->disp_info;

	switch (state) {
	case CTRL_PWR_ON:
		if (panel_t->pwrstate != STATE_PWR_OFF)
			break;
		usleep_range(1000, 1010);
		dsic_exec_cmdlist(panel_t, info->init_seq);
		panel_t->pwrstate = STATE_SCREEN_OFF;
		break;
	case CTRL_PWR_OFF:
		if (panel_t->pwrstate == STATE_PWR_OFF)
			break;
		dsic_panel_on(panel_t, false);
		dsic_exec_cmdlist(panel_t, info->slp_in_seq);
		dsic_hw_reset(true);
		panel_t->pwrstate = STATE_PWR_OFF;
		break;
	case CTRL_SCREEN_ON:
		if (panel_t->pwrstate != STATE_SCREEN_OFF)
			break;
		dsic_panel_on(panel_t, true);
		panel_t->pwrstate = STATE_SCREEN_ON;
		panel.display_enabled = 1;
		break;
	case CTRL_SCREEN_OFF:
		if (panel_t->pwrstate != STATE_SCREEN_ON)
			break;
		panel.display_enabled = 0;
		dsic_panel_on(panel_t, false);
		panel_t->pwrstate = STATE_SCREEN_OFF;
		break;
	default:
		dev_err(&dsi_pdev->dev, "Invalid State[%d]\n", state);
		res = -EINVAL;
		break;
	}
	return res;
}

int dsic_update(void *drv_h, dma_addr_t buff)
{
	struct dispdrv_panel *panel_t = (struct dispdrv_panel *)drv_h;
	struct dsi_lcd_upd_req req;
	struct dispdrv_win *p_win;
	int res  = 0;
	unsigned int offset;

	if (panel_t->pwrstate == STATE_PWR_OFF) {
		dev_err(&dsi_pdev->dev, "Skip Due To Power State\n");
		return -EINVAL;
	}

	p_win =	&panel_t->win_dim;

	offset = (p_win->t * panel_t->disp_info->width + p_win->l)
			* panel_t->disp_info->bpp;

	req.buff = buff + offset;
	req.line_len_p = p_win->w;
	req.line_count = p_win->h;
	req.xstride_b = panel_t->disp_info->width - p_win->w;
	req.buff_bpp = panel_t->disp_info->bpp;

	res = dsic_update_vmvc(panel_t->dsicmvc_handle, &req);
	if (res)	{
		dev_err(&dsi_pdev->dev, "DSI_CTRL_Update error %d\n", res);
		res = -EINVAL;
	}

	return res;
}
