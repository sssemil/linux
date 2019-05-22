/* Copyright (c) 2013-2014, Hisilicon Tech. Co., Ltd. All rights reserved.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 and
* only version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
* GNU General Public License for more details.
*
*/
#include "hisi_fb.h"
#include "hisi_dp.h"
#include "hisi_fb_def.h"

#include "avgen.h"
#include "dp_aux.h"
#include "link.h"
#include "intr.h"
#include "core.h"


/*lint -save -e* */
static void dptx_notify(struct dp_ctrl *dptx)
{
	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return;
	}

	wake_up_interruptible(&dptx->waitq);
}

void dptx_notify_shutdown(struct dp_ctrl *dptx)
{
	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return;
	}

	atomic_set(&dptx->shutdown, 1);
	wake_up_interruptible(&dptx->waitq);
}

int handle_hotunplug(struct hisi_fb_data_type *hisifd)
{
	struct dp_ctrl *dptx;

	if (!hisifd) {
		HISI_FB_ERR("hisifd is NULL!\n");
		return -EINVAL;
	}

	HISI_FB_INFO("+.\n");

	dptx = &(hisifd->dp);

	hisifd->hpd_release_sub_fnc(hisifd->fbi);

	/* Clear xmit enables */
	dptx_phy_enable_xmit(dptx, 4, false);

	/* Disable AUX Block */
	dptx_aux_disreset(dptx, false);

	/* Power down all lanes */
	/* TODO */
	dptx_phy_set_lanes_status(dptx, false);

#ifdef CONFIG_DP_HDCP13_ENABLE
	if (dptx->hparams.hdcp13_is_en) {
		dptx_en_dis_hdcp13(dptx, 0);
	}
#endif

	atomic_set(&dptx->sink_request, 0);
	atomic_set(&dptx->aux.event, 0);
	dptx->link.trained = false;

	dptx_send_cable_notification(dptx, 0);
	dptx_disable_default_video_stream(dptx);

	dptx->threaded_irq_unplug = true;

	HISI_FB_INFO("-.\n");

	return 0;
}

static int dptx_read_edid_block(struct dp_ctrl *dptx,
	unsigned int block)
{
	int retval;
	uint8_t offset = block * 128;
	uint8_t segment = block >> 1;
#ifdef CONFIG_DP_EDID_DEBUG
	int i;
#endif

	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return -EINVAL;
	}

	HISI_FB_INFO("block=%d.\n", block);

	retval = dptx_write_bytes_to_i2c(dptx, 0x30, &segment, 1);
	if (retval) {
		HISI_FB_ERR("failed to  dptx_write_bytes_to_i2c 1!\n");
		return retval;
	}

	/* TODO Skip if no E-DDC */

	retval = dptx_write_bytes_to_i2c(dptx, 0x50, &offset, 1);
	if (retval) {
		HISI_FB_ERR("failed to  dptx_write_bytes_to_i2c 2!\n");
		return retval;
	}

	retval = dptx_read_bytes_from_i2c(dptx, 0x50,
		&dptx->edid[block * 128], 128);
	if (retval) {
		HISI_FB_ERR("failed to  dptx_read_bytes_from_i2c 2!\n");
		return retval;
	}

#ifdef CONFIG_DP_EDID_DEBUG
	for (i = 0; i < 128; i++) {
		if (i != 0 && !(i % 16))
			printk(" @@@@ EDID : \n");

		if (!(i % 16))
			printk("%04x: ", i);

		printk("%02x ", dptx->edid[block * 128 + i]);
	}
#endif

	return 0;
}

static int dptx_read_edid(struct dp_ctrl *dptx)
{
	int i;
	int retval = 0;
	unsigned int ext_blocks = 0;
	uint8_t *first_edid_block;

	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return -EINVAL;
	}

	if (!(dptx->edid)) {
		HISI_FB_ERR("edid is NULL!\n");
		return -EINVAL;
	}

	memset(dptx->edid, 0, DPTX_DEFAULT_EDID_BUFLEN);
	retval = dptx_read_edid_block(dptx, 0);
	if (retval) {
		HISI_FB_ERR("failed to dptx_read_edid_block!\n");
		return -EINVAL;
	}

	ext_blocks = dptx->edid[126];

	first_edid_block = kmalloc(128, GFP_KERNEL);
	if (first_edid_block == NULL) {
		HISI_FB_ERR("Allocate buffer error\n");
		return -EINVAL;
	}
	memcpy(first_edid_block, dptx->edid, 128);

	if (dptx->edid) {
		kfree(dptx->edid);
		dptx->edid = NULL;
	}

	dptx->edid = kzalloc(128 * ext_blocks + 128, GFP_KERNEL);
	if (!dptx->edid) {
		HISI_FB_ERR("Allocate edid buffer error!\n");
		return -EINVAL;
	}

	memcpy(dptx->edid, first_edid_block, 128);
	for (i = 1; i <= ext_blocks; i++) {
		retval = dptx_read_edid_block(dptx, i);
		if (retval) {
			goto fail;
		}
	}

	return 0;

fail:
	if (first_edid_block) {
		kfree(first_edid_block);
		first_edid_block = NULL;
	}

	return retval;
}

static int dptx_check_edid(struct dp_ctrl *dptx)
{
	int i;
	u32 edid_sum = 0;

	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return -EINVAL;
	}

	if (!(dptx->edid)) {
		HISI_FB_ERR("edid is NULL!\n");
		return -EINVAL;
	}

	for (i = 0; i < 128; i++)
		edid_sum += dptx->edid[i];

	if (edid_sum & 0xFF) {
		HISI_FB_ERR("Invalid EDID checksum\n");
		return -EINVAL;
	}

	return 0;
}

static int dptx_resolution_switch(struct hisi_fb_data_type *hisifd)
{
	struct dtd *mdtd;
	struct dp_ctrl *dptx;
	struct hisi_panel_info *pinfo;

	if (!hisifd) {
		HISI_FB_ERR("hisifd is NULL!\n");
		return -EINVAL;
	}

	dptx = &(hisifd->dp);
	mdtd = &(dptx->vparams.mdtd);

	pinfo = &(hisifd->panel_info);

	pinfo->xres = mdtd->h_active;
	pinfo->yres = mdtd->v_active;
	pinfo->ldi.h_back_porch =
		(mdtd->h_blanking - mdtd->h_sync_offset -mdtd->h_sync_pulse_width);
	pinfo->ldi.h_front_porch = mdtd->h_sync_offset;
	pinfo->ldi.h_pulse_width = mdtd->h_sync_pulse_width;
	pinfo->ldi.v_back_porch =
		(mdtd->v_blanking - mdtd->v_sync_offset - mdtd->v_sync_pulse_width);
	pinfo->ldi.v_front_porch = mdtd->v_sync_offset;
	pinfo->ldi.v_pulse_width = mdtd->v_sync_pulse_width;
	pinfo->ldi.hsync_plr = 1 - mdtd->h_sync_polarity;
	pinfo->ldi.vsync_plr = 1 - mdtd->v_sync_polarity;
	pinfo->pxl_clk_rate_div = 1;

	pinfo->pxl_clk_rate = mdtd->pixel_clock * 1000;
	if (pinfo->pxl_clk_rate % 1000000) {
		pinfo->pxl_clk_rate = ((pinfo->pxl_clk_rate /1000000) + 1) * 1000000;
	}

	hisifd->fbi->var.xres = pinfo->xres;
	hisifd->fbi->var.yres = pinfo->yres;

	HISI_FB_INFO("xres=%d\n"
		"yres=%d\n"
		"h_back_porch=%d\n"
		"h_front_porch=%d\n"
		"h_pulse_width=%d\n"
		"v_back_porch=%d\n"
		"v_front_porch=%d\n"
		"v_pulse_width=%d\n"
		"hsync_plr=%d\n"
		"vsync_plr=%d\n"
		"pxl_clk_rate_div=%d\n"
		"pxl_clk_rate=%llu\n"
		"var.xres=%d\n"
		"var.yres=%d\n",
		pinfo->xres,
		pinfo->yres,
		pinfo->ldi.h_back_porch,
		pinfo->ldi.h_front_porch,
		pinfo->ldi.h_pulse_width,
		pinfo->ldi.v_back_porch,
		pinfo->ldi.v_front_porch,
		pinfo->ldi.v_pulse_width,
		pinfo->ldi.hsync_plr,
		pinfo->ldi.vsync_plr,
		pinfo->pxl_clk_rate_div,
		pinfo->pxl_clk_rate,
		hisifd->fbi->var.xres,
		hisifd->fbi->var.yres);

	dptx_send_cable_notification(dptx, 1);
	hisifd->hpd_open_sub_fnc(hisifd->fbi);
	//mdelay(10);
	dptx_enable_default_video_stream(dptx);

	dptx->threaded_irq_unplug = false;

	return 0;
}

static int handle_hotplug(struct hisi_fb_data_type *hisifd)
{
#if 0
	uint32_t phyifctrl;
	u8 preferred_vic[18];
#endif
	uint8_t rev;
	int retval;
	uint8_t vector;
	uint8_t checksum = 0;
	uint8_t blocks = 0;
	u8 preferred_vic[18];
	uint8_t test;

	struct video_params *vparams;
	struct hdcp_params *hparams;
	struct dtd mdtd;
	struct dp_ctrl *dptx;

	if (!hisifd) {
		HISI_FB_ERR("hisifd is NULL!\n");
		return -EINVAL;
	}

	HISI_FB_INFO("+.\n");

	dptx = &(hisifd->dp);

	vparams = &dptx->vparams;
	hparams = &dptx->hparams;

	dptx_audio_config(dptx);

	//temprary
	dptx->vparams.mode = 16;

	dptx_video_config(dptx);

	dptx_soft_reset(dptx,
		DPTX_SRST_CTRL_PHY | DPTX_SRST_CTRL_HDCP | DPTX_SRST_CTRL_AUX);

	/* Enable AUX Block */
	dptx_aux_disreset(dptx, true);
	dptx_core_init_phy(dptx);

#if 0
	phyifctrl = inp32(dptx->base + DPTX_PHYIF_CTRL);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK(0);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK(1);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK(2);
	phyifctrl &= ~DPTX_PHYIF_CTRL_LANE_PWRDOWN_MASK(3);
	outp32(dptx->base + DPTX_PHYIF_CTRL, phyifctrl);
#endif

	retval = dptx_read_dpcd(dptx, DP_DEVICE_SERVICE_IRQ_VECTOR, &vector);
	if (retval) {
		HISI_FB_ERR("failed to  dptx_read_dpcd DP_DEVICE_SERVICE_IRQ_VECTOR, retval=%d.", retval);
		return retval;
	}

	retval = dptx_read_edid(dptx);
	if (retval) {
		HISI_FB_ERR("failed to  dptx_read_edid, retval=%d.", retval);
		return retval;
	}

#if 1
	retval = dptx_check_edid(dptx);
	if (retval) {
		vparams->video_format = VCEA;
		dptx_dtd_fill(&mdtd, 1, vparams->refresh_rate, vparams->video_format);
	} else {
		memcpy(preferred_vic, dptx->edid + 0x36, 0x12);
		retval = dptx_dtd_parse(dptx, &mdtd, preferred_vic);
		if (retval) {
			vparams->video_format = VCEA;
			dptx_dtd_fill(&mdtd, 1, vparams->refresh_rate, vparams->video_format);
		}
	}
	memcpy(&(dptx->vparams.mdtd), &mdtd, sizeof(mdtd));
#else
	mdtd = dptx->vparams.mdtd;
#endif

	retval = dptx_read_dpcd(dptx, DP_DPCD_REV, &rev);
	if (retval) {
		/* Abort bringup */
		/* Reset core and try again */
		/* Abort all aux, and other work, reset the core */
		HISI_FB_ERR("failed to dptx_read_dpcd DP_DPCD_REV, retval=%d.\n", retval);
		return retval;
	}
	HISI_FB_INFO("DP Revision %x.%x .\n", (rev & 0xf0) >> 4, rev & 0xf);

	memset(dptx->rx_caps, 0, DPTX_RECEIVER_CAP_SIZE);
	retval = dptx_read_bytes_from_dpcd(dptx, DP_DPCD_REV,
		dptx->rx_caps, DPTX_RECEIVER_CAP_SIZE);
	if (retval) {
		HISI_FB_ERR("failed to dptx_read_bytes_from_dpcd DP_DPCD_REV, retval=%d.\n", retval);
		return retval;
	}

	/*
	* The TEST_EDID_READ is asserted on HOTPLUG. Check for it and
	* handle it here.
	*/
	if (vector & DP_AUTOMATED_TEST_REQUEST) {
		HISI_FB_INFO("DP_AUTOMATED_TEST_REQUEST");
		retval = dptx_read_dpcd(dptx, DP_TEST_REQUEST, &test);
		if (retval) {
			HISI_FB_ERR("failed to dptx_read_dpcd DP_TEST_REQUEST, retval=%d.\n", retval);
			return retval;
		}

		if (test & DP_TEST_LINK_EDID_READ) {
			blocks = dptx->edid[126];
			checksum = dptx->edid[127 + 128 * blocks];

			retval = dptx_write_dpcd(dptx, DP_TEST_EDID_CHECKSUM, checksum);
			if (retval) {
				HISI_FB_ERR("failed to dptx_write_dpcd DP_TEST_EDID_CHECKSUM, retval=%d.\n", retval);
				return retval;
			}

			retval = dptx_write_dpcd(dptx, DP_TEST_RESPONSE, DP_TEST_EDID_CHECKSUM_WRITE);
			if (retval) {
				HISI_FB_ERR("failed to dptx_write_dpcd DP_TEST_RESPONSE, retval=%d.\n", retval);
				return retval;
			}
		}
	}

	/* TODO No other IRQ should be set on hotplug */
	retval = dptx_link_training(dptx, dptx->max_rate, dptx->max_lanes);
	if (retval) {
		HISI_FB_ERR("failed to  dptx_link_training, retval=%d.\n", retval);
		return retval;
	}

	retval = dptx_video_ts_calculate(dptx, dptx->link.lanes,
		dptx->link.rate, vparams->bpc, vparams->pix_enc, mdtd.pixel_clock);
	if (retval) {
		HISI_FB_INFO("Can't change to the preferred video mode: frequency = %d\n",
						mdtd.pixel_clock);
		HISI_FB_INFO("Changing to the default video mode\n");
		vparams->video_format = VCEA;
		retval = dptx_video_mode_change(dptx, 1);
		if (retval) {
			HISI_FB_ERR("Change mode error!\n");
			return retval;
		}
	} else {
		vparams->mdtd = mdtd;
		HISI_FB_INFO("pixel_frequency=%d.\n", mdtd.pixel_clock);

	#if 0
		/* MMCM */
		dptx_video_reset(dptx, 1);
		retval = dptx_video_pixel_freq_change(dptx, mdtd.pixel_clock);
		if (retval) {
			HISI_FB_ERR("failed to  dptx_video_pixel_freq_change, retval=%d.\n", retval);
			dptx_video_reset(dptx, 0);
			return retval;
		}
		dptx_video_reset(dptx, 0);
		dptx_video_timing_change(dptx);
	#else
		dptx_video_timing_change(dptx); //temp remove
		//dptx_enable_default_video_stream(dptx);
	#endif
	}

#ifdef CONFIG_DP_HDCP13_ENABLE
	if (hparams->hdcp13_is_en) {
		dptx_en_dis_hdcp13(dptx, 1);
		hparams->auth_fail_count = 0;
	}
#endif

	dptx_resolution_switch(hisifd);

	HISI_FB_INFO("-.\n");

	return 0;
}

#ifdef CONFIG_DP_HDCP13_ENABLE
static void handle_hdcp_intr(struct dp_ctrl *dptx)
{
	u32 hdcpintsts;
	struct hdcp_params *hparams;

	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return ;
	}

	hparams = &dptx->hparams;

	hdcpintsts = dptx_readl(dptx, DPTX_HDCP_INT_STS);
	HISI_FB_DEBUG("DPTX_HDCP_INT_STS=0x%08x.\n", hdcpintsts);

	if (hdcpintsts & DPTX_HDCP_KSV_ACCESS_INT)
		HISI_FB_INFO("KSV memory access guaranteed for read, write access\n");

	if (hdcpintsts & DPTX_HDCP_KSV_SHA1)
		HISI_FB_INFO("SHA1 verification has been done\n");

	if (hdcpintsts & DPTX_HDCP_AUX_RESP_TIMEOUT) {
		dptx_en_dis_hdcp13(dptx, 0);
		HISI_FB_INFO("DPTX_HDCP_AUX_RESP_TIMEOUT\n");
	}

	if (hdcpintsts & DPTX_HDCP_FAILED) {
		hparams->auth_fail_count++;
		if (hparams->auth_fail_count > DPTX_HDCP_MAX_AUTH_RETRY) {
			dptx_en_dis_hdcp13(dptx, 0);
			HISI_FB_INFO("Reach max allowed retries count=%d.\n", hparams->auth_fail_count);
		}
		HISI_FB_INFO("HDCP authentication process was failed!\n");
	}

	if (hdcpintsts & DPTX_HDCP_ENGAGED) {
		if (hparams->hdcp13_is_en)
			hparams->auth_fail_count = 0;
		HISI_FB_INFO("HDCP authentication process was successful.\n");
	}

	if (hdcpintsts & DPTX_HDCP22_GPIOINT)
		HISI_FB_INFO("HDCP22_GPIOINT\n");

	dptx_writel(dptx, DPTX_HDCP_INT_CLR, hdcpintsts);
}
#endif

static void handle_aux_reply(struct dp_ctrl *dptx)
{
	uint32_t auxsts;
	uint32_t status;
	uint32_t auxm;
	uint32_t br;

	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return ;
	}

	auxsts = inp32(dptx->base + DPTX_AUX_STS);

	status = (auxsts & DPTX_AUX_STS_STATUS_MASK) >> DPTX_AUX_STS_STATUS_SHIFT;
	auxm = (auxsts & DPTX_AUX_STS_AUXM_MASK) >> DPTX_AUX_STS_AUXM_SHIFT;
	br = (auxsts & DPTX_AUX_STS_BYTES_READ_MASK) >> DPTX_AUX_STS_BYTES_READ_SHIFT;

	HISI_FB_DEBUG("DPTX_AUX_STS=0x%08x: sts=%d, auxm=%d, br=%d, "
		"replyrcvd=%d, replyerr=%d, timeout=%d, disconn=%d.\n",
		auxsts, status, auxm, br,
		!!(auxsts & DPTX_AUX_STS_REPLY_RECEIVED),
		!!(auxsts & DPTX_AUX_STS_REPLY_ERR),
		!!(auxsts & DPTX_AUX_STS_TIMEOUT),
		!!(auxsts & DPTX_AUX_STS_SINK_DWA));

	switch (status) {
	case DPTX_AUX_STS_STATUS_ACK:
		HISI_FB_DEBUG("DPTX_AUX_STS_STATUS_ACK!\n");
		break;
	case DPTX_AUX_STS_STATUS_NACK:
		HISI_FB_DEBUG("DPTX_AUX_STS_STATUS_NACK!\n");
		break;
	case DPTX_AUX_STS_STATUS_DEFER:
		HISI_FB_DEBUG("DPTX_AUX_STS_STATUS_DEFER!\n");
		break;
	case DPTX_AUX_STS_STATUS_I2C_NACK:
		HISI_FB_DEBUG("DPTX_AUX_STS_STATUS_I2C_NACK!\n");
		break;
	case DPTX_AUX_STS_STATUS_I2C_DEFER:
		HISI_FB_DEBUG("DPTX_AUX_STS_STATUS_I2C_DEFER!\n");
		break;
	default:
		HISI_FB_ERR("Invalid AUX status 0x%x.\n", status);
		break;
	}

	dptx->aux.data[0] = inp32(dptx->base + DPTX_AUX_DATA0);
	dptx->aux.data[1] = inp32(dptx->base + DPTX_AUX_DATA1);
	dptx->aux.data[2] = inp32(dptx->base + DPTX_AUX_DATA2);
	dptx->aux.data[3] = inp32(dptx->base + DPTX_AUX_DATA3);
	dptx->aux.sts = auxsts;

	atomic_set(&dptx->aux.event, 1);
	dptx_notify(dptx);
}

irqreturn_t dptx_threaded_irq(int irq, void *dev)
{
	int retval = 0;
	struct hisi_fb_data_type *hisifd;
	struct dp_ctrl *dptx;
	uint32_t hpdsts;
	uint32_t bit_hpd_status;

	hisifd = dev;
	if (!hisifd) {
		HISI_FB_ERR("hisifd is NULL!\n");
		return IRQ_HANDLED;
	}

	dptx = &(hisifd->dp);

	mutex_lock(&dptx->dptx_mutex);
	if (!dptx->dptx_enable) {
		HISI_FB_WARNING("dptx has already off!\n");
		mutex_unlock(&dptx->dptx_mutex);
		return IRQ_HANDLED;
	}

	if (g_dss_version_tag == FB_ACCEL_KIRIN970) {
		bit_hpd_status = DPTX_HPDSTS_STATUS_GA;
	} else {
		bit_hpd_status = DPTX_HPDSTS_STATUS_EA03;
	}

	/*
	 * TODO this should be set after all AUX transactions that are
	 * queued are aborted. Currently we don't queue AUX and AUX is
	 * only started from this function.
	 */
	atomic_set(&dptx->aux.abort, 0);
	if (atomic_read(&dptx->c_connect)) {
		atomic_set(&dptx->c_connect, 0);

		hpdsts = inp32(dptx->base + DPTX_HPDSTS);
		HISI_FB_INFO("HPDSTS = 0x%08x.\n", hpdsts);

		if (hpdsts & bit_hpd_status) {
			retval = handle_hotplug(hisifd);
			if (retval) {
				HISI_FB_ERR("DP Device Hotplug error %d\n", retval);
			}
		} else {
			retval = handle_hotunplug(hisifd);
			if (retval) {
				HISI_FB_ERR("DP Device Hotplug error %d\n", retval);
			}
		}
	}

	if (atomic_read(&dptx->sink_request)) {
		atomic_set(&dptx->sink_request, 0);
	#if 0
		retval = handle_sink_request(dptx);
		if (retval) {
			HISI_FB_ERR("Unable to handle sink request %d\n", retval);
		}
	#endif
	}

	mutex_unlock(&dptx->dptx_mutex);

	return IRQ_HANDLED;
}

void dptx_hpd_handler(struct dp_ctrl *dptx, bool plugin, uint8_t dp_lanes)
{
	u32 reg = 0;

	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return ;
	}

	/* need to  check dp lanes */

	dptx->max_lanes = dp_lanes;

	reg = dptx_readl(dptx, DPTX_CCTL);
	if (plugin) {
		reg |= DPTX_CCTL_FORCE_HPD;
	} else {
		reg &= ~DPTX_CCTL_FORCE_HPD;
	}

	outp32(dptx->base + DPTX_CCTL, reg);
}

void dptx_hpd_irq_handler(struct dp_ctrl *dptx)
{
	if (!dptx) {
		HISI_FB_ERR("dptx is NULL!\n");
		return ;
	}

	atomic_set(&dptx->sink_request, 1);
	dptx_notify(dptx);
}

irqreturn_t dptx_irq(int irq, void *dev)
{
	irqreturn_t retval = IRQ_HANDLED;
	struct hisi_fb_data_type *hisifd;
	struct dp_ctrl *dptx = NULL;
	uint32_t ists;
	uint32_t ien;
	uint32_t hpdsts;

	hisifd = (struct hisi_fb_data_type *)dev;
	if (!hisifd) {
		HISI_FB_ERR("hisifd is NULL!\n");
		return IRQ_HANDLED;
	}

	dptx = &(hisifd->dp);

	ists = inp32(dptx->base + DPTX_ISTS);
	ien = inp32(dptx->base + DPTX_IEN);
	hpdsts = inp32(dptx->base + DPTX_HPDSTS);
	HISI_FB_DEBUG("DPTX_ISTS=0x%08x, DPTX_IEN=0x%08x, DPTX_HPDSTS=0x%08x.\n",
		ists, ien, hpdsts);

	if (!(ists & DPTX_ISTS_ALL_INTR)) {
		HISI_FB_INFO("IRQ_NONE, DPTX_ISTS=0x%08x.\n", ists);
		retval = IRQ_NONE;
		return retval;
	}

	if (ists & DPTX_ISTS_AUX_REPLY) {
		ists &= ~DPTX_ISTS_AUX_REPLY;
		handle_aux_reply(dptx);
		outp32(dptx->base + DPTX_ISTS, DPTX_ISTS_AUX_REPLY);
	}

	if (ists & DPTX_ISTS_AUX_CMD_INVALID) {
		/* TODO abort AUX */
		/* handle_aux_reply(dptx); */
		outp32(dptx->base + DPTX_ISTS, DPTX_ISTS_AUX_CMD_INVALID);
	}

	if (ists & DPTX_ISTS_HDCP) {
		/* handle_hdcp_intr(dptx); */
	}

	if (ists & DPTX_ISTS_SDP) {
		/* TODO Handle and clear */
	}

	if (ists & DPTX_ISTS_AUDIO_FIFO_OVERFLOW) {
		if (ien & DPTX_IEN_AUDIO_FIFO_OVERFLOW) {
			HISI_FB_INFO("DPTX_ISTS_AUDIO_FIFO_OVERFLOW!\n");
			outp32(dptx->base + DPTX_ISTS, DPTX_ISTS_AUDIO_FIFO_OVERFLOW);
		}
	}

	if (ists & DPTX_ISTS_VIDEO_FIFO_OVERFLOW) {
		if (ien & DPTX_IEN_VIDEO_FIFO_OVERFLOW) {
			HISI_FB_ERR("DPTX_ISTS_VIDEO_FIFO_OVERFLOW!\n");
			outp32(dptx->base + DPTX_ISTS, DPTX_ISTS_VIDEO_FIFO_OVERFLOW);
		}
	}

	if (ists & DPTX_ISTS_TYPE_C) {
		/* TODO Handle and clear */
		outp32(dptx->base + DPTX_TYPE_C_CTRL, DPTX_TYPEC_INTERRUPT_STATUS);

		HISI_FB_DEBUG("\n DPTX_TYPE_C_CTRL: [%02x] PRE", inp32(dptx->base + DPTX_TYPE_C_CTRL));
		dptx_typec_reset_ack(dptx);
		HISI_FB_DEBUG("\n DPTX_TYPE_C_CTRL: [%02x] AFT", inp32(dptx->base + DPTX_TYPE_C_CTRL));
	}

	if (ists & DPTX_ISTS_HPD) {
		if (hpdsts & DPTX_HPDSTS_IRQ) {
			outp32(dptx->base + DPTX_HPDSTS, DPTX_HPDSTS_IRQ);
			dptx_hpd_irq_handler(dptx);
			retval = IRQ_WAKE_THREAD;
		}

		if (hpdsts & DPTX_HPDSTS_HOT_PLUG) {
			outp32(dptx->base + DPTX_HPDSTS, DPTX_HPDSTS_HOT_PLUG);

			/* if (hpdsts & DPTX_HPDSTS_STATUS) { */
			if (1) {
				atomic_set(&dptx->aux.abort, 1);
				atomic_set(&dptx->c_connect, 1);
				dptx_notify(dptx);
				retval = IRQ_WAKE_THREAD;
			} else {
				HISI_FB_INFO("Hot plug - not connected\n");
			}
		}

		if (hpdsts & DPTX_HPDSTS_HOT_UNPLUG) {
			outp32(dptx->base + DPTX_HPDSTS, DPTX_HPDSTS_HOT_UNPLUG);

			/* if (!(hpdsts & DPTX_HPDSTS_STATUS)) { */
			if (1) {
				atomic_set(&dptx->aux.abort, 1);
				atomic_set(&dptx->c_connect, 1);
				dptx_notify(dptx);
				retval = IRQ_WAKE_THREAD;
			} else {
				HISI_FB_INFO("Hot unplug - not disconnected\n");
			}
		}

		if (hpdsts & 0x80) {
			HISI_FB_INFO("DPTX_HPDSTS[7] HOTPLUG DEBUG INTERRUPT!\n");
			outp32(dptx->base + DPTX_HPDSTS, 0x80 | DPTX_HPDSTS_HOT_UNPLUG);
		}
	}

	return retval;
}
/*lint -restore*/
