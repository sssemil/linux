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

#include "dp_aux.h"
#include "link.h"
#include "avgen.h"
#include "core.h"
#include "../hisi_fb_def.h"
/*lint -save -e* */
static int dptx_link_read_status(struct dp_ctrl *dptx)
{
	return dptx_read_bytes_from_dpcd(dptx, DP_LANE0_1_STATUS,
					 dptx->link.status,
					 DP_LINK_STATUS_SIZE);
}

static int dptx_link_check_cr_done(struct dp_ctrl *dptx, bool *out_done)
{
	int retval;
	uint8_t byte;
	uint32_t reg;

	HISI_FB_INFO(":\n");

	if (WARN_ON(!out_done))
		return -EINVAL;

	*out_done = false;

	retval = dptx_read_dpcd(dptx, DP_TRAINING_AUX_RD_INTERVAL, &byte);
	if (retval)
		return retval;

	reg = min_t(uint32_t, (byte & 0x7f), 4);
	reg *= 4000;
	if (!reg)
		reg = 400;

	udelay(reg);

	retval = dptx_link_read_status(dptx);
	if (retval)
		return retval;

	*out_done = drm_dp_clock_recovery_ok(dptx->link.status,
					     dptx->link.lanes);

	HISI_FB_INFO("CR_DONE = %d\n", *out_done);

	return 0;
}

static int dptx_link_check_ch_eq_done(struct dp_ctrl *dptx,
				      bool *out_cr_done,
				      bool *out_ch_eq_done)
{
	int retval;
	bool done;

	HISI_FB_INFO("\n");

	if (WARN_ON(!out_cr_done || !out_ch_eq_done))
		return -EINVAL;

	retval = dptx_link_check_cr_done(dptx, &done);
	if (retval)
		return retval;

	*out_cr_done = false;
	*out_ch_eq_done = false;

	if (!done)
		return 0;

	*out_cr_done = true;
	*out_ch_eq_done = drm_dp_channel_eq_ok(dptx->link.status,
					       dptx->link.lanes);

	HISI_FB_INFO("CH_EQ_DONE = %d\n", *out_ch_eq_done);

	return 0;
}

void dptx_link_set_preemp_vswing(struct dp_ctrl *dptx)
{
	uint32_t i;
	uint8_t pe;
	uint8_t vs;

	for (i = 0; i < dptx->link.lanes; i++) {
		pe = dptx->link.preemp_level[i];
		vs = dptx->link.vswing_level[i];

		dptx_phy_set_pre_emphasis(dptx, i, pe);
		dptx_phy_set_vswing(dptx, i, vs);
	}
}

int dptx_link_training_lanes_set(struct dp_ctrl *dptx)
{
	int retval;
	uint32_t i;
	uint8_t bytes[4] = { 0xff, 0xff, 0xff, 0xff };
	uint8_t byte = 0;

	for (i = 0; i < dptx->link.lanes; i++) {
		byte |= ((dptx->link.vswing_level[i] <<
			  DP_TRAIN_VOLTAGE_SWING_SHIFT) &
			 DP_TRAIN_VOLTAGE_SWING_MASK);

		if (dptx->link.vswing_level[i] == 3)
			byte |= DP_TRAIN_MAX_SWING_REACHED;

		byte |= ((dptx->link.preemp_level[i] <<
			  DP_TRAIN_PRE_EMPHASIS_SHIFT) &
			 DP_TRAIN_PRE_EMPHASIS_MASK);

		if (dptx->link.preemp_level[i] == 3)
			byte |= DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

		bytes[i] = byte;
	}

	retval = dptx_write_bytes_to_dpcd(dptx, DP_TRAINING_LANE0_SET, bytes,
					  dptx->link.lanes);
	if (retval)
		return retval;

	return 0;
}

int dptx_link_adjust_drive_settings(struct dp_ctrl *dptx, int *out_changed)
{
	int retval;
	uint32_t lanes;
	uint32_t i;
	uint8_t byte;
	uint8_t adj[4] = { 0 };
	int changed = false;

	lanes = dptx->link.lanes;

	switch (lanes) {
	case 4:
		retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE2_3, &byte);
		if (retval)
			return retval;

		adj[2] = byte & 0x0f;
		adj[3] = (byte & 0xf0) >> 4;
	case 2:
	case 1:
		retval = dptx_read_dpcd(dptx, DP_ADJUST_REQUEST_LANE0_1, &byte);
		if (retval)
			return retval;

		adj[0] = byte & 0x0f;
		adj[1] = (byte & 0xf0) >> 4;
		break;
	default:
		WARN(1, "Invalid number of lanes %d\n", lanes);
		return -EINVAL;
	}

	/* Save the drive settings */
	for (i = 0; i < lanes; i++) {
		uint8_t vs = adj[i] & 0x3;
		uint8_t pe = (adj[i] & 0xc) >> 2;

		if (dptx->link.vswing_level[i] != vs)
			changed = true;

		dptx->link.vswing_level[i] = vs;
		dptx->link.preemp_level[i] = pe;
	}

	dptx_link_set_preemp_vswing(dptx);

	retval = dptx_link_training_lanes_set(dptx);
	if (retval)
		return retval;

	if (out_changed)
		*out_changed = changed;

	return 0;
}

static int dptx_link_training_init(struct dp_ctrl *dptx,
				   uint8_t rate,
				   uint8_t lanes)
{
	uint8_t sink_max_rate;
	uint8_t sink_max_lanes;

	HISI_FB_INFO("lanes=%d, rate=%d\n", lanes, rate);

	if (WARN(rate > DPTX_PHYIF_CTRL_RATE_HBR3,
		 "Invalid rate %d\n", rate))
		rate = DPTX_PHYIF_CTRL_RATE_RBR;

	if (WARN(!lanes || (lanes == 3) || (lanes > 4),
		 "Invalid lanes %d\n", lanes))
		lanes = 1;

	/* Initialize link parameters */
	memset(dptx->link.preemp_level, 0, sizeof(uint8_t) * 4);
	memset(dptx->link.vswing_level, 0, sizeof(uint8_t) * 4);
	memset(dptx->link.status, 0, DP_LINK_STATUS_SIZE);

	sink_max_lanes = drm_dp_max_lane_count(dptx->rx_caps);

	if (lanes > sink_max_lanes)
		lanes = sink_max_lanes;

	sink_max_rate = dptx->rx_caps[DP_MAX_LINK_RATE];
	sink_max_rate = dptx_bw_to_phy_rate(sink_max_rate);

	if (rate > sink_max_rate)
		rate = sink_max_rate;

	dptx->link.lanes = lanes;
	dptx->link.rate = rate;
	dptx->link.trained = false;

	return 0;
}

int dptx_link_training_pattern_set(struct dp_ctrl *dptx, uint8_t pattern)
{
	int retval;

	retval = dptx_write_dpcd(dptx, DP_TRAINING_PATTERN_SET, pattern);
	if (retval)
		return retval;

	return 0;
}

static int dptx_link_training_start(struct dp_ctrl *dptx)
{
	int retval;
	uint32_t cctl;
	uint8_t byte;

	dptx_phy_set_lanes_status(dptx, false);

#if UDP_ENABLE
	/* Wait for PHY busy */
	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		HISI_FB_ERR("Timed out 1 waiting for PHY BUSY\n");
		return retval;
	}
#endif

	/* Initialize PHY */
	dptx_phy_set_lanes(dptx, dptx->link.lanes);
	dptx_phy_set_rate(dptx, dptx->link.rate);

	/* Set SSC_DIS = 1? */
#if  UDP_ENABLE
	/* Wait for PHY busy */
	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		HISI_FB_ERR("Timed out waiting for PHY BUSY\n");
		return retval;
	}
#endif

	dptx_phy_set_lanes_status(dptx, true);

	/* Set SSC_DIS = 1? */
#if UDP_ENABLE
	/* Wait for PHY busy */
	retval = dptx_phy_wait_busy(dptx, dptx->link.lanes);
	if (retval) {
		HISI_FB_ERR("Timed out 3 waiting for PHY BUSY\n");
		return retval;
	}
#endif

	/* Set PHY_TX_EQ */
	dptx_link_set_preemp_vswing(dptx);

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);
	retval = dptx_link_training_pattern_set(dptx,
						DP_TRAINING_PATTERN_DISABLE);
	if (retval)
		return retval;

	dptx_phy_enable_xmit(dptx, dptx->link.lanes, true);

	retval = dptx_phy_rate_to_bw(dptx->link.rate);
	if (retval < 0)
		return retval;

	byte = (uint8_t)retval;
	retval = dptx_write_dpcd(dptx, DP_LINK_BW_SET, byte);
	if (retval)
		return retval;

	byte = dptx->link.lanes;
	cctl = inp32(dptx->base + DPTX_CCTL);

	if (drm_dp_enhanced_frame_cap(dptx->rx_caps)) {
		byte |= DP_ENHANCED_FRAME_CAP;
		cctl |= DPTX_CCTL_ENH_FRAME_EN;
	} else {
		cctl &= ~DPTX_CCTL_ENH_FRAME_EN;
	}

	outp32(dptx->base + DPTX_CCTL, cctl);

	retval = dptx_write_dpcd(dptx, DP_LANE_COUNT_SET, byte);
	if (retval)
		return retval;

	byte = DP_SPREAD_AMP_0_5;
	retval = dptx_write_dpcd(dptx, DP_DOWNSPREAD_CTRL, byte);
	if (retval)
		return retval;

	byte = 1;
	retval = dptx_write_dpcd(dptx, DP_MAIN_LINK_CHANNEL_CODING_SET, byte);
	if (retval)
		return retval;

	return 0;
}

int dptx_link_wait_cr_and_adjust(struct dp_ctrl *dptx, bool ch_eq)
{
	int i;
	int retval;
	int changed = 0;
	bool done = false;

	HISI_FB_INFO(":\n");

	retval = dptx_link_check_cr_done(dptx, &done);
	if (retval)
		return retval;

	if (done)
		return 0;

	/* Try each adjustment setting 5 times */
	for (i = 0; i < 5; i++) {
		retval = dptx_link_adjust_drive_settings(dptx, &changed);
		if (retval)
			return retval;

		/* Reset iteration count if we changed settings */
		if (changed)
			i = 0;

		retval = dptx_link_check_cr_done(dptx, &done);
		if (retval)
			return retval;

		if (done)
			return 0;

		/* TODO check for all lanes? */
		/* Failed and reached the maximum voltage swing */
		if (dptx->link.vswing_level[0] == 3)
			return -EPROTO;
	}

	return -EPROTO;
}

int dptx_link_cr(struct dp_ctrl *dptx)
{
	int retval;

	HISI_FB_INFO(":\n");

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_1);

	retval = dptx_link_training_pattern_set(dptx,
						DP_TRAINING_PATTERN_1 | 0x20);

	if (retval)
		return retval;

	return dptx_link_wait_cr_and_adjust(dptx, false);
}

int dptx_link_ch_eq(struct dp_ctrl *dptx)
{
	int retval;
	bool cr_done;
	bool ch_eq_done;
	uint32_t pattern;
	uint32_t i;
	uint8_t dp_pattern;

	HISI_FB_INFO(":\n");
	//switch (dptx->max_rate) {   // temp remove
	switch (dptx->link.rate) {
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		if (drm_dp_tps4_supported(dptx->rx_caps)) {
			pattern = DPTX_PHYIF_CTRL_TPS_4;
			dp_pattern = DP_TRAINING_PATTERN_4;
			break;
		}
		/* Fall through */
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		if (drm_dp_tps3_supported(dptx->rx_caps)) {
			pattern = DPTX_PHYIF_CTRL_TPS_3;
			dp_pattern = DP_TRAINING_PATTERN_3;
			break;
		}
		/* Fall through */
	case DPTX_PHYIF_CTRL_RATE_RBR:
	case DPTX_PHYIF_CTRL_RATE_HBR:
		pattern = DPTX_PHYIF_CTRL_TPS_2;
		dp_pattern = DP_TRAINING_PATTERN_2;
		break;
	default:
		WARN(1, "Invalid rate %d\n", dptx->link.rate);
		return -EINVAL;
	}

	dptx_phy_set_pattern(dptx, pattern);

	/* TODO this needs to be different for other versions of
	 * DPRX
	 */
	retval = dptx_link_training_pattern_set(dptx,
						dp_pattern | 0x20);
	if (retval)
		return retval;

	for (i = 0; i < 6; i++) {
		retval = dptx_link_check_ch_eq_done(dptx,
						    &cr_done,
						    &ch_eq_done);

		if (retval)
			return retval;

		if (!cr_done)
			return -EPROTO;

		if (ch_eq_done)
			return 0;

		retval = dptx_link_adjust_drive_settings(dptx, NULL);

		if (retval)
			return retval;
	}

	return -EPROTO;
}

int dptx_link_reduce_rate(struct dp_ctrl *dptx)
{
	uint32_t rate = dptx->link.rate;

	switch (rate) {
	case DPTX_PHYIF_CTRL_RATE_RBR:
		return -EPROTO;
	case DPTX_PHYIF_CTRL_RATE_HBR:
		rate = DPTX_PHYIF_CTRL_RATE_RBR;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR2:
		rate = DPTX_PHYIF_CTRL_RATE_HBR;
		break;
	case DPTX_PHYIF_CTRL_RATE_HBR3:
		rate = DPTX_PHYIF_CTRL_RATE_HBR2;
		break;
	}

	HISI_FB_INFO("Reducing rate from %d to %d\n",
					dptx->link.rate, rate);
	dptx->link.rate = rate;
	return 0;
}

int dptx_link_reduce_lanes(struct dp_ctrl *dptx)
{
	uint32_t lanes;

	switch (dptx->link.lanes) {
	case 4:
		lanes = 2;
		break;
	case 2:
		lanes = 1;
		break;
	case 1:
	default:
		return -EPROTO;
	}

	HISI_FB_INFO("Reducing lanes from %d to %d\n",
					dptx->link.lanes, lanes);
	dptx->link.lanes = lanes;
	return 0;
}

int dptx_link_training(struct dp_ctrl *dptx, uint8_t rate, uint8_t lanes)
{
	int retval;
	uint8_t byte;

	retval = dptx_link_training_init(dptx, rate, lanes);
	if (retval)
		goto fail;

again:
	HISI_FB_INFO("Starting link training\n");
	retval = dptx_link_training_start(dptx);
	if (retval)
		goto fail;

	retval = dptx_link_cr(dptx);
	if (retval) {
		if (retval == -EPROTO) {
			if (dptx_link_reduce_rate(dptx)) {
				/* TODO If CR_DONE bits for some lanes
				 * are set, we should reduce lanes to
				 * those lanes.
				 */
				if (dptx_link_reduce_lanes(dptx)) {
					retval = -EPROTO;
					goto fail;
				}
			}

			dptx_link_training_init(dptx,
						dptx->link.rate,
						dptx->link.lanes);
			goto again;
		} else {
			goto fail;
		}
	}

	retval = dptx_link_ch_eq(dptx);
	if (retval) {
		if (retval == -EPROTO) {
			if (dptx_link_reduce_rate(dptx)) {
				if (dptx_link_reduce_lanes(dptx)) {
					retval = -EPROTO;
					goto fail;
				}
			}

			dptx_link_training_init(dptx,
						dptx->link.rate,
						dptx->link.lanes);
			goto again;
		} else {
			goto fail;
		}
	}

	dptx_phy_set_pattern(dptx, DPTX_PHYIF_CTRL_TPS_NONE);

	retval = dptx_link_training_pattern_set(dptx,
						DP_TRAINING_PATTERN_DISABLE);
	if (retval)
		goto fail;

	//dptx_enable_default_video_stream(dptx);
	dptx_phy_enable_xmit(dptx, dptx->link.lanes, true);
	dptx->link.trained = true;

	/* Branch device detection */
	retval = dptx_read_dpcd(dptx, DP_SINK_COUNT, &byte);
	if (retval)
		return retval;
	dptx_video_ts_change(dptx);
	HISI_FB_INFO("Link training succeeded rate=%d lanes=%d\n",
		 dptx->link.rate, dptx->link.lanes);

	return 0;

fail:
	HISI_FB_ERR("Link training failed %d\n", retval);
	return retval;
}

int dptx_link_check_status(struct dp_ctrl *dptx)
{
	int retval;
	uint8_t byte;
	uint8_t bytes[2];

	retval = dptx_read_bytes_from_dpcd(dptx, DP_SINK_COUNT,
					   bytes, 2);
	if (retval)
		return retval;

	retval = dptx_link_read_status(dptx);
	if (retval)
		return retval;

	byte = dptx->link.status[DP_LANE_ALIGN_STATUS_UPDATED -
				 DP_LANE0_1_STATUS];

	if (!(byte & DP_LINK_STATUS_UPDATED))
		return 0;

	/* Check if need to retrain link */
	if (dptx->link.trained &&
	    (!drm_dp_channel_eq_ok(dptx->link.status, dptx->link.lanes) ||
	     !drm_dp_clock_recovery_ok(dptx->link.status, dptx->link.lanes))) {
		HISI_FB_INFO("Retraining link\n");

		return dptx_link_training(dptx,
					  DPTX_MAX_LINK_RATE,
					  DPTX_MAX_LINK_LANES);
	}

	return 0;
}
/*lint -restore*/