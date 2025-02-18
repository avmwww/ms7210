/*
 * MacroSilicon MS7210 HDMI transmitter driver
 *
 * Copyright 2023 Andrey Mitrofanov avmwww@gmail.com
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/hdmi.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#include <drm/drmP.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_edid.h>

#include <media/cec.h>

#include "ms7210.h"

enum ms7210_type {
	MS7210 = 0,
};

enum DVIN_CS_MODE {
	DVIN_CS_MODE_RGB,
	DVIN_CS_MODE_YUV444,
	DVIN_CS_MODE_YUV422
};

enum DVIN_BW_MODE {
	DVIN_BW_MODE_16_20_24BIT,
	DVIN_BW_MODE_8_10_12BIT
};

enum DVIN_SQ_MODE {
	DVIN_SQ_MODE_NONSEQ,
	DVIN_SQ_MODE_SEQ
};

enum DVIN_DR_MODE {
	DVIN_DR_MODE_SDR,
	DVIN_DR_MODE_DDR
};

enum DVIN_SY_MODE {
	DVIN_SY_MODE_HSVSDE,      // 8/16/24-bit BT601
	DVIN_SY_MODE_HSVS,
	DVIN_SY_MODE_VSDE,        // non suport interlace mode
	DVIN_SY_MODE_DEONLY,
	DVIN_SY_MODE_EMBEDDED,    // 16-bit BT1120 or 8bit BT656
	DVIN_SY_MODE_2XEMBEDDED,  // 8-bit BT1120
	DVIN_SY_MODE_BTAT1004     // 16-bit BTA-T1004
};

/* HDMI audio */
enum HDMI_AUDIO_MODE {
	HDMI_AUD_MODE_AUDIO_SAMPLE  = 0x00,
	HDMI_AUD_MODE_HBR           = 0x01,
	HDMI_AUD_MODE_DSD           = 0x02,
	HDMI_AUD_MODE_DST           = 0x03
};

enum HDMI_AUDIO_RATE {
	HDMI_AUD_RATE_44K1  = 0x00,
	HDMI_AUD_RATE_48K   = 0x02,
	HDMI_AUD_RATE_32K   = 0x03,
	HDMI_AUD_RATE_88K2  = 0x08,
	HDMI_AUD_RATE_96K   = 0x0A,
	HDMI_AUD_RATE_176K4 = 0x0C,
	HDMI_AUD_RATE_192K  = 0x0E
};

enum HDMI_AUDIO_LENGTH {
	HDMI_AUD_LENGTH_16BITS    = 0x00,
	HDMI_AUD_LENGTH_20BITS    = 0x01,
	HDMI_AUD_LENGTH_24BITS    = 0x02
};

enum HDMI_AUDIO_CHN {
	HDMI_AUD_2CH    = 0x01,
	HDMI_AUD_3CH    = 0x02,
	HDMI_AUD_4CH    = 0x03,
	HDMI_AUD_5CH    = 0x04,
	HDMI_AUD_6CH    = 0x05,
	HDMI_AUD_7CH    = 0x06,
	HDMI_AUD_8CH    = 0x07
};

enum HDMI_CLK_REPEAT {
	HDMI_X1CLK      = 0x00,
	HDMI_X2CLK      = 0x01,
	HDMI_X3CLK      = 0x02,
	HDMI_X4CLK      = 0x03,
	HDMI_X5CLK      = 0x04,
	HDMI_X6CLK      = 0x05,
	HDMI_X7CLK      = 0x06,
	HDMI_X8CLK      = 0x07,
	HDMI_X9CLK      = 0x08,
	HDMI_X10CLK     = 0x09
};

enum HDMI_SCAN_INFO {
	HDMI_OVERSCAN     = 0x01,    //television type
	HDMI_UNDERSCAN    = 0x02     //computer type
};

enum HDMI_ASPECT_RATIO {
	HDMI_4X3     = 0x01,
	HDMI_16X9    = 0x02
};

enum HDMI_CS {
	HDMI_RGB        = 0x00,
	HDMI_YCBCR422   = 0x01,
	HDMI_YCBCR444   = 0x02,
	HDMI_YUV420     = 0x03
};

enum HDMI_COLOR_DEPTH {
	HDMI_COLOR_DEPTH_8BIT    = 0x00,
	HDMI_COLOR_DEPTH_10BIT   = 0x01,
	HDMI_COLOR_DEPTH_12BIT   = 0x02,
	HDMI_COLOR_DEPTH_16BIT   = 0x03
};

enum HDMI_COLORIMETRY {
	HDMI_COLORIMETRY_601    = 0x00,
	HDMI_COLORIMETRY_709    = 0x01,
	HDMI_COLORIMETRY_656    = 0x02,
	HDMI_COLORIMETRY_1120   = 0x03,
	HDMI_COLORIMETRY_SMPTE  = 0x04,
	HDMI_COLORIMETRY_XVYCC601 = 0x05,
	HDMI_COLORIMETRY_XVYCC709 = 0x06
};

enum HDMI_VIDEO_FORMAT {
	HDMI_NO_ADD_FORMAT,
	HDMI_4Kx2K_FORMAT,
	HDMI_3D_FORMAT
};

enum HDMI_4Kx2K_VIC {
	HDMI_4Kx2K_30HZ = 0x01,
	HDMI_4Kx2K_25HZ,
	HDMI_4Kx2K_24HZ,
	HDMI_4Kx2K_24HZ_SMPTE
};

enum HDMI_3D_STRUCTURE {
	HDMI_FRAME_PACKING,
	HDMI_FIELD_ALTERNATIVE,
	HDMI_LINE_ALTERNATIVE,
	HDMI_SIDE_BY_SIDE_FULL,
	L_DEPTH,
	L_DEPTH_GRAPHICS,
	SIDE_BY_SIDE_HALF = 8
};


struct dvin_config {
	enum DVIN_CS_MODE cs_mode;
	enum DVIN_BW_MODE bw_mode;
	enum DVIN_SQ_MODE sq_mode;
	enum DVIN_DR_MODE dr_mode;
	enum DVIN_SY_MODE sy_mode;
};

struct videotiming {
	u8  polarity;
	u16 htotal;
	u16 vtotal;
	u16 hactive;
	u16 vactive;
	u16 pixclk;     /*10000hz*/
	u16 vfreq;      /*0.01hz*/
	u16 hoffset;    /* h sync start to h active*/
	u16 voffset;    /* v sync start to v active*/
	u16 hsyncwidth;
	u16 vsyncwidth;
};

struct hdmi_config {
	u8 hdmi_flag;				// 0 = dvi out;  1 = hdmi out
	u8 vic;				// reference to CEA-861 VIC
	u16 video_clk;			// TMDS video clk, uint 10000Hz
	enum HDMI_CLK_REPEAT clk_rpt;		// X2CLK = 480i/576i, others = X1CLK
	enum HDMI_SCAN_INFO scan_info;
	enum HDMI_ASPECT_RATIO aspect_ratio;
	enum HDMI_CS color_space;
	enum HDMI_COLOR_DEPTH color_depth;
	enum HDMI_COLORIMETRY colorimetry;	// IT601 = 480i/576i/480p/576p, ohters = IT709
	enum HDMI_VIDEO_FORMAT video_format;
	enum HDMI_4Kx2K_VIC h4Kx2K_vic;
	enum HDMI_3D_STRUCTURE h3D_structure;
	enum HDMI_AUDIO_MODE audio_mode;
	enum HDMI_AUDIO_RATE audio_rate;
	enum HDMI_AUDIO_LENGTH audio_bits;
	enum HDMI_AUDIO_CHN audio_channels;
	u8 audio_speaker_locations;		// 0~255, refer to CEA-861 audio infoframe, BYTE4
};

struct ms7210 {
	struct i2c_client *i2c_main;
	struct i2c_client *i2c_edid;

	struct regmap *regmap;
	struct clk *pixclk;

	enum drm_connector_status status;
	bool powered;

	struct drm_display_mode curr_mode;

	unsigned int f_tmds;
	unsigned int f_audio;
	unsigned int audio_source;

	unsigned int current_edid_segment;
	u8 edid_buf[256];
	bool edid_read;

	bool always_on;

	wait_queue_head_t wq;
	struct work_struct hpd_work;

	struct drm_bridge bridge;
	struct drm_connector connector;

	enum DVIN_CS_MODE cs_mode;
	enum DVIN_SQ_MODE sq_mode;
	enum DVIN_BW_MODE bw_mode;
	enum DVIN_DR_MODE dr_mode;
	enum DVIN_SY_MODE sy_mode;

	enum ms7210_type type;

	int spdif;
	unsigned int hdmi_tx_chan;

	struct cec_adapter *cec_adap;
	bool rgb;



#if 0

	bool embedded_sync;
	enum adv7511_sync_polarity vsync_polarity;
	enum adv7511_sync_polarity hsync_polarity;

	struct gpio_desc *gpio_pd;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	/* ADV7533 DSI RX related params */
	struct device_node *host_node;
	struct mipi_dsi_device *dsi;
	u8 num_dsi_lanes;
	bool use_timing_gen;

	struct platform_device *audio_pdev;
#endif
};

#if 0
/**
 * struct ms7210_link_config - Describes ms7210 hardware configuration
 * @input_color_depth:		Number of bits per color component (8, 10 or 12)
 * @input_colorspace:		The input colorspace (RGB, YUV444, YUV422)
 * @input_clock:		The input video clock style (1x, 2x, DDR)
 * @input_style:		The input component arrangement variant
 * @input_justification:	Video input format bit justification
 * @clock_delay:		Clock delay for the input clock (in ps)
 * @embedded_sync:		Video input uses BT.656-style embedded sync
 * @sync_pulse:			Select the sync pulse
 * @vsync_polarity:		vsync input signal configuration
 * @hsync_polarity:		hsync input signal configuration
 */
struct ms7210_link_config {
	unsigned int input_color_depth;
	enum hdmi_colorspace input_colorspace;
	enum ms7210_input_clock input_clock;
	unsigned int input_style;
	enum ms7210_input_justification input_justification;

	int clock_delay;

	bool embedded_sync;
	enum ms7210_input_sync_pulse sync_pulse;
	enum ms7210_sync_polarity vsync_polarity;
	enum ms7210_sync_polarity hsync_polarity;
};
#endif

static const struct regmap_config ms7210_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,

	.cache_type = REGCACHE_NONE,
};

static inline unsigned int ms7210_swap_reg(unsigned int reg)
{
	return (reg >> 8) | (reg << 8);
}

static int ms7210_write(struct ms7210 *ms, unsigned int reg, unsigned int val)
{
	reg = ms7210_swap_reg(reg);
	return regmap_write(ms->regmap, reg, val);
}

static int ms7210_read(struct ms7210 *ms, unsigned int reg,
		       unsigned int *val)
{
	reg = ms7210_swap_reg(reg);
	return regmap_read(ms->regmap, reg, val);
}

static int ms7210_update_bits(struct ms7210 *ms, unsigned int reg,
			      unsigned int mask, unsigned int val)
{
	reg = ms7210_swap_reg(reg);
	return regmap_update_bits(ms->regmap, reg, mask, val);
}

static int ms7210_write16(struct ms7210 *ms, unsigned int reg, unsigned int val)
{
	int err;

	if ((err = ms7210_write(ms, reg, val)) < 0)
		return err;

	return ms7210_write(ms, reg + 1, val >> 8);
}

static int ms7210_tx_chan_write(struct ms7210 *ms, unsigned int chan,
				 unsigned int reg, unsigned int val)
{
	return ms7210_write(ms, reg +
			    chan * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
			    val);
}

static int ms7210_tx_chan_update_bits(struct ms7210 *ms, unsigned int chan,
				      unsigned int reg,
				      unsigned int mask, unsigned int val)
{
	return ms7210_update_bits(ms, reg +
				  chan * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
				  mask, val);
}

static int ms7210_tx_chan_read(struct ms7210 *ms, unsigned int chan,
			       unsigned int reg,
			       unsigned int *val)
{
	return ms7210_read(ms, reg +
			   chan * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
			   val);
}

/*
 *
 */
static void ms7210_hdmi_tx_phy_output_enable(struct ms7210 *ms, bool enable)
{
	if (enable) {
		/* output data drive control */
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
					   MS7210_HDMI_TX_PHY_DATA_DRV_REG,
					   0x07, 0x07);
		/* output clk drive control */
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
					   MS7210_HDMI_TX_PHY_POWER_REG,
					   0x04, 0x04);
	} else {
		/* output clk drive control */
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
					   MS7210_HDMI_TX_PHY_POWER_REG,
					   0x04, 0);
		/* output data drive control */
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
					   MS7210_HDMI_TX_PHY_DATA_DRV_REG,
					   0x07, 0);
	}
}

static void ms7210_hdmi_tx_phy_power_enable(struct ms7210 *ms,
					    bool enable)
{
	/* PLL power control */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PLL_POWER_REG, 0x07,
			   (enable) ? 0 : 0x07);

	/* PHY power control */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POWER_REG, 0x0a,
			   (enable) ? 0x0a : 0);
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POWER_REG, 0x20,
				   (enable) ? 0 : 0x20);
}

/*
 * RC25M = 27M
 */
static void ms7210_rc_freq_set(struct ms7210 *ms)
{
	ms7210_write(ms, MS7210_RC_CTRL1_REG, 0x81);
	ms7210_write(ms, MS7210_RC_CTRL1_REG, 0x54);
}

static void ms7210_csc_config_input(struct ms7210 *ms)
{
	ms7210_update_bits(ms, MS7210_CSC_CTRL1_REG, 0x03, 0x10 | ms->cs_mode);
}

static int ms7210drv_dvin_mode_config(struct ms7210 *ms)
{
	unsigned int clk_ratio = 0;

	switch (ms->cs_mode) {
		case DVIN_CS_MODE_RGB:
		case DVIN_CS_MODE_YUV444:
			ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x07, 0x00);
			ms->sq_mode = DVIN_SQ_MODE_SEQ;
			break;
		case DVIN_CS_MODE_YUV422:
			switch (ms->bw_mode) {
				case DVIN_BW_MODE_16_20_24BIT:
#if 0
					/* 16 bits */
					ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x07, 0x01);
					/* 20 bits */
					ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x07, 0x02);
#endif
					/* 24 bits */
					ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x07, 0x03);
					break;
				case DVIN_BW_MODE_8_10_12BIT:
#if 0
					/* 8 bits */
					ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x07, 0x04);
					/* 10 bits */
					ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x07, 0x05);
#endif
					/* 12 bits */
					ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x07, 0x06);
					break;
			}
			break;
	}

	/* dvin_data_sel */
	ms7210_update_bits(ms, MS7210_DVIN_SWAP_REG, 0x10, (ms->sq_mode) ? 0x00 : 0x10);

	/* dvin_ddr_mode */
	switch (ms->dr_mode) {
		case DVIN_DR_MODE_SDR:
			/* sdr mode */
			ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x08, 0x00);
			/* clk div2 */
			clk_ratio = (ms->bw_mode == DVIN_BW_MODE_8_10_12BIT) ? 2 : 0;
			break;
		case DVIN_DR_MODE_DDR:
			if ((ms->cs_mode != DVIN_CS_MODE_YUV422) &&
			    (ms->bw_mode == DVIN_BW_MODE_8_10_12BIT)) {
				/* dvin_12b444 */
				ms7210_update_bits(ms, MS7210_DVIN_12B444_REG, 0x01, 0x01);
			} else {
				/* ddr mode */
				ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x08, 0x08);
				if ((ms->cs_mode == DVIN_CS_MODE_YUV422) &&
				    (ms->bw_mode == DVIN_BW_MODE_16_20_24BIT)) {
					/* dvin_ddr_edge_inv */
					ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0x10, 0x10);
				}
			}
			/* clk x2 */
			clk_ratio = (ms->bw_mode == DVIN_BW_MODE_8_10_12BIT) ? 0 : 1;
			break;
	}
	ms7210_update_bits(ms, MS7210_MISC_DIG_CLK_SEL_REG, 0x03, clk_ratio);

	/* dvin_sync_fmt
	 * dvin_ccir_cut_en, dvin_bt1120_mode, dvin_bt656_mode
	 */
	switch (ms->sy_mode) {
		case DVIN_SY_MODE_HSVSDE:
			ms7210_update_bits(ms, MS7210_DVIN_SRC_SEL_REG, 0x0c, 0x04);
			ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0xe0, 0x00);
			break;
		case DVIN_SY_MODE_HSVS:
			ms7210_update_bits(ms, MS7210_DVIN_SRC_SEL_REG, 0x0c, 0x00);
			ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0xe0, 0x00);
			break;
		case DVIN_SY_MODE_VSDE:
		case DVIN_SY_MODE_DEONLY:
			ms7210_update_bits(ms, MS7210_DVIN_SRC_SEL_REG, 0x0c, 0x08);
			ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0xe0, 0x00);
			break;
		case DVIN_SY_MODE_EMBEDDED:
		case DVIN_SY_MODE_BTAT1004:
			ms7210_update_bits(ms, MS7210_DVIN_SRC_SEL_REG, 0x0c, 0x0c);
			ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0xe0, 0xa0);
			break;
		case DVIN_SY_MODE_2XEMBEDDED:
			ms7210_update_bits(ms, MS7210_DVIN_SRC_SEL_REG, 0x0c, 0x0c);
			ms7210_update_bits(ms, MS7210_DVIN_MODE_REG, 0xe0,
					((ms->bw_mode == DVIN_BW_MODE_8_10_12BIT) &&
					 (ms->dr_mode == DVIN_DR_MODE_DDR) &&
					 (ms->sy_mode == DVIN_SY_MODE_2XEMBEDDED)) ? 0xa0 : 0xe0);
			break;
	}
	/* r_de_only_mode */
	ms7210_update_bits(ms, MS7210_DVIN_DEONLY_REG, 0x01,
			   (ms->sy_mode == DVIN_SY_MODE_DEONLY) ? 0x01 : 0x00);
	/* dvin_bt_1004_mode */
	ms7210_update_bits(ms, MS7210_DVIN_BT1004_REG, 0x01,
			   ((ms->sy_mode == DVIN_SY_MODE_BTAT1004) ||
			    ((ms->cs_mode == DVIN_CS_MODE_YUV422) &&
			     (ms->bw_mode == DVIN_BW_MODE_8_10_12BIT) &&
			     (ms->dr_mode == DVIN_DR_MODE_DDR) &&
			     (ms->sy_mode == DVIN_SY_MODE_EMBEDDED))) ? 0x01 : 0x00);

	/* dvin_deregen_en */
	ms7210_update_bits(ms, MS7210_DVIN_SYNCOUT_FLIP_REG, 0x01,
			   (ms->sy_mode == DVIN_SY_MODE_HSVSDE) ? 0x00 : 0x01);

	return (clk_ratio == 1);
}

static int ms7210_dvin_timing_config(struct ms7210 *ms,
		struct dvin_config *dc, struct videotiming *vt,
		enum HDMI_CLK_REPEAT *rpt)
{
	unsigned int val = 0, pix_shift = 3, line_shift = 2;
	int clkx2 = 0;
	struct videotiming svt;

	ms7210_read(ms, MS7210_MISC_DIG_CLK_SEL_REG, &val);
	if (val & 1) {
		clkx2 = 1;
		vt->htotal /= *rpt;
		vt->hsyncwidth /= *rpt;
		vt->hoffset /= *rpt;
		vt->hactive /= *rpt;
		vt->pixclk /= *rpt;
		*rpt = 0;
	}

	svt = *vt;

	if (*rpt) {
		clkx2 = 1;
		svt.htotal /= *rpt;
		svt.hsyncwidth /= *rpt;
		svt.hoffset /= *rpt;
		svt.hactive /= *rpt;
		svt.pixclk /= *rpt;
		*rpt = 1;
	}

	if ((dc->bw_mode == DVIN_BW_MODE_8_10_12BIT) &&
	    (dc->dr_mode == DVIN_DR_MODE_SDR)) {
		svt.htotal *= 2;
		svt.hsyncwidth *= 2;
		svt.hoffset *= 2;
		svt.hactive *= 2;
	}

	switch (dc->sy_mode) {
		case DVIN_SY_MODE_HSVS:
			break;
		case DVIN_SY_MODE_HSVSDE:
		case DVIN_SY_MODE_VSDE:
			pix_shift += svt.hoffset;
			break;
		case DVIN_SY_MODE_DEONLY:
			pix_shift += svt.hoffset;
			line_shift += svt.voffset;
			break;
		case DVIN_SY_MODE_EMBEDDED:
			pix_shift += svt.hoffset + svt.hactive;
			line_shift += svt.voffset + svt.vactive / ((svt.polarity & 0x01) ? 1 : 2);
			break;
		case DVIN_SY_MODE_2XEMBEDDED:
			pix_shift += svt.hoffset + svt.hactive + 4;
			line_shift += svt.voffset + svt.vactive / ((svt.polarity & 0x01) ? 1 : 2);
			break;
		case DVIN_SY_MODE_BTAT1004:
			pix_shift += svt.hoffset + svt.hactive - 2;
			line_shift += svt.voffset + svt.vactive / ((svt.polarity & 0x01) ? 1 : 2);
			break;
	}

	ms7210_write16(ms, MS7210_DVIN_HT_L_REG, svt.htotal);
	ms7210_write16(ms, MS7210_DVIN_VT_L_REG, svt.vtotal);
	ms7210_write16(ms, MS7210_DVIN_PXL_SHIFT_L_REG, pix_shift);
	ms7210_write16(ms, MS7210_DVIN_LN_SHIFT_L_REG, line_shift);
	ms7210_write16(ms, MS7210_DVIN_HSW_L_REG, svt.hsyncwidth);
	ms7210_write16(ms, MS7210_DVIN_VSW_L_REG, svt.vsyncwidth);
	ms7210_write16(ms, MS7210_DVIN_HDE_ST_L_REG, svt.hoffset);
	ms7210_write16(ms, MS7210_DVIN_HDE_SP_L_REG, svt.hoffset + svt.hactive);
	ms7210_write16(ms, MS7210_DVIN_VDE_OST_L_REG, svt.voffset);
	ms7210_write16(ms, MS7210_DVIN_VDE_OSP_L_REG,
		       svt.voffset + svt.vactive /
		       ((svt.polarity & 0x01) ? 1 : 2));
	ms7210_write16(ms, MS7210_DVIN_VDE_EST_L_REG,
			svt.voffset + svt.vtotal / 2 + 1);
	ms7210_write16(ms, MS7210_DVIN_VDE_ESP_L_REG,
			svt.voffset + svt.vtotal / 2 + svt.vactive / 2 + 1);

	ms7210_update_bits(ms, MS7210_DVIN_SYNCIN_FLIP_REG,
			0x01, (svt.polarity & 0x02) ? 0x00 : 0x01); /* hsync */
	ms7210_update_bits(ms, MS7210_DVIN_SYNCIN_FLIP_REG,
			0x02, (svt.polarity & 0x04) ? 0x00 : 0x02); /* vsync */
	ms7210_update_bits(ms, MS7210_DVIN_SYNCOUT_FLIP_REG,
			0x10, (svt.polarity & 0x02) ? 0x00 : 0x10); /* hsync */
	ms7210_update_bits(ms, MS7210_DVIN_SYNCOUT_FLIP_REG,
			0x20, (svt.polarity & 0x04) ? 0x00 : 0x20); /* vsync */
	ms7210_update_bits(ms, MS7210_DVIN_SYNCOUT_FLIP_REG,
			0x04, (svt.polarity & 0x01) ? 0x04 : 0x00); /* de */

	return clkx2;
}

static void ms7210_hdmi_tx_phy_set_clk_ratio(struct ms7210 *ms, int ratio)
{
	unsigned int v = 0;

	switch (ratio) {
		case 0:
			v = 0;
			break;
		case 1:   /* x2 clk */
			v = 0x04;
			break;
		case 2:   /* 1/2 clk */
			v = 0x40;
			break;
	}
	ms7210_update_bits(ms, MS7210_HDMI_TX_PLL_CTRL11_REG, 0x44, v);
}

/*
 * 0 = floating
 * 1 = pull up
 * 2 = pull down
 */
static void ms7210_dig_pads_pull_set(struct ms7210 *ms, unsigned int pull)
{
	ms7210_update_bits(ms, MS7210_IO_CTRL3_REG, 0x30, (pull & 3) << 4);
}

/*
 * 0 = i2s
 * 1 = spdif & mclk
 * 2 = spdif
 */
static void ms7210_misc_audio_pad_in_spdif(struct ms7210 *ms, unsigned int spdif)
{
	if (spdif == 2) {
		/* aupll sel xtal */
		ms7210_update_bits(ms, MS7210_RX_PLL_SEL_REG, 0x0c, 0x0c);
		ms7210_update_bits(ms, MS7210_AUPLL_CTRL2_REG, 0x10, 0x10);
		/* audio clk enable */
		ms7210_update_bits(ms, MS7210_CLK_CTRL1_REG, 0x01, 0x01);
		/* aupll */
		/* power down */
		ms7210_update_bits(ms, MS7210_AUPLL_PWR_REG, 0x02, 0x00);
		ms7210_update_bits(ms, MS7210_AUPLL_PWR_REG, 0x01, 0x01);
		/* M */
		ms7210_update_bits(ms, MS7210_AUPLL_M_REG, 0xff, 0x1c);
		/* spdiv div */
		ms7210_update_bits(ms, MS7210_AUPLL_CFG_CTRL_REG, 0xc0, 0x40);
		/* power on */
		ms7210_update_bits(ms, MS7210_AUPLL_PWR_REG, 0x01, 0x00);

		udelay(100);

		ms7210_update_bits(ms, MS7210_AUPLL_PWR_REG, 0x02, 0x02);
		/* clk sel cdr */
		ms7210_update_bits(ms, MS7210_CLK_CTRL5_REG, 0x03, 0x03);
	} else {
		/* clk sel pad */
		ms7210_update_bits(ms, MS7210_CLK_CTRL5_REG, 0x03, 0x00);
		if (spdif)
			ms7210_update_bits(ms, MS7210_CLK_CTRL5_REG, 0x04, 0x04);
	}
	ms7210_update_bits(ms, MS7210_PINMUX_REG, 0x18, spdif << 3);
}

static void ms7210_hdmi_tx_shell_set_audio_mode(struct ms7210 *ms, int audio_mode)
{
	switch (audio_mode) {
		case HDMI_AUD_MODE_AUDIO_SAMPLE:
			ms7210_update_bits(ms,
					MS7210_HDMI_TX_AUDIO_CFG_REG +
					ms->hdmi_tx_chan * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
					0x01, 0x00);
			break;
		case HDMI_AUD_MODE_HBR:
			/* spdif_cdr_sw_rstb */
			ms7210_update_bits(ms, MS7210_HDMI_TX_MISC_RST_CTRL1_REG, 0x02, 0x02);

			ms7210_update_bits(ms,
					MS7210_HDMI_TX_AUDIO_CFG_REG +
					ms->hdmi_tx_chan * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
					0x01, 0x01);
			break;
		case HDMI_AUD_MODE_DSD:
			/* dsd_sw_rstb */
			ms7210_update_bits(ms, MS7210_HDMI_TX_MISC_RST_CTRL1_REG, 0x01, 0x01);

			ms7210_update_bits(ms,
					MS7210_HDMI_TX_AUDIO_CFG_REG +
					ms->hdmi_tx_chan * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
					0x01, 0x00);
			break;
		case HDMI_AUD_MODE_DST:
			ms7210_update_bits(ms,
					MS7210_HDMI_TX_AUDIO_CFG_REG +
					ms->hdmi_tx_chan * MS7210_HDMI_TX_CHN_REG_ADDRESS_OFST,
					0x01, 0x01);
			break;

		default:
			break;
	}
}

static void ms7210_hdmi_tx_clk_sel(struct ms7210 *ms, unsigned int sel)
{
	switch (sel) {
		case 0:
			/* clk from rx */
			ms7210_update_bits(ms, MS7210_MISC_CLK_CTRL4_REG,
					   0x0c, 1 << 2);
			break;
		case 1:
			/* clk form dvin */
			ms7210_update_bits(ms, MS7210_MISC_CLK_CTRL4_REG,
					   0x0d, (2 << 2) | 0x01);
			ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
						   MS7210_HDMI_TX_PLL_CTRL1_REG,
						   0x18, 0 << 3);
			break;
	}
}

static void ms7210_hdmi_tx_phy_set_clk(struct ms7210 *ms, unsigned int clk)
{
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PLL_TRIG_REG, 0x01);
	/* delay > 100us for PLLV clk stable */
	mdelay(10);
}

static void ms7210_hdmi_tx_phy_init(struct ms7210 *ms, unsigned int tmds)
{
	unsigned int main_po, post_po;

	/* tmds_clk > 200MHz */
	main_po = (tmds > 20000) ? 9 : 4;
	post_po = (tmds > 20000) ? 9 : 7;

	/* PLL init, use Hardware auto mode. */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PLL_CFG_SEL_REG, 0x07, 0x00);
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PLL_CTRL1_REG, 0x01, 0x01);
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PLL_CTRL4_REG, 0x08, 0x08);
	/* misc clk and reset */
	ms7210_update_bits(ms, MS7210_HDMI_TX_MISC_ACLK_SEL_REG, 0xf0, 0xf0);
	ms7210_update_bits(ms, MS7210_HDMI_TX_MISC_RST_CTRL1_REG, 0xf0, 0xf0);

	/* clk drive */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_PREC_2_REG,
				   0x70, 4 << 4); /* clk main pre */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_POC_2_REG,
				   0xf0, 2 << 4); /* clk main po */

	/* data0 drive */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_PRE0_1_REG,
				   0x07, 4 << 0); /* data main pre */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG,
				   0x0f, main_po << 0); /* data main po,  9 */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POST_PRE_REG,
				   0x01, 1 << 0); /* data post pre */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POST_PO0_1_REG,
				   0x0f, post_po << 0); /* data post po, 9 */

	/* data1 drive */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_PRE0_1_REG,
				   0x70, 4 << 4); /* data main pre */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG,
				   0xf0, main_po << 4); /* data main po */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POST_PRE_REG,
				   0x02, 1 << 1); /* data post pre */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POST_PO0_1_REG,
				   0xf0, post_po << 4); /* data post po */

	/* data2 drive */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_PREC_2_REG,
				   0x07, 4 << 0); /* data main pre */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_MAIN_POC_2_REG,
				   0x07, main_po << 0); /* data main po */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POST_PRE_REG,
				   0x04, 1 << 2); /* data post pre */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PHY_POST_PO2_REG,
				   0x0f, post_po << 0); /* data post po */

	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
			MS7210_HDMI_TX_PLL_POWER_REG, 0x20);

	if ((tmds / 100) > 100) {
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PLL_CTRL6_REG,
				0x04, 0x04);
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_POWER_REG,
				0x40, 0x40);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG, 0xdd);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_MAIN_POC_2_REG, 0x0d);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_POST_PO0_1_REG, 0x88);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_POST_PO2_REG, 0x08);
	} else {
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PLL_CTRL6_REG, 0x04, 0x00);
		ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_POWER_REG, 0x40, 0x00);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_MAIN_PO0_1_REG, 0x11);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_MAIN_POC_2_REG, 0x01);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_POST_PO0_1_REG, 0x11);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_PHY_POST_PO2_REG, 0x01);
	}

	/* HBR audio mode clk config, spdif div clk, expected value.
	 * The bigger the better.
	 * But do not greater than 350MHz
	 */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_PLL_CTRL5_REG,
				   0x03, 0x02);
}

static void ms7210_misc_freqm_pclk_enable(struct ms7210 *ms)
{
	ms7210_write(ms, MS7210_MISC_FREQ_CTRL_REG, 0x07);
}

static int ms7210_hdmi_tx_shell_timing_stable(struct ms7210 *ms)
{
	unsigned int val;

	ms7210_tx_chan_read(ms, ms->hdmi_tx_chan,
			    MS7210_HDMI_TX_SHELL_CTS_STABLE_REG, &val);
	return val & 0x10;
}

static void ms7210_hdmi_tx_hdcp_enable(struct ms7210 *ms, bool enable)
{
	if (enable) {
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				      MS7210_HDMI_TX_HDCP_CONTROL_REG, 0x0b);
	} else {
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				      MS7210_HDMI_TX_HDCP_CONTROL_REG, 0x04);
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				      MS7210_HDMI_TX_HDCP_CONTROL_REG, 0x00);
	}
}

static void ms7210_hdmi_tx_shell_set_gcp_packet_avmute(struct ms7210 *ms,
						       bool mute)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_CTRL_REG, 0x80,
				   (mute) ? 0x80 : 0x00);
}

static void ms7210_csc_config_output(struct ms7210 *ms, enum HDMI_CS cs)
{
	unsigned int c;

	switch (cs) {
		case HDMI_RGB:
			c = 0;
			break;
		case HDMI_YCBCR444:
			c = 1;
			break;
		case HDMI_YCBCR422:
			c = 2;
			break;
		default:
			c = 3;
			break;
	}
	ms7210_update_bits(ms, MS7210_CSC_CTRL1_REG, 0x0c, c << 2);
}

static void ms7210_hdmi_tx_shell_reset_enable(struct ms7210 *ms, bool en)
{
	/* video and audio reset */
	ms7210_update_bits(ms, MS7210_HDMI_TX_MISC_HDMI_RST_REG,
			   0x11 << ms->hdmi_tx_chan,
			   (en) ? 0x00 : (0x11 << ms->hdmi_tx_chan));
}

static void ms7210_hdmi_tx_shell_init(struct ms7210 *ms)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_VOLUME_CFG0_REG, 0x01, 0x01);

	/* video and audio clk enable */
	ms7210_update_bits(ms, MS7210_HDMI_TX_MISC_HDMI_CLK_REG,
			   0x11 << ms->hdmi_tx_chan,
			   0x11 << ms->hdmi_tx_chan);

	/*  audio */
	ms7210_update_bits(ms, MS7210_HDMI_TX_MISC_AUD_CTRL, 0x0f, 0x00);
	/*  audio enable */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_AUDIO_RATE_REG, 0x80, 0x80);
}

static void ms7210_hdmi_tx_shell_set_hdmi_out(struct ms7210 *ms, bool hdmi_out)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_DVI_REG,
				   0x01, (hdmi_out) ? 0x00 : 0x01);
}

static void ms7210_hdmi_tx_shell_set_clk_repeat(struct ms7210 *ms, unsigned int rpt)
{
	if (rpt > HDMI_X10CLK)
		rpt = HDMI_X1CLK;

	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_MODE_REG,
				   0x30, rpt << 4);
}

static void ms7210_hdmi_tx_shell_set_color_space(struct ms7210 *ms, unsigned int cs)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_MODE_REG,
				   0xc0, cs << 6);
}

static void ms7210_hdmi_tx_shell_set_color_depth(struct ms7210 *ms, unsigned int depth)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_MODE_REG,
				   0x04, (depth > 0) ? 0x04 : 0x00);
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_MODE_REG,
				   0x03, depth);
}

static void ms7210_hdmi_tx_shell_set_audio_rate(struct ms7210 *ms, unsigned int audio_rate)
{
	switch (audio_rate) {
		case HDMI_AUD_RATE_96K:
			ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
						   MS7210_HDMI_TX_AUDIO_RATE_REG, 0x7F, 0x02);
			break;

		case HDMI_AUD_RATE_32K:
			ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
						   MS7210_HDMI_TX_AUDIO_RATE_REG, 0x7F, 0x40);
			break;

		case HDMI_AUD_RATE_88K2:
			ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
						   MS7210_HDMI_TX_AUDIO_RATE_REG, 0x7F, 0x10);
			break;

		case HDMI_AUD_RATE_176K4: 
			ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
						   MS7210_HDMI_TX_AUDIO_RATE_REG, 0x7F, 0x08);
			break;

		case HDMI_AUD_RATE_192K:
			ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
						   MS7210_HDMI_TX_AUDIO_RATE_REG, 0x7F, 0x01);
			break;        

		case HDMI_AUD_RATE_44K1:
		case HDMI_AUD_RATE_48K:
		default:
			ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
						   MS7210_HDMI_TX_AUDIO_RATE_REG, 0x7F, 0x04);
			break; 
	}
}

static void ms7210_hdmi_tx_shell_set_audio_bits(struct ms7210 *ms,
					        unsigned int audio_bits)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_AUDIO_CFG_I2S_REG,
				   0x30, audio_bits ? 0x00 : 0x10);
}

static void ms7210_hdmi_tx_shell_set_audio_channels(struct ms7210 *ms,
						    unsigned int audio_channels)
{
	if (audio_channels <= HDMI_AUD_2CH)
		audio_channels = 0x01;
	else if (audio_channels <= HDMI_AUD_4CH)
		audio_channels = 0x03;
	else if (audio_channels <= HDMI_AUD_6CH)
		audio_channels = 0x07;
	else
		audio_channels = 0x0f;

	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_AUDIO_CH_EN_REG,
				   0x0f, audio_channels);
}

static void ms7210_hdmi_tx_shell_set_video_infoframe(struct ms7210 *ms,
						     struct hdmi_config *hc)
{
	int i;
	u8 frame[14];

	/* "AVI packet is enabled, active is high" */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_CTRL_REG, 0x40, 0x00);

	memset(frame, 0, sizeof(frame));

	frame[1] = ((hc->color_space << 5) & 0x60) |
		   0x10 | /* Active Format Information Present, Active */
		   (hc->scan_info & 0x03);

	if (hc->colorimetry == HDMI_COLORIMETRY_601) {
		frame[2] = 0x40;
	} else if ((hc->colorimetry == HDMI_COLORIMETRY_709) || (hc->colorimetry == HDMI_COLORIMETRY_1120)) {
		frame[2] = 0x80;
	} else if (hc->colorimetry == HDMI_COLORIMETRY_XVYCC601) {
		frame[2] = 0xc0;
		frame[3] &= ~0x70;
	} else if (hc->colorimetry == HDMI_COLORIMETRY_XVYCC709) {
		frame[2] = 0xc0;
		frame[3] = 0x10;
	} else {
		frame[2] &= ~0xc0;
		frame[3] &= ~0x70;
	}

	if (hc->aspect_ratio == HDMI_4X3)
		frame[2] |= 0x10;
	else if (hc->aspect_ratio == HDMI_16X9)
		frame[2] |= 0x20;

	/* R3...R0, default to 0x08
	   Same as coded frame aspect ratio.*/
	frame[2] |= 0x08;

	/* According to HDMI1.3 spec. VESA mode should be set to 0 */
	if (hc->vic >= 64)
		frame[4] = 0;
	else
		frame[4] = hc->vic;

	frame[5] = hc->clk_rpt;

	/* Calculate the check-sum */
	frame[0] = 0x82 + 0x02 + 0x0D;
	for (i = 1; i < 14; i++)
		frame[0] += frame[i];

	frame[0] = 0x100 - frame[0];
	/* Write data into hdmi shell. */
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_TYPE_REG, 0x82);
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_VER_REG, 0x02);
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_LEN_REG, 0x0D);

	for (i = 0; i < 14; i++)
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_PACK_REG, frame[i]);

	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_CTRL_REG, 0x40, 0x40);
}

static void ms7210_hdmi_tx_shell_set_audio_infoframe(struct ms7210 *ms,
						     struct hdmi_config *hc)
{
	int i;
	u8 frame[14];

	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_CTRL_REG, 0x20, 0x00);

	memset(frame, 0, sizeof(frame));
	frame[0] = 0x84;
	frame[1] = 0x01;
	frame[2] = 0x0A;
	frame[3] = 0x84 + 0x01 + 0x0A;
	frame[4] = hc->audio_channels;
	frame[5] = 0;
	frame[7] = hc->audio_speaker_locations;
	/* Calculate the checksum */
	for (i = 4; i < 14; i ++)
		frame[3] += frame[i];

	frame[3] = 0x100 - frame[3];
	/* Write packet info. */
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_TYPE_REG, frame[0]);
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_VER_REG, frame[1]);
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_LEN_REG, frame[2]);

	for (i = 3; i < 14; i ++)
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_PACK_REG, frame[i]);

	/* open the 'cfg_audio_en_o", tell the hw infoframe is ready. */
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_CTRL_REG, 0x20, 0x20);
}

static void ms7210_hdmi_tx_shell_set_vendor_specific_infoframe(struct ms7210 *ms,
							       struct hdmi_config *hc)
{
	int i;
	u8 frame[32];

	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_CFG_REG, 0x01, 0x00);

	memset(frame, 0, sizeof(frame));

	frame[0] = 0x81;
	frame[1] = 0x01;
	frame[2] = 0x1B;
	frame[3] = 0x81 + 0x01 + 0x1B;
	/* 24bit IEEE */
	frame[4] = 0x03;
	frame[5] = 0x0C;
	frame[6] = 0x00;
	/* HDMI_VIDEO_FORMAT */
	frame[7] = hc->video_format << 5;
	/* HDMI_VIC */
	frame[8] = hc->h4Kx2K_vic;

	/* Calculate the checksum */
	for (i = 4; i < 31; i ++)
		frame[3] += frame[i];

	frame[3] = 0x100 - frame[3];

	/* Write packet info. */
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_TYPE_REG, frame[0]);
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_VER_REG, frame[1]);
	ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_LEN_REG, frame[2]);

	for (i = 3; i < 31; i ++)
		ms7210_tx_chan_write(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_INFO_PACK_REG, frame[i]);

	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				MS7210_HDMI_TX_SHELL_CFG_REG, 0x01, 0x01);
}

static void ms7210_hdmi_tx_shell_video_mute_enable(struct ms7210 *ms,
						   bool en)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				  MS7210_HDMI_TX_SHELL_AVMUTE_REG,
				  0x02, en ? 0x02 : 0x00);
}

static void ms7210_hdmi_tx_shell_audio_mute_enable(struct ms7210 *ms,
						   bool en)
{
	ms7210_tx_chan_update_bits(ms, ms->hdmi_tx_chan,
				   MS7210_HDMI_TX_SHELL_AVMUTE_REG,
				   0x04, en ? 0x04 : 0x00);
}

static void ms7210_dvin_clk_reset_release(struct ms7210 *ms, bool release)
{
	if (release) {
		ms7210_update_bits(ms, MS7210_MISC_DVIN_ENABLE_REG, 0x04, 0x04);
		ms7210_update_bits(ms, MS7210_MISC_DVIN_RESET_REG, 0x01, 0x01);
	} else {
		ms7210_update_bits(ms, MS7210_MISC_DVIN_RESET_REG, 0x01, 0x00);
		ms7210_update_bits(ms, MS7210_MISC_DVIN_ENABLE_REG, 0x04, 0x00);
	}
}

static void ms7210_hdmi_tx_phy_config(struct ms7210 *ms, unsigned int video_clk)
{
	ms7210_hdmi_tx_clk_sel(ms, 1);

	ms7210_hdmi_tx_phy_init(ms, video_clk);
	ms7210_hdmi_tx_phy_power_enable(ms, true);
	/* delay > 100us for PLLV power stable */
	mdelay(10);
	ms7210_hdmi_tx_phy_set_clk(ms, video_clk);
}

static void ms7210_hdmi_tx_shell_config(struct ms7210 *ms,
					struct hdmi_config *hc)
{
	ms7210_hdmi_tx_shell_reset_enable(ms, true);
	ms7210_hdmi_tx_shell_init(ms);
	ms7210_hdmi_tx_shell_set_hdmi_out(ms, hc->hdmi_flag);
	ms7210_hdmi_tx_shell_set_clk_repeat(ms, hc->clk_rpt);

	/* if input is YUV422 and deep color mode, color space must set to RGB */
	if (hc->color_space == HDMI_YCBCR422 && hc->color_depth != HDMI_COLOR_DEPTH_8BIT)
		ms7210_hdmi_tx_shell_set_color_space(ms, HDMI_RGB);
	else
		ms7210_hdmi_tx_shell_set_color_space(ms, hc->color_space);

	ms7210_hdmi_tx_shell_set_color_depth(ms, hc->color_depth);

	ms7210_hdmi_tx_shell_set_audio_rate(ms, hc->audio_rate);
	ms7210_hdmi_tx_shell_set_audio_bits(ms, hc->audio_bits);
	ms7210_hdmi_tx_shell_set_audio_channels(ms, hc->audio_channels);

	ms7210_hdmi_tx_shell_reset_enable(ms, false);

	ms7210_hdmi_tx_shell_set_video_infoframe(ms, hc);
	ms7210_hdmi_tx_shell_set_audio_infoframe(ms, hc);
	if (hc->video_format)
		ms7210_hdmi_tx_shell_set_vendor_specific_infoframe(ms, hc);
}

static void ms7210_dvin_init(struct ms7210 *ms)
{
	ms->cs_mode = DVIN_CS_MODE_RGB; // TODO: from dts
	ms->bw_mode = DVIN_BW_MODE_16_20_24BIT;
	ms->sq_mode = DVIN_SQ_MODE_NONSEQ;
	ms->dr_mode = DVIN_DR_MODE_SDR;
	ms->sy_mode = DVIN_SY_MODE_HSVSDE; // TODO: from dts

	ms7210_rc_freq_set(ms);
	ms7210_csc_config_input(ms);

	if (ms7210drv_dvin_mode_config(ms))
		ms7210_hdmi_tx_phy_set_clk_ratio(ms, 1);

	/* all digital pads set to pull down */
	ms7210_dig_pads_pull_set(ms, 2);
	ms7210_misc_audio_pad_in_spdif(ms, ms->spdif);
	ms7210_hdmi_tx_shell_set_audio_mode(ms, ms->spdif ? 1 : 0);
	ms7210_misc_freqm_pclk_enable(ms);
}

static void ms7210_timing_config(struct ms7210 *ms,
				 struct dvin_config *dc,
				 struct videotiming *vt,
				 struct hdmi_config *hc)
{
	int clkx2;

	clkx2 = ms7210_dvin_timing_config(ms, dc, vt, &hc->clk_rpt);
	ms7210_hdmi_tx_phy_set_clk_ratio(ms, clkx2);
	hc->video_clk = vt->pixclk;
}

static void ms7210_hdmi_tx_output_config(struct ms7210 *ms,
					 struct hdmi_config *hc)
{
	ms7210_hdmi_tx_phy_output_enable(ms, false);
	ms7210_hdmi_tx_hdcp_enable(ms, false);
	ms7210_hdmi_tx_shell_set_gcp_packet_avmute(ms, false);

	ms7210_csc_config_output(ms, hc->color_space);

	ms7210_hdmi_tx_phy_config(ms, hc->video_clk);
	ms7210_hdmi_tx_shell_config(ms, hc);

	ms7210_hdmi_tx_shell_video_mute_enable(ms, false);
	ms7210_hdmi_tx_shell_audio_mute_enable(ms, false);
	ms7210_hdmi_tx_phy_output_enable(ms, true);
}

static void ms7210_dvin_video_config(struct ms7210 *ms, bool release)
{
	ms7210_dvin_clk_reset_release(ms, release);
}

static void ms7210_power_on(struct ms7210 *ms)
{
	if (ms->powered)
		return;

	ms7210_dvin_init(ms);
	ms->powered = true;
	ms->current_edid_segment = -1;
}

static void ms7210_power_off(struct ms7210 *ms, bool force)
{
	if (!ms->powered && !force)
		return;

	ms7210_hdmi_tx_phy_output_enable(ms, false);
	ms7210_hdmi_tx_phy_power_enable(ms, false);
	ms7210_hdmi_tx_hdcp_enable(ms, false);
	ms->powered = false;
}

static bool ms7210_hdmi_tx_ddc_is_busy(struct ms7210 *ms)
{
	unsigned int val;

	if (ms7210_read(ms, MS7210_HDMI_TX_MISC_DDC_DEBUG_REG, &val) < 0)
		return true;

	val &= 0x0f;

	if (val != 0x09)
		return false;

	if (ms7210_read(ms, MS7210_HDMI_TX_MISC_DDC_RO_REG, &val) < 0)
		return true;

	val >>= (ms->hdmi_tx_chan << 1);
	return ((val & 0x03) != 0x03);
}

static void ms7210_hdmi_tx_ddc_enable(struct ms7210 *ms, bool en)
{
	if (en) {
		/* I2C speed bus should be 20 kbps */
		/* Prepare the GPIO */
		ms7210_write(ms, MS7210_HDMI_TX_MISC_DDC_CTRL_REG, 0xff);
		ms7210_write(ms, MS7210_HDMI_TX_MISC_DDC_ENABLE_REG,
				(~(1 << ms->hdmi_tx_chan)) & 0x0f);
	} else {
		ms7210_write(ms, MS7210_HDMI_TX_MISC_DDC_ENABLE_REG, 0x0f);
	}
}

static int ms7210_wait_for_edid(struct ms7210 *ms, int timeout)
{
	int ret;

	if (ms->i2c_main->irq) {
		ret = wait_event_interruptible_timeout(ms->wq,
				ms->edid_read, msecs_to_jiffies(timeout));
	} else {
		for (; timeout > 0; timeout -= 25) {
			if (!ms7210_hdmi_tx_ddc_is_busy(ms)) {
				ms->edid_read = true;
				break;
			}

			msleep(25);
		}
	}

	return ms->edid_read ? 0 : -EBUSY;
}

static const u8 edid_emu[] = {
	0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x05, 0xe3, 0x70, 0x22, 0xb3, 0x9d, 0x02, 0x00,
	0x13, 0x20, 0x01, 0x03, 0x80, 0x30, 0x1b, 0x78, 0x2a, 0x39, 0x35, 0xa2, 0x59, 0x52, 0xa1, 0x27,
	0x0c, 0x50, 0x54, 0xbf, 0xef, 0x00, 0xd1, 0xc0, 0xb3, 0x00, 0x95, 0x00, 0x81, 0x80, 0x81, 0x40,
	0x81, 0xc0, 0x01, 0x01, 0x01, 0x01, 0x02, 0x3a, 0x80, 0x18, 0x71, 0x38, 0x2d, 0x40, 0x58, 0x2c,
	0x45, 0x00, 0xdd, 0x0c, 0x11, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xfd, 0x00, 0x32, 0x4c, 0x1e,
	0x53, 0x11, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x32,
	0x32, 0x37, 0x30, 0x57, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0xff,
	0x00, 0x47, 0x58, 0x50, 0x4e, 0x35, 0x48, 0x41, 0x31, 0x37, 0x31, 0x34, 0x34, 0x33, 0x01, 0x0c,
	0x02, 0x03, 0x1e, 0xf1, 0x4b, 0x10, 0x1f, 0x05, 0x14, 0x04, 0x13, 0x03, 0x12, 0x02, 0x11, 0x01,
	0x23, 0x09, 0x07, 0x07, 0x83, 0x01, 0x00, 0x00, 0x65, 0x03, 0x0c, 0x00, 0x10, 0x00, 0x8c, 0x0a,
	0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00, 0xdd, 0x0c, 0x11, 0x00, 0x00, 0x18,
	0x01, 0x1d, 0x00, 0x72, 0x51, 0xd0, 0x1e, 0x20, 0x6e, 0x28, 0x55, 0x00, 0xdd, 0x0c, 0x11, 0x00,
	0x00, 0x1e, 0x8c, 0x0a, 0xd0, 0x8a, 0x20, 0xe0, 0x2d, 0x10, 0x10, 0x3e, 0x96, 0x00, 0xdd, 0x0c,
	0x11, 0x00, 0x00, 0x18, 0x8c, 0x0a, 0xd0, 0x90, 0x20, 0x40, 0x31, 0x20, 0x0c, 0x40, 0x55, 0x00,
	0xdd, 0x0c, 0x11, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x45,
};

static int __ms7210_get_edid_block(void *data, u8 *buf, unsigned int block,
				 size_t len)
{
	struct ms7210 *ms = data;
	struct i2c_msg xfer[2];
	uint8_t offset;
	unsigned int i;
	int ret;

	if (ms->current_edid_segment != block / 2) {

		if (ms7210_hdmi_tx_ddc_is_busy(ms)) {
			ms->edid_read = false;

			ret = ms7210_wait_for_edid(ms, 200);
			if (ret < 0)
				return ret;
		}

		ms7210_hdmi_tx_ddc_enable(ms, true);

		/* Break this apart, hopefully more I2C controllers will
		 * support 64 byte transfers than 256 byte transfers
		 */

		xfer[0].addr = ms->i2c_edid->addr;
		xfer[0].flags = 0;
		xfer[0].len = 1;
		xfer[0].buf = &offset;
		xfer[1].addr = ms->i2c_edid->addr;
		xfer[1].flags = I2C_M_RD;
		xfer[1].len = 64;
		xfer[1].buf = ms->edid_buf;

		offset = 0;

		for (i = 0; i < 4; ++i) {
			ret = i2c_transfer(ms->i2c_edid->adapter, xfer,
					   ARRAY_SIZE(xfer));
			if (ret < 0)
				return ret;
			else if (ret != 2)
				return -EIO;

			xfer[1].buf += 64;
			offset += 64;
		}

		ms->current_edid_segment = block / 2;

		ms7210_hdmi_tx_ddc_enable(ms, false);
	}
	if (block % 2 == 0)
		memcpy(buf, ms->edid_buf, len);
	else
		memcpy(buf, ms->edid_buf + 128, len);

	return 0;
}

static int ms7210_get_edid_block(void *data, u8 *buf, unsigned int block,
				 size_t len)
{
	struct ms7210 *ms = data;

	if (len > 128)
		return -EINVAL;

	if (ms->status == connector_status_connected)
		return  __ms7210_get_edid_block(data, buf, block, len);

	if (!ms->always_on)
		return -EIO;

	/* DSUB */
	ms->current_edid_segment = block / 2;
	memcpy(ms->edid_buf, edid_emu, 256);

	if (block % 2 == 0)
		memcpy(buf, ms->edid_buf, len);
	else
		memcpy(buf, ms->edid_buf + 128, len);


	return 0;
}

static void ms7210_set_config_csc(struct ms7210 *ms,
				  struct drm_connector *connector,
				  bool rgb, bool hdmi_mode)
{
#if 0
	struct adv7511_video_config config;
	bool output_format_422, output_format_ycbcr;
	unsigned int mode;
	uint8_t infoframe[17];

	config.hdmi_mode = hdmi_mode;

	hdmi_avi_infoframe_init(&config.avi_infoframe);

	config.avi_infoframe.scan_mode = HDMI_SCAN_MODE_UNDERSCAN;

	if (rgb) {
		config.csc_enable = false;
		config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
	} else {
		config.csc_scaling_factor = ADV7511_CSC_SCALING_4;
		config.csc_coefficents = adv7511_csc_ycbcr_to_rgb;

		if ((connector->display_info.color_formats &
		     DRM_COLOR_FORMAT_YCRCB422) &&
		    config.hdmi_mode) {
			config.csc_enable = false;
			config.avi_infoframe.colorspace =
				HDMI_COLORSPACE_YUV422;
		} else {
			config.csc_enable = true;
			config.avi_infoframe.colorspace = HDMI_COLORSPACE_RGB;
		}
	}

	if (config.hdmi_mode) {
		mode = ADV7511_HDMI_CFG_MODE_HDMI;

		switch (config.avi_infoframe.colorspace) {
		case HDMI_COLORSPACE_YUV444:
			output_format_422 = false;
			output_format_ycbcr = true;
			break;
		case HDMI_COLORSPACE_YUV422:
			output_format_422 = true;
			output_format_ycbcr = true;
			break;
		default:
			output_format_422 = false;
			output_format_ycbcr = false;
			break;
		}
	} else {
		mode = ADV7511_HDMI_CFG_MODE_DVI;
		output_format_422 = false;
		output_format_ycbcr = false;
	}

	adv7511_packet_disable(adv7511, ADV7511_PACKET_ENABLE_AVI_INFOFRAME);

	adv7511_set_colormap(adv7511, config.csc_enable,
			     config.csc_coefficents,
			     config.csc_scaling_factor);

	regmap_update_bits(adv7511->regmap, ADV7511_REG_VIDEO_INPUT_CFG1, 0x81,
			   (output_format_422 << 7) | output_format_ycbcr);

	regmap_update_bits(adv7511->regmap, ADV7511_REG_HDCP_HDMI_CFG,
			   ADV7511_HDMI_CFG_MODE_MASK, mode);

	hdmi_avi_infoframe_pack(&config.avi_infoframe, infoframe,
				sizeof(infoframe));

	/* The AVI infoframe id is not configurable */
	regmap_bulk_write(adv7511->regmap, ADV7511_REG_AVI_INFOFRAME_VERSION,
			  infoframe + 1, sizeof(infoframe) - 1);

	adv7511_packet_enable(adv7511, ADV7511_PACKET_ENABLE_AVI_INFOFRAME);
#endif
}


/* -----------------------------------------------------------------------------
 * Interrupt and hotplug detection
 */

static bool ms7210_hpd(struct ms7210 *ms)
{
	if (!ms7210_hdmi_tx_shell_timing_stable(ms))
		return false;


	return true;
}

static int ms7210_get_modes(struct ms7210 *ms,
			    struct drm_connector *connector)
{
	struct edid *edid;
	unsigned int count;

#if 0
	/* Reading the EDID only works if the device is powered */
	if (!adv7511->powered) {
		unsigned int edid_i2c_addr =
					(adv7511->i2c_edid->addr << 1);

		__adv7511_power_on(adv7511);

		/* Reset the EDID_I2C_ADDR register as it might be cleared */
		regmap_write(adv7511->regmap, ADV7511_REG_EDID_I2C_ADDR,
			     edid_i2c_addr);
	}
#endif

	edid = drm_do_get_edid(connector, ms7210_get_edid_block, ms);
	if (!edid)
		return 0;

#if 0
	print_hex_dump(KERN_INFO, "EDID buffer: ",
		       DUMP_PREFIX_NONE, 16, 1,
		       edid, sizeof(struct edid), false);
#endif

#if 0
	if (!adv7511->powered)
		__adv7511_power_off(adv7511);
#endif

	drm_connector_update_edid_property(connector, edid);
	count = drm_add_edid_modes(connector, edid);

	ms7210_set_config_csc(ms, connector, ms->rgb,
			      drm_detect_hdmi_monitor(edid));

	cec_s_phys_addr_from_edid(ms->cec_adap, edid);

	kfree(edid);

	return count;
}

static enum drm_connector_status
ms7210_detect(struct ms7210 *ms, struct drm_connector *connector)
{
	enum drm_connector_status status;
	unsigned int val;
	bool hpd;
	int ret;

	ret = ms7210_tx_chan_read(ms, ms->hdmi_tx_chan,
				  MS7210_HDMI_TX_SHELL_STATUS_REG, &val);
	if (ret < 0)
		return connector_status_disconnected;

	if (val & 1)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	hpd = ms7210_hpd(ms);

	if (ms->status != status) {
		ms->status = status;
		if (status == connector_status_connected) {
			ms7210_power_on(ms);
			if (ms7210_get_modes(ms, connector) == 0)
				status = ms->status = connector_status_disconnected;
		} else {
			//ms7210_power_off(ms, false);
		}
		dev_err(&ms->i2c_main->dev, "Connector status = %d, hpd = %d, val = %02x", status, hpd, val);
	} else {
#if 0
		/* Renable HPD sensing */
#endif
	}

	if (ms->always_on)
		status = connector_status_connected;

	return status;
}

static void ms7210_hpd_work(struct work_struct *work)
{
#if 0
	struct ms7210 *ms = container_of(work, struct ms7210, hpd_work);
	enum drm_connector_status status;
	unsigned int val;
	int ret;

	ret = ms7210_tx_chan_read(ms, ms->hdmi_tx_chan,
				  MS7210_HDMI_TX_SHELL_STATUS_REG, &val);
	if (ret < 0)
		status = connector_status_disconnected;
	else if (val & 1)
		status = connector_status_connected;
	else
		status = connector_status_disconnected;

	/*
	 * The bridge resets its registers on unplug. So when we get a plug
	 * event and we're already supposed to be powered, cycle the bridge to
	 * restore its state.
	 */
	if (status == connector_status_connected &&
	    ms->connector.status == connector_status_disconnected &&
	    ms->powered) {
		ms7210_power_on(ms);
	}

	if (ms->connector.status != status) {
		ms->connector.status = status;
		if (status == connector_status_disconnected)
			cec_phys_addr_invalidate(ms->cec_adap);

		drm_kms_helper_hotplug_event(ms->connector.dev);
	}
#endif
}

static int ms7210_irq_process(struct ms7210 *ms, bool process_hpd)
{
#if 0
	unsigned int irq0, irq1;
	int ret;

	ret = regmap_read(adv7511->regmap, ADV7511_REG_INT(0), &irq0);
	if (ret < 0)
		return ret;

	ret = regmap_read(adv7511->regmap, ADV7511_REG_INT(1), &irq1);
	if (ret < 0)
		return ret;

	regmap_write(adv7511->regmap, ADV7511_REG_INT(0), irq0);
	regmap_write(adv7511->regmap, ADV7511_REG_INT(1), irq1);

	if (process_hpd && irq0 & ADV7511_INT0_HPD && adv7511->bridge.encoder)
		schedule_work(&adv7511->hpd_work);

	if (irq0 & ADV7511_INT0_EDID_READY || irq1 & ADV7511_INT1_DDC_ERROR) {
		adv7511->edid_read = true;

		if (adv7511->i2c_main->irq)
			wake_up_all(&adv7511->wq);
	}

#ifdef CONFIG_DRM_I2C_ADV7511_CEC
	adv7511_cec_irq_process(adv7511, irq1);
#endif

#endif
	return 0;
}

static irqreturn_t ms7210_irq_handler(int irq, void *devid)
{
	struct ms7210 *ms = devid;
	int ret;

	dev_err(&ms->i2c_main->dev, "irq handler\n");

	ret = ms7210_irq_process(ms, true);
	return ret < 0 ? IRQ_NONE : IRQ_HANDLED;
}

static enum drm_mode_status ms7210_mode_valid(struct ms7210 *ms,
					      struct drm_display_mode *mode)
{
	if (mode->clock > 165000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void ms7210_mode_set(struct ms7210 *ms,
			    struct drm_display_mode *mode,
			    struct drm_display_mode *adj_mode)
{
	struct videotiming vt;

	struct dvin_config dc = {
		DVIN_CS_MODE_RGB, // TODO: from dts
		DVIN_BW_MODE_16_20_24BIT,
		DVIN_SQ_MODE_NONSEQ,
		DVIN_DR_MODE_SDR,
		DVIN_SY_MODE_HSVSDE // TODO: from dts
	};

	struct hdmi_config hc = {
		.hdmi_flag = true,
		.vic = 0,
		//.video_clk = 7425,
		.clk_rpt = HDMI_X1CLK,
		.scan_info = HDMI_OVERSCAN,
		.aspect_ratio = HDMI_16X9,
		.color_space = HDMI_RGB,
		.color_depth = HDMI_COLOR_DEPTH_8BIT,
		.colorimetry = HDMI_COLORIMETRY_709,

		.video_format = HDMI_NO_ADD_FORMAT,
		.h4Kx2K_vic = HDMI_4Kx2K_30HZ,
		.h3D_structure = HDMI_FRAME_PACKING,

		.audio_mode = HDMI_AUD_MODE_AUDIO_SAMPLE,
		.audio_rate = HDMI_AUD_RATE_48K,
		.audio_bits = HDMI_AUD_LENGTH_16BITS,
		.audio_channels = HDMI_AUD_2CH,
		.audio_speaker_locations = 0,
	};

	hc.vic = drm_match_cea_mode(adj_mode);
	if ((hc.vic == 6) || (hc.vic == 7) ||
			(hc.vic == 21) || (hc.vic == 22) ||
			(hc.vic == 2) || (hc.vic == 3) ||
			(hc.vic == 17) || (hc.vic == 18))
		hc.colorimetry = HDMI_COLORIMETRY_ITU_601;
	else
		hc.colorimetry = HDMI_COLORIMETRY_ITU_709;

	vt.polarity = 0;
	if (adj_mode->flags & DRM_MODE_FLAG_NHSYNC)
		vt.polarity |= 2;

	if (adj_mode->flags & DRM_MODE_FLAG_NVSYNC)
		vt.polarity |= 4;

	vt.htotal = adj_mode->crtc_htotal;
	vt.vtotal = adj_mode->crtc_vtotal;
	vt.hactive = adj_mode->crtc_hdisplay;
	vt.vactive = adj_mode->crtc_vdisplay;

	vt.pixclk = adj_mode->crtc_clock / 10;     /* 10 000 hz*/

	vt.vfreq = drm_mode_vrefresh(mode) * 100;      /*0.01hz*/

	vt.hoffset = adj_mode->crtc_hsync_start -
		     adj_mode->crtc_hdisplay;
	vt.voffset = adj_mode->crtc_vsync_start -
		     adj_mode->crtc_vdisplay;

	vt.hsyncwidth = adj_mode->crtc_hsync_end -
			adj_mode->crtc_hsync_start;
	vt.vsyncwidth = adj_mode->crtc_vsync_end -
			adj_mode->crtc_vsync_start;

	dev_dbg(&ms->i2c_main->dev, "Mode set vic %d, pixclk %d, vfreq %d, polarity 0x%02x",
			hc.vic, vt.pixclk, vt.vfreq, vt.polarity);

	dev_dbg(&ms->i2c_main->dev, "Mode set htotal %d, vtotal %d, hactive %d, "
				    "vactive %d, hoffset %d, voffset %d, hsyncwidth %d, vsyncwidth %d",
			vt.htotal, vt.vtotal, vt.hactive, vt.vactive, vt.hoffset, vt.voffset,
			vt.hsyncwidth, vt.vsyncwidth);
	ms7210_timing_config(ms, &dc, &vt, &hc);

	ms7210_dvin_video_config(ms, true);

	ms7210_hdmi_tx_output_config(ms, &hc);

	drm_mode_copy(&ms->curr_mode, adj_mode);

	ms->f_tmds = mode->clock;
}

/* Connector funcs */
static struct ms7210 *connector_to_ms7210(struct drm_connector *connector)
{
	return container_of(connector, struct ms7210, connector);
}

static int ms7210_connector_get_modes(struct drm_connector *connector)
{
	struct ms7210 *ms = connector_to_ms7210(connector);

	return ms7210_get_modes(ms, connector);
}

static enum drm_mode_status
ms7210_connector_mode_valid(struct drm_connector *connector,
			     struct drm_display_mode *mode)
{
	struct ms7210 *ms = connector_to_ms7210(connector);

	return ms7210_mode_valid(ms, mode);
}

static struct drm_connector_helper_funcs ms7210_connector_helper_funcs = {
	.get_modes = ms7210_connector_get_modes,
	.mode_valid = ms7210_connector_mode_valid,
};

static enum drm_connector_status
ms7210_connector_detect(struct drm_connector *connector, bool force)
{
	struct ms7210 *ms = connector_to_ms7210(connector);

	return ms7210_detect(ms, connector);
}

static const struct drm_connector_funcs ms7210_connector_funcs = {
	.fill_modes = drm_helper_probe_single_connector_modes,
	.detect = ms7210_connector_detect,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

/* Bridge funcs */
static inline struct ms7210 *bridge_to_ms7210(struct drm_bridge *bridge)
{
	return container_of(bridge, struct ms7210, bridge);
}

static void ms7210_bridge_enable(struct drm_bridge *bridge)
{
	struct ms7210 *ms = bridge_to_ms7210(bridge);

	ms7210_power_on(ms);
}

static void ms7210_bridge_disable(struct drm_bridge *bridge)
{
	struct ms7210 *ms = bridge_to_ms7210(bridge);

	ms7210_power_off(ms, true);
}

static void ms7210_bridge_mode_set(struct drm_bridge *bridge,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adj_mode)
{
	struct ms7210 *ms = bridge_to_ms7210(bridge);

	ms7210_mode_set(ms, mode, adj_mode);
}

static int ms7210_bridge_attach(struct drm_bridge *bridge)
{
	struct ms7210 *ms = bridge_to_ms7210(bridge);
	int ret;

	if (!bridge->encoder) {
		DRM_ERROR("Parent encoder object not found");
		return -ENODEV;
	}

	if (ms->i2c_main->irq)
		ms->connector.polled = DRM_CONNECTOR_POLL_HPD;
	else
		ms->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
				DRM_CONNECTOR_POLL_DISCONNECT;

	ret = drm_connector_init(bridge->dev, &ms->connector,
				 &ms7210_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}
	drm_connector_helper_add(&ms->connector,
				 &ms7210_connector_helper_funcs);
	drm_connector_attach_encoder(&ms->connector, bridge->encoder);

	if (ms->i2c_main->irq)
		ms7210_write(ms, MS7210_HDMI_TX_SHELL_INTERRUPT_REG, 0x01);

	return ret;
}

static const struct drm_bridge_funcs ms7210_bridge_funcs = {
	.enable = ms7210_bridge_enable,
	.disable = ms7210_bridge_disable,
	.mode_set = ms7210_bridge_mode_set,
	.attach = ms7210_bridge_attach,
};

static int ms7210_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
//	struct ms7210_link_config link_config;
	struct ms7210 *ms;
	struct device *dev = &i2c->dev;
	unsigned int val;
	int ret;

	if (!dev->of_node)
		return -EINVAL;

	ms = devm_kzalloc(dev, sizeof(*ms), GFP_KERNEL);
	if (!ms)
		return -ENOMEM;

	ms->i2c_main = i2c;
	ms->powered = false;
	ms->status = connector_status_disconnected;

	if (dev->of_node)
		ms->type = (enum ms7210_type)of_device_get_match_data(dev);
	else
		ms->type = id->driver_data;

	if (of_property_read_bool(dev->of_node, "ms,always-on"))
		ms->always_on = true;
	else
		ms->always_on = false;

#if 0
	memset(&link_config, 0, sizeof(link_config));

	ret = ms7210_parse_dt(dev->of_node, &link_config);
	if (ret)
		return ret;

	ret = ms7210_init_regulators(ms);
	if (ret) {
		dev_err(dev, "failed to init regulators\n");
		return ret;
	}
#endif

	ms->pixclk = devm_clk_get(dev, "pixclk");
	if (PTR_ERR(ms->pixclk) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	ret = clk_prepare_enable(ms->pixclk);
	if (ret)
		return ret;

	ms->regmap = devm_regmap_init_i2c(i2c, &ms7210_regmap_config);
	if (IS_ERR(ms->regmap)) {
		ret = PTR_ERR(ms->regmap);
		goto uninit_regulators;
	}

	ret = regmap_read(ms->regmap, MS7210_CHIPID0_REG, &val);
	if (ret)
		goto uninit_regulators;

	dev_dbg(dev, "Chip ID %02x\n", val);

	ms->i2c_edid = i2c_new_secondary_device(i2c, "edid",
					MS7210_EDID_I2C_ADDR);
	if (!ms->i2c_edid) {
		dev_err(dev, "edid i2c device add failed\n");
		ret = -EINVAL;
		goto uninit_regulators;
	}

	if (i2c->irq)
		dev_dbg(dev, "Using i2c interrupt to hpd\n");
	else
		dev_dbg(dev, "Using poll to hpd\n");

	INIT_WORK(&ms->hpd_work, ms7210_hpd_work);

	if (i2c->irq) {
		init_waitqueue_head(&ms->wq);

		ret = devm_request_threaded_irq(dev, i2c->irq, NULL,
						ms7210_irq_handler,
						IRQF_ONESHOT, dev_name(dev),
						ms);
		if (ret)
			goto err_i2c_unregister_edid;
	}

	ms7210_power_off(ms, true);

	i2c_set_clientdata(i2c, ms);

//	ms7210_set_link_config(ms, &link_config);

	/* FIXME: fill this from dts */
	ms->rgb = true;

	ms->bridge.funcs = &ms7210_bridge_funcs;
	ms->bridge.of_node = dev->of_node;

	drm_bridge_add(&ms->bridge);

//	ms7210_audio_init(dev, ms);
	return 0;

err_i2c_unregister_edid:
	i2c_unregister_device(ms->i2c_edid);
uninit_regulators:
//	ms7210_uninit_regulators(ms);

	return ret;
}

static int ms7210_remove(struct i2c_client *i2c)
{
	struct ms7210 *ms = i2c_get_clientdata(i2c);

//	ms7210_uninit_regulators(ms);

	drm_bridge_remove(&ms->bridge);

//	ms7210_audio_exit(ms);

	cec_unregister_adapter(ms->cec_adap);

	i2c_unregister_device(ms->i2c_edid);

	return 0;
}

static const struct i2c_device_id ms7210_i2c_ids[] = {
	{ "ms7210", MS7210 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ms7210_i2c_ids);

static const struct of_device_id ms7210_of_ids[] = {
	{ .compatible = "ms,ms7210", .data = (void *)MS7210 },
	{ }
};
MODULE_DEVICE_TABLE(of, ms7210_of_ids);

static struct i2c_driver ms7210_driver = {
	.driver = {
		.name = "ms7210",
		.of_match_table = ms7210_of_ids,
	},
	.id_table = ms7210_i2c_ids,
	.probe = ms7210_probe,
	.remove = ms7210_remove,
};

static int __init ms7210_init(void)
{
	return i2c_add_driver(&ms7210_driver);
}
module_init(ms7210_init);

static void __exit ms7210_exit(void)
{
	i2c_del_driver(&ms7210_driver);
}
module_exit(ms7210_exit);

MODULE_AUTHOR("Andrey Mitrofanov <avmwww@gmail.com>");
MODULE_DESCRIPTION("MS7210 HDMI transmitter driver");
MODULE_LICENSE("GPL");
