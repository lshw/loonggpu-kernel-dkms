#include <drm/drm_atomic_helper.h>
#include <linux/delay.h>
#include <gsgpu.h>
#include "gsgpu_helper.h"
#include "gsgpu_dc.h"
#include "gsgpu_dc_resource.h"
#include "gsgpu_dc_plane.h"
#include "gsgpu_dc_crtc.h"
#include "gsgpu_dc_hdmi.h"
#include "gsgpu_dc_reg.h"
#include "gsgpu_backlight.h"
#include "gsgpu_helper.h"

static unsigned int
cal_freq(unsigned int pixclock_khz, struct pixel_clock *pll_config)
{
	unsigned int pstdiv, loopc, frefc;
	unsigned long a, b, c;
	unsigned long min = 50;

	for (pstdiv = 1; pstdiv < 64; pstdiv++) {
		a = (unsigned long)pixclock_khz * pstdiv;
		for (frefc = 3; frefc < 6; frefc++) {
			for (loopc = 24; loopc < 161; loopc++) {
				if ((loopc < 12 * frefc) ||
				    (loopc > 32 * frefc))
					continue;

				b = 100000L * loopc / frefc;
				c = (a > b) ? (a * 10000 / b - 10000) :
					(b * 10000 / a - 10000);
				if (c < min) {
					min = c;
					pll_config->l2_div = pstdiv;
					pll_config->l1_loopc = loopc;
					pll_config->l1_frefc = frefc;
				}
			}
		}
	}

	if (min < 50)
		return 1;

	return 0;
}

static u32 dc_io_rreg(void *base, u32 offset)
{
	return readl(base + offset);
}

static void dc_io_wreg(void *base, u32 offset, u32 val)
{
	writel(val, base + offset);
}

static bool config_pll(struct gsgpu_device *adev, u32 clock, unsigned long pll_reg)
{
	u32 val;
	u32 count = 0;
	struct pixel_clock pll_cfg = {0};

	void __iomem *io_base = adev->io_base;
	if (io_base == NULL)
		return false;

	cal_freq(clock, &pll_cfg);

	/* clear sel_pll_out0 */
	val = dc_io_rreg(io_base, pll_reg + 0x4);
	val &= ~(1UL << 8);
	dc_io_wreg(io_base, pll_reg + 0x4, val);

	/* set pll_pd */
	val = dc_io_rreg(io_base, pll_reg + 0x4);
	val |= (1UL << 13);
	dc_io_wreg(io_base, pll_reg + 0x4, val);

	/* clear set_pll_param */
	val = dc_io_rreg(io_base, pll_reg + 0x4);
	val &= ~(1UL << 11);
	dc_io_wreg(io_base, pll_reg + 0x4, val);

	/* clear old value & config new value */
	val = dc_io_rreg(io_base, pll_reg + 0x4);
	val &= ~(0x7fUL << 0);
	val |= (pll_cfg.l1_frefc << 0); /* refc */
	dc_io_wreg(io_base, pll_reg + 0x4, val);
	val = dc_io_rreg(io_base, pll_reg + 0x0);
	val &= ~(0x7fUL << 0);
	val |= (pll_cfg.l2_div << 0); /* div */
	val &= ~(0x1ffUL << 21);
	val |= (pll_cfg.l1_loopc << 21); /* loopc */
	dc_io_wreg(io_base, pll_reg + 0x0, val);

	/* set set_pll_param */
	val = dc_io_rreg(io_base, pll_reg + 0x4);
	val |= (1UL << 11);
	dc_io_wreg(io_base, pll_reg + 0x4, val);
	/* clear pll_pd */
	val = dc_io_rreg(io_base, pll_reg + 0x4);
	val &= ~(1UL << 13);
	dc_io_wreg(io_base, pll_reg + 0x4, val);

	while (!(dc_io_rreg(io_base, pll_reg + 0x4) & 0x80)) {
		cpu_relax();
		count++;
		if (count >= 1000) {
			DRM_ERROR("loongson-7A PLL lock failed\n");
			return false;
		}
		schedule_timeout(usecs_to_jiffies(100));
	}

	val = dc_io_rreg(io_base, pll_reg + 0x4);
	val |= (1UL << 8);
	dc_io_wreg(io_base, pll_reg + 0x4, val);

	return true;
}

bool dc_set_pll(struct gsgpu_dc_crtc *crtc, u32 clock)
{
	struct gsgpu_device *adev = crtc->dc->adev;
	u32 link;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	gsgpu_hdmi_enable(adev, link, false);
	if (config_pll(adev, clock, CURRENT_REG(DC_IO_PIX_PLL, link)) == false)
		return false;

	hdmi_phy_pll_config(adev, link, clock);
	gsgpu_hdmi_enable(adev, link, true);
	return true;
}

bool dc_crtc_timing_set(struct gsgpu_dc_crtc *crtc, struct dc_timing_info *timing)
{
	struct gsgpu_device *adev = crtc->dc->adev;
	u32 depth;
	u32 link;
	u32 value;
	u32 stride_reg, cur_stride_reg;
	u32 dc_cfg, cur_dc_cfg;
	u32 hdisplay, cur_hdisplay;
	u32 hsync, cur_hsync;
	u32 vdisplay, cur_vdisplay;
	u32 vsync, cur_vsync;
	u32 panel_cfg, cur_panel_cfg;

	if (IS_ERR_OR_NULL(crtc) || IS_ERR_OR_NULL(timing))
		return false;

	DRM_DEBUG_DRIVER("crtc %d timing set: clock %d, stride %d\n",
		crtc->resource->base.link, timing->clock, timing->stride);
	DRM_DEBUG_DRIVER("hdisplay %d, hsync_start %d, hsync_end %d, htotal %d\n",
		timing->hdisplay, timing->hsync_start, timing->hsync_end, timing->htotal);
	DRM_DEBUG_DRIVER("vdisplay %d, vsync_start %d, vsync_end %d, vtotal %d\n",
		timing->vdisplay, timing->vsync_start, timing->vsync_end, timing->vtotal);
	DRM_DEBUG_DRIVER("depth %d, use_dma %d\n", timing->depth, timing->use_dma);

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	dc_cfg = CRTC_CFG_RESET;

	switch (crtc->array_mode) {
	case 0:
		dc_cfg &= CRTC_CFG_LINEAR;
		if (adev->chip == dev_2k2000) {
			dc_cfg &= CRTC_CFG_TILE8_DISABLE;
			dc_cfg &= CRTC_CFG_ZIP_DISABLE;
		}
		stride_reg = timing->stride;
		break;
	case 2:
		dc_cfg |= CRTC_CFG_TILE4x4;
		if (adev->chip == dev_2k2000) {
			dc_cfg &= CRTC_CFG_TILE8_DISABLE;
			dc_cfg |= CRTC_CFG_ZIP_ENABLE;
		}
		stride_reg = timing->stride * 4;
		break;
	}

	dc_cfg &= ~CRTC_CFG_DMA_MASK;
	if (timing->use_dma)
		dc_cfg |= timing->use_dma;
	else if (!(timing->hdisplay % 64))
		dc_cfg |= CRTC_CFG_DMA_256;
	else if (!(timing->hdisplay % 32))
		dc_cfg |= CRTC_CFG_DMA_128;
	else if (!(timing->hdisplay % 16))
		dc_cfg |= CRTC_CFG_DMA_64;
	else if (!(timing->hdisplay % 8))
		dc_cfg |= CRTC_CFG_DMA_32;

	dc_cfg &= ((~CRTC_CFG_FORMAT_MASK) << 0);
	depth = timing->depth;
	switch (depth) {
	case 12:
		dc_cfg |= (DC_FB_FORMAT12 & CRTC_CFG_FORMAT_MASK);
		break;
	case 15:
		dc_cfg |= (DC_FB_FORMAT15 & CRTC_CFG_FORMAT_MASK);
		break;
	case 16:
		dc_cfg |= (DC_FB_FORMAT16 & CRTC_CFG_FORMAT_MASK);
		break;
	case 32:
	case 24:
	default:
		dc_cfg |= (DC_FB_FORMAT32 & CRTC_CFG_FORMAT_MASK);
		break;
	}

	dc_cfg = dc_cfg | CRTC_CFG_ENABLE;

	hdisplay = ((timing->hdisplay & CRTC_HPIXEL_MASK) << CRTC_HPIXEL_SHIFT);
	hdisplay |= ((timing->htotal & CRTC_HTOTAL_MASK) << CRTC_HTOTAL_SHIFT);

	hsync = CRTC_HSYNC_POLSE;
	hsync |= ((timing->hsync_start & CRTC_HSYNC_START_MASK) << CRTC_HSYNC_START_SHIFT);
	hsync |= ((timing->hsync_end & CRTC_HSYNC_END_MASK) << CRTC_HSYNC_END_SHIFT);

	vdisplay = ((timing->vdisplay & CRTC_VPIXEL_MASK) << CRTC_VPIXEL_SHIFT);
	vdisplay |= ((timing->vtotal & CRTC_VTOTAL_MASK) << CRTC_VTOTAL_SHIFT);

	vsync = CRTC_VSYNC_POLSE;
	vsync |= ((timing->vsync_start  & CRTC_VSYNC_START_MASK) << CRTC_VSYNC_START_SHIFT);
	vsync |= ((timing->vsync_end & CRTC_VSYNC_END_MASK) << CRTC_VSYNC_END_SHIFT);

	panel_cfg = CRTC_PANCFG_BASE | CRTC_PANCFG_DE | CRTC_PANCFG_CLKEN;
	if (gsgpu_panel_cfg_clk_pol != -1)
		panel_cfg |= (gsgpu_panel_cfg_clk_pol & 1) << 9;
	else
		panel_cfg |= CRTC_PANCFG_CLKPOL;

	if (gsgpu_panel_cfg_de_pol != -1)
		panel_cfg |= (gsgpu_panel_cfg_de_pol & 1) << 1;

	cur_stride_reg = dc_readl(adev, CURRENT_REG(DC_CRTC_STRIDE_REG, link));
	cur_hdisplay = dc_readl(adev, CURRENT_REG(DC_CRTC_HDISPLAY_REG, link));
	cur_hsync = dc_readl(adev, CURRENT_REG(DC_CRTC_HSYNC_REG, link));
	cur_vdisplay = dc_readl(adev, CURRENT_REG(DC_CRTC_VDISPLAY_REG, link));
	cur_vsync = dc_readl(adev, CURRENT_REG(DC_CRTC_VSYNC_REG, link));
	cur_panel_cfg = dc_readl(adev, CURRENT_REG(DC_CRTC_PANELCFG_REG, link));
	cur_dc_cfg = dc_readl(adev, CURRENT_REG(DC_CRTC_CFG_REG, link));
	cur_dc_cfg &= CRTC_CFG_MASK;

	if (cur_stride_reg == stride_reg &&
		cur_hdisplay == hdisplay &&
		cur_hsync == hsync &&
		cur_vdisplay == vdisplay &&
		cur_vsync == vsync &&
		cur_panel_cfg == panel_cfg &&
		cur_dc_cfg == dc_cfg &&
		crtc->timing->clock == timing->clock)
		return false;

	value = dc_readl(adev, CURRENT_REG(DC_CRTC_CFG_REG, link));
	value &= CRTC_CFG_MASK;
	dc_writel_check(adev, CURRENT_REG(DC_CRTC_CFG_REG, link), value & (~CRTC_CFG_ENABLE));
	dc_writel(adev, CURRENT_REG(DC_CRTC_STRIDE_REG, link), stride_reg);
	dc_writel(adev, CURRENT_REG(DC_CRTC_HDISPLAY_REG, link), hdisplay);
	dc_writel(adev, CURRENT_REG(DC_CRTC_HSYNC_REG, link), hsync);
	dc_writel(adev, CURRENT_REG(DC_CRTC_VDISPLAY_REG, link), vdisplay);
	dc_writel(adev, CURRENT_REG(DC_CRTC_VSYNC_REG, link), vsync);
	dc_writel(adev, CURRENT_REG(DC_CRTC_PANELCFG_REG, link), panel_cfg);
	dc_writel(adev, CURRENT_REG(DC_CRTC_PANELTIM_REG, link), 0);

	if (crtc->timing->clock != timing->clock) {
		if (dc_set_pll(crtc, timing->clock))
			crtc->timing->clock = timing->clock;
	}

	dc_writel_check(adev, CURRENT_REG(DC_CRTC_CFG_REG, link), dc_cfg);
	return true;
}

bool dc_crtc_enable(struct gsgpu_crtc *acrtc, bool enable)
{
	struct gsgpu_device *adev = acrtc->base.dev->dev_private;
	struct gsgpu_backlight *ls_bl;
	u32 crtc_cfg, crtc_pan;
	u32 hsync_val, vsync_val;
	u32 crtc_id;

	if (IS_ERR_OR_NULL(acrtc))
		return false;

	crtc_id = acrtc->crtc_id;
	if (crtc_id >= DC_DVO_MAXLINK)
		return false;

	crtc_cfg = dc_readl(adev, CURRENT_REG(DC_CRTC_CFG_REG, crtc_id));
	crtc_cfg &= CRTC_CFG_MASK;
	if (enable && (crtc_cfg & CRTC_CFG_ENABLE))
		return true;

	if (!enable && (!(crtc_cfg & CRTC_CFG_ENABLE)))
		return true;

	crtc_pan = dc_readl(adev, CURRENT_REG(DC_CRTC_PANELCFG_REG, crtc_id));
	hsync_val = dc_readl(adev, CURRENT_REG(DC_CRTC_HSYNC_REG, crtc_id));
	vsync_val = dc_readl(adev, CURRENT_REG(DC_CRTC_VSYNC_REG, crtc_id));

	if (enable) {
		crtc_cfg |= CRTC_CFG_ENABLE;
		crtc_pan |= CRTC_PANCFG_DE;
		crtc_pan |= CRTC_PANCFG_CLKEN;
		hsync_val |= CRTC_HSYNC_POLSE;
		vsync_val |= CRTC_VSYNC_POLSE;

		lg_loongson_screen_state(enable);
		dc_writel(adev, CURRENT_REG(DC_CRTC_PANELCFG_REG, crtc_id), crtc_pan);
		dc_writel(adev, CURRENT_REG(DC_CRTC_HSYNC_REG, crtc_id), hsync_val);
		dc_writel(adev, CURRENT_REG(DC_CRTC_VSYNC_REG, crtc_id), vsync_val);
		dc_writel_check(adev, CURRENT_REG(DC_CRTC_CFG_REG, crtc_id), crtc_cfg);
	} else {
		crtc_cfg &= ~CRTC_CFG_ENABLE;
		crtc_pan &= ~CRTC_PANCFG_DE;
		crtc_pan &= ~CRTC_PANCFG_CLKEN;
		hsync_val &= ~CRTC_HSYNC_POLSE;
		vsync_val &= ~CRTC_VSYNC_POLSE;

		lg_loongson_screen_state(enable);
		dc_writel_check(adev, CURRENT_REG(DC_CRTC_CFG_REG, crtc_id), crtc_cfg);
		dc_writel(adev, CURRENT_REG(DC_CRTC_PANELCFG_REG, crtc_id), crtc_pan);
		dc_writel(adev, CURRENT_REG(DC_CRTC_HSYNC_REG, crtc_id), hsync_val);
		dc_writel(adev, CURRENT_REG(DC_CRTC_VSYNC_REG, crtc_id), vsync_val);
	}

	ls_bl = adev->mode_info.backlights[crtc_id];
	if (ls_bl && ls_bl->power)
		ls_bl->power(ls_bl, enable);

	return true;
}

bool dc_crtc_vblank_enable(struct gsgpu_dc_crtc *crtc, bool enable)
{
	u32 link;
	u32 value;
	struct gsgpu_device *adev = crtc->dc->adev;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	value = dc_readl(adev, DC_INT_REG);
	switch (link) {
	case 0:
		if (enable)
			value |= INT_VSYNC0_ENABLE;
		else
			value &= ~INT_VSYNC0_ENABLE;
		break;
	case 1:
		if (enable)
			value |= INT_VSYNC1_ENABLE;
		else
			value &= ~INT_VSYNC1_ENABLE;
		break;
	default:
		return false;
	}

	dc_writel(adev, DC_INT_REG, value);

	return true;
}

u32 dc_vblank_get_counter(struct gsgpu_device *adev, int crtc_num)
{
	if (crtc_num >= adev->mode_info.num_crtc)
		return 0;
	else {
		return dc_readl(adev, DC_VSYNC_COUNTER_REG + (0x10 * crtc_num));
	}

	return 0;
}

int dc_crtc_get_scanoutpos(struct gsgpu_device *adev, int crtc_num,
				  u32 *vbl, u32 *position)
{
	struct gsgpu_dc *dc = adev->dc;
	uint32_t v_blank_start, v_blank_end, h_position, v_position;
	int reg_val = 0;

	if (IS_ERR_OR_NULL(dc) || (crtc_num >= dc->links))
		return false;

	if (IS_ERR_OR_NULL(dc->link_info[crtc_num].crtc))
		return false;

	if ((crtc_num < 0) || (crtc_num >= adev->mode_info.num_crtc))
		return -EINVAL;
	else {
		reg_val = dc_readl(adev, DC_CRTC_DISPLAY_POS_REG + (0x10 * crtc_num));

		v_blank_start = 0;
		v_blank_end = 0;
		h_position = (reg_val & 0xffff);
		v_position = (reg_val >> 16);

		position = 0;
		vbl = 0;
	}

	return 0;
}

bool dc_crtc_vblank_ack(struct gsgpu_dc_crtc *crtc)
{
	u32 link;
	u32 value;
	struct gsgpu_device *adev = crtc->dc->adev;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	value = dc_readl(adev, DC_INT_REG);

	switch (link) {
	case 0:
		value |= INT_VSYNC0_ENABLE;
		break;
	case 1:
		value |= INT_VSYNC1_ENABLE;
		break;
	}

	dc_writel(adev, DC_INT_REG, value);

	return true;
}

static bool crtc_update_fb_address(struct gsgpu_dc_crtc *crtc,
				   union plane_address address)
{
	u32 link;
	struct gsgpu_device *adev;

	if (IS_ERR_OR_NULL(crtc))
		return false;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	adev = crtc->dc->adev;

	dc_writel(adev, CURRENT_REG(DC_CRTC_FBADDR0_LO_REG, link), address.low_part);
	dc_writel(adev, CURRENT_REG(DC_CRTC_FBADDR1_LO_REG, link), address.low_part);

	dc_writel(adev, CURRENT_REG(DC_CRTC_FBADDR0_HI_REG, link), address.high_part);
	dc_writel(adev, CURRENT_REG(DC_CRTC_FBADDR1_HI_REG, link), address.high_part);

	return true;
}

static bool crtc_primary_plane_set(struct gsgpu_dc_crtc *crtc,
					 struct dc_primary_plane *primary)
{
	if (IS_ERR_OR_NULL(crtc) || IS_ERR_OR_NULL(primary))
		return false;

	return crtc_update_fb_address(crtc, primary->address);
}

bool dc_crtc_plane_update(struct gsgpu_dc_crtc *crtc, struct dc_plane_update *update)
{
	bool ret;

	if (IS_ERR_OR_NULL(crtc) || IS_ERR_OR_NULL(update))
		return false;

	switch (update->type) {
	case DC_PLANE_CURSOR:
		ret = crtc_cursor_set(crtc, &update->cursor);
		break;
	case DC_PLANE_PRIMARY:
		ret = crtc_primary_plane_set(crtc, &update->primary);
		break;
	case DC_PLANE_OVERLAY:
	default:
		pr_err("%s 7A1000 not support overlay \n", __func__);
		ret = false;
		break;
	}

	return ret;
}

struct gsgpu_dc_crtc *dc_crtc_construct(struct gsgpu_dc *dc, struct crtc_resource *resource)
{
	struct gsgpu_dc_crtc *crtc;
	u32 link;

	if (IS_ERR_OR_NULL(dc) || IS_ERR_OR_NULL(resource))
		return NULL;

	crtc = kzalloc(sizeof(*crtc), GFP_KERNEL);
	crtc->timing = kzalloc(sizeof(struct dc_timing_info), GFP_KERNEL);

	if (IS_ERR_OR_NULL(crtc))
		return NULL;

	crtc->dc = dc;
	crtc->resource = resource;

	link = crtc->resource->base.link;
	if (link >= DC_DVO_MAXLINK)
		return false;

	list_add_tail(&crtc->node, &dc->crtc_list);

	return crtc;
}

void dc_crtc_destroy(struct gsgpu_dc_crtc *crtc)
{
	if (IS_ERR_OR_NULL(crtc))
		return;

	list_del(&crtc->node);
	kfree(crtc->timing);
	kfree(crtc);
	crtc = NULL;
}

static int crtc_helper_atomic_check(struct drm_crtc *crtc,
				    lg_atomic_check_state_arg *state)
{
	return 0;
}

static enum drm_mode_status gsgpu_dc_mode_valid(struct drm_crtc *crtc,
				const struct drm_display_mode *mode)
{
	struct gsgpu_device *adev = crtc->dev->dev_private;

	switch (adev->chip) {
	case dev_7a2000:
		if (mode->hdisplay > 4096)
			return MODE_BAD;
		if (mode->vdisplay > 2160)
			return MODE_BAD;
		if (mode->hdisplay == 1680)
			return MODE_BAD;
		if (mode->hdisplay == 1440)
			return MODE_BAD;
		break;
	case dev_2k2000:
		if (mode->hdisplay > 1920 && crtc->index == 1)
			return MODE_BAD;
		else if (mode->hdisplay > 4096)
			return MODE_BAD;
		if (mode->vdisplay > 1200 && crtc->index == 1)
			return MODE_BAD;
		else if (mode->vdisplay > 2160)
			return MODE_BAD;
		break;
	}

	if (mode->hdisplay % 8)
		return MODE_BAD;
	if (mode->clock > 340000)
		return MODE_CLOCK_HIGH;
	if (mode->vdisplay < 480)
		return MODE_BAD;

	return MODE_OK;
}

static bool gsgpu_crtc_helper_scanout_position(struct drm_crtc *crtc,
					bool in_vblank_irq, int *vpos,
					int *hpos, ktime_t *stime, ktime_t *etime,
					const struct drm_display_mode *mode)
{
	struct drm_device *dev = crtc->dev;
	unsigned int pipe = crtc->index;

	return gsgpu_display_get_crtc_scanoutpos(dev, pipe, 0, vpos, hpos, stime, etime, mode);
}

static const struct drm_crtc_helper_funcs gsgpu_dc_crtc_helper_funcs = {
	.atomic_check = crtc_helper_atomic_check,
	.mode_valid = gsgpu_dc_mode_valid,
	lg_get_scanout_position_setting(gsgpu_crtc_helper_scanout_position)
};

static inline int dc_set_vblank(struct drm_crtc *crtc, bool enable)
{
	enum dc_irq_source irq_source;
	struct gsgpu_crtc *acrtc = to_gsgpu_crtc(crtc);
	struct gsgpu_device *adev = crtc->dev->dev_private;

	irq_source = DC_IRQ_TYPE_VSYNC + acrtc->crtc_id;
	return dc_interrupt_enable(adev->dc, irq_source, enable) ? 0 : -EBUSY;
}

static int dc_enable_vblank(struct drm_crtc *crtc)
{
	return dc_set_vblank(crtc, true);
}

static void dc_disable_vblank(struct drm_crtc *crtc)
{
	dc_set_vblank(crtc, false);
}

static u32 gsgpu_get_vblank_counter_crtc(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	unsigned int pipe = crtc->index;

	return gsgpu_get_vblank_counter_kms(dev, pipe);
}

static const struct drm_crtc_funcs gsgpu_dc_crtc_funcs = {
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = dc_enable_vblank,
	.disable_vblank = dc_disable_vblank,
	.get_vblank_counter = gsgpu_get_vblank_counter_crtc
};

int gsgpu_dc_crtc_init(struct gsgpu_device *adev,
		       struct drm_plane *plane, uint32_t crtc_index)
{
	struct gsgpu_crtc *acrtc = NULL;
	struct gsgpu_plane *cursor_plane;
	int res = -ENOMEM;

	cursor_plane = kzalloc(sizeof(*cursor_plane), GFP_KERNEL);
	if (!cursor_plane)
		goto fail;

	cursor_plane->base.type = DRM_PLANE_TYPE_CURSOR;
	res = gsgpu_dc_plane_init(adev, cursor_plane, 0);

	acrtc = kzalloc(sizeof(struct gsgpu_crtc), GFP_KERNEL);
	if (!acrtc)
		goto fail;

	res = drm_crtc_init_with_planes(adev->ddev, &acrtc->base, plane,
			&cursor_plane->base, &gsgpu_dc_crtc_funcs, NULL);
	if (!res)
		acrtc->crtc_id = crtc_index;
	else {
		acrtc->crtc_id = -1;
		goto fail;
	}

	drm_crtc_helper_add(&acrtc->base, &gsgpu_dc_crtc_helper_funcs);

	mutex_init(&acrtc->cursor_lock);
	acrtc->max_cursor_width = 64;
	acrtc->max_cursor_height = 64;

	acrtc->irq_source_vsync = DC_IRQ_TYPE_VSYNC + crtc_index;
	acrtc->base.enabled = false;

	adev->mode_info.crtcs[crtc_index] = acrtc;

	gsgpu_hdmi_init(adev);

	return 0;

fail:
	kfree(acrtc);
	kfree(cursor_plane);
	return res;
}
