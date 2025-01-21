#include <linux/delay.h>
#include "gsgpu.h"
#include "gsgpu_dc.h"
#include "gsgpu_dc_hdmi.h"
#include "gsgpu_dc_crtc.h"
#include "gsgpu_dc_reg.h"
#include "gsgpu_helper.h"

void hdmi_phy_pll_config(struct gsgpu_device *adev, int index, int clock)
{
	int val;
	int count = 0;

	if (adev->chip == dev_2k2000 && index == 1)
		return;

	dc_writel(adev, CURRENT_REG(DC_HDMI_PHY_PLLCFG_REG, index), 0x0);

	if (clock >= 170000)
		val = (0x0 << 13) | (0x28 << 6) | (0x10 << 1) | (0 << 0);
	else if (clock >= 85000  && clock < 170000)
		val = (0x1 << 13) | (0x28 << 6) | (0x8 << 1) | (0 << 0);
	else if (clock >= 42500 && clock < 85000)
		val = (0x2 << 13) | (0x28 << 6) | (0x4 << 1) | (0 << 0);
	else if (clock >= 21250 && clock < 42500)
		val = (0x3 << 13) | (0x28 << 6) | (0x2 << 1) | (0 << 0);

	dc_writel(adev, CURRENT_REG(DC_HDMI_PHY_PLLCFG_REG, index), val);
	mdelay(5);
	val |= (1 << 0);
	dc_writel(adev, CURRENT_REG(DC_HDMI_PHY_PLLCFG_REG, index), val);

	/* wait pll lock */
	while (!(dc_readl(adev, CURRENT_REG(DC_HDMI_PHY_PLLCFG_REG, index)) & 0x10000)) {
		count++;
		if (count >= 1000) {
			DRM_ERROR("GSGPU HDMI PHY PLL lock failed\n");
			return;
		}
	}
}

bool gsgpu_hdmi_enable(struct gsgpu_device *adev, u32 link, bool enable)
{
	u32 hdmi_ctrl, hdmi_phy;

	if (IS_ERR_OR_NULL(adev))
		return false;

	if (link >= DC_DVO_MAXLINK)
		return false;

	hdmi_phy = 0xf02;
	hdmi_phy |= dc_readl(adev, CURRENT_REG(DC_HDMI_PHY_CTRL_REG, link));
	if (enable && (hdmi_phy & HDMI_PHY_CTRL_ENABLE))
		return true;

	if (!enable && (!(hdmi_phy & HDMI_PHY_CTRL_ENABLE)))
		return true;

	if (enable) {
		hdmi_phy |= HDMI_PHY_CTRL_ENABLE;
		hdmi_ctrl = dc_readl(adev, DC_HDMI_CTRL_REG + (link * 0x10));
		hdmi_ctrl |= HDMI_CTRL_ENABLE;
		dc_writel(adev, CURRENT_REG(DC_HDMI_CTRL_REG, link), hdmi_ctrl);
	} else {
		hdmi_ctrl = dc_readl(adev, DC_HDMI_CTRL_REG + (link * 0x10));
		hdmi_ctrl &= ~HDMI_CTRL_ENABLE;
		dc_writel(adev, CURRENT_REG(DC_HDMI_CTRL_REG, link), hdmi_ctrl);
		msleep(50);
		hdmi_phy &= ~HDMI_PHY_CTRL_ENABLE;
	}

	dc_writel(adev, CURRENT_REG(DC_HDMI_PHY_CTRL_REG, link), hdmi_phy);

	return true;
}

void gsgpu_hdmi_suspend(struct gsgpu_device *adev)
{
	u32 link = 0;

	for (link = 0; link < 2; link++) {
		adev->dc->hdmi_ctrl_reg[link] = dc_readl(adev, DC_HDMI_CTRL_REG + (link * 0x10));
	}
}


int gsgpu_hdmi_resume(struct gsgpu_device *adev)
{
	u32 link = 0;

	for (link = 0; link < 2; link++) {
		dc_writel(adev, DC_HDMI_CTRL_REG + (link * 0x10), adev->dc->hdmi_ctrl_reg[link]);
		if (adev->dc->hdmi_ctrl_reg[link] & HDMI_CTRL_AUDIO_ENABLE) {
			dc_writel(adev, DC_HDMI_ZONEIDLE_REG + (link * 0x10), 0x00400040);

			dc_writel(adev, DC_HDMI_AUDIO_NCFG_REG + (link * 0x10), 6272);
			dc_writel(adev, DC_HDMI_AUDIO_CTSCFG_REG + (link * 0x10), 0x80000000);
			dc_writel(adev, DC_HDMI_AUDIO_INFOFRAME_REG + (link * 0x10), 0x15);
			dc_writel(adev, DC_HDMI_AUDIO_SAMPLE_REG + (link * 0x10), 0x1);
		}
	}

	return 0;
}

int gsgpu_hdmi_init(struct gsgpu_device *adev)
{
	u32 link;

	for (link = 0; link < 2; link++) {
		dc_writel(adev, DC_HDMI_CTRL_REG + (link * 0x10), 0x288);
		dc_writel(adev, DC_HDMI_PHY_CTRL_REG + (link * 0x10), 0xf02);
		dc_writel(adev, DC_HDMI_ZONEIDLE_REG + (link * 0x10), 0x00400040);
	}

	return 0;
}

int gsgpu_hdmi_audio_init(struct gsgpu_device *adev, u32 link)
{
	u32 val;

	val = dc_readl(adev, DC_HDMI_CTRL_REG + (link * 0x10));
	if ((val & 0x2) && (val & 0x4))
		return 0;

	/* enable hdmi audio, but don't touch EN bit which will be
	 * update in gsgpu_hdmi_enable() */
	dc_writel(adev, DC_HDMI_CTRL_REG + (link * 0x10), 0x286 | (val & 0x1));

	/* Audio N 
	 * 44.1KHz * 4, dynamic update N && CTS value */
	dc_writel(adev, DC_HDMI_AUDIO_NCFG_REG + (link * 0x10), 6272);

	/* Enable Send CTS */
	dc_writel(adev, DC_HDMI_AUDIO_CTSCFG_REG + (link * 0x10), 0x80000000);

	/* Audio AIF
	 * enable AIF,set freq,and set CC = 1, CA = 0
	 * Update AIF */
	dc_writel(adev, DC_HDMI_AUDIO_INFOFRAME_REG + (link * 0x10), 0x15);

	/* Audio Sample Packet */
	dc_writel(adev, DC_HDMI_AUDIO_SAMPLE_REG + (link * 0x10), 0x1);

	return 0;
}
