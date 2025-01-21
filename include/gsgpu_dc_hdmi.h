#ifndef __GSGPU_HDMI_H__
#define __GSGPU_HDMI_H__

int gsgpu_hdmi_audio_init(struct gsgpu_device *adev, u32 link);
int gsgpu_hdmi_init(struct gsgpu_device *adev);
void gsgpu_hdmi_suspend(struct gsgpu_device *adev);
int gsgpu_hdmi_resume(struct gsgpu_device *adev);
bool gsgpu_hdmi_enable(struct gsgpu_device *adev, u32 link, bool enable);
void hdmi_phy_pll_config(struct gsgpu_device *adev, int index, int clock);

#endif /* __GSGPU_HDMI_H__ */
