#include <linux/compat.h>

#include "gsgpu.h"
#include "gsgpu_drm.h"
#include "gsgpu_drv.h"

long gsgpu_kms_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	unsigned int nr = DRM_IOCTL_NR(cmd);
	int ret;

	if (nr < DRM_COMMAND_BASE)
		return 0;

	ret = gsgpu_drm_ioctl(filp, cmd, arg);

	return ret;
}
