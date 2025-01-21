#ifndef __GSGPU_SCHED_H__
#define __GSGPU_SCHED_H__

#include "gsgpu.h"

int gsgpu_to_sched_priority(int gsgpu_priority, enum drm_sched_priority *prio);
int gsgpu_sched_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *filp);

#endif /* __GSGPU_SCHED_H__ */
