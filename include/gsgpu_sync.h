#ifndef __GSGPU_SYNC_H__
#define __GSGPU_SYNC_H__

#include <linux/hashtable.h>

#include "gsgpu_dma_resv_helper.h"

struct dma_fence;
struct gsgpu_device;
struct gsgpu_ring;
struct dma_resv;
struct gsgpu_job;

enum gsgpu_sync_mode {
	GSGPU_SYNC_ALWAYS,
	GSGPU_SYNC_NE_OWNER,
	GSGPU_SYNC_EQ_OWNER,
	GSGPU_SYNC_EXPLICIT
};

/*
 * Container for fences used to sync command submissions.
 */
struct gsgpu_sync {
	DECLARE_HASHTABLE(fences, 4);
	struct dma_fence	*last_vm_update;
};

void gsgpu_sync_create(struct gsgpu_sync *sync);
int gsgpu_sync_fence(struct gsgpu_device *adev, struct gsgpu_sync *sync,
		      struct dma_fence *f, bool explicit);
int gsgpu_sync_resv(struct gsgpu_device *adev,
		     struct gsgpu_sync *sync,
		     lg_dma_resv_t *resv,
		     enum gsgpu_sync_mode mode,
		     void *owner,
		     bool explicit_sync);
struct dma_fence *gsgpu_sync_peek_fence(struct gsgpu_sync *sync,
				     struct gsgpu_ring *ring);
struct dma_fence *gsgpu_sync_get_fence(struct gsgpu_sync *sync, bool *explicit);
int gsgpu_sync_clone(struct gsgpu_sync *source, struct gsgpu_sync *clone);
int gsgpu_sync_wait(struct gsgpu_sync *sync, bool intr);
void gsgpu_sync_free(struct gsgpu_sync *sync);
int gsgpu_sync_init(void);
void gsgpu_sync_fini(void);

#endif /* __GSGPU_SYNC_H__ */
