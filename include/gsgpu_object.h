#ifndef __GSGPU_OBJECT_H__
#define __GSGPU_OBJECT_H__

#include "gsgpu_drm.h"
#include "gsgpu.h"
#include "gsgpu_helper.h"
#if defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
#include <drm/ttm/ttm_range_manager.h>
#endif
#include "gsgpu_gtt_mgr_helper.h"

#define GSGPU_BO_INVALID_OFFSET	LONG_MAX
#define GSGPU_BO_MAX_PLACEMENTS	3

struct gsgpu_bo_param {
	unsigned long			size;
	int				byte_align;
	u32				domain;
	u32				preferred_domain;
	u64				flags;
	enum ttm_bo_type		type;
	lg_dma_resv_t	*resv;
};

/* User space allocated BO in a VM */
struct gsgpu_bo_va {
	struct gsgpu_vm_bo_base	base;

	/* protected by bo being reserved */
	unsigned			ref_count;

	/* all other members protected by the VM PD being reserved */
	struct dma_fence	        *last_pt_update;

	/* mappings for this bo_va */
	struct list_head		invalids;
	struct list_head		valids;

	/* If the mappings are cleared or filled */
	bool				cleared;
};

struct gsgpu_bo {
	/* Protected by tbo.reserved */
	u32				preferred_domains;
	u32				allowed_domains;
	struct ttm_place		placements[GSGPU_BO_MAX_PLACEMENTS];
	struct ttm_placement		placement;
	struct ttm_buffer_object	tbo;
	struct ttm_bo_kmap_obj		kmap;
	u64				flags;
	unsigned			pin_count;
	u64				tiling_flags;
	u64				node_offset;
	u64				metadata_flags;
	void				*metadata;
	u32				metadata_size;
	unsigned			prime_shared_count;
	bool                            is_other_gpu_share;
	/* list of all virtual address to which this bo is associated to */
	struct list_head		va;
	/* Constant after initialization */
	struct drm_gem_object		gem_base;
	struct gsgpu_bo		*parent;
	struct gsgpu_bo		*shadow;

	struct ttm_bo_kmap_obj		dma_buf_vmap;
	struct gsgpu_mn		*mn;

	union {
		struct list_head	mn_list;
		struct list_head	shadow_list;
	};
};

static inline struct gsgpu_bo *ttm_to_gsgpu_bo(struct ttm_buffer_object *tbo)
{
	return container_of(tbo, struct gsgpu_bo, tbo);
}

/**
 * gsgpu_mem_type_to_domain - return domain corresponding to mem_type
 * @mem_type:	ttm memory type
 *
 * Returns corresponding domain of the ttm mem_type
 */
static inline unsigned gsgpu_mem_type_to_domain(u32 mem_type)
{
	switch (mem_type) {
	case TTM_PL_VRAM:
		return GSGPU_GEM_DOMAIN_VRAM;
	case TTM_PL_TT:
		return GSGPU_GEM_DOMAIN_GTT;
	case TTM_PL_SYSTEM:
		return GSGPU_GEM_DOMAIN_CPU;
	default:
		break;
	}
	return 0;
}

/**
 * gsgpu_bo_reserve - reserve bo
 * @bo:		bo structure
 * @no_intr:	don't return -ERESTARTSYS on pending signal
 *
 * Returns:
 * -ERESTARTSYS: A wait for the buffer to become unreserved was interrupted by
 * a signal. Release all buffer reservations and return to user-space.
 */
static inline int gsgpu_bo_reserve(struct gsgpu_bo *bo, bool no_intr)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
	int r;

	r = ttm_bo_reserve(&bo->tbo, !no_intr, false, NULL);
	if (unlikely(r != 0)) {
		if (r != -ERESTARTSYS)
			dev_err(adev->dev, "%p reserve failed\n", bo);
		return r;
	}
	return 0;
}

static inline void gsgpu_bo_unreserve(struct gsgpu_bo *bo)
{
	ttm_bo_unreserve(&bo->tbo);
}

static inline unsigned long gsgpu_bo_size(struct gsgpu_bo *bo)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	return bo->tbo.base.size;
#else
	return bo->tbo.num_pages << PAGE_SHIFT;
#endif
}

static inline unsigned long lg_tbo_to_num_pages(struct ttm_buffer_object *tbo)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	return PFN_UP(tbo->base.size);
#else
	return tbo->num_pages;
#endif
}

static inline unsigned gsgpu_bo_ngpu_pages(struct gsgpu_bo *bo)
{
#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	return bo->tbo.base.size / GSGPU_GPU_PAGE_SIZE;
#else
	return (bo->tbo.num_pages << PAGE_SHIFT) / GSGPU_GPU_PAGE_SIZE;
#endif
}

#if defined(LG_TTM_BUFFER_OBJECT_HAS_BASE) || defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
#define to_dma_resv(bo) ((bo)->tbo.base.resv)
#else
#define to_dma_resv(bo) ((bo)->tbo.resv)
#endif

/**
 * gsgpu_bo_mmap_offset - return mmap offset of bo
 * @bo:	gsgpu object for which we query the offset
 *
 * Returns mmap offset of the object.
 */
static inline u64 gsgpu_bo_mmap_offset(struct gsgpu_bo *bo)
{
#if defined(LG_TTM_BUFFER_OBJECT_HAS_BASE) || defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
	return drm_vma_node_offset_addr(&bo->tbo.base.vma_node);
#else
	return drm_vma_node_offset_addr(&bo->tbo.vma_node);
#endif
}

/**
 * gsgpu_bo_gpu_accessible - return whether the bo is currently in memory that
 * is accessible to the GPU.
 */
static inline bool gsgpu_bo_gpu_accessible(struct gsgpu_bo *bo)
{
	switch (lg_get_bo_mem_type(bo)) {
	case TTM_PL_TT:
	#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
		return gsgpu_gtt_mgr_has_gart_addr(bo->tbo.resource);
	#else
		return gsgpu_gtt_mgr_has_gart_addr(&bo->tbo.mem);
	#endif
	case TTM_PL_VRAM: return true;
	default: return false;
	}
}

/**
 * gsgpu_bo_in_cpu_visible_vram - check if BO is (partly) in visible VRAM
 */
static inline bool gsgpu_bo_in_cpu_visible_vram(struct gsgpu_bo *bo)
{
	struct gsgpu_device *adev = gsgpu_ttm_adev(bo->tbo.bdev);
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	struct gsgpu_res_cursor cursor;
#else
	unsigned fpfn = adev->gmc.visible_vram_size >> PAGE_SHIFT;
	struct drm_mm_node *node = bo->tbo.mem.mm_node;
	unsigned long pages_left;
#endif

	if (!lg_tbo_to_mem(&bo->tbo) || lg_get_bo_mem_type(bo) != TTM_PL_VRAM)
		return false;

#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	gsgpu_res_first(bo->tbo.resource, 0, bo->tbo.base.size, &cursor);

	while (cursor.remaining) {
		if (cursor.start < adev->gmc.visible_vram_size)
			return true;

		gsgpu_res_next(&cursor, cursor.size);
	}
#else
	for (pages_left = bo->tbo.mem.num_pages; pages_left; pages_left -= node->size, node++)
		if (node->start < fpfn)
			return true;
#endif
	return false;
}

/**
 * gsgpu_bo_explicit_sync - return whether the bo is explicitly synced
 */
static inline bool gsgpu_bo_explicit_sync(struct gsgpu_bo *bo)
{
	return bo->flags & GSGPU_GEM_CREATE_EXPLICIT_SYNC;
}

bool gsgpu_bo_is_gsgpu_bo(struct ttm_buffer_object *bo);
void gsgpu_bo_placement_from_domain(struct gsgpu_bo *abo, u32 domain);

int gsgpu_bo_create(struct gsgpu_device *adev,
		     struct gsgpu_bo_param *bp,
		     struct gsgpu_bo **bo_ptr);
int gsgpu_bo_create_reserved(struct gsgpu_device *adev,
			      unsigned long size, int align,
			      u32 domain, struct gsgpu_bo **bo_ptr,
			      u64 *gpu_addr, void **cpu_addr);
int gsgpu_bo_create_kernel(struct gsgpu_device *adev,
			    unsigned long size, int align,
			    u32 domain, struct gsgpu_bo **bo_ptr,
			    u64 *gpu_addr, void **cpu_addr);
void gsgpu_bo_free_kernel(struct gsgpu_bo **bo, u64 *gpu_addr,
			   void **cpu_addr);
int gsgpu_bo_kmap(struct gsgpu_bo *bo, void **ptr);
void *gsgpu_bo_kptr(struct gsgpu_bo *bo);
void gsgpu_bo_kunmap(struct gsgpu_bo *bo);
struct gsgpu_bo *gsgpu_bo_ref(struct gsgpu_bo *bo);
void gsgpu_bo_unref(struct gsgpu_bo **bo);
int gsgpu_bo_pin(struct gsgpu_bo *bo, u32 domain);
int gsgpu_bo_pin_restricted(struct gsgpu_bo *bo, u32 domain,
			     u64 min_offset, u64 max_offset);
int gsgpu_bo_unpin(struct gsgpu_bo *bo);
int gsgpu_bo_evict_vram(struct gsgpu_device *adev);
int gsgpu_bo_init(struct gsgpu_device *adev);
int gsgpu_bo_late_init(struct gsgpu_device *adev);
void gsgpu_bo_fini(struct gsgpu_device *adev);
int gsgpu_bo_fbdev_mmap(struct gsgpu_bo *bo,
				struct vm_area_struct *vma);
int gsgpu_bo_set_tiling_flags(struct gsgpu_bo *bo, u64 tiling_flags);
void gsgpu_bo_get_tiling_flags(struct gsgpu_bo *bo, u64 *tiling_flags);
int gsgpu_bo_set_metadata (struct gsgpu_bo *bo, void *metadata,
			    uint32_t metadata_size, uint64_t flags);
int gsgpu_bo_get_metadata(struct gsgpu_bo *bo, void *buffer,
			   size_t buffer_size, uint32_t *metadata_size,
			   uint64_t *flags);
void gsgpu_bo_move_notify(struct ttm_buffer_object *bo,
			   bool evict,
			   lg_ttm_mem_t *new_mem);
int gsgpu_bo_fault_reserve_notify(struct ttm_buffer_object *bo);
void gsgpu_bo_fence(struct gsgpu_bo *bo, struct dma_fence *fence,
		     bool shared);
u64 gsgpu_bo_gpu_offset(struct gsgpu_bo *bo);
int gsgpu_bo_backup_to_shadow(struct gsgpu_device *adev,
			       struct gsgpu_ring *ring,
			       struct gsgpu_bo *bo,
			       lg_dma_resv_t *resv,
			       struct dma_fence **fence, bool direct);
int gsgpu_bo_validate(struct gsgpu_bo *bo);
int gsgpu_bo_restore_from_shadow(struct gsgpu_device *adev,
				  struct gsgpu_ring *ring,
				  struct gsgpu_bo *bo,
				  lg_dma_resv_t *resv,
				  struct dma_fence **fence,
				  bool direct);
uint32_t gsgpu_bo_get_preferred_pin_domain(struct gsgpu_device *adev,
					    uint32_t domain);

/*
 * sub allocation
 */

static inline uint64_t gsgpu_sa_bo_gpu_addr(struct gsgpu_sa_bo *sa_bo)
{
	return sa_bo->manager->gpu_addr + sa_bo->soffset;
}

static inline void *gsgpu_sa_bo_cpu_addr(struct gsgpu_sa_bo *sa_bo)
{
	return sa_bo->manager->cpu_ptr + sa_bo->soffset;
}

int gsgpu_sa_bo_manager_init(struct gsgpu_device *adev,
				     struct gsgpu_sa_manager *sa_manager,
				     unsigned size, u32 align, u32 domain);
void gsgpu_sa_bo_manager_fini(struct gsgpu_device *adev,
				      struct gsgpu_sa_manager *sa_manager);
int gsgpu_sa_bo_manager_start(struct gsgpu_device *adev,
				      struct gsgpu_sa_manager *sa_manager);
int gsgpu_sa_bo_new(struct gsgpu_sa_manager *sa_manager,
		     struct gsgpu_sa_bo **sa_bo,
		     unsigned size, unsigned align);
void gsgpu_sa_bo_free(struct gsgpu_device *adev,
			      struct gsgpu_sa_bo **sa_bo,
			      struct dma_fence *fence);
#if defined(CONFIG_DEBUG_FS)
void gsgpu_sa_bo_dump_debug_info(struct gsgpu_sa_manager *sa_manager,
					 struct seq_file *m);
#endif

#endif /* __GSGPU_OBJECT_H__ */
