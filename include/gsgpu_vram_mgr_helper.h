#ifndef __GSGPU_VRAM_MGR_HELPER_H__
#define __GSGPU_VRAM_MGR_HELPER_H__

#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
static inline void lg_drm_buddy_free_list(struct drm_buddy *mm,
					struct list_head *objects,
					unsigned int flags)
{
#if defined(LG_DRM_BUDDY_FREE_LIST_HAS_FLAGS)
	drm_buddy_free_list(mm, objects, flags);
#else
	drm_buddy_free_list(mm, objects);
#endif
}
static inline void lg_drm_buddy_block_trim(struct drm_buddy *mm,
					   u64 *start,
					   u64 new_size,
					   struct list_head *blocks)
{
#if defined(LG_DRM_BUDDY_BLOCK_TRIM_HAS_START)
	drm_buddy_block_trim(mm, start, new_size, blocks);
#else
	drm_buddy_block_trim(mm, new_size, blocks);
#endif
}
int gsgpu_vram_mgr_new_buddy(struct ttm_resource_manager *man,
			     struct ttm_buffer_object *tbo,
			     const struct ttm_place *place,
			     struct ttm_resource **res);
void gsgpu_vram_mgr_del_buddy(struct ttm_resource_manager *man,
				struct ttm_resource *res);
void gsgpu_vram_mgr_debug_buddy(struct ttm_resource_manager *man,
				struct drm_printer *printer);
bool gsgpu_vram_mgr_intersects(struct ttm_resource_manager *man,
				struct ttm_resource *res,
				const struct ttm_place *place,
				size_t size);
bool gsgpu_vram_mgr_compatible(struct ttm_resource_manager *man,
				struct ttm_resource *res,
				const struct ttm_place *place,
				size_t size);

#define lg_vram_mgr_get_usage_size	struct gsgpu_vram_mgr_resource *vres = to_gsgpu_vram_mgr_resource(mem); \
					struct drm_buddy_block *block; \
					list_for_each_entry(block, &vres->blocks, link) \
						usage += gsgpu_vram_mgr_vis_size(adev, block);
#else
#define lg_vram_mgr_get_usage_size	struct drm_mm_node *nodes = mem->mm_node; \
					unsigned pages = mem->num_pages; \
					for (usage = 0; nodes && pages; pages -= nodes->size, nodes++) \
						usage += gsgpu_vram_mgr_vis_size(adev, nodes);
#endif

#if defined(LG_TTM_RESOURCE_MANAGER)
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
#define lg_vram_mgr_func_setting	.alloc = gsgpu_vram_mgr_new_buddy, \
					.free  = gsgpu_vram_mgr_del_buddy, \
					.intersects = gsgpu_vram_mgr_intersects, \
					.compatible = gsgpu_vram_mgr_compatible, \
					.debug = gsgpu_vram_mgr_debug_buddy
#else
#define lg_vram_mgr_func_setting	.alloc = gsgpu_vram_mgr_new, \
					.free  = gsgpu_vram_mgr_del, \
					.debug = gsgpu_vram_mgr_debug
#endif
#else
#define lg_vram_mgr_func_setting	.init = gsgpu_vram_mgr_init, \
					.takedown = gsgpu_vram_mgr_fini, \
					.get_node = gsgpu_vram_mgr_new, \
					.put_node = gsgpu_vram_mgr_del, \
					.debug    = gsgpu_vram_mgr_debug
#endif

static inline struct gsgpu_vram_mgr *lg_man_to_vram_mgr(lg_ttm_manager_t *man)
{
#if defined(LG_TTM_RESOURCE_MANAGER)
	return container_of(man, struct gsgpu_vram_mgr, manager);
#else
	return man->priv;
#endif
}

static inline struct gsgpu_device *vram_mgr_to_gsgpu_device(struct gsgpu_vram_mgr *mgr)
{
#if defined(LG_TTM_RESOURCE_MANAGER)
	return container_of(mgr, struct gsgpu_device, mman.vram_mgr);
#endif
	return NULL;
}

static inline struct gsgpu_device *lg_vram_mgr_man_to_gsgpu(lg_ttm_manager_t *man, struct gsgpu_vram_mgr *mgr)
{
#if !defined(LG_TTM_RESOURCE_MANAGER)
	return gsgpu_ttm_adev(man->bdev);
#else
	return vram_mgr_to_gsgpu_device(mgr);
#endif
}

#if defined(LG_TTM_RESOURCE_MANAGER)
#define lg_vram_mgr_init_mgr(man, mgr)	mgr = container_of(man, struct gsgpu_vram_mgr, manager);
#else
#define lg_vram_mgr_init_mgr(man, mgr)	mgr = kzalloc(sizeof(*mgr), GFP_KERNEL); \
					if (!mgr) \
						return -ENOMEM;
#endif

static inline int lg_vram_mgr_init(struct gsgpu_device *adev)
{
#if defined(LG_TTM_BO_INIT_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return ttm_bo_init_mm(&adev->mman.bdev, TTM_PL_VRAM,
			adev->gmc.real_vram_size >> PAGE_SHIFT);
#else
	return gsgpu_vram_mgr_init_buddy(adev);
#endif
}

static inline void lg_vram_mgr_init_man_func(lg_ttm_manager_t *man, struct gsgpu_device *adev, u64 size,
					     const lg_ttm_mem_func_t *func)
{
#if !(defined(LG_TTM_BO_INIT_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT))
	man->func = func;
#if defined(LG_TTM_RESOURCE_MANAGER_INIT_HAS_BDEV)
	ttm_resource_manager_init(man, &adev->mman.bdev, size);
#else
	ttm_resource_manager_init(man, size);
#endif
#endif
}

static inline void lg_vram_mgr_init_drv_ptr_used(lg_ttm_manager_t *man, struct gsgpu_vram_mgr *mgr)
{
#if defined(LG_TTM_BO_INIT_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	man->priv = mgr;
#else
	ttm_set_driver_manager(&container_of(mgr, struct gsgpu_device, mman.vram_mgr)->mman.bdev,
				TTM_PL_VRAM, &mgr->manager);
	ttm_resource_manager_set_used(man, true);
#endif
}

static inline void lg_gsgpu_vram_mgr_fini(struct gsgpu_device *adev)
{
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	gsgpu_vram_mgr_fini_buddy(adev);
#endif
}

static inline int lg_vram_mgr_fini_clean(lg_ttm_manager_t *man, struct gsgpu_vram_mgr *mgr)
{
#if defined(LG_TTM_BO_CLEAN_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	return 0;
#else
	ttm_resource_manager_set_used(man, false);
	#if defined(LG_TTM_RESOURCE_MANAGER_EVICT_ALL)
		return ttm_resource_manager_evict_all(&vram_mgr_to_gsgpu_device(mgr)->mman.bdev, man);
	#else
		return ttm_resource_manager_force_list_clean(&vram_mgr_to_gsgpu_device(mgr)->mman.bdev, man);
	#endif
#endif
}

static inline void lg_vram_mgr_fini_free(lg_ttm_manager_t *man, struct gsgpu_vram_mgr *mgr)
{
#if defined(LG_TTM_BO_CLEAN_MM) && defined(LG_DRM_TTM_TTM_BO_API_H_PRESENT)
	kfree(mgr);
	man->priv = NULL;
#else
	ttm_resource_manager_cleanup(man);
	ttm_set_driver_manager(&vram_mgr_to_gsgpu_device(mgr)->mman.bdev, TTM_PL_VRAM, NULL);
#endif
}

static inline void lg_vram_init_mgr(struct gsgpu_vram_mgr *mgr)
{
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	mutex_init(&mgr->m_lock);
	INIT_LIST_HEAD(&mgr->reservations_pending);
	INIT_LIST_HEAD(&mgr->reserved_pages);
	mgr->default_page_size = PAGE_SIZE;
#else
	spin_lock_init(&mgr->lock);
#endif
}

static inline int lg_vram_init_mgr_mm(struct gsgpu_vram_mgr *mgr, u64 start, u64 size)
{
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	return drm_buddy_init(&mgr->b_mm, size, PAGE_SIZE);
#else
	drm_mm_init(&mgr->mm, start, size);
	return 0;
#endif
}

static inline void lg_vram_mm_fini(struct gsgpu_vram_mgr *mgr)
{
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	mutex_lock(&mgr->m_lock);
	drm_buddy_fini(&mgr->b_mm);
	mutex_unlock(&mgr->m_lock);
#else
	spin_lock(&mgr->lock);
	drm_mm_takedown(&mgr->mm);
	spin_unlock(&mgr->lock);
#endif
}

#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
#define lg_vram_mgr_vis_size_arg struct drm_buddy_block *block
#define lg_vram_mgr_vis_size_start gsgpu_vram_mgr_block_start(block)
#define lg_vram_mgr_vis_size_end start + gsgpu_vram_mgr_block_size(block)
#else
#define lg_vram_mgr_vis_size_arg struct drm_mm_node *node
#define lg_vram_mgr_vis_size_start node->start << PAGE_SHIFT
#define lg_vram_mgr_vis_size_end (node->size + node->start) << PAGE_SHIFT;
#endif

#if defined(LG_DRM_TTM_TTM_BO_H_PRESENT)
#define lg_bo_to_mem bo->tbo.resource
#else
#define lg_bo_to_mem &bo->tbo.mem
#endif
u64 gsgpu_vram_mgr_vis_size(struct gsgpu_device *adev,
				lg_vram_mgr_vis_size_arg);
#endif
