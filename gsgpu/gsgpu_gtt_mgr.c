#include "gsgpu.h"
#include "gsgpu_gtt_mgr_helper.h"

/**
 * gsgpu_gtt_mgr_init - init GTT manager and DRM MM
 *
 * @man: TTM memory type manager
 * @p_size: maximum size of GTT
 *
 * Allocate and initialize the GTT manager.
 */
int gsgpu_gtt_mgr_init(lg_ttm_manager_t *man,
			       unsigned long p_size)
{
	struct gsgpu_device *adev;
	struct gsgpu_gtt_mgr *mgr;
	uint64_t start, size;

	lg_gtt_mgr_init_mgr(man, mgr);

        adev = lg_gtt_mgr_man_to_adev(man, mgr);
	lg_gtt_mgr_func_setting(man, adev, &gsgpu_gtt_mgr_func, p_size);
	start = GSGPU_GTT_MAX_TRANSFER_SIZE * GSGPU_GTT_NUM_TRANSFER_WINDOWS;
	size = (adev->gmc.gart_size >> PAGE_SHIFT) - start;
	drm_mm_init(&mgr->mm, start, size);
	spin_lock_init(&mgr->lock);
	atomic64_set(&mgr->available, p_size);
	lg_gtt_mgr_set_man_priv(man, mgr);
	lg_gtt_mgr_init_set_man_and_used(man, &adev->mman.bdev);
	return 0;
}

/**
 * gsgpu_gtt_mgr_fini - free and destroy GTT manager
 *
 * @man: TTM memory type manager
 *
 * Destroy and free the GTT manager, returns -EBUSY if ranges are still
 * allocated inside it.
 */
int gsgpu_gtt_mgr_fini(lg_ttm_manager_t *man)
{
	int ret;
	struct gsgpu_gtt_mgr *mgr = lg_man_to_gtt_mgr(man);
	struct gsgpu_device *adev = lg_gtt_mgr_man_to_adev(man, mgr);

	ret = lg_gtt_mgr_fini_set_used_clean_list(man, &adev->mman.bdev);
	if (ret)
		return ret;

	spin_lock(&mgr->lock);
	drm_mm_takedown(&mgr->mm);
	spin_unlock(&mgr->lock);
	lg_gtt_mgr_fini_free_mgr(mgr);
	lg_gtt_mgr_set_man_priv(man, NULL);
	lg_gtt_mgr_fini_clean_up(man, &adev->mman.bdev);
	return 0;
}

/**
 * gsgpu_gtt_mgr_has_gart_addr - Check if mem has address space
 *
 * @mem: the mem object to check
 *
 * Check if a mem object has already address space allocated.
 */
bool gsgpu_gtt_mgr_has_gart_addr(lg_ttm_mem_t *mem)
{
#if defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	if (mem->mem_type != TTM_PL_TT)
		return true;

	struct ttm_range_mgr_node *node = to_ttm_range_mgr_node(mem);

        return drm_mm_node_allocated(&node->mm_nodes[0]);
#else
	struct drm_mm_node *node = lg_res_to_drm_node(mem);
	return (node->start != GSGPU_BO_INVALID_OFFSET);
#endif
}

/**
 * gsgpu_gtt_mgr_alloc - allocate new ranges
 *
 * @man: TTM memory type manager
 * @tbo: TTM BO we need this range for
 * @place: placement flags and restrictions
 * @mem: the resulting mem object
 *
 * Allocate the address space for a node.
 */
static int gsgpu_gtt_mgr_alloc(lg_ttm_manager_t *man,
				struct ttm_buffer_object *tbo,
				const struct ttm_place *place,
				lg_ttm_mem_t *mem)
{
	struct gsgpu_gtt_mgr *mgr = lg_man_to_gtt_mgr(man);
	struct gsgpu_device *adev = lg_gtt_mgr_man_to_adev(man, mgr);
	lg_gsgpu_gtt_node *node = lg_res_to_gtt_node(mem);
	enum drm_mm_insert_mode mode;
	unsigned long fpfn, lpfn;
	int r;

	if (gsgpu_gtt_mgr_has_gart_addr(mem))
		return 0;

	if (place)
		fpfn = place->fpfn;
	else
		fpfn = 0;

	if (place && place->lpfn)
		lpfn = place->lpfn;
	else
		lpfn = adev->gart.num_cpu_pages;

	mode = DRM_MM_INSERT_BEST;
	if (place && place->flags & TTM_PL_FLAG_TOPDOWN)
		mode = DRM_MM_INSERT_HIGH;

	spin_lock(&mgr->lock);
	r = drm_mm_insert_node_in_range(&mgr->mm, lg_gtt_node_to_drm_node(node), lg_gtt_get_pages(tbo, mem),
					lg_tbo_get_page_alignment(tbo), 0, fpfn, lpfn,
					mode);
	spin_unlock(&mgr->lock);

	if (!r)
		mem->start = lg_gtt_node_to_drm_node(node)->start;

	return r;
}

/**
 * gsgpu_gtt_mgr_new - allocate a new node
 *
 * @man: TTM memory type manager
 * @tbo: TTM BO we need this range for
 * @place: placement flags and restrictions
 * @mem: the resulting mem object
 *
 * Dummy, allocate the node but no space for it yet.
 */
static int gsgpu_gtt_mgr_new(lg_ttm_manager_t *man,
			      struct ttm_buffer_object *tbo,
			      const struct ttm_place *place,
			      lg_gtt_mgr_new_mem_reg)
{
#if defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	struct gsgpu_gtt_mgr *mgr = container_of(man, struct gsgpu_gtt_mgr, manager);
	uint32_t num_pages = PFN_UP(tbo->base.size);
	struct ttm_range_mgr_node *node;
	int r;

	node = kzalloc(struct_size(node, mm_nodes, 1), GFP_KERNEL);
	if (!node)
		return -ENOMEM;

	ttm_resource_init(tbo, place, &node->base);
	if (!(place->flags & TTM_PL_FLAG_TEMPORARY) &&
	   ttm_resource_manager_usage(man) > man->size) {
		r = -ENOSPC;
		goto err_free;
	}

	if (place->lpfn) {
		spin_lock(&mgr->lock);
		r = drm_mm_insert_node_in_range(&mgr->mm, &node->mm_nodes[0],
						num_pages, tbo->page_alignment,
						0, place->fpfn, place->lpfn,
						DRM_MM_INSERT_BEST);
		spin_unlock(&mgr->lock);
		if (unlikely(r))
			goto err_free;

		node->base.start = node->mm_nodes[0].start;
	} else {
		node->mm_nodes[0].start = 0;
		node->mm_nodes[0].size = PFN_UP(node->base.size);
		node->base.start = GSGPU_BO_INVALID_OFFSET;
	}

	*mem = &node->base;
	return 0;

err_free:
	ttm_resource_fini(man, &node->base);
	kfree(node);
	return r;
#else
	struct gsgpu_gtt_mgr *mgr = lg_man_to_gtt_mgr(man);
	lg_gsgpu_gtt_node *gtt_node;
	struct drm_mm_node *mm_node;
	int r;
	uint32_t num_pages = lg_gtt_get_pages(tbo, mem);

	spin_lock(&mgr->lock);
	if ((lg_gtt_mgr_new_check_mem || lg_tbo_to_mem(tbo)->mem_type != TTM_PL_TT) &&
		atomic64_read(&mgr->available) < num_pages) {
		spin_unlock(&mgr->lock);
		return LG_GTT_MGR_NEW_RET_VAL;
	}
	atomic64_sub(num_pages, &mgr->available);
	spin_unlock(&mgr->lock);

	lg_alloc_gtt_node(gtt_node);
	if (!gtt_node) {
		r = -ENOMEM;
		goto err_out;
	}

	lg_ttm_resource_init(tbo, place, gtt_node);
	mm_node = lg_gtt_node_to_drm_node(gtt_node);
	mm_node->start = GSGPU_BO_INVALID_OFFSET;
	mm_node->size = num_pages;
	lg_gtt_node_set_tbo(tbo)
	lg_gtt_node_set_mm_node(gtt_node)

	if (place->fpfn || place->lpfn || place->flags & TTM_PL_FLAG_TOPDOWN) {
		r = gsgpu_gtt_mgr_alloc(man, tbo, place, lg_gtt_mgr_alloc_arg);
		if (unlikely(r)) {
			kfree(gtt_node);
			lg_gtt_node_set_mm_node(NULL)
			r = 0;
			goto err_out;
		}
	} else {
		lg_gtt_node_set_mm_start(mm_node->start)
	}

	lg_gtt_mgr_set_res_ptr
	return 0;
err_out:
	atomic64_add(num_pages, &mgr->available);
	lg_ttm_resource_fini(man, gtt_node);
	return r;
#endif
}

/**
 * gsgpu_gtt_mgr_del - free ranges
 *
 * @man: TTM memory type manager
 * @tbo: TTM BO we need this range for
 * @place: placement flags and restrictions
 * @mem: TTM memory object
 *
 * Free the allocated GTT again.
 */
static void gsgpu_gtt_mgr_del(lg_ttm_manager_t *man,
			       lg_ttm_mem_t *mem)
{
	struct gsgpu_gtt_mgr *mgr = lg_man_to_gtt_mgr(man);
	lg_gsgpu_gtt_node *node = lg_res_to_gtt_node(mem);
	struct drm_mm_node *mm_node;

	if (!node)
		return;
#if defined(LG_DRM_TTM_TTM_RANGE_MANAGER_H_PRESENT)
	spin_lock(&mgr->lock);
	if (drm_mm_node_allocated(&node->mm_nodes[0]))
		drm_mm_remove_node(&node->mm_nodes[0]);
	spin_unlock(&mgr->lock);

	ttm_resource_fini(man, mem);
	kfree(node);
#else
	mm_node = lg_gtt_node_to_drm_node(node);
	spin_lock(&mgr->lock);
	if (mm_node->start != GSGPU_BO_INVALID_OFFSET)
		drm_mm_remove_node(mm_node);
	spin_unlock(&mgr->lock);
	atomic64_add(lg_gtt_del_mgr_num_pages, &mgr->available);
	lg_ttm_resource_fini(man, node);
	kfree(node);
	lg_gtt_node_set_mm_node(NULL)
#endif
}

/**
 * gsgpu_gtt_mgr_usage - return usage of GTT domain
 *
 * @man: TTM memory type manager
 *
 * Return how many bytes are used in the GTT domain
 */
uint64_t gsgpu_gtt_mgr_usage(lg_ttm_manager_t *man)
{
#if defined(LG_DRM_DRM_BUDDY_H_PRESENT)
	return ttm_resource_manager_usage(man);
#else
	struct gsgpu_gtt_mgr *mgr = lg_man_to_gtt_mgr(man);
	s64 result = man->size - atomic64_read(&mgr->available);

	return (result > 0 ? result : 0) * PAGE_SIZE;
#endif
}

int gsgpu_gtt_mgr_recover(lg_ttm_manager_t *man)
{
	struct gsgpu_gtt_mgr *mgr = lg_man_to_gtt_mgr(man);
	lg_gsgpu_gtt_node *node;
	struct drm_mm_node *mm_node;
	int r = 0;

	spin_lock(&mgr->lock);
	drm_mm_for_each_node(mm_node, &mgr->mm) {
		node = lg_drm_node_to_gtt_node(mm_node);
		r = gsgpu_ttm_recover_gart(lg_gtt_node_to_tbo(node));
		if (r)
			break;
	}
	spin_unlock(&mgr->lock);

	return r;
}

/**
 * gsgpu_gtt_mgr_debug - dump VRAM table
 *
 * @man: TTM memory type manager
 * @printer: DRM printer to use
 *
 * Dump the table content using printk.
 */
static void gsgpu_gtt_mgr_debug(lg_ttm_manager_t *man,
				 struct drm_printer *printer)
{
	struct gsgpu_gtt_mgr *mgr = lg_man_to_gtt_mgr(man);

	spin_lock(&mgr->lock);
	drm_mm_print(&mgr->mm, printer);
	spin_unlock(&mgr->lock);

	drm_printf(printer, "man size:%llu pages, gtt available:%lld pages, usage:%lluMB\n",
		   man->size, (u64)atomic64_read(&mgr->available),
		   gsgpu_gtt_mgr_usage(man) >> 20);
}

const lg_ttm_mem_func_t gsgpu_gtt_mgr_func = {
	lg_gtt_mgr_set_funcs
};
