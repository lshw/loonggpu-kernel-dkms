#include <linux/pci.h>
#include "gsgpu.h"
#include "gsgpu_drm.h"

/*
 * GART
 * The GART (Graphics Aperture Remapping Table) is an aperture
 * in the GPU's address space.  System pages can be mapped into
 * the aperture and look like contiguous pages from the GPU's
 * perspective.  A page table maps the pages in the aperture
 * to the actual backing pages in system memory.
 *
 * GSGPU GPUs support both an internal GART, as described above,
 * and AGP.  AGP works similarly, but the GART table is configured
 * and maintained by the northbridge rather than the driver.
 * GSGPU hw has a separate AGP aperture that is programmed to
 * point to the AGP aperture provided by the northbridge and the
 * requests are passed through to the northbridge aperture.
 * Both AGP and internal GART can be used at the same time, however
 * that is not currently supported by the driver.
 *
 * This file handles the common internal GART management.
 */

/*
 * Common GART table functions.
 */

/**
 * gsgpu_dummy_page_init - init dummy page used by the driver
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate the dummy page used by the driver (all asics).
 * This dummy page is used by the driver as a filler for gart entries
 * when pages are taken out of the GART
 * Returns 0 on sucess, -ENOMEM on failure.
 */
static int gsgpu_gart_dummy_page_init(struct gsgpu_device *adev)
{
	struct page *dummy_page = lg_get_ttm_bo_glob(&adev->mman.bdev)->dummy_read_page;
	void *dummy_addr;

	if (adev->dummy_page_addr)
		return 0;

	dummy_addr = page_address(dummy_page);
	memset(dummy_addr, 0xdd, PAGE_SIZE);

	return lg_map_page(adev, dummy_page);
}

/**
 * gsgpu_dummy_page_fini - free dummy page used by the driver
 *
 * @adev: gsgpu_device pointer
 *
 * Frees the dummy page used by the driver (all asics).
 */
static void gsgpu_gart_dummy_page_fini(struct gsgpu_device *adev)
{
	if (!adev->dummy_page_addr)
		return;

	lg_unmap_page(adev);

	adev->dummy_page_addr = 0;
}

/**
 * gsgpu_gart_table_vram_alloc - allocate vram for gart page table
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate video memory for GART page table
 * (pcie r4xx, r5xx+).  These asics require the
 * gart table to be in video memory.
 * Returns 0 for success, error for failure.
 */
int gsgpu_gart_table_vram_alloc(struct gsgpu_device *adev)
{
	int r;

	if (adev->gart.robj == NULL) {
		struct gsgpu_bo_param bp;

		memset(&bp, 0, sizeof(bp));
		bp.size = adev->gart.table_size;
		bp.byte_align = PAGE_SIZE;
		bp.domain = GSGPU_GEM_DOMAIN_VRAM;
		bp.flags = GSGPU_GEM_CREATE_CPU_ACCESS_REQUIRED |
			GSGPU_GEM_CREATE_VRAM_CONTIGUOUS;
		bp.type = ttm_bo_type_kernel;
		bp.resv = NULL;
		r = gsgpu_bo_create(adev, &bp, &adev->gart.robj);
		if (r) {
			return r;
		}
	}
	return 0;
}

/**
 * gsgpu_gart_table_vram_pin - pin gart page table in vram
 *
 * @adev: gsgpu_device pointer
 *
 * Pin the GART page table in vram so it will not be moved
 * by the memory manager (pcie r4xx, r5xx+).  These asics require the
 * gart table to be in video memory.
 * Returns 0 for success, error for failure.
 */
int gsgpu_gart_table_vram_pin(struct gsgpu_device *adev)
{
	int r;

	r = gsgpu_bo_reserve(adev->gart.robj, false);
	if (unlikely(r != 0))
		return r;
	r = gsgpu_bo_pin(adev->gart.robj, GSGPU_GEM_DOMAIN_VRAM);
	if (r) {
		gsgpu_bo_unreserve(adev->gart.robj);
		return r;
	}
	r = gsgpu_bo_kmap(adev->gart.robj, &adev->gart.ptr);
	if (r)
		gsgpu_bo_unpin(adev->gart.robj);
	gsgpu_bo_unreserve(adev->gart.robj);
	adev->gart.table_addr = gsgpu_bo_gpu_offset(adev->gart.robj);
	return r;
}

/**
 * gsgpu_gart_table_vram_unpin - unpin gart page table in vram
 *
 * @adev: gsgpu_device pointer
 *
 * Unpin the GART page table in vram (pcie r4xx, r5xx+).
 * These asics require the gart table to be in video memory.
 */
void gsgpu_gart_table_vram_unpin(struct gsgpu_device *adev)
{
	int r;

	if (adev->gart.robj == NULL) {
		return;
	}
	r = gsgpu_bo_reserve(adev->gart.robj, true);
	if (likely(r == 0)) {
		gsgpu_bo_kunmap(adev->gart.robj);
		gsgpu_bo_unpin(adev->gart.robj);
		gsgpu_bo_unreserve(adev->gart.robj);
		adev->gart.ptr = NULL;
	}
}

/**
 * gsgpu_gart_table_vram_free - free gart page table vram
 *
 * @adev: gsgpu_device pointer
 *
 * Free the video memory used for the GART page table
 * (pcie r4xx, r5xx+).  These asics require the gart table to
 * be in video memory.
 */
void gsgpu_gart_table_vram_free(struct gsgpu_device *adev)
{
	if (adev->gart.robj == NULL) {
		return;
	}
	gsgpu_bo_unref(&adev->gart.robj);
}

/*
 * Common gart functions.
 */
/**
 * gsgpu_gart_unbind - unbind pages from the gart page table
 *
 * @adev: gsgpu_device pointer
 * @offset: offset into the GPU's gart aperture
 * @pages: number of pages to unbind
 *
 * Unbinds the requested pages from the gart page table and
 * replaces them with the dummy page (all asics).
 * Returns 0 for success, -EINVAL for failure.
 */
int gsgpu_gart_unbind(struct gsgpu_device *adev, uint64_t offset,
			int pages)
{
	unsigned t;
	unsigned p;
	int i, j;
	u64 page_base;
	/* Starting from VEGA10, system bit must be 0 to mean invalid. */
	uint64_t flags = 0;

	if (!adev->gart.ready) {
		WARN(1, "trying to unbind memory from uninitialized GART !\n");
		return -EINVAL;
	}

	t = offset / GSGPU_GPU_PAGE_SIZE;
	p = t / GSGPU_GPU_PAGES_IN_CPU_PAGE;
	for (i = 0; i < pages; i++, p++) {
#ifdef CONFIG_DRM_GSGPU_GART_DEBUGFS
		adev->gart.pages[p] = NULL;
#endif
		page_base = adev->dummy_page_addr;
		if (!adev->gart.ptr)
			continue;

		for (j = 0; j < GSGPU_GPU_PAGES_IN_CPU_PAGE; j++, t++) {
			gsgpu_gmc_set_pte_pde(adev, adev->gart.ptr,
					       t, page_base, flags);
			page_base += GSGPU_GPU_PAGE_SIZE;
		}
	}
	mb();
	gsgpu_gmc_flush_gpu_tlb(adev, 0);
	return 0;
}

/**
 * gsgpu_gart_map - map dma_addresses into GART entries
 *
 * @adev: gsgpu_device pointer
 * @offset: offset into the GPU's gart aperture
 * @pages: number of pages to bind
 * @dma_addr: DMA addresses of pages
 *
 * Map the dma_addresses into GART entries (all asics).
 * Returns 0 for success, -EINVAL for failure.
 */
int gsgpu_gart_map(struct gsgpu_device *adev, uint64_t offset,
		    int pages, dma_addr_t *dma_addr, uint64_t flags,
		    void *dst)
{
	uint64_t page_base;
	unsigned i, j, t;

	if (!adev->gart.ready) {
		WARN(1, "trying to bind memory to uninitialized GART !\n");
		return -EINVAL;
	}

	t = offset / GSGPU_GPU_PAGE_SIZE;

	for (i = 0; i < pages; i++) {
		page_base = dma_addr[i];
		for (j = 0; j < GSGPU_GPU_PAGES_IN_CPU_PAGE; j++, t++) {
			gsgpu_gmc_set_pte_pde(adev, dst, t, page_base, flags);
			page_base += GSGPU_GPU_PAGE_SIZE;
		}
	}
	return 0;
}

/**
 * gsgpu_gart_bind - bind pages into the gart page table
 *
 * @adev: gsgpu_device pointer
 * @offset: offset into the GPU's gart aperture
 * @pages: number of pages to bind
 * @pagelist: pages to bind
 * @dma_addr: DMA addresses of pages
 *
 * Binds the requested pages to the gart page table
 * (all asics).
 * Returns 0 for success, -EINVAL for failure.
 */
int gsgpu_gart_bind(struct gsgpu_device *adev, uint64_t offset,
		     int pages, struct page **pagelist, dma_addr_t *dma_addr,
		     uint64_t flags)
{
#ifdef CONFIG_DRM_GSGPU_GART_DEBUGFS
	unsigned i, t, p;
#endif
	int r;

	if (!adev->gart.ready) {
		WARN(1, "trying to bind memory to uninitialized GART !\n");
		return -EINVAL;
	}

#ifdef CONFIG_DRM_GSGPU_GART_DEBUGFS
	t = offset / GSGPU_GPU_PAGE_SIZE;
	p = t / GSGPU_GPU_PAGES_IN_CPU_PAGE;
	for (i = 0; i < pages; i++, p++)
		adev->gart.pages[p] = pagelist ? pagelist[i] : NULL;
#endif

	if (!adev->gart.ptr)
		return 0;

	r = gsgpu_gart_map(adev, offset, pages, dma_addr, flags,
		    adev->gart.ptr);
	if (r)
		return r;

	mb();
	gsgpu_gmc_flush_gpu_tlb(adev, 0);
	return 0;
}

/**
 * gsgpu_gart_init - init the driver info for managing the gart
 *
 * @adev: gsgpu_device pointer
 *
 * Allocate the dummy page and init the gart driver info (all asics).
 * Returns 0 for success, error for failure.
 */
int gsgpu_gart_init(struct gsgpu_device *adev)
{
	int r;

	if (adev->dummy_page_addr)
		return 0;

	/* We need PAGE_SIZE >= GSGPU_GPU_PAGE_SIZE */
	if (PAGE_SIZE < GSGPU_GPU_PAGE_SIZE) {
		DRM_ERROR("Page size is smaller than GPU page size!\n");
		return -EINVAL;
	}
	r = gsgpu_gart_dummy_page_init(adev);
	if (r)
		return r;
	/* Compute table size */
	adev->gart.num_cpu_pages = adev->gmc.gart_size / PAGE_SIZE;
	adev->gart.num_gpu_pages = adev->gmc.gart_size / GSGPU_GPU_PAGE_SIZE;
	DRM_INFO("GART: num cpu pages %u, num gpu pages %u\n",
		 adev->gart.num_cpu_pages, adev->gart.num_gpu_pages);

#ifdef CONFIG_DRM_GSGPU_GART_DEBUGFS
	/* Allocate pages table */
	adev->gart.pages = vzalloc(array_size(sizeof(void *),
					      adev->gart.num_cpu_pages));
	if (adev->gart.pages == NULL)
		return -ENOMEM;
#endif

	return 0;
}

/**
 * gsgpu_gart_fini - tear down the driver info for managing the gart
 *
 * @adev: gsgpu_device pointer
 *
 * Tear down the gart driver info and free the dummy page (all asics).
 */
void gsgpu_gart_fini(struct gsgpu_device *adev)
{
#ifdef CONFIG_DRM_GSGPU_GART_DEBUGFS
	vfree(adev->gart.pages);
	adev->gart.pages = NULL;
#endif
	gsgpu_gart_dummy_page_fini(adev);
}
