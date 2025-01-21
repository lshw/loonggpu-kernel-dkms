#if !defined(LG_SYNC_HELPER_H)
#define LG_SYNC_HELPER_H

void *gsgpu_sync_get_owner(struct dma_fence *f);
bool gsgpu_sync_same_dev(struct gsgpu_device *adev, struct dma_fence *f);

static bool gsgpu_sync_test_fence(struct gsgpu_device *adev,
                                  enum gsgpu_sync_mode mode,
                                  void *owner, struct dma_fence *f)
{
        void *fence_owner = gsgpu_sync_get_owner(f);

        /* Always sync to moves, no matter what */
        if (fence_owner == GSGPU_FENCE_OWNER_UNDEFINED)
                return true;

        /* We only want to trigger KFD eviction fences on
         * evict or move jobs. Skip KFD fences otherwise.
         */
        if (fence_owner == GSGPU_FENCE_OWNER_KFD &&
            owner != GSGPU_FENCE_OWNER_UNDEFINED)
                return false;

        /* Never sync to VM updates either. */
        if (fence_owner == GSGPU_FENCE_OWNER_VM &&
            owner != GSGPU_FENCE_OWNER_UNDEFINED)
                return false;

        /* Ignore fences depending on the sync mode */
        switch (mode) {
        case GSGPU_SYNC_ALWAYS:
                return true;

        case GSGPU_SYNC_NE_OWNER:
                if (gsgpu_sync_same_dev(adev, f) &&
                    fence_owner == owner)
                        return false;
                break;

        case GSGPU_SYNC_EQ_OWNER:
                if (gsgpu_sync_same_dev(adev, f) &&
                    fence_owner != owner)
                        return false;
                break;

        case GSGPU_SYNC_EXPLICIT:
                return false;
        }

        return true;
}

static inline int lg_gsgpu_sync_resv(struct gsgpu_device *adev,
                     struct gsgpu_sync *sync,
                     lg_dma_resv_t *resv,
                     enum gsgpu_sync_mode mode,
                     void *owner, bool explicit_sync)
{
#if defined(dma_resv_for_each_fence)
        struct dma_resv_iter cursor;
        struct dma_fence *f;
        int r;

        if (resv == NULL)
                return -EINVAL;

        dma_resv_for_each_fence(&cursor, resv, DMA_RESV_USAGE_BOOKKEEP, f) {
                dma_fence_chain_for_each(f, f) {
                        struct dma_fence *tmp = dma_fence_chain_contained(f);
                        if (gsgpu_sync_test_fence(adev, mode, owner, tmp)) {
                                r = gsgpu_sync_fence(adev, sync, f, explicit_sync);
                                dma_fence_put(f);
                                if (r)
                                        return r;

                                break;
                        }
                }
        }
        return 0;
#else
        lg_dma_resv_list_t *flist;
        struct dma_fence *f;
        void *fence_owner;
        unsigned i;
        int r = 0;

        /* always sync to the exclusive fence */
        f = lg_dma_resv_get_excl(resv);
        r = gsgpu_sync_fence(adev, sync, f, false);

        flist = lg_dma_resv_get_list(resv);
        if (!flist || r)
                return r;

        for (i = 0; i < flist->shared_count; ++i) {
                f = rcu_dereference_protected(flist->shared[i],
                                              lg_dma_resv_held(resv));
                /* We only want to trigger KFD eviction fences on
                 * evict or move jobs. Skip KFD fences otherwise.
                 */
                fence_owner = gsgpu_sync_get_owner(f);
                if (fence_owner == GSGPU_FENCE_OWNER_KFD &&
                    owner != GSGPU_FENCE_OWNER_UNDEFINED)
                        continue;

                if (gsgpu_sync_same_dev(adev, f)) {
                        /* VM updates are only interesting
                         * for other VM updates and moves.
                         */
                        if ((owner != GSGPU_FENCE_OWNER_UNDEFINED) &&
                            (fence_owner != GSGPU_FENCE_OWNER_UNDEFINED) &&
                            ((owner == GSGPU_FENCE_OWNER_VM) !=
                             (fence_owner == GSGPU_FENCE_OWNER_VM)))
                                continue;

                        /* Ignore fence from the same owner and explicit one as
                         * long as it isn't undefined.
                         */
                        if (owner != GSGPU_FENCE_OWNER_UNDEFINED &&
                            (fence_owner == owner || explicit_sync))
                                continue;
                }

                r = gsgpu_sync_fence(adev, sync, f, false);
                if (r)
                        break;
        }
        return r;
#endif
}

#endif /* LG_SYNC_HELPER_H */
