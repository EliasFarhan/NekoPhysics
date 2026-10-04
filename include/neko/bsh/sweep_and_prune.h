#ifndef NEKO_BSH_SWEEP_AND_PRUNE_H_
#define NEKO_BSH_SWEEP_AND_PRUNE_H_

#include "neko/bsh/bsh.h"
#include "neko/core/allocator.h"

#include <cstddef>

namespace neko
{

// Sort-and-sweep broadphase on the x axis, the default broadphase of PhysicsWorld.
//
// Deterministic (ties are broken by collider index), allocation-free after construction and stateless between
// steps, so it needs no snapshot for rollback. Pairs are exactly the colliders whose AABBs intersect.
class SweepAndPrune final : public BoundingSurfaceHierarchy
{
public:
    SweepAndPrune(std::size_t max_colliders, std::size_t max_pairs, Allocator& allocator = DefaultAllocator());

    void Insert(const ColliderAabb& collider_aabb) override;
    void CalculatePairs() override;
    void Clear() override;
    void SetWorldAabb([[maybe_unused]] const Aabbf& world_aabb) override {}
    [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs() const override { return possible_pairs_; }

private:
    std::size_t max_colliders_;
    std::size_t max_pairs_;
    ArrayList<ColliderAabb> colliders_;
    ArrayList<ColliderPair> possible_pairs_;
};

}  // namespace neko

#endif  // NEKO_BSH_SWEEP_AND_PRUNE_H_
