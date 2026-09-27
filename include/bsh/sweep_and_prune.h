#pragma once

#include "bsh/bsh.h"
#include "core/allocator.h"

#include <cstddef>

namespace neko
{

/**
 * @brief Sort-and-sweep broadphase on the x axis, the default broadphase of PhysicsWorld.
 *
 * Deterministic (ties are broken by collider index), allocation-free after construction and stateless between
 * steps, so it needs no snapshot for rollback. Pairs are exactly the colliders whose AABBs intersect.
 */
class SweepAndPrune final : public BoundingSurfaceHierarchy
{
public:
    SweepAndPrune(std::size_t maxColliders, std::size_t maxPairs, Allocator& allocator = DefaultAllocator());

    void Insert(const ColliderAabb& colliderAabb) override;
    void CalculatePairs() override;
    void Clear() override;
    void SetWorldAabb([[maybe_unused]] const Aabbf& worldAabb) override {}
    [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs() const override { return possiblePairs_; }

private:
    std::size_t maxColliders_;
    std::size_t maxPairs_;
    ArrayList<ColliderAabb> colliders_;
    ArrayList<ColliderPair> possiblePairs_;
};

} // namespace neko
