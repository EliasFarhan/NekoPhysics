#include "neko/bsh/sweep_and_prune.h"

#include <algorithm>
#include <cassert>

#include "neko/core/platform.h"
#include "neko/core/profile.h"

namespace neko
{

SweepAndPrune::SweepAndPrune(std::size_t maxColliders, std::size_t maxPairs, Allocator& allocator) :
    maxColliders_(maxColliders),
    maxPairs_(maxPairs),
    colliders_(StandardAllocator<ColliderAabb>{allocator}),
    possiblePairs_(StandardAllocator<ColliderPair>{allocator})
{
    colliders_.reserve(maxColliders_);
    possiblePairs_.reserve(maxPairs_);
}

void SweepAndPrune::Insert(const ColliderAabb& colliderAabb)
{
    if (colliders_.size() < maxColliders_)
    {
        colliders_.push_back(colliderAabb);
    }
    else
    {
        assert(false && "SweepAndPrune: maxColliders reached");
    }
}

NEKO_HOT void SweepAndPrune::CalculatePairs()
{
    NEKO_PROFILE_FUNCTION();
    // std::sort doesn't allocate; the order is total, so every implementation gives the same result
    std::sort(colliders_.begin(), colliders_.end(), [](const ColliderAabb& a, const ColliderAabb& b)
    {
        if (a.aabb.minBound.x != b.aabb.minBound.x)
        {
            return a.aabb.minBound.x < b.aabb.minBound.x;
        }
        return a.colliderIndex < b.colliderIndex;
    });
    for (auto it = colliders_.begin(); it != colliders_.end(); ++it)
    {
        for (auto other = it + 1; other != colliders_.end() && !(it->aabb.maxBound.x < other->aabb.minBound.x); ++other)
        {
            if (it->aabb.maxBound.y < other->aabb.minBound.y || other->aabb.maxBound.y < it->aabb.minBound.y)
            {
                continue;
            }
            if (possiblePairs_.size() < maxPairs_)
            {
                possiblePairs_.emplace_back(it->colliderIndex, other->colliderIndex);
            }
            else
            {
                assert(false && "SweepAndPrune: maxPairs reached");
            }
        }
    }
}

void SweepAndPrune::Clear()
{
    colliders_.clear();
    possiblePairs_.clear();
}

} // namespace neko
