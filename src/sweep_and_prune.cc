#include "neko/bsh/sweep_and_prune.h"

#include <algorithm>
#include <cassert>

#include "neko/core/platform.h"
#include "neko/core/profile.h"

namespace neko
{

SweepAndPrune::SweepAndPrune(std::size_t max_colliders, std::size_t max_pairs, Allocator& allocator) :
    max_colliders_(max_colliders),
    max_pairs_(max_pairs),
    colliders_(StandardAllocator<ColliderAabb>{allocator}),
    possible_pairs_(StandardAllocator<ColliderPair>{allocator})
{
    colliders_.reserve(max_colliders_);
    possible_pairs_.reserve(max_pairs_);
}

void SweepAndPrune::Insert(const ColliderAabb& collider_aabb)
{
    if (colliders_.size() < max_colliders_)
    {
        colliders_.push_back(collider_aabb);
    }
    else
    {
        assert(false && "SweepAndPrune: max_colliders reached");
    }
}

NEKO_HOT void SweepAndPrune::CalculatePairs()
{
    NEKO_PROFILE_FUNCTION();
    // std::sort doesn't allocate; the order is total, so every implementation gives the same result
    std::sort(colliders_.begin(), colliders_.end(), [](const ColliderAabb& a, const ColliderAabb& b)
    {
        if (a.aabb.min_bound().x() != b.aabb.min_bound().x())
        {
            return a.aabb.min_bound().x() < b.aabb.min_bound().x();
        }
        return a.collider_index < b.collider_index;
    });
    for (auto it = colliders_.begin(); it != colliders_.end(); ++it)
    {
        for (auto other = it + 1; other != colliders_.end() && !(it->aabb.max_bound().x() < other->aabb.min_bound().x()); ++other)
        {
            if (it->aabb.max_bound().y() < other->aabb.min_bound().y() || other->aabb.max_bound().y() < it->aabb.min_bound().y())
            {
                continue;
            }
            if (possible_pairs_.size() < max_pairs_)
            {
                possible_pairs_.emplace_back(it->collider_index, other->collider_index);
            }
            else
            {
                assert(false && "SweepAndPrune: max_pairs reached");
            }
        }
    }
}

void SweepAndPrune::Clear()
{
    colliders_.clear();
    possible_pairs_.clear();
}

} // namespace neko
