#include "neko/bsh/quadtree.h"
#include "neko/bsh/sweep_and_prune.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>

namespace
{
using neko::Scalar;

/**
 * @brief Small deterministic generator, so the scenes are the same on every platform
 */
class Lcg
{
public:
    explicit Lcg(std::uint32_t seed) : state_(seed) {}
    float Range(float min, float max)
    {
        state_ = state_ * 1664525u + 1013904223u;
        return min + (max - min) * static_cast<float>(state_ >> 8) / static_cast<float>(1u << 24);
    }
private:
    std::uint32_t state_;
};

std::vector<neko::ColliderAabb> RandomAabbs(std::size_t count, std::uint32_t seed)
{
    Lcg random{ seed };
    std::vector<neko::ColliderAabb> aabbs;
    for (std::size_t i = 0; i < count; ++i)
    {
        const neko::Vec2f center{ Scalar{random.Range(-10, 10)}, Scalar{random.Range(-8, 8)} };
        const neko::Vec2f half_size{ Scalar{random.Range(0.05f, 1.5f)}, Scalar{random.Range(0.05f, 1.5f)} };
        aabbs.push_back({ neko::Aabbf::FromCenter(center, half_size), neko::ColliderIndex{static_cast<neko::Index>(i)} });
    }
    // Same minimum x, to exercise the tie-break: move the whole box so it stays valid
    const auto shift = aabbs[0].aabb.min_bound().x() - aabbs[1].aabb.min_bound().x();
    aabbs[1].aabb.set_min_bound(aabbs[1].aabb.min_bound() + neko::Vec2f{ shift, neko::Scalar{ 0 } });
    aabbs[1].aabb.set_max_bound(aabbs[1].aabb.max_bound() + neko::Vec2f{ shift, neko::Scalar{ 0 } });
    return aabbs;
}

std::set<neko::ColliderPair> IntersectingPairs(const std::vector<neko::ColliderAabb>& aabbs)
{
    std::set<neko::ColliderPair> pairs;
    for (std::size_t i = 0; i < aabbs.size(); ++i)
    {
        for (std::size_t j = i + 1; j < aabbs.size(); ++j)
        {
            if (neko::Intersect(aabbs[i].aabb, aabbs[j].aabb))
            {
                pairs.emplace(aabbs[i].collider_index, aabbs[j].collider_index);
            }
        }
    }
    return pairs;
}

neko::Aabbf Bounds(const std::vector<neko::ColliderAabb>& aabbs)
{
    auto bounds = aabbs.front().aabb;
    for (const auto& aabb : aabbs)
    {
        bounds.set_min_bound({ neko::Min(bounds.min_bound().x(), aabb.aabb.min_bound().x()), neko::Min(bounds.min_bound().y(), aabb.aabb.min_bound().y()) });
        bounds.set_max_bound({ neko::Max(bounds.max_bound().x(), aabb.aabb.max_bound().x()), neko::Max(bounds.max_bound().y(), aabb.aabb.max_bound().y()) });
    }
    return bounds;
}

std::vector<neko::ColliderPair> CollectPairs(neko::BoundingSurfaceHierarchy& broadphase, const std::vector<neko::ColliderAabb>& aabbs)
{
    broadphase.Clear();
    broadphase.SetWorldAabb(Bounds(aabbs));
    for (const auto& aabb : aabbs)
    {
        broadphase.Insert(aabb);
    }
    broadphase.CalculatePairs();
    const auto& pairs = broadphase.GetPossiblePairs();
    return { pairs.begin(), pairs.end() };
}

constexpr std::size_t kColliderCount = 200;
} // namespace

TEST(SweepAndPrune, FindsExactlyTheIntersectingPairs)
{
    neko::SweepAndPrune sap{ kColliderCount, kColliderCount * kColliderCount / 2 };
    for (std::uint32_t seed = 1; seed <= 5; ++seed)
    {
        const auto aabbs = RandomAabbs(kColliderCount, seed);
        const auto pairs = CollectPairs(sap, aabbs);
        const std::set<neko::ColliderPair> found{ pairs.begin(), pairs.end() };
        EXPECT_EQ(found.size(), pairs.size()) << "duplicate pairs, seed " << seed;
        EXPECT_EQ(found, IntersectingPairs(aabbs)) << "seed " << seed;
    }
}

TEST(SweepAndPrune, IsDeterministicAcrossClears)
{
    neko::SweepAndPrune sap{ kColliderCount, kColliderCount * kColliderCount / 2 };
    const auto aabbs = RandomAabbs(kColliderCount, 42);
    const auto first = CollectPairs(sap, aabbs);
    auto reversed = aabbs;
    std::reverse(reversed.begin(), reversed.end());
    // Same set of colliders inserted in another order: same pairs in the same order
    EXPECT_EQ(CollectPairs(sap, reversed), first);
}

TEST(QuadTree, FindsAllIntersectingPairsOnce)
{
    for (std::uint32_t seed = 1; seed <= 5; ++seed)
    {
        neko::QuadTree quad_tree{ kColliderCount, kColliderCount * kColliderCount / 2 };
        const auto aabbs = RandomAabbs(kColliderCount, seed);
        const auto pairs = CollectPairs(quad_tree, aabbs);
        const std::set<neko::ColliderPair> found{ pairs.begin(), pairs.end() };
        EXPECT_EQ(found.size(), pairs.size()) << "duplicate pairs, seed " << seed;
        for (const auto& pair : IntersectingPairs(aabbs))
        {
            EXPECT_TRUE(found.contains(pair)) << pair.c1().index() << "-" << pair.c2().index() << ", seed " << seed;
        }
        // The tree must actually prune something
        EXPECT_LT(pairs.size(), kColliderCount * (kColliderCount - 1) / 2);
    }
}

// The split copied kMaxSize colliders whatever max_size was: out of bounds below 4, dropped colliders above
TEST(QuadTree, AnyNodeSizeKeepsAllPairs)
{
    const auto aabbs = RandomAabbs(64, 7);
    const auto expected = IntersectingPairs(aabbs);
    for (std::size_t max_size : { 1u, 2u, 3u, 8u, 16u })
    {
        neko::QuadTree quad_tree{ 64, 64 * 63 / 2, 4, max_size };
        const auto pairs = CollectPairs(quad_tree, aabbs);
        const std::set<neko::ColliderPair> found{ pairs.begin(), pairs.end() };
        for (const auto& pair : expected)
        {
            EXPECT_TRUE(found.contains(pair)) << "max_size " << max_size;
        }
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
