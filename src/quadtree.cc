#include "neko/bsh/quadtree.h"

#include <cassert>

#include "neko/core/profile.h"

namespace neko
{
namespace
{
/**
 * @brief Number of nodes of a full tree: 1 + 4 + ... + 4^maxDepth
 */
constexpr std::size_t FullTreeNodeCount(std::size_t maxDepth)
{
    std::size_t count = 0;
    std::size_t levelCount = 1;
    for (std::size_t depth = 0; depth <= maxDepth; ++depth)
    {
        count += levelCount;
        levelCount *= 4;
    }
    return count;
}
} // namespace

QuadTree::QuadTree(std::size_t maxColliders, std::size_t maxPairs, std::size_t maxDepth, std::size_t maxSize, Allocator& allocator) :
    maxColliders_(maxColliders),
    maxPairs_(maxPairs),
    maxDepth_(maxDepth),
    maxSize_(maxSize),
    nodes_(FullTreeNodeCount(maxDepth), QuadNode{}, StandardAllocator<QuadNode>{allocator}),
    entries_(StandardAllocator<Entry>{allocator}),
    possiblePairs_(StandardAllocator<ColliderPair>{allocator})
{
    assert(nodes_.size() < QuadNode::NONE && maxColliders < QuadNode::NONE);
    entries_.reserve(maxColliders_);
    possiblePairs_.reserve(maxPairs_);
}

void QuadTree::Insert(const ColliderAabb& colliderAabb)
{
    NEKO_PROFILE_FUNCTION();
    if (entries_.size() >= maxColliders_)
    {
        assert(false && "QuadTree: maxColliders reached");
        return;
    }
    entries_.push_back({ colliderAabb, QuadNode::NONE });
    InsertEntry(static_cast<Index>(entries_.size() - 1), ROOT, 0);
}

void QuadTree::CalculatePairs()
{
    NEKO_PROFILE_FUNCTION();
    AddPairs(ROOT);
}

void QuadTree::Clear()
{
    NEKO_PROFILE_FUNCTION();
    entries_.clear();
    possiblePairs_.clear();
    usedNodes_ = 1;
    nodes_[ROOT] = QuadNode{ nodes_[ROOT].aabb };
}

void QuadTree::SetWorldAabb(const Aabbf& worldAabb)
{
    nodes_[ROOT].aabb = worldAabb;
}

void QuadTree::InsertEntry(Index entry, Index node, std::size_t depth)
{
    const auto& quadNode = nodes_[node];
    if (quadNode.IsLeaf())
    {
        if (depth < maxDepth_ && quadNode.entryCount >= maxSize_ && usedNodes_ + 4 <= nodes_.size())
        {
            Split(node, depth);
            InsertEntry(entry, node, depth);
        }
        else
        {
            PushEntry(entry, node);
        }
        return;
    }
    // Go down only when a single child overlaps the collider, otherwise it stays in this node
    const auto& aabb = entries_[entry].colliderAabb.aabb;
    Index target = QuadNode::NONE;
    int overlapCount = 0;
    for (Index child = quadNode.firstChild; child < quadNode.firstChild + 4; ++child)
    {
        if (Intersect(aabb, nodes_[child].aabb))
        {
            ++overlapCount;
            target = child;
        }
    }
    if (overlapCount == 1)
    {
        InsertEntry(entry, target, depth + 1);
    }
    else
    {
        PushEntry(entry, node);
    }
}

void QuadTree::PushEntry(Index entry, Index node)
{
    auto& quadNode = nodes_[node];
    entries_[entry].next = quadNode.firstEntry;
    quadNode.firstEntry = entry;
    ++quadNode.entryCount;
}

void QuadTree::Split(Index node, std::size_t depth)
{
    const std::size_t firstChild = usedNodes_;
    usedNodes_ += 4;
    // Exact quadrants: no gap between children because of rounding
    const auto [minBound, maxBound] = nodes_[node].aabb;
    const auto center = nodes_[node].aabb.GetCenter();
    nodes_[firstChild] = QuadNode{ {minBound, center} };
    nodes_[firstChild + 1] = QuadNode{ {{center.x, minBound.y}, {maxBound.x, center.y}} };
    nodes_[firstChild + 2] = QuadNode{ {{minBound.x, center.y}, {center.x, maxBound.y}} };
    nodes_[firstChild + 3] = QuadNode{ {center, maxBound} };

    auto entry = nodes_[node].firstEntry;
    nodes_[node].firstChild = static_cast<Index>(firstChild);
    nodes_[node].firstEntry = QuadNode::NONE;
    nodes_[node].entryCount = 0;
    while (entry != QuadNode::NONE)
    {
        const auto next = entries_[entry].next;
        InsertEntry(entry, node, depth);
        entry = next;
    }
}

void QuadTree::AddPairs(Index node)
{
    const auto& quadNode = nodes_[node];
    for (auto entry = quadNode.firstEntry; entry != QuadNode::NONE; entry = entries_[entry].next)
    {
        for (auto other = entries_[entry].next; other != QuadNode::NONE; other = entries_[other].next)
        {
            AddPair(entry, other);
        }
        if (!quadNode.IsLeaf())
        {
            for (Index child = quadNode.firstChild; child < quadNode.firstChild + 4; ++child)
            {
                AddPairsWithSubtree(entry, child);
            }
        }
    }
    if (!quadNode.IsLeaf())
    {
        for (Index child = quadNode.firstChild; child < quadNode.firstChild + 4; ++child)
        {
            AddPairs(child);
        }
    }
}

void QuadTree::AddPairsWithSubtree(Index entry, Index node)
{
    const auto& quadNode = nodes_[node];
    for (auto other = quadNode.firstEntry; other != QuadNode::NONE; other = entries_[other].next)
    {
        AddPair(entry, other);
    }
    if (!quadNode.IsLeaf())
    {
        for (Index child = quadNode.firstChild; child < quadNode.firstChild + 4; ++child)
        {
            AddPairsWithSubtree(entry, child);
        }
    }
}

void QuadTree::AddPair(Index entry1, Index entry2)
{
    const auto& colliderAabb1 = entries_[entry1].colliderAabb;
    const auto& colliderAabb2 = entries_[entry2].colliderAabb;
    if (!Intersect(colliderAabb1.aabb, colliderAabb2.aabb))
    {
        return;
    }
    if (possiblePairs_.size() < maxPairs_)
    {
        possiblePairs_.emplace_back(colliderAabb1.colliderIndex, colliderAabb2.colliderIndex);
    }
    else
    {
        assert(false && "QuadTree: maxPairs reached");
    }
}

}
