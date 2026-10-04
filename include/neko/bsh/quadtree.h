#ifndef NEKO_BSH_QUADTREE_H_
#define NEKO_BSH_QUADTREE_H_

#include "neko/bsh/bsh.h"
#include "neko/core/allocator.h"

#include <cstddef>
#include <limits>

namespace neko
{

struct QuadNode
{
    static constexpr Index NONE = std::numeric_limits<Index>::max();

    Aabbf aabb{};
    /**
     * @brief The four children are contiguous, starting at firstChild
     */
    Index firstChild = NONE;
    /**
     * @brief Head of the linked list of colliders stored in this node
     */
    Index firstEntry = NONE;
    Index entryCount = 0;

    [[nodiscard]] constexpr bool IsLeaf() const noexcept { return firstChild == NONE; }
};

/**
 * @brief Quadtree broadphase. Nodes and colliders live in arrays sized at construction and link to each other with
 * indices: no allocation after construction, and copies are safe.
 */
class QuadTree final : public BoundingSurfaceHierarchy
{
public:
    static constexpr std::size_t MAX_DEPTH = 3;
    static constexpr std::size_t MAX_SIZE = 4;
    static constexpr Index ROOT = 0;

    /**
     * @param maxDepth depth of the deepest nodes (the root is at depth 0)
     * @param maxSize number of colliders above which a leaf splits
     */
    explicit QuadTree(std::size_t maxColliders = 256, std::size_t maxPairs = 2048,
        std::size_t maxDepth = MAX_DEPTH, std::size_t maxSize = MAX_SIZE, Allocator& allocator = DefaultAllocator());

    void Insert(const ColliderAabb& colliderAabb) override;
    void CalculatePairs() override;
    void Clear() override;
    void SetWorldAabb(const Aabbf& worldAabb) override;
    [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs() const override { return possiblePairs_; }

    [[nodiscard]] const QuadNode& GetNode(Index index) const { return nodes_[index]; }

private:
    struct Entry
    {
        ColliderAabb colliderAabb;
        Index next = QuadNode::NONE;
    };

    void InsertEntry(Index entry, Index node, std::size_t depth);
    void PushEntry(Index entry, Index node);
    void Split(Index node, std::size_t depth);
    void AddPairs(Index node);
    void AddPairsWithSubtree(Index entry, Index node);
    void AddPair(Index entry1, Index entry2);

    std::size_t maxColliders_;
    std::size_t maxPairs_;
    std::size_t maxDepth_;
    std::size_t maxSize_;
    ArrayList<QuadNode> nodes_;
    ArrayList<Entry> entries_;
    ArrayList<ColliderPair> possiblePairs_;
    std::size_t usedNodes_ = 1;
};

}

#endif  // NEKO_BSH_QUADTREE_H_
