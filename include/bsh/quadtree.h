#pragma once

#include "bsh/bsh.h"
#include "core/allocator.h"

#include <array>
#include <vector>

namespace neko
{

struct QuadNode
{
    explicit QuadNode(Allocator& allocator);
    Aabbf aabb{};
    std::array<QuadNode*, 4> nodes{};
    ArrayList<ColliderAabb> colliders;
};

class QuadTree : public BoundingSurfaceHierarchy
{
public:
    explicit QuadTree(Allocator& allocator = DefaultAllocator());
    QuadTree(std::size_t maxDepth, std::size_t maxSize, Allocator& allocator = DefaultAllocator());
    // Nodes point to each other
    QuadTree(const QuadTree&) = delete;
    QuadTree& operator=(const QuadTree&) = delete;
    void Insert(const ColliderAabb& colliderAabb) override;
    void CalculatePairs() override;
    void Clear() override;
    void SetWorldAabb(const Aabbf& worldAabb) override;
    [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs() const override { return possiblePairs_; }

    [[nodiscard]] const QuadNode& GetRootNode() const { return nodes_[0]; }

    static constexpr std::size_t MAX_DEPTH = 3;
    static constexpr std::size_t MAX_SIZE = 4;
private:
    void Insert(const ColliderAabb& colliderAabb, QuadNode* node, int depth);
    void GoDownTree(const QuadNode*);
    void InsertPairs(const QuadNode*, ColliderIndex);
    
    Allocator* allocator_;
    ArrayList<ColliderPair> possiblePairs_;
    std::size_t nodeAllocationIndex_ = 1;
    ArrayList<QuadNode> nodes_;
    std::size_t maxDepth_ = MAX_DEPTH;
    std::size_t maxSize_ = MAX_SIZE;
};

}
