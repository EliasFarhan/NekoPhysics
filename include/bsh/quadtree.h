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
    QuadTree();
    QuadTree(size_t maxDepth, size_t maxSize);
    void Insert(const ColliderAabb& colliderAabb) override;
    void CalculatePairs() override;
    void Clear() override;
    void SetWorldAabb(const Aabbf& worldAabb) override;
    [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs() const override { return possiblePairs_; }

    [[nodiscard]] const QuadNode& GetRootNode() const { return nodes_[0]; }

    static constexpr std::size_t MAX_DEPTH = 7;
    static constexpr std::size_t MAX_SIZE = 8;
private:
    void Insert(const ColliderAabb& colliderAabb, QuadNode* node, int depth);
    void GoDownTree(const QuadNode*);
    void InsertPairs(const QuadNode*, ColliderIndex);
    
    HeapAllocator heapAllocator_;
    ArrayList<ColliderPair> possiblePairs_{StandardAllocator<ColliderPair>{heapAllocator_}};
    std::size_t nodeAllocationIndex_ = 1;
    ArrayList<QuadNode> nodes_;
    size_t maxDepth_ = MAX_DEPTH;
    size_t maxSize_ = MAX_SIZE;
};

}
