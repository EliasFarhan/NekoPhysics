#pragma once

#include "bsh/bsh.h"
#include "core/allocator.h"

#include <array>
#include <vector>

namespace neko
{

struct QuadNode
{
    QuadNode(Allocator& allocator);
    Aabbf aabb{};
    std::array<QuadNode*, 4> nodes{};
    ArrayList<ColliderAabb> colliders;
};

class QuadTree : public BoundingSurfaceHierarchy
{
public:
    QuadTree();
    void Insert(const ColliderAabb& colliderAabb) override;
    void CalculatePairs() override;
    void Clear() override;
    void SetWorldAabb(const Aabbf& worldAabb) override;
    [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs() const override { return possiblePairs_; }

    const QuadNode& GetRootNode() { return nodes_[0]; }

    static constexpr std::size_t MAX_DEPTH = 7;
    static constexpr std::size_t MAX_SIZE = 8;
private:
    void Insert(const ColliderAabb& colliderAabb, QuadNode* node, int depth);
    void GoDownTree(const QuadNode*);
    void InsertPairs(const QuadNode*, ColliderIndex);
    
    HeapAllocator heapAllocator_;
    ArrayList<ColliderPair> possiblePairs_{{heapAllocator_}};
    std::size_t nodeAllocationIndex_ = 1;
    ArrayList<QuadNode> nodes_;
};

}
