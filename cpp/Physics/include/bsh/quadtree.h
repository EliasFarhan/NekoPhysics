#pragma once
#include <array>
#include <vector>

#include "bsh/bsh.h"
#include "core/allocator.h"

namespace neko
{

struct QuadNode
{
    Aabbf aabb{};
    std::array<QuadNode*, 4> nodes{};
    std::vector<ColliderAabb> colliders{};
};

class QuadTree : public BoundingSurfaceHierarchy
{
public:
    QuadTree();
    void Insert(const ColliderAabb& colliderAabb) override;
    void CalculatePairs() override;
    void Clear() override;
    void SetWorldAabb(const Aabbf& worldAabb) override;
    [[nodiscard]] const ArrayList<TriggerPair>& GetPossiblePairs() const override { return possiblePairs_; }


    static constexpr std::size_t depth = 5;
    static constexpr std::size_t maxSize = 32;
private:
    void Insert(const ColliderAabb& colliderAabb, QuadNode* node);
    void GoDownTree(const QuadNode*);
    void InsertPairs(const QuadNode*, ColliderIndex);
    
    HeapAllocator heapAllocator_;
    ArrayList<TriggerPair> possiblePairs_{{heapAllocator_}};
    std::size_t index_ = 1;
    ArrayList<QuadNode> nodes_{{heapAllocator_}};
};

}
