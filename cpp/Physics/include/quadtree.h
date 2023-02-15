#pragma once
#include <array>
#include <unordered_set>
#include <vector>

#include "physics_type.h"
#include "shape.h"

namespace neko
{

struct ColliderAabb
{
    Aabbf aabb;
    ColliderIndex colliderIndex = INVALID_COLLIDER_INDEX;
};

struct QuadNode
{
    Aabbf aabb;
    std::array<QuadNode*, 4> nodes{};
    std::vector<ColliderAabb> colliders;
};

class QuadTree
{
public:
    QuadTree();
    void Insert(const ColliderAabb& colliderAabb);
    void CalculatePairs();
    void Clear();
    void SetWorldAabb(const Aabbf& worldAabb);
    [[nodiscard]] const auto& GetPossiblePairs() const { return possiblePairs_; }
private:
    void Insert(const ColliderAabb& colliderAabb, QuadNode* node);
    void GoDownTree(const QuadNode*);
    void InsertPairs(const QuadNode*, ColliderIndex);
    std::vector<TriggerPair> possiblePairs_;
    std::size_t index_ = 1;
    std::vector<QuadNode> nodes_;
};

}
