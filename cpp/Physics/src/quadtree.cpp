#include <utility>

#include "bsh/quadtree.h"
#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif
namespace neko
{

template<typename T>
constexpr T pow(T base, T exponent)
{
    T result = 1;
    for (T i = 0; i < exponent; ++i)
    {
        result *= base;
    }
    return result;
}
template<typename T>
constexpr T quadCount(T n)
{
    T result = 1;
    for (T i = 1; i < n; ++i)
    {
        result += pow<T>(4, i);
    }
    return result;
}


QuadNode::QuadNode(Allocator& allocator) : colliders(StandardAllocator<ColliderAabb>(allocator))
{
}

QuadTree::QuadTree() : nodes_(StandardAllocator<QuadNode>(heapAllocator_))
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    possiblePairs_.reserve(pow(static_cast<std::size_t>(4), MAX_DEPTH));
    nodes_.resize(quadCount(MAX_DEPTH), {heapAllocator_});
    for(auto& node : nodes_)
    {
        node.colliders.reserve(maxSize);
    }
}

void QuadTree::Insert(const ColliderAabb& colliderAabb)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    //Insert into root node
    Insert(colliderAabb, &nodes_[0], MAX_DEPTH);
}

void QuadTree::CalculatePairs()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    GoDownTree(nodes_.data());
}

void QuadTree::Clear()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    nodeAllocationIndex_ = 1;
    possiblePairs_.clear();
    for(auto& node: nodes_)
    {
        node.colliders.clear();
        std::fill(node.nodes.begin(), node.nodes.end(), nullptr);
    }
}

void QuadTree::SetWorldAabb(const Aabbf& worldAabb)
{
    nodes_[0].aabb = worldAabb;
}

void QuadTree::Insert(const ColliderAabb& colliderAabb, QuadNode* node, int depth)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    if(node->nodes[0] == nullptr && depth != 0)
    {
       if(node->colliders.size() >= maxSize && nodeAllocationIndex_ < nodes_.size())
       {
           //Break into 4 new nodes
#ifdef TRACY_ENABLE
           ZoneNamedN(Break4, "BreakNode", true);
#endif
           constexpr static std::array<Vec2f, 4> direction
           {
               Vec2f{Scalar{-1}, Scalar{-1}},
               Vec2f{Scalar{-1}, Scalar{1} },
               Vec2f{Scalar{1},  Scalar{-1} },
               Vec2f{Scalar{1},  Scalar{1} },
           };
           for(std::size_t i = 0; i < direction.size(); i++)
           {
               node->nodes[i] = &nodes_[nodeAllocationIndex_+i];
               node->nodes[i]->aabb = Aabbf::FromCenter(
                   node->aabb.GetCenter() + direction[i] * node->aabb.GetHalfSize()*Scalar{0.5f},
                   node->aabb.GetHalfSize()*Scalar{0.5f});
           }
           std::array<ColliderAabb, maxSize> tmp;
           for(std::size_t i = 0; i < maxSize; ++i)
           {
               tmp[i] = node->colliders[i];
           }
           node->colliders.clear();
           nodeAllocationIndex_ += 4; // node index for quad node linear allocation
           for(const auto& localAabb: tmp)
           {
               Insert(localAabb, node, depth);
           }
           Insert(colliderAabb, node, depth);
       }
       else
       {
           node->colliders.push_back(colliderAabb);
       }
    }
    else
    {
        //If I have children
        int count = 0;
        QuadNode* insertNode = nullptr;
        for(const auto& childNode: node->nodes)
        {
            if(childNode == nullptr)
                continue;
            if(Intersect(colliderAabb.aabb, childNode->aabb))
            {
                count++;
                insertNode = childNode;
            }
        }
        if(count > 1 || depth == 0)
        {
            node->colliders.push_back(colliderAabb);
        }
        else
        {
            Insert(colliderAabb, insertNode, depth-1);
        }
    }
}

void QuadTree::GoDownTree(const QuadNode* node)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    for (std::size_t i = 0; i < node->colliders.size(); ++i)
    {
        const auto colliderIndex = node->colliders[i].colliderIndex;
        for (std::size_t j = i + 1; j < node->colliders.size(); ++j)
        {
            possiblePairs_.push_back({ colliderIndex, node->colliders[j].colliderIndex });
        }
        if (node->nodes[0] != nullptr)
        {
            for (const auto* childNode : node->nodes)
            {
                InsertPairs(childNode, colliderIndex);
            }
        }
    }
    if (node->nodes[0] != nullptr)
    {
        for (const auto* childNode : node->nodes)
        {
            GoDownTree(childNode);
        }
    }
}

void QuadTree::InsertPairs(const QuadNode* node, ColliderIndex colliderIndex)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    for (auto& colliderAabb : node->colliders)
    {
        possiblePairs_.push_back({ colliderAabb.colliderIndex, colliderIndex });
    }
    if (node->nodes[0] != nullptr)
    {
        for (const auto* childNode : node->nodes)
        {
            InsertPairs(childNode, colliderIndex);
        }
    }
}
}
