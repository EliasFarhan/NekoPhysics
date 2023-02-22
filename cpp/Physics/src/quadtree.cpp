#include "bsh/quadtree.h"
#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif
namespace neko
{
constexpr std::size_t depth = 5;
constexpr std::size_t maxSize = 32;

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


QuadTree::QuadTree()
{
    possiblePairs_.reserve(pow(static_cast<std::size_t>(4), depth));
    nodes_.resize(quadCount(depth), {});
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
    Insert(colliderAabb, nodes_.data());
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
    possiblePairs_.clear();
    for(auto& node: nodes_)
    {
        node.colliders.clear();
        std::ranges::fill(node.nodes, nullptr);
    }
}

void QuadTree::SetWorldAabb(const Aabbf& worldAabb)
{
    nodes_[0].aabb = worldAabb;
}

void QuadTree::Insert(const ColliderAabb& colliderAabb, QuadNode* node)
{
    if(node->nodes[0] == nullptr)
    {
       if(node->colliders.size() == maxSize && index_ < nodes_.size())
       {
           //Break into 4 new nodes
#ifdef TRACY_ENABLE
           ZoneNamedN(Break4, "BreakNode", true);
#endif
           constexpr static std::array<Vec2f, 4> direction
           {
               Vec2f{-1,-1},
               Vec2f{-1,1},
               Vec2f{1,-1},
               Vec2f{1,1},
           };
           for(int i = 0; i < 4; i++)
           {
               node->nodes[i] = &nodes_[index_+i];
               node->nodes[i]->aabb = Aabbf::FromCenter(
                   node->aabb.GetCenter() + direction[i] * node->aabb.GetHalfSize() / 2.0f, 
                   node->aabb.GetHalfSize() / 2);
           }
           std::array<ColliderAabb, maxSize> tmp;
           for(std::size_t i = 0; i < maxSize; ++i)
           {
               tmp[i] = node->colliders[i];
           }
           node->colliders.clear();
           index_ += 4;
           for(const auto& localAabb: tmp)
           {
               Insert(localAabb, node);
           }
           Insert(colliderAabb, node);
       }
       else
       {
           node->colliders.push_back(colliderAabb);
       }
    }
    else
    {
        int count = 0;
        QuadNode* insertNode = nullptr;
        for(const auto& childNode: node->nodes)
        {
            if(Intersect(colliderAabb.aabb, node->aabb))
            {
                count++;
                insertNode = childNode;
            }
        }
        if(count > 1)
        {
            node->colliders.push_back(colliderAabb);
        }
        else
        {
            Insert(colliderAabb, insertNode);
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
