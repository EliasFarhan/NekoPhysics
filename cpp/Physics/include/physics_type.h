#pragma once

#include <functional>

namespace neko
{

struct BodyIndex
{
    int index;
    bool operator ==(const BodyIndex& rhs) const
    {
        return index == rhs.index;
    }
};


struct ShapeIndex
{
    int index = -1;
    bool operator ==(const ShapeIndex& rhs) const
    {
        return index == rhs.index;
    }
};


enum class ColliderType
{
    AABB,
    CIRCLE,
    NONE
};

struct ColliderIndex
{
    int index = -1;
    ShapeIndex shapeIndex{};
    ColliderType type = ColliderType::NONE;
    bool operator ==(const ColliderIndex& rhs) const
    {
        return index == rhs.index && shapeIndex == rhs.shapeIndex && type == rhs.type;
    }
};

struct TriggerPair
{
    BodyIndex b1{};
    ColliderIndex c1{};
    BodyIndex b2{};
    ColliderIndex c2{};
    bool operator ==(const TriggerPair& rhs) const
    {
        return (b1 == rhs.b1) && (b2 == rhs.b2) && (c1 == rhs.c1) && (c2 == rhs.c2);
    }
};

struct TriggerHash
{
    auto operator()(const TriggerPair& p) const noexcept
    {
        const auto h1 = std::hash<int>{}(p.c1.index);
        const auto h2 = std::hash<int>{}(p.c2.index);
        return h1 + h2; // or use boost::hash_combine
    }
};
}
