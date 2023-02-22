#pragma once

#include <functional>

namespace neko
{

using Scalar = float;

struct BodyIndex
{
    int index = -1;
    bool operator ==(const BodyIndex& rhs) const
    {
        return index == rhs.index;
    }
};

constexpr auto INVALID_BODY_INDEX = BodyIndex{ -1 };

struct ShapeIndex
{
    int index = -1;
    bool operator ==(const ShapeIndex& rhs) const
    {
        return index == rhs.index;
    }
};

constexpr auto INVALID_SHAPE_INDEX = ShapeIndex{ -1 };

enum class ColliderType
{
    AABB,
    CIRCLE,
    NONE
};

struct ColliderIndex
{
    int index = -1;
   
    bool operator ==(const ColliderIndex& rhs) const
    {
        return index == rhs.index;
    }
};

constexpr auto INVALID_COLLIDER_INDEX = ColliderIndex{ -1 };

struct TriggerPair
{
    ColliderIndex c1{};
    ColliderIndex c2{};
    bool operator ==(const TriggerPair& rhs) const
    {
        return (c1 == rhs.c1 && c2 == rhs.c2) || (c1 == rhs.c2 && c2 == rhs.c1);
    }
};

struct TriggerHash
{
    auto operator()(const TriggerPair& p) const noexcept
    {
        const auto h1 = std::hash<int>{}(p.c1.index);
        const auto h2 = std::hash<int>{}(p.c2.index);
        return h1 + h2;
    }
};
}
