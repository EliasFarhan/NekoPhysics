#pragma once

#include <functional>
#include "math/fixed.h"

namespace neko
{


struct BodyIndex
{
    std::int32_t index = -1;
    constexpr bool operator ==(const BodyIndex& rhs) const
    {
        return index == rhs.index;
    }
};

constexpr auto INVALID_BODY_INDEX = BodyIndex{ -1 };

struct ShapeIndex
{
    std::int32_t index = -1;
    constexpr bool operator ==(const ShapeIndex& rhs) const
    {
        return index == rhs.index;
    }
};

constexpr auto INVALID_SHAPE_INDEX = ShapeIndex{ -1 };

enum class ShapeType : short
{
    AABB,
    CIRCLE,
    PLANE,
    NONE
};

struct ColliderIndex
{
    std::int32_t index = -1;
   
    constexpr bool operator ==(const ColliderIndex& rhs) const
    {
        return index == rhs.index;
    }
	constexpr bool operator !=(const ColliderIndex& rhs) const
	{
		return index != rhs.index;
	}
};

constexpr auto INVALID_COLLIDER_INDEX = ColliderIndex{ -1 };

struct ColliderPair
{
    ColliderIndex c1{};
    ColliderIndex c2{};
    constexpr bool operator ==(const ColliderPair& rhs) const noexcept
    {
        return (c1 == rhs.c1 && c2 == rhs.c2) || (c1 == rhs.c2 && c2 == rhs.c1);
    }

    [[nodiscard]] constexpr bool contains(ColliderIndex c) const
    {
        return c == c1 || c == c2;
    }
};

struct ColliderEqual
{
	bool operator()(const ColliderPair& lhs, const ColliderPair& rhs) const
	{
		return lhs == rhs;
	}
};

struct ColliderHash
{
    auto operator()(const ColliderPair& p) const noexcept
    {
        const auto h1 = std::hash<int>{}(p.c1.index);
        const auto h2 = std::hash<int>{}(p.c2.index);
        return h1 + h2;
    }
};
}
