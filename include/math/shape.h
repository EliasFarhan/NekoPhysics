#pragma once

#include "physics/physics_type.h"
#include "vec2.h"
#include <array>
namespace neko
{

template<typename T>
struct Aabb
{
    Vec2<T> minBound{};
    Vec2<T> maxBound{};



    static constexpr Aabb FromCenter(Vec2<T> center, Vec2<T> halfSize)
    {
        return { center - halfSize, center + halfSize };
    }
    [[nodiscard]] constexpr Vec2<T> GetCenter() const { return minBound + GetHalfSize(); }
    [[nodiscard]] constexpr Vec2<T> GetHalfSize() const { return (maxBound - minBound) / T{2}; }

    [[nodiscard]] constexpr bool Contains(Vec2<T> point) const noexcept
    {
        const auto center = GetCenter();
        const auto halfSize = GetHalfSize();
        return Abs(point.x - center.x) < halfSize.x && Abs(point.y - center.y) < halfSize.y;
    }
};


template<typename T>
struct Circle
{
    Vec2<T> position;
    T radius;
    [[nodiscard]] constexpr Aabb<T> GetAabb() const
    {
        return Aabb<T>::FromCenter(position, { radius, radius });
    }
    [[nodiscard]] constexpr bool Contains(Vec2<T> point) const noexcept
    {
        return (point - position).SquareLength() < radius * radius;
    }
};


template<typename T>
constexpr bool Intersect(const Aabb<T>& a, const Aabb<T>& b)
{
    if (a.maxBound.x < b.minBound.x || a.minBound.x > b.maxBound.x) return false;
    if (a.maxBound.y < b.minBound.y || a.minBound.y > b.maxBound.y) return false;
    return true;
}

template<typename T>
constexpr static bool Intersect(const Circle<T>& c1, const Circle<T>& c2)
{
    const auto r = c1.radius + c2.radius;
    return r * r >= (c1.position - c2.position).SquareLength();
}
template<typename T>
constexpr bool Intersect(const Circle<T>& circle, const Aabb<T>& aabb)
{

    const Aabb<T> aabb1{ {aabb.minBound.x-circle.radius, aabb.minBound.y}, {aabb.maxBound.x+circle.radius, aabb.maxBound.y} };
    const Aabb<T> aabb2{ {aabb.minBound.x, aabb.minBound.y-circle.radius}, {aabb.maxBound.x, aabb.maxBound.y+circle.radius} };
    if (aabb1.Contains(circle.position))
    {
        return true;
    }
    if(aabb2.Contains(circle.position))
    {
        return true;
    }
    const std::array<Vec2<T>, 4> edges =
    {
        aabb.minBound,
        aabb.maxBound,
        aabb.GetCenter()+Vec2<T>::up()*aabb.GetHalfSize().y,
        aabb.GetCenter()+Vec2<T>::right()*aabb.GetHalfSize().x,
    };
    for(auto edge: edges)
    {
        if((edge-circle.position).SquareLength() < circle.radius*circle.radius)
        {
            return true;
        }
    }
    return false;
}

template<typename T>
constexpr bool Intersect(const Aabb<T>& aabb, const Circle<T>& circle)
{
    return Intersect(circle, aabb);
}


using Aabbf = Aabb<Scalar>;
using Circlef = Circle<Scalar>;
} // namespace neko