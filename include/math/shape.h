#pragma once

#include "physics/physics_type.h"
#include "vec2.h"
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
        return CompareLength(point - position, radius) < 0;
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
[[nodiscard]] constexpr bool Intersect(const Circle<T>& c1, const Circle<T>& c2)
{
    return CompareLength(c1.position - c2.position, c1.radius + c2.radius) <= 0;
}

/**
 * @brief Point of the box closest to point (point itself when it is inside)
 */
template<typename T>
[[nodiscard]] constexpr Vec2<T> ClosestPoint(const Aabb<T>& aabb, Vec2<T> point)
{
    return Vec2<T>::Clamp(point, aabb.minBound, aabb.maxBound);
}

template<typename T>
[[nodiscard]] constexpr bool Intersect(const Circle<T>& circle, const Aabb<T>& aabb)
{
    return CompareLength(circle.position - ClosestPoint(aabb, circle.position), circle.radius) <= 0;
}

template<typename T>
constexpr bool Intersect(const Aabb<T>& aabb, const Circle<T>& circle)
{
    return Intersect(circle, aabb);
}


using Aabbf = Aabb<Scalar>;
using Circlef = Circle<Scalar>;
} // namespace neko