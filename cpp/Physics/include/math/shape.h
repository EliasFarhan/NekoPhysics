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
    [[nodiscard]] constexpr Vec2<T> GetHalfSize() const { return (maxBound - minBound)/2; }
};

template<typename T>
constexpr bool Intersect(const Aabb<T>& a, const Aabb<T>& b)
{
    if (a.maxBound.x < b.minBound.x || a.minBound.x > b.maxBound.x) return false;
    if (a.maxBound.y < b.minBound.y || a.minBound.y > b.maxBound.y) return false;
    return true;
}

template<typename T>
struct Circle
{
    Vec2<T> position;
    T radius;
    [[nodiscard]] constexpr Aabb<T> GetAabb() const
    {
        return Aabb<T>::FromCenter(position, { radius, radius });
    }
};
template<typename T>
constexpr static bool Intersect(const Circle<T>& c1, const Circle<T>& c2)
{
    const auto r = c1.radius + c2.radius;
    return r * r >= (c1.position - c2.position).SquareLength();
}
template<typename T>
constexpr bool Intersect(const Circle<T>& circle, const Aabb<T>& aabb)
{
    const auto halfSize = aabb.GetHalfSize();
    const auto delta = Vec2<T>::Clamp(aabb.GetCenter() - circle.position, -halfSize, halfSize);
    const auto closestPoint = aabb.GetCenter() + delta;
    return (closestPoint-circle.position).SquareLength() > circle.radius * circle.radius;
}

template<typename T>
constexpr bool Intersect(const Aabb<T>& aabb, const Circle<T>& circle)
{
    return Intersect(circle, aabb);
}


using Aabbf = Aabb<Scalar>;
using Circlef = Circle<Scalar>;
} // namespace neko