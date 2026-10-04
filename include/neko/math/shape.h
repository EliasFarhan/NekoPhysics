#ifndef NEKO_MATH_SHAPE_H_
#define NEKO_MATH_SHAPE_H_

#include "neko/physics/physics_type.h"
#include "neko/math/vec2.h"
namespace neko
{

template<typename T>
struct Aabb
{
    Vec2<T> min_bound{};
    Vec2<T> max_bound{};



    static constexpr Aabb FromCenter(Vec2<T> center, Vec2<T> half_size)
    {
        return { center - half_size, center + half_size };
    }
    [[nodiscard]] constexpr Vec2<T> GetCenter() const { return min_bound + GetHalfSize(); }
    [[nodiscard]] constexpr Vec2<T> GetHalfSize() const { return (max_bound - min_bound) / T{2}; }

    [[nodiscard]] constexpr bool Contains(Vec2<T> point) const noexcept
    {
        const auto center = GetCenter();
        const auto half_size = GetHalfSize();
        return Abs(point.x - center.x) < half_size.x && Abs(point.y - center.y) < half_size.y;
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
    if (a.max_bound.x < b.min_bound.x || a.min_bound.x > b.max_bound.x) return false;
    if (a.max_bound.y < b.min_bound.y || a.min_bound.y > b.max_bound.y) return false;
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
    return Vec2<T>::Clamp(point, aabb.min_bound, aabb.max_bound);
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

#endif  // NEKO_MATH_SHAPE_H_
