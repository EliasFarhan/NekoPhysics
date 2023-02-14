#pragma once

#include "vec2.h"

namespace neko
{

template<typename T>
struct Aabb
{
    Vec2<T> minBound{};
    Vec2<T> maxBound{};

    constexpr static bool Intersect(const Aabb& a, const Aabb& b)
    { 
        if(a.maxBound.x < b.minBound.x || a.minBound.x > b.maxBound.x) return false;
        if(a.maxBound.y < b.minBound.y || a.minBound.y > b.maxBound.y) return false;
        return true;
    }

    static constexpr Aabb FromCenter(Vec2<T> center, Vec2<T> halfSize)
    {
        return { center - halfSize, center + halfSize };
    }

    [[nodiscard]] constexpr Vec2<T> GetHalfSize() const { return maxBound - minBound; }
}; 

} // namespace neko
