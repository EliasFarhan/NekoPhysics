#pragma once

namespace neko
{

template<typename T>
struct Circle
{
    Vec2<T> position;
    T radius;

    constexpr static bool Intersect(const Circle& c1, const Circle& c2)
    {
        const auto r = c1.radius + c2.radius;
        return r * r < (c1.position + c2.position).SquareLength();
    }
};
} // namespace neko
