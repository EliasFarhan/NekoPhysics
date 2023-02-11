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

    }
};
} // namespace neko
