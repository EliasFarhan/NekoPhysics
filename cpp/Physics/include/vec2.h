#pragma once

#include <type_traits>
#include <cmath>

namespace neko
{
template<typename T>
requires std::is_arithmetic_v<T>
struct Vec2
{
    T x{}, y{};

    constexpr Vec2 operator+(Vec2 other) const
    {
        return { x + other.x, y + other.y };
    }
    constexpr Vec2& operator+=(Vec2 other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }


    constexpr Vec2 operator*(T other) const
    {
        return { x * other, y * other };
    }

    constexpr Vec2 operator/(T other) const
    {
        return { x / other, y / other };
    }
    constexpr Vec2& operator*=(T other)
    {
        x *= other;
        y *= other;
        return *this;
    }
    static constexpr T Dot(Vec2 v1, Vec2 v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }
    static constexpr Vec2 zero() { return {}; }
    static constexpr Vec2 up() { return {0,1}; }

    Vec2 Rotate(T angle) noexcept
    {
        const float sin = std::sin(angle);
        const float cos = std::cos(angle);
        return { (cos * x) - (sin * y) , (sin * x) + (cos * y) };
    }
};

template<typename T>
constexpr Vec2<T> operator*(float f, Vec2<T> other)
{
    return { f * other.x, f * other.y };
}


} // namespace neko
