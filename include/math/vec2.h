#pragma once

#include <type_traits>
#include <cmath>
#include <algorithm>

#include "math/const.h"
#include "math/fixed_lut.h"

namespace neko
{
template<typename T>
struct Vec2
{
    T x{}, y{};

    constexpr Vec2(T xArg, T yArg): x(xArg), y(yArg){}
    constexpr Vec2()= default;
    constexpr Vec2(T scalar): x(scalar), y(scalar){}
	template<typename OtherT>
	constexpr explicit Vec2(Vec2<OtherT> v): x(static_cast<T>(v.x)), y(static_cast<T>(v.y)){}
	template<typename OtherT>
	constexpr explicit Vec2(OtherT x, OtherT y): x(static_cast<T>(x)), y(static_cast<T>(y)){}
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
    constexpr Vec2 operator-() const
    {
        return { -x, -y };
    }
    constexpr Vec2 operator-(Vec2 other) const
    {
        return { x - other.x, y - other.y };
    }
    constexpr Vec2& operator-=(Vec2 other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }
    constexpr Vec2 operator*(Vec2 other) const
    {
        return { x * other.x, y * other.y };
    }
    constexpr Vec2& operator*=(T other)
    {
        x *= other;
        y *= other;
        return *this;
    }
    constexpr Vec2 operator*(T other) const
    {
        return { x * other, y * other };
    }

    constexpr Vec2 operator/(T other) const
    {
        if (other == T{ 0 })
        {
            return Vec2{ std::numeric_limits<T>::max(), std::numeric_limits<T>::max() };
        }
        return Vec2{ x/other, y/other };
    }
    constexpr Vec2& operator/=(T other)
    {
        x /= other;
        y /= other;
        return *this;
    }
    static constexpr T Dot(Vec2 v1, Vec2 v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }
    static constexpr Vec2 zero() { return {}; }
    static constexpr Vec2 up() { return Vec2{ Scalar{0},Scalar{1} }; }
    static constexpr Vec2 down() { return Vec2{ Scalar{0},Scalar{-1} }; }
    static constexpr Vec2 left() { return Vec2{ Scalar{-1},Scalar{0} }; }
    static constexpr Vec2 right() { return Vec2{ Scalar{1},Scalar{0} }; }
    static constexpr Vec2 one() { return Vec2{ Scalar{1},Scalar{1} }; }

    /**
     *  @brief Rotate the 2d vector of a radian angle
     */
    [[nodiscard]] Vec2 Rotate(T angle) const noexcept
    {
        const auto sin = Sin(angle);
        const auto cos = Cos(angle);
        return { (cos * x) - (sin * y) , (sin * x) + (cos * y) };
    }

    [[nodiscard]] constexpr Vec2 Perpendicular() const
    {
        return { y, -x };
    }

    [[nodiscard]] constexpr Vec2 Perpendicular2() const
    {
        return { -y, x };
    }

    [[nodiscard]] constexpr T SquareLength() const
    {
        return Dot(*this, *this);
    }

    [[nodiscard]] T Length() const
    {
        return Sqrt(SquareLength());
    }
    [[nodiscard]] Vec2 Normalized() const
    {
        return *this / Length();
    }
    [[nodiscard]] constexpr static Vec2 Clamp(Vec2 v, Vec2 minV, Vec2 maxV)
    {
        return { neko::Clamp(v.x, minV.x, maxV.x), neko::Clamp(v.y, minV.y, maxV.y) };
    }

    constexpr bool operator==(Vec2 other) const
    {
        return x == other.x && y == other.y;
    }

	constexpr bool operator!=(Vec2 other) const
	{
		return x != other.x || y != other.y;
	}
};

template<typename T>
constexpr Vec2<T> operator*(Scalar f, Vec2<T> other)
{
    return { f * other.x, f * other.y };
}


using Vec2f = Vec2<Scalar>;
using Vec2u = Vec2<unsigned>;
using Vec2i = Vec2<int>;
} // namespace neko
