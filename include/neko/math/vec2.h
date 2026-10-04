#ifndef NEKO_MATH_VEC2_H_
#define NEKO_MATH_VEC2_H_

#include <type_traits>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <limits>

#include "neko/math/const.h"
#include "neko/math/fixed_trig.h"
#include "neko/math/func.h"

namespace neko
{
namespace detail
{
/**
 * @brief Square that can't overflow: Q32.32 in 64 bits for Fixed16 (16.16 overflows past ~181), T otherwise
 */
template<typename T>
[[nodiscard]] constexpr auto WideSquare(T value) noexcept
{
    if constexpr (std::is_same_v<T, Fixed16>)
    {
        const auto raw = static_cast<std::int64_t>(value.underlying_value());
        return static_cast<std::uint64_t>(raw * raw);
    }
    else
    {
        return value * value;
    }
}
} // namespace detail

template<typename T>
struct Vec2
{
    T x{}, y{};

    constexpr Vec2(T x_arg, T y_arg): x(x_arg), y(y_arg){}
    constexpr Vec2()= default;
    constexpr Vec2(T scalar): x(scalar), y(scalar){}
	template<typename OtherT>
	constexpr explicit Vec2(Vec2<OtherT> v): x(static_cast<T>(v.x)), y(static_cast<T>(v.y)){}

	template<typename OtherT>
	constexpr explicit Vec2(const OtherT& v)
	{
		x = static_cast<T>(v.x);
		y = static_cast<T>(v.y);
	}

	template<typename OtherT>
	constexpr explicit Vec2(OtherT x_arg, OtherT y_arg): x(static_cast<T>(x_arg)), y(static_cast<T>(y_arg)){}
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
    static constexpr Vec2 Zero() { return {}; }
    static constexpr Vec2 Up() { return Vec2{ Scalar{0},Scalar{1} }; }
    static constexpr Vec2 Down() { return Vec2{ Scalar{0},Scalar{-1} }; }
    static constexpr Vec2 Left() { return Vec2{ Scalar{-1},Scalar{0} }; }
    static constexpr Vec2 Right() { return Vec2{ Scalar{1},Scalar{0} }; }
    static constexpr Vec2 One() { return Vec2{ Scalar{1},Scalar{1} }; }

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
        if constexpr (std::is_same_v<T, Fixed16>)
        {
            // Sum of squares in Q32.32: SquareLength() overflows past ~181 and rounds to zero below ~1/256
            const auto root = detail::ISqrt64(detail::WideSquare(x) + detail::WideSquare(y));
            constexpr auto max_raw = static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max());
            return T::FromUnderlyingValue(static_cast<std::int32_t>(root > max_raw ? max_raw : root));
        }
        else
        {
            return Sqrt(SquareLength());
        }
    }
    /**
     * @brief Unit vector with the same direction, or zero for a zero vector
     */
    [[nodiscard]] Vec2 Normalized() const
    {
        const auto length = Length();
        if (length == T{ 0 })
        {
            return Zero();
        }
        return { x / length, y / length };
    }
    [[nodiscard]] constexpr static Vec2 Clamp(Vec2 v, Vec2 min_v, Vec2 max_v)
    {
        return { neko::Clamp(v.x, min_v.x, max_v.x), neko::Clamp(v.y, min_v.y, max_v.y) };
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

/**
 * @brief Compares the length of v with length (>= 0) without computing a square root or overflowing
 * @return negative if |v| < length, 0 if equal, positive if |v| > length
 */
template<typename T>
[[nodiscard]] constexpr int CompareLength(Vec2<T> v, T length) noexcept
{
    const auto square_length = detail::WideSquare(v.x) + detail::WideSquare(v.y);
    const auto length_square = detail::WideSquare(length);
    if (square_length < length_square)
    {
        return -1;
    }
    return length_square < square_length ? 1 : 0;
}

template<typename T>
constexpr Vec2<T> operator*(Scalar f, Vec2<T> other)
{
    return { f * other.x, f * other.y };
}


using Vec2f = Vec2<Scalar>;
using Vec2u = Vec2<unsigned>;
using Vec2i = Vec2<int>;
} // namespace neko

#endif  // NEKO_MATH_VEC2_H_
