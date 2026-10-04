#pragma once

#include "neko/math/fixed.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace neko
{
namespace detail
{
inline constexpr std::size_t SIN_TABLE_BITS = 10;
inline constexpr std::size_t SIN_TABLE_SIZE = std::size_t{ 1 } << SIN_TABLE_BITS;

/**
 * @brief Compile-time sine (Taylor series on [-pi/2, pi/2]), only used to build the table
 */
consteval double ConstevalSin(double x)
{
    constexpr double pi = 3.14159265358979323846;
    while (x > pi)
    {
        x -= 2.0 * pi;
    }
    while (x < -pi)
    {
        x += 2.0 * pi;
    }
    if (x > pi / 2.0)
    {
        x = pi - x;
    }
    else if (x < -pi / 2.0)
    {
        x = -pi - x;
    }
    const double square = x * x;
    double term = x;
    double sum = x;
    for (int n = 1; n < 12; ++n)
    {
        term *= -square / static_cast<double>((2 * n) * (2 * n + 1));
        sum += term;
    }
    return sum;
}

/**
 * @brief sin over one full turn in 16.16, plus the first value again so interpolation never wraps
 */
consteval std::array<std::int32_t, SIN_TABLE_SIZE + 1> MakeSinTable()
{
    constexpr double pi = 3.14159265358979323846;
    std::array<std::int32_t, SIN_TABLE_SIZE + 1> table{};
    for (std::size_t i = 0; i <= SIN_TABLE_SIZE; ++i)
    {
        const double value = ConstevalSin(2.0 * pi * static_cast<double>(i) / static_cast<double>(SIN_TABLE_SIZE)) * 65536.0;
        table[i] = static_cast<std::int32_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
    }
    return table;
}

inline constexpr auto sinTable = MakeSinTable();

/**
 * @brief Angle in radians to a phase in 1/2^32 of a turn. One multiplication, no division, and the wrap-around to
 * [0, 2 pi) is the uint32_t overflow.
 */
[[nodiscard]] constexpr std::uint32_t RadiansToPhase(Fixed16 radians) noexcept
{
    // round(2^32 / (2 pi))
    constexpr std::int64_t phasePerRadian = 683565276;
    return static_cast<std::uint32_t>((static_cast<std::int64_t>(radians.underlyingValue()) * phasePerRadian) >> 16);
}

[[nodiscard]] constexpr Fixed16 SinFromPhase(std::uint32_t phase) noexcept
{
    constexpr std::uint32_t fractionBits = 32 - SIN_TABLE_BITS;
    const auto index = phase >> fractionBits;
    const auto fraction = static_cast<std::int64_t>(phase & ((std::uint32_t{ 1 } << fractionBits) - 1));
    const auto v1 = sinTable[index];
    const auto v2 = sinTable[index + 1];
    constexpr std::int64_t half = std::int64_t{ 1 } << (fractionBits - 1);
    const auto interpolation = (static_cast<std::int64_t>(v2 - v1) * fraction + half) >> fractionBits;
    return Fixed16::fromUnderlyingValue(static_cast<std::int32_t>(v1 + interpolation));
}
} // namespace detail

/**
 * @brief Sine of an angle in radians. Table + linear interpolation, error below 5e-5, no division.
 */
[[nodiscard]] constexpr Fixed16 Sin(Fixed16 radians) noexcept
{
    return detail::SinFromPhase(detail::RadiansToPhase(radians));
}

[[nodiscard]] constexpr Fixed16 Cos(Fixed16 radians) noexcept
{
    // cos(x) = sin(x + quarter turn)
    return detail::SinFromPhase(detail::RadiansToPhase(radians) + (std::uint32_t{ 1 } << 30));
}

[[nodiscard]] constexpr Fixed16 Tan(Fixed16 radians) noexcept
{
    return Sin(radians) / Cos(radians);
}

} // namespace neko
