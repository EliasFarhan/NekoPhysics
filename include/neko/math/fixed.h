#ifndef NEKO_MATH_FIXED_H_
#define NEKO_MATH_FIXED_H_

#include <compare>
#include <cstdint>
#include <limits>
#include <type_traits>

#if defined(NEKO_PLATFORM_NDS)
#include <nds/arm9/math.h>
#endif

namespace neko
{

namespace internal
{
// Floor of the square root of a 64-bit integer, computed bit by bit.
// Exact, so it matches the DS hardware square root unit (REG_SQRTCNT, 64-bit mode).
[[nodiscard]] constexpr std::uint32_t ISqrt64Portable(std::uint64_t value) noexcept
{
    std::uint64_t result = 0;
    std::uint64_t bit = std::uint64_t{ 1 } << 62;
    while (bit > value)
    {
        bit >>= 2;
    }
    while (bit != 0)
    {
        if (value >= result + bit)
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return static_cast<std::uint32_t>(result);
}

// 64/32 signed division, truncated toward zero, keeping the low 32 bits of the quotient. den must not be 0.
[[nodiscard]] constexpr std::int32_t Div64Portable(std::int64_t num, std::int32_t den) noexcept
{
    return static_cast<std::int32_t>(num / den);
}

// DS hardware math hooks: the divider and the square root unit are shared memory-mapped registers, so the physics
// must not run inside IRQ handlers (an interrupted operation would return the handler's result).
// Constant evaluation keeps the portable path, and both paths give bit-exact results.

// Floor of the square root of a 64-bit integer (REG_SQRTCNT 64-bit mode on the DS).
[[nodiscard]] constexpr std::uint32_t ISqrt64(std::uint64_t value) noexcept
{
#if defined(NEKO_PLATFORM_NDS)
    // From 2^62 up (roots >= 2^31), melonDS 1.1 returns the root + 1; not checked on hardware, so these inputs
    // (only vector lengths >= 32768) stay on the portable path
    if !consteval
    {
        if (value < (std::uint64_t{ 1 } << 62))
        {
            return sqrt64(static_cast<long long>(value));
        }
    }
#endif
    return ISqrt64Portable(value);
}

// Same as Div64Portable (REG_DIVCNT 64/32 mode on the DS). den must not be 0.
[[nodiscard]] constexpr std::int32_t Div64(std::int64_t num, std::int32_t den) noexcept
{
#if defined(NEKO_PLATFORM_NDS)
    if !consteval
    {
        return div64(num, den);
    }
#endif
    return Div64Portable(num, den);
}
}  // namespace internal

// Fixed-point number
//
// Rounding rules, which the DS port must reproduce bit for bit:
// - multiplication rounds to nearest, ties away from zero;
// - division truncates toward zero (like the DS divider) and saturates when dividing by zero;
// - conversion from floating point rounds to nearest, conversion to integer truncates toward zero.
template<typename T=std::int32_t, T Exp=16, typename MulT=std::int64_t>
class Fixed
{
public:
    constexpr Fixed() = default;

    template<class OtherT>
    requires std::is_floating_point_v<OtherT>
    constexpr explicit Fixed(OtherT f)
    {
        const OtherT scaled = f * static_cast<OtherT>(kOne);
        underlying_value_ = static_cast<T>(scaled >= OtherT{ 0 } ? scaled + OtherT{ 0.5 } : scaled - OtherT{ 0.5 });
    }

    template<class OtherT>
    requires std::is_integral_v<OtherT>
    constexpr explicit Fixed(OtherT n) : underlying_value_(static_cast<T>(static_cast<T>(n) * kOne))
    {
    }

    constexpr explicit Fixed(T up, T down) : underlying_value_((Fixed{ up } / Fixed{ down }).underlying_value_)
    {
    }

    template<typename OtherT, OtherT OtherExp, typename OtherMulT>
    constexpr explicit Fixed(Fixed<OtherT, OtherExp, OtherMulT> other)
    {
        const auto other_value = static_cast<MulT>(other.underlying_value());
        if constexpr (Exp >= OtherExp)
        {
            underlying_value_ = static_cast<T>(other_value * (MulT{ 1 } << (Exp - OtherExp)));
        }
        else
        {
            underlying_value_ = static_cast<T>(other_value / (MulT{ 1 } << (OtherExp - Exp)));
        }
    }

    [[nodiscard]] explicit constexpr operator float() const noexcept
    {
        return static_cast<float>(underlying_value_) / static_cast<float>(kOne);
    }

    [[nodiscard]] explicit constexpr operator T() const noexcept
    {
        return static_cast<T>(underlying_value_ / kOne);
    }

    [[nodiscard]] constexpr Fixed operator+(Fixed other) const noexcept
    {
        return FromUnderlyingValue(static_cast<T>(underlying_value_ + other.underlying_value_));
    }

    constexpr Fixed& operator+=(Fixed other) noexcept
    {
        return *this = *this + other;
    }

    [[nodiscard]] constexpr Fixed operator-(Fixed other) const noexcept
    {
        return FromUnderlyingValue(static_cast<T>(underlying_value_ - other.underlying_value_));
    }

    constexpr Fixed& operator-=(Fixed other) noexcept
    {
        return *this = *this - other;
    }

    [[nodiscard]] constexpr Fixed operator-() const noexcept
    {
        return FromUnderlyingValue(static_cast<T>(-underlying_value_));
    }

    [[nodiscard]] constexpr Fixed operator*(Fixed other) const noexcept
    {
        const MulT product = static_cast<MulT>(underlying_value_) * static_cast<MulT>(other.underlying_value_);
        // + half, minus one ulp of the product when negative, then floor: rounds half away from zero
        constexpr MulT half = MulT{ 1 } << (Exp - 1);
        const MulT negative = product < 0 ? MulT{ 1 } : MulT{ 0 };
        return FromUnderlyingValue(static_cast<T>((product + half - negative) >> Exp));
    }

    constexpr Fixed& operator*=(Fixed other) noexcept
    {
        return *this = *this * other;
    }

    [[nodiscard]] constexpr Fixed operator/(Fixed other) const noexcept
    {
        if (other.underlying_value_ == 0)
        {
            if (underlying_value_ == 0)
            {
                return Fixed{};
            }
            return FromUnderlyingValue(underlying_value_ > 0 ? std::numeric_limits<T>::max() : std::numeric_limits<T>::min());
        }
        const MulT numerator = static_cast<MulT>(underlying_value_) * static_cast<MulT>(kOne);
        if constexpr (std::is_same_v<T, std::int32_t> && std::is_same_v<MulT, std::int64_t>)
        {
            // Hardware divider on the DS
            return FromUnderlyingValue(internal::Div64(numerator, other.underlying_value_));
        }
        else
        {
            return FromUnderlyingValue(static_cast<T>(numerator / static_cast<MulT>(other.underlying_value_)));
        }
    }

    constexpr Fixed& operator/=(Fixed other) noexcept
    {
        return *this = *this / other;
    }

    [[nodiscard]] constexpr bool operator==(const Fixed& other) const noexcept = default;
    [[nodiscard]] constexpr auto operator<=>(const Fixed& other) const noexcept = default;

    [[nodiscard]] static constexpr Fixed FromUnderlyingValue(T value) noexcept
    {
        Fixed f;
        f.underlying_value_ = value;
        return f;
    }
    [[nodiscard]] constexpr T underlying_value() const noexcept { return underlying_value_; }

private:
    static constexpr T kOne = T{ 1 } << Exp;
    T underlying_value_ = 0;
};

using Fixed16 = Fixed<std::int32_t, 16, std::int64_t>;
using Fixed8 = Fixed<std::int8_t, 6, std::int16_t>;

// Named like a standard type trait (std::is_floating_point_v)
// NOLINTBEGIN(readability-identifier-naming)
template<typename T>
inline constexpr bool is_fixed_v = false;

template<typename T, T Exp, typename MulT>
inline constexpr bool is_fixed_v<Fixed<T, Exp, MulT>> = true;
// NOLINTEND(readability-identifier-naming)

// Exact square root (floor), negative values return 0.
[[nodiscard]] constexpr Fixed16 Sqrt(Fixed16 value) noexcept
{
    if (value.underlying_value() <= 0)
    {
        return Fixed16{};
    }
    // sqrt(raw / 2^16) * 2^16 == sqrt(raw * 2^16)
    const auto squared = static_cast<std::uint64_t>(value.underlying_value()) << 16;
    return Fixed16::FromUnderlyingValue(static_cast<std::int32_t>(internal::ISqrt64(squared)));
}

}  // namespace neko

// Standard library interface names
// NOLINTBEGIN(readability-identifier-naming)
template<typename T, T Exp, typename MulT>
class std::numeric_limits<neko::Fixed<T, Exp, MulT>>
{
    using Fixed = neko::Fixed<T, Exp, MulT>;
public:
    static constexpr bool is_specialized = true;
    static constexpr bool is_signed = std::numeric_limits<T>::is_signed;
    static constexpr bool is_integer = false;
    static constexpr bool is_exact = true;
    static constexpr bool has_infinity = false;
    static constexpr bool has_quiet_NaN = false;
    static constexpr bool has_signaling_NaN = false;
    static constexpr bool is_bounded = true;
    static constexpr bool is_modulo = false;
    static constexpr int radix = 2;
    static constexpr int digits = std::numeric_limits<T>::digits;
    static constexpr std::float_round_style round_style = std::round_to_nearest;

    [[nodiscard]] static constexpr Fixed min() noexcept { return Fixed::FromUnderlyingValue(T{ 1 }); }
    [[nodiscard]] static constexpr Fixed max() noexcept { return Fixed::FromUnderlyingValue(std::numeric_limits<T>::max()); }
    [[nodiscard]] static constexpr Fixed lowest() noexcept { return Fixed::FromUnderlyingValue(std::numeric_limits<T>::min()); }
    [[nodiscard]] static constexpr Fixed epsilon() noexcept { return Fixed::FromUnderlyingValue(T{ 1 }); }
    [[nodiscard]] static constexpr Fixed round_error() noexcept { return Fixed{ 0.5f }; }
};
// NOLINTEND(readability-identifier-naming)

#endif  // NEKO_MATH_FIXED_H_
