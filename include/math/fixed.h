#pragma once

#include <compare>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace neko
{

namespace detail
{
/**
 * @brief Floor of the square root of a 64-bit integer, computed bit by bit.
 * Exact, so it matches the DS hardware square root unit (REG_SQRTCNT, 64-bit mode).
 */
[[nodiscard]] constexpr std::uint32_t ISqrt64(std::uint64_t value) noexcept
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
} // namespace detail

/**
 * @brief Fixed-point number
 *
 * Rounding rules, which the DS port must reproduce bit for bit:
 * - multiplication rounds to nearest, ties away from zero;
 * - division truncates toward zero (like the DS divider) and saturates when dividing by zero;
 * - conversion from floating point rounds to nearest, conversion to integer truncates toward zero.
 */
template<typename T=std::int32_t, T Exp=16, typename MulT=std::int64_t>
class Fixed
{
public:
    constexpr Fixed() = default;

    template<class OtherT>
    requires std::is_floating_point_v<OtherT>
    constexpr explicit Fixed(OtherT f)
    {
        const OtherT scaled = f * static_cast<OtherT>(one);
        underlyingValue_ = static_cast<T>(scaled >= OtherT{ 0 } ? scaled + OtherT{ 0.5 } : scaled - OtherT{ 0.5 });
    }

    template<class OtherT>
    requires std::is_integral_v<OtherT>
    constexpr explicit Fixed(OtherT n) : underlyingValue_(static_cast<T>(static_cast<T>(n) * one))
    {
    }

    constexpr explicit Fixed(T up, T down) : underlyingValue_((Fixed{ up } / Fixed{ down }).underlyingValue_)
    {
    }

    template<typename OtherT, OtherT OtherExp, typename OtherMulT>
    constexpr explicit Fixed(Fixed<OtherT, OtherExp, OtherMulT> other)
    {
        const auto otherValue = static_cast<MulT>(other.underlyingValue());
        if constexpr (Exp >= OtherExp)
        {
            underlyingValue_ = static_cast<T>(otherValue * (MulT{ 1 } << (Exp - OtherExp)));
        }
        else
        {
            underlyingValue_ = static_cast<T>(otherValue / (MulT{ 1 } << (OtherExp - Exp)));
        }
    }

    [[nodiscard]] explicit constexpr operator float() const noexcept
    {
        return static_cast<float>(underlyingValue_) / static_cast<float>(one);
    }

    [[nodiscard]] explicit constexpr operator T() const noexcept
    {
        return static_cast<T>(underlyingValue_ / one);
    }

    [[nodiscard]] constexpr Fixed operator+(Fixed other) const noexcept
    {
        return fromUnderlyingValue(static_cast<T>(underlyingValue_ + other.underlyingValue_));
    }

    constexpr Fixed& operator+=(Fixed other) noexcept
    {
        return *this = *this + other;
    }

    [[nodiscard]] constexpr Fixed operator-(Fixed other) const noexcept
    {
        return fromUnderlyingValue(static_cast<T>(underlyingValue_ - other.underlyingValue_));
    }

    constexpr Fixed& operator-=(Fixed other) noexcept
    {
        return *this = *this - other;
    }

    [[nodiscard]] constexpr Fixed operator-() const noexcept
    {
        return fromUnderlyingValue(static_cast<T>(-underlyingValue_));
    }

    [[nodiscard]] constexpr Fixed operator*(Fixed other) const noexcept
    {
        const MulT product = static_cast<MulT>(underlyingValue_) * static_cast<MulT>(other.underlyingValue_);
        // + half, minus one ulp of the product when negative, then floor: rounds half away from zero
        constexpr MulT half = MulT{ 1 } << (Exp - 1);
        const MulT negative = product < 0 ? MulT{ 1 } : MulT{ 0 };
        return fromUnderlyingValue(static_cast<T>((product + half - negative) >> Exp));
    }

    constexpr Fixed& operator*=(Fixed other) noexcept
    {
        return *this = *this * other;
    }

    [[nodiscard]] constexpr Fixed operator/(Fixed other) const noexcept
    {
        if (other.underlyingValue_ == 0)
        {
            if (underlyingValue_ == 0)
            {
                return Fixed{};
            }
            return fromUnderlyingValue(underlyingValue_ > 0 ? std::numeric_limits<T>::max() : std::numeric_limits<T>::min());
        }
        // DS port: this 64/32 division is what the hardware divider does (REG_DIVCNT, 64/32 mode)
        const MulT numerator = static_cast<MulT>(underlyingValue_) * static_cast<MulT>(one);
        return fromUnderlyingValue(static_cast<T>(numerator / static_cast<MulT>(other.underlyingValue_)));
    }

    constexpr Fixed& operator/=(Fixed other) noexcept
    {
        return *this = *this / other;
    }

    [[nodiscard]] constexpr bool operator==(const Fixed& other) const noexcept = default;
    [[nodiscard]] constexpr auto operator<=>(const Fixed& other) const noexcept = default;

    [[nodiscard]] static constexpr Fixed fromUnderlyingValue(T value) noexcept
    {
        Fixed f;
        f.underlyingValue_ = value;
        return f;
    }
    [[nodiscard]] constexpr T underlyingValue() const noexcept { return underlyingValue_; }

private:
    static constexpr T one = T{ 1 } << Exp;
    T underlyingValue_ = 0;
};

using Fixed16 = Fixed<std::int32_t, 16, std::int64_t>;
using Fixed8 = Fixed<std::int8_t, 6, std::int16_t>;

template<typename T>
inline constexpr bool is_fixed_v = false;

template<typename T, T Exp, typename MulT>
inline constexpr bool is_fixed_v<Fixed<T, Exp, MulT>> = true;

/**
 * @brief Exact square root (floor), negative values return 0.
 */
[[nodiscard]] constexpr Fixed16 Sqrt(Fixed16 value) noexcept
{
    if (value.underlyingValue() <= 0)
    {
        return Fixed16{};
    }
    // sqrt(raw / 2^16) * 2^16 == sqrt(raw * 2^16)
    const auto squared = static_cast<std::uint64_t>(value.underlyingValue()) << 16;
    return Fixed16::fromUnderlyingValue(static_cast<std::int32_t>(detail::ISqrt64(squared)));
}

} // namespace neko

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

    [[nodiscard]] static constexpr Fixed min() noexcept { return Fixed::fromUnderlyingValue(T{ 1 }); }
    [[nodiscard]] static constexpr Fixed max() noexcept { return Fixed::fromUnderlyingValue(std::numeric_limits<T>::max()); }
    [[nodiscard]] static constexpr Fixed lowest() noexcept { return Fixed::fromUnderlyingValue(std::numeric_limits<T>::min()); }
    [[nodiscard]] static constexpr Fixed epsilon() noexcept { return Fixed::fromUnderlyingValue(T{ 1 }); }
    [[nodiscard]] static constexpr Fixed round_error() noexcept { return Fixed{ 0.5f }; }
};
