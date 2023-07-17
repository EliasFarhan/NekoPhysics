#pragma once

#include <cstdint>

namespace neko
{

/**
 * @brief Fixed-point number
 */
template<typename T=std::int32_t, T Exp=16, typename MulT=std::int64_t>
class Fixed
{
public:
    constexpr Fixed() = default;
    constexpr explicit Fixed(float f)
    {
        underlyingValue_ = static_cast<int>((f) * (1 << Exp));
    }

    constexpr explicit Fixed(std::size_t n)
    {
        underlyingValue_ = static_cast<T>(n) << Exp;
    }

    constexpr explicit Fixed(T i)
    {
        underlyingValue_ = i << Exp;
    }

    constexpr explicit Fixed(T up, T down)
    {
        constexpr auto tmp = Fixed(up) / Fixed(down);
        underlyingValue_ = tmp.underlyingValue();
    }

    explicit constexpr operator float() const
    {
        return (static_cast<float>(underlyingValue_) / (1 << Exp));
    }

    explicit constexpr operator int() const
    {
        return underlyingValue_ >> Exp;
    }

    explicit constexpr operator std::size_t() const
    {
        return static_cast<std::size_t>(underlyingValue_ >> Exp);
    }

    constexpr Fixed operator+(Fixed other) const
    {
        Fixed result;
        result.underlyingValue_ = underlyingValue_ + other.underlyingValue_;
        return result;
    }

    constexpr Fixed& operator+=(Fixed other)
    {
        underlyingValue_ += other.underlyingValue_;
        return *this;
    }

    constexpr Fixed operator-(Fixed other) const
    {
        Fixed result;
        result.underlyingValue_ = underlyingValue_ - other.underlyingValue_;
        return result;
    }

    constexpr Fixed& operator-=(Fixed other)
    {
        underlyingValue_ -= other.underlyingValue_;
        return *this;
    }

    constexpr Fixed operator-() const
    {
        Fixed result;
        result.underlyingValue_ = -underlyingValue_;
        return result;
    }

    constexpr Fixed operator*(Fixed other) const
    {
        Fixed result;
        MulT leftOp = static_cast<MulT>(underlyingValue_);
        MulT rightOp = static_cast<MulT>(other.underlyingValue_);
        result.underlyingValue_ = static_cast<T>((leftOp * rightOp) >> static_cast<MulT>(Exp));
        return result;
    }

    constexpr Fixed& operator*=(Fixed other)
    {
        MulT leftOp = static_cast<MulT>(underlyingValue_);
        MulT rightOp = static_cast<MulT>(other.underlyingValue_);
        underlyingValue_ = static_cast<T>((leftOp * rightOp) >> static_cast<MulT>(Exp));
        return *this;
    }

    constexpr Fixed operator/(Fixed other) const
    {
        Fixed result;
        MulT leftOp = static_cast<MulT>(underlyingValue_) << static_cast<MulT>(Exp);
        MulT rightOp = static_cast<MulT>(other.underlyingValue_);
        result.underlyingValue_ = static_cast<T>((leftOp / rightOp));
        return result;
    }

    constexpr Fixed& operator/=(Fixed other)
    {
        Fixed result;
        MulT leftOp = static_cast<MulT>(underlyingValue_) << static_cast<MulT>(Exp);
        MulT rightOp = static_cast<MulT>(other.underlyingValue_);
        underlyingValue_ = static_cast<T>((leftOp / rightOp));
        return *this;
    }

    constexpr bool operator==(Fixed f2) const
    {
        return underlyingValue_ == f2.underlyingValue_;
    }

    constexpr bool operator!=(Fixed f2) const
    {
        return underlyingValue_ != f2.underlyingValue_;
    }

    constexpr bool operator>(Fixed f2) const
    {
        return underlyingValue_ > f2.underlyingValue_;
    }

    constexpr bool operator>=(Fixed f2) const
    {
        return underlyingValue_ >= f2.underlyingValue_;
    }

    constexpr bool operator<(Fixed f2) const
    {
        return underlyingValue_ < f2.underlyingValue_;
    }

    constexpr bool operator<=(Fixed f2) const
    {
        return underlyingValue_ <= f2.underlyingValue_;
    }

    static Fixed Sqrt(Fixed f)
    {
        return Fixed{ std::sqrt(float{f}) };
    }

    static Fixed Sin(Fixed f)
    {
        return Fixed{ std::sin(float{f}) };
    }

    static Fixed Cos(Fixed f)
    {
        return Fixed{ std::cos(float{f}) };
    }

    [[nodiscard]] constexpr int underlyingValue() const { return underlyingValue_; }
    [[nodiscard]] int& underlyingValue() { return underlyingValue_; }
private:
    T underlyingValue_ = 0;
};

using Fixed16 = Fixed<std::int32_t, 16, std::int64_t>;



template<typename T>
T Sqrt(T value)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::sqrt(value);
    }
    else
    {
        return T::Sqrt(value);
    }
}

template<typename T>
T Sin(T value)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::sin(value);
    }
    else
    {
        return T::Sin(value);
    }
}

template<typename T>
T Cos(T value)
{
    if constexpr (std::is_floating_point_v<T>)
    {
        return std::cos(value);
    }
    else
    {
        return T::Cos(value);
    }
}


} // namespace neko
