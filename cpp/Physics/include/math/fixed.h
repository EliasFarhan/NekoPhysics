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

    constexpr explicit Fixed(T i)
    {
        underlyingValue_ = i << Exp;
    }

    explicit constexpr operator float() const
    {
        return (static_cast<float>(underlyingValue_) / (1 << Exp));
    }

    explicit constexpr operator int() const
    {
        return underlyingValue_ >> Exp;
    }

    constexpr Fixed operator+(Fixed other) const
    {
        Fixed result;
        result.underlyingValue_ = underlyingValue_ + other.underlyingValue_;
        return result;
    }

    constexpr Fixed& operator+=(Fixed other)
    {
        underlyingValue_ += underlyingValue_ + other.underlyingValue_;
        return *this;
    }

    constexpr Fixed operator-(Fixed other) const
    {
        Fixed result;
        result.underlyingValue_ = underlyingValue_ - other.underlyingValue_;
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

    constexpr Fixed operator/(Fixed other) const
    {
        Fixed result;
        MulT leftOp = static_cast<MulT>(underlyingValue_) << static_cast<MulT>(Exp);
        MulT rightOp = static_cast<MulT>(other.underlyingValue_);
        result.underlyingValue_ = static_cast<T>((leftOp / rightOp));
        return result;
    }

    [[nodiscard]] constexpr int underlyingValue() const { return underlyingValue_; }
    [[nodiscard]] int& underlyingValue() { return underlyingValue_; }
private:
    T underlyingValue_ = 0;
};

} // namespace neko
