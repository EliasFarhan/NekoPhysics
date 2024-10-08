#pragma once

#include <cstdint>
#include <cmath>

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

	template<class OtherT>
	requires std::is_floating_point_v<OtherT>
    constexpr explicit Fixed(OtherT f)
    {
        underlyingValue_ = static_cast<T>((f) * (1 << Exp));
    }

	template<class OtherT>
	requires std::is_integral_v<OtherT>
    constexpr explicit Fixed(OtherT n)
    {
        underlyingValue_ = static_cast<T>(n) << Exp;
    }

    constexpr explicit Fixed(T up, T down)
    {
        underlyingValue_ = (Fixed{ up } / Fixed{down}).underlyingValue_;
    }

	template<typename OtherT, OtherT OtherExp=16, typename OtherMulT>
	constexpr explicit Fixed(Fixed<OtherT, OtherExp, OtherMulT> other)
	{
		underlyingValue_ = static_cast<T>(other.underlyingValue()) << (Exp-OtherExp);
	}

    explicit constexpr operator float() const
    {
        return (static_cast<float>(underlyingValue_) / (1 << Exp));
    }

    explicit constexpr operator T() const
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
        Fixed result{};
        MulT leftOp = static_cast<MulT>(underlyingValue_) * (static_cast<MulT>(1) << static_cast<MulT>(Exp));
        MulT rightOp = static_cast<MulT>(other.underlyingValue_);
        result.underlyingValue_ = static_cast<T>((leftOp / rightOp));
        return result;
    }

    constexpr Fixed& operator/=(Fixed other)
    {
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

    static constexpr Fixed fromUnderlyingValue(T value)
    {
        Fixed f;
        f.underlyingValue_ = value;
        return f;
    }
    [[nodiscard]] constexpr T underlyingValue() const { return underlyingValue_; }
    [[nodiscard]] T& underlyingValue() { return underlyingValue_; }
private:
    T underlyingValue_ = 0;
};

using Fixed16 = Fixed<std::int32_t, 16, std::int64_t>;
using Fixed8 = Fixed<std::int8_t, 6, std::int16_t>;

constexpr Fixed pi = Fixed16::fromUnderlyingValue(205887);
constexpr Fixed exp = Fixed16::fromUnderlyingValue(178145);


} // namespace neko
