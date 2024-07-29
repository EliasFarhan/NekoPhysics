#include "math/fixed_lut.h"
#include "math/const.h"

#include "generated/sin_const.h"
#include "generated/cos_const.h"
#include "generated/sqrt_const.h"

extern int sqrt_table[];
extern int sin_table[];
extern int cos_table[];

namespace neko
{
template<>
Fixed16 Sqrt(Fixed16 value)
{
    constexpr Fixed16 minValue = Fixed16::fromUnderlyingValue(sqrt_min);
    constexpr Fixed16 maxValue = Fixed16::fromUnderlyingValue(sqrt_max);
    Fixed16 result{ 1 };
    while(maxValue < value)
    {
        result *= Sqrt(maxValue);
        value /= maxValue;
    }
    constexpr Fixed16 delta = maxValue-minValue;
    const auto index = (value - minValue) / (delta) * Fixed16 { sqrt_len }; // v = a + (b-a)*t; <=> (v-a)/(b-a)
    const auto intIndex = static_cast<int>(index);
    const auto ratio = index - Fixed16{intIndex};
    if (intIndex == sqrt_len && ratio == Fixed{0})
    {
        return Fixed16::fromUnderlyingValue(sqrt_table[sqrt_len - 1]);
    }
    const auto v1 = Fixed16::fromUnderlyingValue(sqrt_table[intIndex]);
    const auto v2 = Fixed16::fromUnderlyingValue(sqrt_table[intIndex + 1]);
    return (v1 + (v2 - v1) * ratio) * result;

}

template<>
Fixed16 Sin(Fixed16 value)
{
    if (value < Fixed16{0})
    {
        value.underlyingValue() = (value.underlyingValue() % (pi.underlyingValue() * 2)) + (pi.underlyingValue() * 2);
    }

    if(value >= pi * Fixed16{2})
    {
        value.underlyingValue() = value.underlyingValue() % (pi.underlyingValue() * 2);
    }
    constexpr Fixed16 minValue = Fixed16::fromUnderlyingValue(0);
    constexpr Fixed16 maxValue = Pi<Fixed16>() * Fixed16{2};
    const auto index = (value-minValue)/(maxValue-minValue) * Fixed16 { sin_len }; // v = a + (b-a)*t; <=> (v-a)/(b-a)
    const auto ratio = index - Fixed16{static_cast<int>(index)};
    const auto v1 = Fixed16::fromUnderlyingValue(sin_table[static_cast<int>(index)]);
    const auto v2 = Fixed16::fromUnderlyingValue(sin_table[static_cast<int>(index)+1]);
    return v1 + (v2 - v1) * ratio;
}

template<>
Fixed16 Cos(Fixed16 value)
{
    if (value < Fixed16{0})
    {
        value.underlyingValue() = (value.underlyingValue() % (pi.underlyingValue() * 2)) + (pi.underlyingValue() * 2);
    }

    if (value >= pi * Fixed16{2})
    {
        value.underlyingValue() = value.underlyingValue() % (pi.underlyingValue() * 2);
    }

    constexpr Fixed16 minValue = Fixed16::fromUnderlyingValue(0);
    constexpr Fixed16 maxValue = Pi<Fixed16>() * Fixed16{2};
    const auto index = (value-minValue)/(maxValue-minValue) * Fixed16 { cos_len }; // v = a + (b-a)*t; <=> (v-a)/(b-a)
    const auto ratio = index - Fixed16{static_cast<int>(index)};
    const auto v1 = Fixed16::fromUnderlyingValue(cos_table[static_cast<int>(index)]);
    const auto v2 = Fixed16::fromUnderlyingValue(cos_table[static_cast<int>(index) + 1]);
    return v1 + (v2 - v1) * ratio;
}
}