#include "math/fixed_lut.h"
#include "math/const.h"

#include "generated/sin_const.h"
#include "generated/cos_const.h"

extern int sin_table[];
extern int cos_table[];

namespace neko
{
namespace
{
// Wraps an angle in radians into [0, 2*pi)
Fixed16 WrapAngle(Fixed16 value)
{
    const auto twoPi = Pi<Fixed16>().underlyingValue() * 2;
    auto raw = value.underlyingValue() % twoPi;
    if (raw < 0)
    {
        raw += twoPi;
    }
    return Fixed16::fromUnderlyingValue(raw);
}
} // namespace

template<>
Fixed16 Sin(Fixed16 value)
{
    value = WrapAngle(value);
    constexpr Fixed16 minValue = Fixed16::fromUnderlyingValue(0);
    constexpr Fixed16 maxValue = Pi<Fixed16>() * Fixed16{2};
    const auto index = (value-minValue)/(maxValue-minValue) * Fixed16 { sin_len }; // v = a + (b-a)*t; <=> (v-a)/(b-a)
    const auto intIndex = static_cast<int>(index);
    if(intIndex >= sin_len - 1)
    {
        return Fixed16::fromUnderlyingValue(sin_table[sin_len - 1]);
    }
    const auto ratio = index - Fixed16{intIndex};
    const auto v1 = Fixed16::fromUnderlyingValue(sin_table[intIndex]);
    const auto v2 = Fixed16::fromUnderlyingValue(sin_table[intIndex+1]);
    return v1 + (v2 - v1) * ratio;
}

template<>
Fixed16 Cos(Fixed16 value)
{
    value = WrapAngle(value);
    constexpr Fixed16 minValue = Fixed16::fromUnderlyingValue(0);
    constexpr Fixed16 maxValue = Pi<Fixed16>() * Fixed16{2};
    const auto index = (value-minValue)/(maxValue-minValue) * Fixed16 { cos_len }; // v = a + (b-a)*t; <=> (v-a)/(b-a)
    const auto intIndex = static_cast<int>(index);
    if(intIndex >= cos_len - 1)
    {
        return Fixed16::fromUnderlyingValue(cos_table[cos_len - 1]);
    }
    const auto ratio = index - Fixed16{intIndex};
    const auto v1 = Fixed16::fromUnderlyingValue(cos_table[intIndex]);
    const auto v2 = Fixed16::fromUnderlyingValue(cos_table[intIndex + 1]);
    return v1 + (v2 - v1) * ratio;
}
}
