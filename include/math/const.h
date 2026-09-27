#pragma once

#include "math/fixed.h"
#ifdef NEKO_SCALAR_SIXIT
#include "math/sixit_float.h"
#endif
#include <cmath>
#include <numbers>

namespace neko
{

#if defined(NEKO_SCALAR_SIXIT)
using Scalar = sixit::dmath::ieee_float_inline_asm;
#elif defined(NEKO_SCALAR_FLOAT)
using Scalar = float;
#else
using Scalar = Fixed16;
#endif

template<typename T>
constexpr T Pi()
{
#ifdef NEKO_SCALAR_SIXIT
    if constexpr (std::is_same_v<T, sixit::dmath::ieee_float_inline_asm>)
    {
        return sixit::dmath::ieee_float_inline_asm(std::numbers::pi_v<float>);
    }
    else
#endif
    {
        return std::numbers::pi_v<T>;
    }
}

template<typename T>
constexpr T Exp()
{
#ifdef NEKO_SCALAR_SIXIT
    if constexpr (std::is_same_v<T, sixit::dmath::ieee_float_inline_asm>)
    {
        return sixit::dmath::ieee_float_inline_asm(std::numbers::e_v<float>);
    }
    else
#endif
    {
        return std::numbers::e_v<T>;
    }
}

template<>
constexpr Fixed16 Pi()
{
    return Fixed16::fromUnderlyingValue(205887);
}

template<>
constexpr Fixed16 Exp()
{
    return Fixed16::fromUnderlyingValue(178145);
}

}