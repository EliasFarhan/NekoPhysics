#pragma once

#include "math/fixed.h"
#include "math/sixit_float.h"
#include <cmath>
#include <numbers>

namespace neko
{

using Scalar = sixit::dmath::ieee_float_inline_asm;

template<typename T>
constexpr T Pi()
{
    if constexpr (std::is_same_v<T, sixit::dmath::ieee_float_inline_asm>)
    {
        return sixit::dmath::ieee_float_inline_asm(std::numbers::pi_v<float>);
    }
    else
    {
        return std::numbers::pi_v<T>;
    }
}

template<typename T>
constexpr T Exp()
{
    if constexpr (std::is_same_v<T, sixit::dmath::ieee_float_inline_asm>)
    {
        return sixit::dmath::ieee_float_inline_asm(std::numbers::e_v<float>);
    }
    else
    {
        return std::numbers::e_v<T>;
    }
}

template<>
constexpr Fixed16 Pi()
{
    return pi;
}

template<>
constexpr Fixed16 Exp()
{
	return exp;
}

}