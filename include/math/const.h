#pragma once

#include "math/fixed.h"
#include "math/sixit_float.h"
#include <cmath>

namespace neko
{

using Scalar = sixit::dmath::ieee_float_inline_asm;

template<typename T>
constexpr T Pi()
{
    return static_cast<T>(3.14159265358979323846264338327950288);
}

template<typename T>
constexpr T Exp()
{
	return static_cast<T>(2.718281828459045);
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