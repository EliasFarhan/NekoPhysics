#pragma once

#include <math/fixed.h>
#include <cmath>

namespace neko
{

using Scalar = Fixed16;

template<typename T>
constexpr T Pi()
{
    return 3.14159265358979323846264338327950288;
}

template<typename T>
constexpr T Exp()
{
	return 2.718281828459045;
}

template<>
constexpr inline Fixed16 Pi()
{
    return pi;
}

template<>
constexpr inline Fixed16 Exp()
{
	return exp;
}

}