#pragma once

#include <math/fixed.h>
#include <cmath>

namespace neko
{

using Scalar = Fixed16;

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