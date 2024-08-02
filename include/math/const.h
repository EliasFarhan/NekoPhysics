#pragma once

#include <math/fixed.h>
#include <cmath>

namespace neko
{

using Scalar = Fixed16;

template<typename T>
constexpr T Pi()
{
    return M_PI;
}

template<>
constexpr inline Fixed16 Pi()
{
    return pi;
}

}