#pragma once

#include <math/fixed.h>

namespace neko
{

using Scalar = float;

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