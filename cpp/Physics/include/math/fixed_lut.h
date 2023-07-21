#pragma once
#include "math/fixed.h"
#include <cmath>

namespace neko
{

template<typename T>
T Sqrt(T value)
{
    return std::sqrt(value);
}

template<>
Fixed16 Sqrt(Fixed16 value);

template<typename T>
T Sin(T value)
{
    return std::sin(value);
}

template<>
Fixed16 Sin(Fixed16 value);

template<typename T>
T Cos(T value)
{
    return std::cos(value);
}

template<>
Fixed16 Cos(Fixed16 value);

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