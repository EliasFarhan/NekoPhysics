#pragma once
#include "math/fixed.h"
#include <cmath>

namespace neko
{

template<typename T>
constexpr T Abs(T value)
{
    if(value >= T{0.0f})
        return value;
    return -value;
}

template<typename T>
constexpr T Clamp(T value, T minValue, T maxValue)
{
	if(value > maxValue)
		return maxValue;
	if(value < minValue)
		return minValue;
	return value;
}

template<typename T>
constexpr T Max(T v1, T v2)
{
	return v1 > v2 ? v1 : v2;
}

template<typename T>
constexpr T Min(T v1, T v2)
{
	return v1 < v2 ? v1 : v2;
}

template<typename T>
constexpr T Sign(T v1)
{
	if(v1 < T{0.0f})
	{
		return T{-1.0f};
	}
	return T{1.0f};
}

template<typename T>
constexpr T Lerp(T begin, T end, T t)
{
	return (end-begin)*t+begin;
}

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



}