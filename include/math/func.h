#pragma once

#include "sixit/core/guidelines.h"
#include <cmath>


constexpr uint16_t operator "" _u16(unsigned long long value)
{
	return sixit::guidelines::narrow_cast<std::uint16_t>(value);
}
constexpr uint8_t operator "" _u8(unsigned long long value)
{
	return sixit::guidelines::narrow_cast<std::uint8_t>(value);
}

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
constexpr const T& Max(const T& v1, const T& v2)
{
	return v1 > v2 ? v1 : v2;
}

template<typename T>
constexpr const T& Min(const T& v1, const T& v2)
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

template<typename T>
T Sin(T value)
{
	return std::sin(value);
}

template<typename T>
T Cos(T value)
{
	return std::cos(value);
}
}
