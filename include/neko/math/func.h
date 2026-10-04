#ifndef NEKO_MATH_FUNC_H_
#define NEKO_MATH_FUNC_H_

#include <cmath>

namespace neko
{

template<typename T>
constexpr T Abs(T value)
{
	if(value >= T{0.0f})
	{
		return value;
	}
	return -value;
}

template<typename T>
constexpr T Clamp(T value, T min_value, T max_value)
{
	if(value > max_value)
	{
		return max_value;
	}
	if(value < min_value)
	{
		return min_value;
	}
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

template<typename T>
T Tan(T value)
{
	return std::tan(value);
}
}  // namespace neko

#endif  // NEKO_MATH_FUNC_H_
