#ifndef NEKO_MATH_FUNC_H_
#define NEKO_MATH_FUNC_H_

#include <cmath>

namespace neko {

/// Absolute value.
///
/// Generic over `float`, @ref Fixed and any type with comparison and
/// unary minus, unlike `std::abs`.
///
/// @tparam T The number type.
/// @param value The value.
/// @return @p value if it is positive or zero, `-value` otherwise.
template <typename T>
constexpr T Abs(T value) {
  if (value >= T{0.0f}) {
    return value;
  }
  return -value;
}

/// Restricts a value to the range [ @p min_value , @p max_value ].
///
/// @tparam T The number type.
/// @param value The value to clamp.
/// @param min_value The lower bound.
/// @param max_value The upper bound.
/// @return @p max_value if @p value is above it, @p min_value if it is
/// below it, @p value otherwise.
/// @pre `min_value <= max_value`.
template <typename T>
constexpr T Clamp(T value, T min_value, T max_value) {
  if (value > max_value) {
    return max_value;
  }
  if (value < min_value) {
    return min_value;
  }
  return value;
}

/// Larger of two values.
///
/// @tparam T The compared type.
/// @param v1 First value.
/// @param v2 Second value.
/// @return A reference to the larger one (@p v2 when they are equal).
template <typename T>
constexpr const T& Max(const T& v1, const T& v2) {
  return v1 > v2 ? v1 : v2;
}

/// Smaller of two values.
///
/// @tparam T The compared type.
/// @param v1 First value.
/// @param v2 Second value.
/// @return A reference to the smaller one (@p v2 when they are equal).
template <typename T>
constexpr const T& Min(const T& v1, const T& v2) {
  return v1 < v2 ? v1 : v2;
}

/// Sign of a value as -1 or 1.
///
/// Zero counts as positive, so the result is never 0: handy to pick a
/// direction without a special case.
///
/// @tparam T The number type.
/// @param v1 The value.
/// @return -1 if @p v1 is negative, 1 otherwise.
template <typename T>
constexpr T Sign(T v1) {
  if (v1 < T{0.0f}) {
    return T{-1.0f};
  }
  return T{1.0f};
}

/// Linear interpolation between two values.
///
/// @tparam T The number type.
/// @param begin The value at `t = 0`.
/// @param end The value at `t = 1`.
/// @param t The interpolation factor, usually in [0, 1] (not clamped).
/// @return `begin + (end - begin) * t`.
template <typename T>
constexpr T Lerp(T begin, T end, T t) {
  return (end - begin) * t + begin;
}

/// Square root, generic version based on `std::sqrt`.
///
/// Used for `float`. @ref Fixed16 has its own exact integer overload,
/// @ref Sqrt(Fixed16), and the sixit float type a specialization.
///
/// @tparam T The number type.
/// @param value The value, not negative.
/// @return The square root of `value`.
template <typename T>
T Sqrt(T value) {
  return std::sqrt(value);
}

/// Sine of an angle in radians, generic version based on `std::sin`.
///
/// @ref Fixed16 uses the table based overload from `neko/math/fixed_trig.h`.
/// Prefer @ref Sin(Radian) in game code so the unit is explicit.
///
/// @tparam T The number type.
/// @param value The angle in radians.
/// @return The sine of `value`.
template <typename T>
T Sin(T value) {
  return std::sin(value);
}

/// Cosine of an angle in radians, generic version based on `std::cos`.
///
/// @tparam T The number type.
/// @param value The angle in radians.
/// @return The cosine of `value`.
template <typename T>
T Cos(T value) {
  return std::cos(value);
}

/// Tangent of an angle in radians, generic version based on `std::tan`.
///
/// @tparam T The number type.
/// @param value The angle in radians.
/// @return The tangent of `value`.
template <typename T>
T Tan(T value) {
  return std::tan(value);
}
}  // namespace neko

#endif  // NEKO_MATH_FUNC_H_
