#ifndef NEKO_MATH_ANGLE_H_
#define NEKO_MATH_ANGLE_H_

#include "neko/math/const.h"
#include "neko/math/fixed_trig.h"
#include "neko/math/func.h"

namespace neko {

class Degree;
/// An angle in radians, a strong type around @ref Scalar.
///
/// One full turn is 2 pi radians. Wrapping the number in a type makes the
/// unit explicit: a function taking a `Radian` cannot be passed degrees by
/// mistake, and the explicit conversions between `Radian` and @ref Degree do
/// the math for you. The value is not wrapped to one turn (0 to 2 pi): 3 pi
/// stays 3 pi.
///
/// `Radian` is what the trigonometric functions @ref Sin, @ref Cos
/// and @ref Tan take.
///
/// @code
/// const neko::Radian quarter{neko::Degree{neko::Scalar{90}}};
/// const neko::Scalar s = neko::Sin(quarter);  // 1
/// const neko::Radian half_turn = quarter * neko::Scalar{2};
/// @endcode
///
/// @see @ref Degree
class Radian {
 public:
  /// Constructs a zero angle.
  constexpr Radian() = default;
  /// Constructs an angle of @p value radians.
  constexpr explicit Radian(Scalar value) : value_(value) {}
  /// Converts a @ref Degree angle to radians (`degrees / 180 * pi`).
  ///
  /// @param angle The angle in degrees.
  explicit constexpr Radian(const Degree& angle);
  /// Returns the angle in radians.
  [[nodiscard]] constexpr Scalar value() const { return value_; }

  /// Sum of two angles.
  constexpr Radian operator+(Radian angle) const {
    return Radian{value_ + angle.value()};
  }
  /// Adds @p angle to this angle.
  constexpr Radian& operator+=(Radian angle) & {
    value_ += angle.value();
    return *this;
  }
  /// Difference of two angles.
  constexpr Radian operator-(Radian angle) const {
    return Radian{value_ - angle.value()};
  }
  /// Subtracts @p angle from this angle.
  constexpr Radian& operator-=(Radian angle) & {
    value_ -= angle.value();
    return *this;
  }
  /// Scales the angle by `value`.
  constexpr Radian operator*(Scalar value) const {
    return Radian{value_ * value};
  }
  /// Divides the angle by `value`.
  constexpr Radian operator/(Scalar value) const {
    return Radian{value_ / value};
  }
  /// Opposite angle.
  constexpr Radian operator-() const { return Radian{-value_}; }

 private:
  Scalar value_{};
};

/// Scales a @ref Radian angle by @p value (scalar on the left).
///
/// @param value The scale factor.
/// @param angle The angle.
/// @return `angle * value`.
constexpr Radian operator*(Scalar value, Radian angle) {
  return Radian{angle.value() * value};
}

/// An angle in degrees, a strong type around @ref Scalar.
///
/// One full turn is 360 degrees. Degrees are easier to read and write for
/// humans (editor values, tuning), while the math uses radians: convert
/// to a @ref Radian (`Radian{degree}`) before calling @ref Sin, @ref Cos
/// or @ref Tan. The value is not wrapped to one turn (0 to 360).
///
/// @see @ref Radian
class Degree {
 public:
  /// Constructs a zero angle.
  constexpr Degree() = default;
  /// Constructs an angle of @p value degrees.
  constexpr explicit Degree(Scalar value) : value_(value) {}
  /// Converts a @ref Radian angle to degrees (`radians / pi * 180`).
  ///
  /// @param angle The angle in radians.
  explicit constexpr Degree(const Radian& angle)
      : value_(angle.value() / Pi<Scalar>() * Scalar{180}) {}
  /// Returns the angle in degrees.
  [[nodiscard]] constexpr Scalar value() const { return value_; }
  /// Sum of two angles.
  constexpr Degree operator+(Degree angle) const {
    return Degree{value_ + angle.value()};
  }
  /// Adds @p angle to this angle.
  constexpr Degree& operator+=(Degree angle) & {
    value_ += angle.value();
    return *this;
  }
  /// Difference of two angles.
  constexpr Degree operator-(Degree angle) const {
    return Degree{value_ - angle.value()};
  }
  /// Subtracts @p angle from this angle.
  constexpr Degree& operator-=(Degree angle) & {
    value_ -= angle.value();
    return *this;
  }
  /// Opposite angle.
  constexpr Degree operator-() const { return Degree{-value_}; }

  /// Scales the angle by `value`.
  constexpr Degree operator*(Scalar value) const {
    return Degree{value_ * value};
  }
  /// Divides the angle by `value`.
  constexpr Degree operator/(Scalar value) const {
    return Degree{value_ / value};
  }

 private:
  Scalar value_{};
};

/// Scales a @ref Degree angle by @p value (scalar on the left).
///
/// @param value The scale factor.
/// @param angle The angle.
/// @return `angle * value`.
constexpr Degree operator*(Scalar value, Degree angle) {
  return Degree{angle.value() * value};
}

constexpr Radian::Radian(const Degree& angle) {
  value_ = angle.value() / Scalar{180} * Pi<Scalar>();
}

/// Returns the same direction as @p angle, wrapped to one turn: at least
/// -pi and less than pi radians.
///
/// Adds or subtracts whole turns (2 pi) until the angle is in range, so an
/// angle already in range comes back unchanged and wrapping costs one
/// comparison per call in the usual case. @ref Rotation keeps its angle
/// wrapped this way, so a body that spins for minutes never overflows.
///
/// @note With @ref Fixed16, 2 pi is stored as 411774 / 65536, 0.83 of a step
/// below the true value: each wrap turns the angle by about 1.3e-5 rad
/// (0.013 rad after 1000 turns). The error is the same on every platform, so
/// it doesn't break determinism.
///
/// @param angle The angle to wrap.
/// @return The wrapped angle, in radians.
[[nodiscard]] constexpr Radian WrapAngle(Radian angle) {
  const Scalar pi = Pi<Scalar>();
  // 2 * pi exactly (no rounding: doubling is exact in fixed and floating
  // point), so a value just past pi lands at or above -pi
  const Scalar two_pi = pi + pi;
  Scalar value = angle.value();
  while (value >= pi) {
    value -= two_pi;
  }
  while (value < -pi) {
    value += two_pi;
  }
  return Radian{value};
}

/// Sine of an angle.
///
/// Forwards to the @ref Scalar overload: the deterministic table lookup for
/// @ref Fixed16, `std::sin` for `float`.
///
/// @param angle The angle.
/// @return The sine of `angle`, in [-1, 1].
inline Scalar Sin(Radian angle) { return Sin(angle.value()); }

/// Cosine of an angle.
///
/// @param angle The angle.
/// @return The cosine of `angle`, in [-1, 1].
inline Scalar Cos(Radian angle) { return Cos(angle.value()); }

/// Tangent of an angle.
///
/// @param angle The angle.
/// @return The tangent of @p angle (very large near +/- 90 degrees).
inline Scalar Tan(Radian angle) { return Tan(angle.value()); }
}  // namespace neko

#endif  // NEKO_MATH_ANGLE_H_
