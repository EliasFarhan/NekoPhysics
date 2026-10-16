#ifndef NEKO_MATH_ROTATION_H_
#define NEKO_MATH_ROTATION_H_

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/vec2.h"

namespace neko {

/// The orientation of a body: an angle with its sine and cosine cached.
///
/// Rotating a vector needs the sine and cosine of the angle. Computing them
/// is the expensive part (a table lookup with @ref Fixed16, `std::sin` and
/// `std::cos` with `float`), while a body's colliders, contacts and drawing
/// all rotate vectors by the same angle many times per step. `Rotation`
/// computes them once, in @ref set_angle, and keeps them private so they can
/// never disagree with the angle.
///
/// The angle is in radians, positive counter-clockwise (the world is y-up),
/// and always wrapped to one turn by @ref WrapAngle: at least -pi and less
/// than pi. A default `Rotation` is the angle 0 (sine 0, cosine 1), which
/// leaves vectors unchanged bit for bit.
///
/// @code
/// neko::Rotation rotation;  // 0
/// rotation.set_angle(neko::Radian{neko::Pi<neko::Scalar>() /
/// neko::Scalar{2}});
/// // A quarter turn counter-clockwise: the local x axis points up
/// const neko::Vec2f up = rotation.Rotate(neko::Vec2f::Right());  // (0, 1)
/// const neko::Vec2f back = rotation.InverseRotate(up);  // (1, 0)
/// @endcode
///
/// @note Determinism: with @ref Fixed16 the sine and cosine come from the
/// table of @ref Sin, identical on every platform. With `float` they come
/// from the standard library, which may differ between compilers.
class Rotation {
 public:
  /// Constructs the angle 0.
  constexpr Rotation() = default;
  /// Constructs the orientation of @p angle radians.
  ///
  /// @param angle The angle, wrapped to one turn.
  explicit Rotation(Radian angle) { set_angle(angle); }

  /// Returns the angle, at least -pi and less than pi radians.
  [[nodiscard]] constexpr Radian angle() const { return angle_; }
  /// Returns the cached sine of the angle.
  [[nodiscard]] constexpr Scalar sin() const { return sin_; }
  /// Returns the cached cosine of the angle.
  [[nodiscard]] constexpr Scalar cos() const { return cos_; }

  /// Sets the angle and recomputes its sine and cosine.
  ///
  /// @param angle The new angle in radians; any value works, it is wrapped
  /// to one turn with @ref WrapAngle.
  void set_angle(Radian angle) {
    angle_ = WrapAngle(angle);
    sin_ = Sin(angle_);
    cos_ = Cos(angle_);
  }

  /// Rotates a vector from the body's local frame to the world frame.
  ///
  /// Turns @p local counter-clockwise by the angle:
  /// `(cos * x - sin * y, sin * x + cos * y)`. A collider offset given in
  /// the body's frame becomes the world-space offset this way.
  ///
  /// @param local The vector in the body's frame.
  /// @return The same vector in the world frame.
  [[nodiscard]] constexpr Vec2f Rotate(Vec2f local) const {
    return {cos_ * local.x() - sin_ * local.y(),
            sin_ * local.x() + cos_ * local.y()};
  }
  /// Rotates a vector from the world frame to the body's local frame.
  ///
  /// The inverse of @ref Rotate: turns @p world clockwise by the angle,
  /// `(cos * x + sin * y, -sin * x + cos * y)`.
  ///
  /// @param world The vector in the world frame.
  /// @return The same vector in the body's frame.
  [[nodiscard]] constexpr Vec2f InverseRotate(Vec2f world) const {
    return {cos_ * world.x() + sin_ * world.y(),
            cos_ * world.y() - sin_ * world.x()};
  }
  /// Returns the body's local x axis in world space, `(cos, sin)`.
  ///
  /// This is the direction the debug drawing shows as the body's
  /// orientation.
  [[nodiscard]] constexpr Vec2f x_axis() const { return {cos_, sin_}; }
  /// Returns the body's local y axis in world space, `(-sin, cos)`.
  [[nodiscard]] constexpr Vec2f y_axis() const { return {-sin_, cos_}; }

  /// Compares the angles (and so the cached sine and cosine).
  constexpr bool operator==(const Rotation& other) const {
    return angle_.value() == other.angle_.value();
  }

 private:
  Radian angle_{};
  Scalar sin_{0};
  Scalar cos_{1};
};

}  // namespace neko

#endif  // NEKO_MATH_ROTATION_H_
