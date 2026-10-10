#ifndef NEKO_MATH_VEC2_H_
#define NEKO_MATH_VEC2_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "neko/math/const.h"
#include "neko/math/fixed_trig.h"
#include "neko/math/func.h"

namespace neko {
namespace internal {
// Square that can't overflow: Q32.32 in 64 bits for Fixed16 (16.16 overflows
// past ~181), T otherwise
template <typename T>
[[nodiscard]] constexpr auto WideSquare(T value) noexcept {
  if constexpr (std::is_same_v<T, Fixed16>) {
    const auto raw = static_cast<std::int64_t>(value.underlying_value());
    return static_cast<std::uint64_t>(raw * raw);
  } else {
    return value * value;
  }
}
}  // namespace internal

/// Two-component vector used for positions, velocities, forces and sizes.
///
/// The physics code uses @ref Vec2f, a `Vec2<Scalar>`: with the default
/// `NEKO_SCALAR=FIXED16` build its components are 16.16 fixed-point numbers
/// (@ref Fixed16), so every operation gives the same bits on every machine.
/// That is what makes the simulation deterministic, which rollback netcode
/// needs: two peers replaying the same inputs must reach exactly the same
/// state. The world is y-up (@ref Up is `(0, 1)`, gravity points to -y), and
/// lengths are in world units (typically meters).
///
/// `Vec2` is a small, trivially copyable value type: pass it by value. The
/// components are private; read them with `x()`/`y()` and write them with
/// `set_x()`/`set_y()`. `x()` returns a copy, and the assignment operators
/// only accept lvalues, so `v.x() += d` does not compile instead of silently
/// changing a temporary.
///
/// @code
/// const neko::Scalar dt = neko::Scalar{1} / neko::Scalar{60};
/// neko::Vec2f position{neko::Scalar{2}, neko::Scalar{3}};
/// const neko::Vec2f velocity = neko::Vec2f::Right() * neko::Scalar{4};
/// position += velocity * dt;  // integrate one step
/// if (neko::CompareLength(velocity, neko::Scalar{10}) > 0) {
///   // faster than 10 units per second
/// }
/// @endcode
///
/// @tparam T Component type: @ref Scalar for physics, `int` or `unsigned`
/// for pixel or grid coordinates.
template <typename T>
class Vec2 {
 public:
  /// Builds the vector (`x_arg`, `y_arg`).
  ///
  /// @param x_arg The x component.
  /// @param y_arg The y component.
  constexpr Vec2(T x_arg, T y_arg) : x_(x_arg), y_(y_arg) {}
  /// Builds the zero vector.
  constexpr Vec2() = default;
  /// Copy constructor.
  constexpr Vec2(const Vec2&) = default;
  /// Move constructor.
  constexpr Vec2(Vec2&&) = default;
  /// Copy assignment, lvalue-only.
  ///
  /// Assigning to a temporary (`v.x() = ...`, `GetVec() = ...`) does not
  /// compile: it would silently modify a copy.
  constexpr Vec2& operator=(const Vec2&) & = default;
  /// Move assignment, lvalue-only.
  constexpr Vec2& operator=(Vec2&&) & = default;
  /// Returns the x component.
  [[nodiscard]] constexpr T x() const noexcept { return x_; }
  /// Returns the y component.
  [[nodiscard]] constexpr T y() const noexcept { return y_; }
  /// Sets the x component.
  ///
  /// @param x_value New x component.
  constexpr void set_x(T x_value) noexcept { x_ = x_value; }
  /// Sets the y component.
  ///
  /// @param y_value New y component.
  constexpr void set_y(T y_value) noexcept { y_ = y_value; }
  /// Builds the vector (`scalar`, `scalar`).
  explicit constexpr Vec2(T scalar) : x_(scalar), y_(scalar) {}
  /// Converts a vector of another component type, casting each component.
  ///
  /// For example `Vec2i{position}` truncates a @ref Vec2f to a @ref Vec2i
  /// (integers) for drawing.
  ///
  /// @tparam OtherT Component type of `v`.
  template <typename OtherT>
  constexpr explicit Vec2(Vec2<OtherT> v)
      : x_(static_cast<T>(v.x())), y_(static_cast<T>(v.y())) {}

  /// Converts any type that has `x()` and `y()` accessors.
  ///
  /// Useful to bridge with another math library's vector type.
  ///
  /// @tparam OtherT Source type, with `x()` and `y()` convertible to `T`.
  template <typename OtherT>
  constexpr explicit Vec2(const OtherT& v) {
    x_ = static_cast<T>(v.x());
    y_ = static_cast<T>(v.y());
  }

  /// Builds the vector from two values of another type, casting each one.
  ///
  /// @tparam OtherT Type of the arguments, convertible to `T`.
  /// @param x_arg The x component.
  /// @param y_arg The y component.
  template <typename OtherT>
  constexpr explicit Vec2(OtherT x_arg, OtherT y_arg)
      : x_(static_cast<T>(x_arg)), y_(static_cast<T>(y_arg)) {}
  /// Returns the component-wise sum.
  constexpr Vec2 operator+(Vec2 other) const {
    return {x_ + other.x_, y_ + other.y_};
  }
  /// Adds @p other in place.
  constexpr Vec2& operator+=(Vec2 other) & {
    x_ += other.x_;
    y_ += other.y_;
    return *this;
  }
  /// Returns the opposite vector.
  constexpr Vec2 operator-() const { return {-x_, -y_}; }
  /// Returns the component-wise difference.
  constexpr Vec2 operator-(Vec2 other) const {
    return {x_ - other.x_, y_ - other.y_};
  }
  /// Subtracts @p other in place.
  constexpr Vec2& operator-=(Vec2 other) & {
    x_ -= other.x_;
    y_ -= other.y_;
    return *this;
  }
  /// Returns the component-wise product `(x * other.x, y * other.y)`.
  ///
  /// @note This is not the dot product; use @ref Dot for that.
  constexpr Vec2 operator*(Vec2 other) const {
    return {x_ * other.x_, y_ * other.y_};
  }
  /// Scales both components by @p other in place.
  constexpr Vec2& operator*=(T other) & {
    x_ *= other;
    y_ *= other;
    return *this;
  }
  /// Returns the vector scaled by `other`.
  constexpr Vec2 operator*(T other) const { return {x_ * other, y_ * other}; }

  /// Returns the vector divided by `other`.
  ///
  /// Dividing by zero does not crash: it returns a vector whose components
  /// are both `std::numeric_limits<T>::max()` (whatever the signs). Check
  /// the divisor yourself when that case matters.
  constexpr Vec2 operator/(T other) const {
    if (other == T{0}) {
      return Vec2{std::numeric_limits<T>::max(), std::numeric_limits<T>::max()};
    }
    return Vec2{x_ / other, y_ / other};
  }
  /// Divides both components by @p other in place.
  ///
  /// @pre @p other is not zero (unlike `operator/`, there is no guard).
  constexpr Vec2& operator/=(T other) & {
    x_ /= other;
    y_ /= other;
    return *this;
  }
  /// Returns the dot product `v1.x * v2.x + v1.y * v2.y`.
  ///
  /// It is `|v1| * |v2| * cos(angle)`: positive when the vectors point the
  /// same way, zero when perpendicular. Projecting a velocity on a contact
  /// normal is a typical use.
  ///
  /// @warning With @ref Fixed16 the products overflow when the components
  /// are larger than about 181 (181 * 181 > 32767).
  static constexpr T Dot(Vec2 v1, Vec2 v2) {
    return v1.x_ * v2.x_ + v1.y_ * v2.y_;
  }
  /// Returns `(0, 0)`.
  static constexpr Vec2 Zero() { return {}; }
  /// Returns `(0, 1)`: the world is y-up.
  static constexpr Vec2 Up() { return Vec2{Scalar{0}, Scalar{1}}; }
  /// Returns `(0, -1)`.
  static constexpr Vec2 Down() { return Vec2{Scalar{0}, Scalar{-1}}; }
  /// Returns `(-1, 0)`.
  static constexpr Vec2 Left() { return Vec2{Scalar{-1}, Scalar{0}}; }
  /// Returns `(1, 0)`.
  static constexpr Vec2 Right() { return Vec2{Scalar{1}, Scalar{0}}; }
  /// Returns `(1, 1)`.
  static constexpr Vec2 One() { return Vec2{Scalar{1}, Scalar{1}}; }

  /// Returns the vector rotated counter-clockwise by @p angle radians.
  ///
  /// With @ref Fixed16, @ref Sin and @ref Cos come from a lookup table built at
  /// compile time, so the result is deterministic and needs no FPU.
  ///
  /// @param angle Rotation in radians (positive is counter-clockwise in the
  /// y-up world).
  /// @return `(cos * x - sin * y, sin * x + cos * y)`.
  [[nodiscard]] Vec2 Rotate(T angle) const noexcept {
    const auto sin = Sin(angle);
    const auto cos = Cos(angle);
    return {(cos * x_) - (sin * y_), (sin * x_) + (cos * y_)};
  }

  /// Returns `(y, -x)`, the vector rotated by 90 degrees clockwise.
  ///
  /// @see @ref Vec2::Perpendicular2 for the counter-clockwise one.
  [[nodiscard]] constexpr Vec2 Perpendicular() const { return {y_, -x_}; }

  /// Returns `(-y, x)`, the vector rotated by 90 degrees counter-clockwise.
  [[nodiscard]] constexpr Vec2 Perpendicular2() const { return {-y_, x_}; }

  /// Returns the squared length `x * x + y * y`.
  ///
  /// Cheaper than @ref Length (no square root) and enough to compare
  /// distances. With @ref Fixed16 it overflows past a length of about 181
  /// and rounds to zero below about 1/256; prefer @ref CompareLength to
  /// compare a length with a threshold.
  [[nodiscard]] constexpr T SquareLength() const { return Dot(*this, *this); }

  /// Returns the Euclidean length `sqrt(x * x + y * y)`.
  ///
  /// With @ref Fixed16 the sum of squares is computed in 64 bits (Q32.32)
  /// and rooted with an integer square root, so it neither overflows nor
  /// loses small vectors; a length beyond the largest @ref Fixed16 saturates
  /// to it.
  [[nodiscard]] T Length() const {
    if constexpr (std::is_same_v<T, Fixed16>) {
      // Sum of squares in Q32.32: SquareLength() overflows past ~181 and rounds
      // to zero below ~1/256
      const auto root = internal::ISqrt64(internal::WideSquare(x_) +
                                          internal::WideSquare(y_));
      constexpr auto max_raw =
          static_cast<std::uint32_t>(std::numeric_limits<std::int32_t>::max());
      return T::FromUnderlyingValue(
          static_cast<std::int32_t>(root > max_raw ? max_raw : root));
    } else {
      return Sqrt(SquareLength());
    }
  }
  /// Returns the unit vector with the same direction.
  ///
  /// @return The vector divided by its @ref Length, or @ref Zero for a zero
  /// vector (no division by zero).
  [[nodiscard]] Vec2 Normalized() const {
    const auto length = Length();
    if (length == T{0}) {
      return Zero();
    }
    return {x_ / length, y_ / length};
  }
  /// Clamps each component of @p v between those of @p min_v and `max_v`.
  ///
  /// Clamping a point to a box gives the point of the box closest to it
  /// (see @ref neko::ClosestPoint).
  ///
  /// @pre `min_v.x() <= max_v.x()` and `min_v.y() <= max_v.y()`.
  /// @return The clamped vector.
  [[nodiscard]] constexpr static Vec2 Clamp(Vec2 v, Vec2 min_v, Vec2 max_v) {
    return {neko::Clamp(v.x_, min_v.x_, max_v.x_),
            neko::Clamp(v.y_, min_v.y_, max_v.y_)};
  }

  /// Returns true when both components are exactly equal.
  ///
  /// Exact comparison is meaningful with @ref Fixed16; with `float` prefer a
  /// tolerance.
  constexpr bool operator==(Vec2 other) const {
    return x_ == other.x_ && y_ == other.y_;
  }

  /// Returns true when a component differs.
  constexpr bool operator!=(Vec2 other) const {
    return x_ != other.x_ || y_ != other.y_;
  }

 private:
  T x_{};
  T y_{};
};

static_assert(sizeof(Vec2<Fixed16>) == 2 * sizeof(Fixed16));
static_assert(sizeof(Vec2<float>) == 8);
static_assert(sizeof(Vec2<int>) == 8);
static_assert(std::is_trivially_copyable_v<Vec2<Fixed16>>);
static_assert(std::is_trivially_copyable_v<Vec2<float>>);

/// Compares the length of @p v with @p length without a square root.
///
/// It compares `x * x + y * y` with `length * length`. With @ref Fixed16 the
/// squares are computed in 64 bits, so unlike `SquareLength() < r * r` it
/// cannot overflow, even for large worlds. The shape intersection tests use
/// it.
///
/// @code
/// // Is the point inside the circle?
/// const bool inside =
///     neko::CompareLength(point - circle.position(), circle.radius()) < 0;
/// @endcode
///
/// @tparam T Component type.
/// @param v Vector to measure.
/// @param length Length to compare with.
/// @pre @p length >= 0.
/// @return A negative value if `|v| < length`, 0 if equal, a positive value
/// if `|v| > length`.
template <typename T>
[[nodiscard]] constexpr int CompareLength(Vec2<T> v, T length) noexcept {
  const auto square_length =
      internal::WideSquare(v.x()) + internal::WideSquare(v.y());
  const auto length_square = internal::WideSquare(length);
  if (square_length < length_square) {
    return -1;
  }
  return length_square < square_length ? 1 : 0;
}

/// Returns @p other scaled by `f`, so that `scalar * v` works like
/// `v * scalar`.
///
/// @tparam T Component type of the vector.
/// @return The scaled vector.
template <typename T>
constexpr Vec2<T> operator*(Scalar f, Vec2<T> other) {
  return {f * other.x(), f * other.y()};
}

/// Vector of @ref Scalar, the type used by the physics engine.
///
/// The `f` is historical: it is fixed point (@ref Fixed16) in the default
/// build and `float` only with `NEKO_SCALAR=FLOAT`.
using Vec2f = Vec2<Scalar>;
/// Vector of `unsigned`, for sizes in pixels or cells.
using Vec2u = Vec2<unsigned>;
/// Vector of `int`, for screen or grid coordinates.
using Vec2i = Vec2<int>;
}  // namespace neko

#endif  // NEKO_MATH_VEC2_H_
