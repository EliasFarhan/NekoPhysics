#ifndef NEKO_MATH_SHAPE_H_
#define NEKO_MATH_SHAPE_H_

#include <type_traits>

#include "neko/math/vec2.h"
#include "neko/physics/physics_type.h"
namespace neko {

/// Axis-aligned bounding box: a rectangle whose sides are parallel to the x
/// and y axes.
///
/// It is stored as two corners, @ref min_bound (bottom-left in the y-up
/// world) and @ref max_bound (top-right). Because the sides never rotate,
/// testing two boxes for overlap is just four comparisons
/// (@ref neko::Intersect). That makes AABBs the building block of the
/// broadphase: every collider, whatever its shape, is wrapped in its AABB,
/// and only pairs whose AABBs overlap get the exact (narrowphase) test.
/// AABBs are also a collider shape of their own (box colliders).
///
/// @code
/// const auto box = neko::Aabbf::FromCenter(
///     neko::Vec2f{neko::Scalar{0}, neko::Scalar{0}},
///     neko::Vec2f{neko::Scalar{2}, neko::Scalar{1}});  // 4 x 2 box
/// const auto other = neko::Aabbf{
///     neko::Vec2f{neko::Scalar{1}, neko::Scalar{0}},
///     neko::Vec2f{neko::Scalar{3}, neko::Scalar{2}}};
/// const bool overlap = neko::Intersect(box, other);  // true
/// @endcode
///
/// @tparam T Coordinate type, usually @ref Scalar (see @ref Aabbf).
/// @note The class does not enforce `min_bound <= max_bound`: keep the
/// corners ordered when building a box by hand.
template <typename T>
class Aabb {
 public:
  /// Builds a degenerate box with both corners at the origin.
  constexpr Aabb() noexcept = default;
  /// Builds the box from its two corners.
  ///
  /// @param min_bound_value Corner with the smallest x and y.
  /// @param max_bound_value Corner with the largest x and y.
  constexpr Aabb(Vec2<T> min_bound_value, Vec2<T> max_bound_value) noexcept
      : min_bound_(min_bound_value), max_bound_(max_bound_value) {}

  /// Returns the corner with the smallest x and y.
  [[nodiscard]] constexpr Vec2<T> min_bound() const noexcept {
    return min_bound_;
  }
  /// Returns the corner with the largest x and y.
  [[nodiscard]] constexpr Vec2<T> max_bound() const noexcept {
    return max_bound_;
  }
  /// Sets the corner with the smallest x and y.
  constexpr void set_min_bound(Vec2<T> value) noexcept { min_bound_ = value; }
  /// Sets the corner with the largest x and y.
  constexpr void set_max_bound(Vec2<T> value) noexcept { max_bound_ = value; }

  /// Builds a box from its center and half size.
  ///
  /// @param center Center of the box.
  /// @param half_size Half of the width and half of the height (>= 0).
  /// @return The box from `center - half_size` to `center + half_size`.
  static constexpr Aabb FromCenter(Vec2<T> center, Vec2<T> half_size) {
    return {center - half_size, center + half_size};
  }
  /// Returns the center of the box.
  [[nodiscard]] constexpr Vec2<T> GetCenter() const {
    return min_bound_ + GetHalfSize();
  }
  /// Returns half of the width and half of the height.
  [[nodiscard]] constexpr Vec2<T> GetHalfSize() const {
    return (max_bound_ - min_bound_) / T{2};
  }

  /// Tests whether @p point is strictly inside the box.
  ///
  /// @return True when the point is inside; a point exactly on an edge is
  /// outside.
  [[nodiscard]] constexpr bool Contains(Vec2<T> point) const noexcept {
    const auto center = GetCenter();
    const auto half_size = GetHalfSize();
    return Abs(point.x() - center.x()) < half_size.x() &&
           Abs(point.y() - center.y()) < half_size.y();
  }

 private:
  Vec2<T> min_bound_{};
  Vec2<T> max_bound_{};
};

static_assert(sizeof(Aabb<Fixed16>) == 4 * sizeof(Fixed16));
static_assert(std::is_trivially_copyable_v<Aabb<Fixed16>>);
static_assert(std::is_trivially_copyable_v<Aabb<float>>);

/// Circle defined by its center and radius.
///
/// Circles are the cheapest shape to test exactly: two circles touch when
/// the distance between their centers is at most the sum of their radii.
/// The tests use @ref neko::CompareLength, so they need no square root and
/// cannot overflow with @ref Fixed16.
///
/// @code
/// const neko::Circlef ball{neko::Vec2f{neko::Scalar{0}, neko::Scalar{5}},
///                          neko::Scalar{1}};
/// const neko::Aabbf bounds = ball.GetAabb();  // (-1, 4) to (1, 6)
/// @endcode
///
/// @tparam T Coordinate type, usually @ref Scalar (see @ref Circlef).
template <typename T>
class Circle {
 public:
  /// Builds a circle of radius 0 at the origin.
  constexpr Circle() noexcept = default;
  /// Builds a circle from its center and radius.
  ///
  /// @param position_value Center of the circle.
  /// @param radius_value Radius (>= 0).
  constexpr Circle(Vec2<T> position_value, T radius_value) noexcept
      : position_(position_value), radius_(radius_value) {}

  /// Returns the center of the circle.
  [[nodiscard]] constexpr Vec2<T> position() const noexcept {
    return position_;
  }
  /// Returns the radius.
  [[nodiscard]] constexpr T radius() const noexcept { return radius_; }
  /// Sets the center of the circle.
  constexpr void set_position(Vec2<T> value) noexcept { position_ = value; }
  /// Sets the radius.
  ///
  /// @param value New radius (>= 0).
  constexpr void set_radius(T value) noexcept { radius_ = value; }
  /// Returns the smallest @ref Aabb containing the circle.
  ///
  /// This is the box the broadphase sees for a circle collider.
  [[nodiscard]] constexpr Aabb<T> GetAabb() const {
    return Aabb<T>::FromCenter(position_, {radius_, radius_});
  }
  /// Tests whether @p point is strictly inside the circle.
  ///
  /// @return True when the distance from the center is less than the
  /// radius; a point exactly on the circle is outside.
  [[nodiscard]] constexpr bool Contains(Vec2<T> point) const noexcept {
    return CompareLength(point - position_, radius_) < 0;
  }

 private:
  Vec2<T> position_{};
  T radius_{};
};

static_assert(sizeof(Circle<Fixed16>) == 3 * sizeof(Fixed16));
static_assert(std::is_trivially_copyable_v<Circle<Fixed16>>);
static_assert(std::is_trivially_copyable_v<Circle<float>>);

/// Tests whether two boxes overlap.
///
/// Two AABBs overlap exactly when their ranges overlap on both axes (the
/// separating axis theorem reduced to x and y). Boxes that only touch along
/// an edge count as overlapping.
///
/// @tparam T Coordinate type.
/// @param a First box.
/// @param b Second box.
/// @return True when the boxes overlap or touch.
template <typename T>
constexpr bool Intersect(const Aabb<T>& a, const Aabb<T>& b) {
  if (a.max_bound().x() < b.min_bound().x() ||
      a.min_bound().x() > b.max_bound().x()) {
    return false;
  }
  if (a.max_bound().y() < b.min_bound().y() ||
      a.min_bound().y() > b.max_bound().y()) {
    return false;
  }
  return true;
}

/// Tests whether two circles overlap.
///
/// @tparam T Coordinate type.
/// @param c1 First circle.
/// @param c2 Second circle.
/// @return True when the distance between the centers is at most the sum
/// of the radii (touching counts).
template <typename T>
[[nodiscard]] constexpr bool Intersect(const Circle<T>& c1,
                                       const Circle<T>& c2) {
  return CompareLength(c1.position() - c2.position(),
                       c1.radius() + c2.radius()) <= 0;
}

/// Returns the point of @p aabb closest to @p point.
///
/// Each coordinate is clamped to the box, so a point inside the box is
/// returned unchanged. The circle-box test uses it: the circle hits the box
/// when this closest point is within the radius of its center.
///
/// @tparam T Coordinate type.
/// @param aabb Box to project on.
/// @param point Point to project.
/// @return The closest point of the box (on its edge or inside it).
template <typename T>
[[nodiscard]] constexpr Vec2<T> ClosestPoint(const Aabb<T>& aabb,
                                             Vec2<T> point) {
  return Vec2<T>::Clamp(point, aabb.min_bound(), aabb.max_bound());
}

/// Tests whether a circle and a box overlap.
///
/// @tparam T Coordinate type.
/// @param circle The circle.
/// @param aabb The box.
/// @return True when the point of the box closest to the circle's center
/// (@ref neko::ClosestPoint) is at most one radius away (touching counts).
template <typename T>
[[nodiscard]] constexpr bool Intersect(const Circle<T>& circle,
                                       const Aabb<T>& aabb) {
  return CompareLength(
             circle.position() - ClosestPoint(aabb, circle.position()),
             circle.radius()) <= 0;
}

/// Tests whether a box and a circle overlap (same as the circle-box
/// overload, arguments swapped).
///
/// @tparam T Coordinate type.
/// @param aabb The box.
/// @param circle The circle.
/// @return True when they overlap or touch.
template <typename T>
constexpr bool Intersect(const Aabb<T>& aabb, const Circle<T>& circle) {
  return Intersect(circle, aabb);
}

/// Box in world coordinates (@ref Scalar), the type the engine uses.
using Aabbf = Aabb<Scalar>;
/// Circle in world coordinates (@ref Scalar), the type the engine uses.
using Circlef = Circle<Scalar>;
}  // namespace neko

#endif  // NEKO_MATH_SHAPE_H_
