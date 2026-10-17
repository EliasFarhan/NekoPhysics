#ifndef NEKO_PHYSICS_QUERY_H_
#define NEKO_PHYSICS_QUERY_H_

#include <cstdint>

#include "neko/math/vec2.h"
#include "neko/physics/physics_type.h"

namespace neko {

/// Which colliders a world query sees: @ref PhysicsWorld::OverlapCircle,
/// @ref PhysicsWorld::OverlapAabb, @ref PhysicsWorld::Raycast and
/// @ref PhysicsWorld::RaycastClosest.
///
/// A collider passes when its @ref Collider::category shares a bit with
/// `mask`, and, for a trigger, when `include_triggers` is set. Colliders of
/// inactive bodies are never seen. The default sees every solid collider.
///
/// @code
/// // Categories of a game: one bit each
/// constexpr std::uint16_t kWall = 1 << 1;
/// constexpr std::uint16_t kCrate = 1 << 2;
/// // Line of sight: only walls and crates block it
/// const neko::QueryFilter sight{.mask = kWall | kCrate};
/// @endcode
struct QueryFilter {
  /// Categories the query sees, one bit each; all of them by default.
  std::uint16_t mask = 0xFFFF;
  /// Whether triggers (pickups, damage zones) are seen too. `false` by
  /// default: a line of sight shouldn't stop on a checkpoint.
  bool include_triggers = false;
};

/// Where a segment cast by @ref PhysicsWorld::Raycast first touches a
/// collider.
///
/// The segment goes from `start` to `end`; a hit at `fraction` t is at
/// `start + t * (end - start)`, so t runs from 0 (at the start) to 1 (at the
/// end) and is the distance along the segment divided by its length.
struct RaycastHit {
  /// The collider hit.
  ColliderIndex collider{};
  /// The body of that collider.
  BodyIndex body{};
  /// World-space point where the segment enters the shape, in meters.
  Vec2f point{};
  /// Unit outward normal of the shape at `point`, facing the start of the
  /// segment.
  ///
  /// Zero when the segment starts inside the shape (`fraction` 0): there is
  /// no surface crossed, so no normal. Each component is a @ref Scalar.
  Vec2f normal{};
  /// Position of the hit along the segment, from 0 to 1.
  Scalar fraction{};
};

}  // namespace neko

#endif  // NEKO_PHYSICS_QUERY_H_
