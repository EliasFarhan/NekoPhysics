#ifndef NEKO_PHYSICS_PHYSICS_TYPE_H_
#define NEKO_PHYSICS_PHYSICS_TYPE_H_

#include <compare>
#include <cstdint>
#include <limits>
#include <type_traits>

namespace neko {

/// Integer type of body, shape and collider indices.
///
/// 16 bits keep snapshots small on the DS (rollback netcode copies the whole
/// world every frame) and cap a world at 65535 objects of each kind.
using Index = std::uint16_t;

/// Strongly typed index of a @ref Body in a @ref PhysicsWorld.
///
/// The world stores bodies in a fixed-size table and hands out indices
/// instead of pointers: indices stay valid when the world is copied for a
/// rollback snapshot, pointers would not. A distinct type per table means the
/// compiler rejects passing a body index where a collider index is expected.
/// A default-constructed index is invalid (see @ref kInvalidBodyIndex).
class BodyIndex {
 public:
  /// Constructs an invalid index, equal to @ref kInvalidBodyIndex.
  constexpr BodyIndex() noexcept = default;
  /// Wraps a raw slot number.
  ///
  /// @param index_value Slot of the body in the world's body table.
  explicit constexpr BodyIndex(Index index_value) noexcept
      : index_(index_value) {}

  /// Returns the raw slot number.
  [[nodiscard]] constexpr Index index() const noexcept { return index_; }

  /// Compares two indices for equality.
  constexpr bool operator==(const BodyIndex& rhs) const noexcept = default;

 private:
  Index index_ = std::numeric_limits<Index>::max();
};

static_assert(sizeof(BodyIndex) == sizeof(Index));
static_assert(std::is_trivially_copyable_v<BodyIndex>);

/// Invalid body index, returned by @ref PhysicsWorld::AddBody when the world
/// is full.
///
/// It is a default-constructed @ref BodyIndex.
constexpr auto kInvalidBodyIndex = BodyIndex{};

/// Strongly typed index of a shape (@ref CircleCollider or
/// @ref AabbCollider) in a @ref PhysicsWorld.
///
/// Circles and boxes live in two separate tables, so a shape index is only
/// meaningful together with the @ref ShapeType of its @ref Collider. Use it
/// with @ref PhysicsWorld::circle or @ref PhysicsWorld::aabb.
class ShapeIndex {
 public:
  /// Constructs an invalid index, equal to @ref kInvalidShapeIndex.
  constexpr ShapeIndex() noexcept = default;
  /// Wraps a raw slot number.
  ///
  /// @param index_value Slot of the shape in its shape table.
  explicit constexpr ShapeIndex(Index index_value) noexcept
      : index_(index_value) {}

  /// Returns the raw slot number.
  [[nodiscard]] constexpr Index index() const noexcept { return index_; }

  /// Compares two indices for equality.
  constexpr bool operator==(const ShapeIndex& rhs) const noexcept = default;

 private:
  Index index_ = std::numeric_limits<Index>::max();
};

static_assert(sizeof(ShapeIndex) == sizeof(Index));
static_assert(std::is_trivially_copyable_v<ShapeIndex>);

/// Invalid shape index, the shape index of an unused @ref Collider slot.
///
/// It is a default-constructed @ref ShapeIndex.
constexpr auto kInvalidShapeIndex = ShapeIndex{};

/// Kind of shape a @ref Collider uses.
enum class ShapeType : std::uint8_t {
  kAabb,    ///< Axis-aligned box, see @ref AabbCollider.
  kCircle,  ///< Circle, see @ref CircleCollider.
  kNone,    ///< No shape: marks a free collider slot.
  /// Box that turns with its body, see @ref OrientedBoxCollider. Listed after
  /// `kNone` so the values of the older shapes, which are part of the
  /// checksums, didn't change when it was added.
  kOrientedBox
};

/// Strongly typed index of a @ref Collider in a @ref PhysicsWorld.
///
/// Collider indices are totally ordered so that @ref ColliderPair can be
/// sorted: the world relies on that order to report contact events
/// deterministically.
class ColliderIndex {
 public:
  /// Constructs an invalid index, equal to @ref kInvalidColliderIndex.
  constexpr ColliderIndex() noexcept = default;
  /// Wraps a raw slot number.
  ///
  /// @param index_value Slot of the collider in the world's collider table.
  explicit constexpr ColliderIndex(Index index_value) noexcept
      : index_(index_value) {}

  /// Returns the raw slot number.
  [[nodiscard]] constexpr Index index() const noexcept { return index_; }

  /// Compares two indices by their slot number.
  constexpr auto operator<=>(const ColliderIndex& rhs) const noexcept = default;

 private:
  Index index_ = std::numeric_limits<Index>::max();
};

static_assert(sizeof(ColliderIndex) == sizeof(Index));
static_assert(std::is_trivially_copyable_v<ColliderIndex>);

/// Invalid collider index, returned by @ref PhysicsWorld::AddCircleCollider
/// and @ref PhysicsWorld::AddAabbCollider when the world is full.
///
/// It is a default-constructed @ref ColliderIndex.
constexpr auto kInvalidColliderIndex = ColliderIndex{};

/// Strongly typed index of a @ref DistanceJoint in a @ref PhysicsWorld.
///
/// Like the other indices, it stays valid when the world is copied for a
/// rollback snapshot. A default-constructed index is invalid (see
/// @ref kInvalidJointIndex).
class JointIndex {
 public:
  /// Constructs an invalid index, equal to @ref kInvalidJointIndex.
  constexpr JointIndex() noexcept = default;
  /// Wraps a raw slot number.
  ///
  /// @param index_value Slot of the joint in the world's joint table.
  explicit constexpr JointIndex(Index index_value) noexcept
      : index_(index_value) {}

  /// Returns the raw slot number.
  [[nodiscard]] constexpr Index index() const noexcept { return index_; }

  /// Compares two indices for equality.
  constexpr bool operator==(const JointIndex& rhs) const noexcept = default;

 private:
  Index index_ = std::numeric_limits<Index>::max();
};

static_assert(sizeof(JointIndex) == sizeof(Index));
static_assert(std::is_trivially_copyable_v<JointIndex>);

/// Invalid joint index, returned by @ref PhysicsWorld::AddDistanceJoint when
/// the world is full.
///
/// It is a default-constructed @ref JointIndex.
constexpr auto kInvalidJointIndex = JointIndex{};

/// Unordered pair of colliders, stored with `c1 < c2` so that it can be
/// compared and sorted.
///
/// `ColliderPair{a, b}` and `ColliderPair{b, a}` are the same pair. This is
/// the type the broadphase outputs (potentially overlapping colliders) and the
/// one a @ref ContactListener receives in its enter and exit events.
class ColliderPair {
 public:
  /// Constructs a pair of two invalid indices.
  constexpr ColliderPair() noexcept = default;
  /// Constructs the pair of @p a and `b`, in any order.
  ///
  /// @param a One collider of the pair.
  /// @param b The other collider of the pair.
  constexpr ColliderPair(ColliderIndex a, ColliderIndex b) noexcept
      : c1_(a < b ? a : b), c2_(a < b ? b : a) {}

  /// Compares pairs lexicographically: by `c1`, then by `c2`.
  constexpr auto operator<=>(const ColliderPair& rhs) const noexcept = default;

  /// Returns the smaller collider index of the pair.
  [[nodiscard]] constexpr ColliderIndex c1() const noexcept { return c1_; }
  /// Returns the larger collider index of the pair.
  [[nodiscard]] constexpr ColliderIndex c2() const noexcept { return c2_; }

  /// Checks whether @p c is one of the two colliders.
  ///
  /// Handy in a @ref ContactListener to test whether an event concerns a
  /// given collider.
  ///
  /// @param c @ref Collider to look for.
  /// @return `true` if @p c is `c1()` or `c2()`.
  [[nodiscard]] constexpr bool Contains(ColliderIndex c) const noexcept {
    return c == c1_ || c == c2_;
  }

 private:
  ColliderIndex c1_{};
  ColliderIndex c2_{};
};

static_assert(sizeof(ColliderPair) == 2 * sizeof(Index));
static_assert(std::is_trivially_copyable_v<ColliderPair>);

}  // namespace neko

#endif  // NEKO_PHYSICS_PHYSICS_TYPE_H_
