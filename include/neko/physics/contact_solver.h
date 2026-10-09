#ifndef NEKO_PHYSICS_CONTACT_SOLVER_H_
#define NEKO_PHYSICS_CONTACT_SOLVER_H_
#include <array>

#include "neko/math/vec2.h"

namespace neko {
struct Collider;
struct Body;

/// One side of a @ref Contact, a body and the collider of it that touches.
///
/// The pointers refer to the world's own tables and are only valid during the
/// step that created the contact.
struct CollidingBody {
  Body* body = nullptr;          ///< Body that owns the collider.
  Collider* collider = nullptr;  ///< Collider that touches the other one.
};

/// Contact between two solid colliders, found by the narrowphase and resolved
/// with an impulse.
///
/// @ref PhysicsWorld::DetectContact fills it in when two colliders overlap;
/// @ref Resolve then fixes velocities and positions. A `Contact` lives only for
/// the step that created it: games normally never build one, they receive
/// @ref ContactListener events instead.
///
/// Conventions: `contact_normal` is a unit vector pointing from `bodies[1]`
/// towards `bodies[0]`, and `penetration` is how deep the shapes overlap along
/// it (positive when they overlap).
struct Contact {
  /// The two touching bodies, with their colliders.
  std::array<CollidingBody, 2> bodies{};
  /// Unit vector from `bodies[1]` towards `bodies[0]`: the direction in which
  /// `bodies[0]` is pushed out.
  Vec2f contact_normal{};
  /// Approximate world-space point where the shapes touch (informative, the
  /// solver doesn't use it).
  Vec2f contact_point{};
  /// Bounciness of the contact, the larger `Collider::restitution` of the two
  /// colliders.
  ///
  /// 0 stops the bodies along the normal (no bounce), 1 is a perfectly elastic
  /// bounce that keeps the speed along the normal.
  Scalar restitution{1};
  /// Overlap depth along `contact_normal`, in world units.
  Scalar penetration{};

  /// Applies the impulse and the position correction that separate the two
  /// bodies.
  ///
  /// Impulse-based solving: instead of simulating contact forces over time,
  /// the solver instantly changes the velocities so that the bodies stop
  /// approaching each other (and bounce back according to `restitution`).
  /// The change is shared by inverse mass, so a light body moves more than a
  /// heavy one. Then the bodies are moved apart by `penetration`, again shared
  /// by inverse mass, so they don't stay sunk into each other.
  ///
  /// Only @ref BodyType::kDynamic bodies are affected: static and kinematic
  /// bodies behave as if their mass was infinite. If neither body is dynamic
  /// (or both have an inverse mass of 0) nothing happens. If the bodies are
  /// already separating, only the position is corrected.
  ///
  /// @pre Both `bodies[i].body` pointers are valid.
  void Resolve();
  /// Returns the relative velocity of the two bodies along the normal.
  ///
  /// Computed as the dot product of `bodies[0].velocity - bodies[1].velocity`
  /// with `contact_normal`. Negative when the bodies are moving towards each
  /// other, positive when they are already separating.
  ///
  /// @pre Both `bodies[i].body` pointers are valid.
  /// @return The separating velocity, in world units per second.
  [[nodiscard]] Scalar CalculateSeparateVelocity() const;
};
}  // namespace neko

#endif  // NEKO_PHYSICS_CONTACT_SOLVER_H_
