#ifndef NEKO_PHYSICS_CONTACT_SOLVER_H_
#define NEKO_PHYSICS_CONTACT_SOLVER_H_
#include <array>
#include <cstdint>

#include "neko/math/vec2.h"

namespace neko {
struct Collider;
struct Body;

/// One side of a @ref Contact, a body and the collider of it that touches.
///
/// The pointers refer to the world's own tables and are only valid during the
/// step that created the contact.
struct CollidingBody {
  Body* body = nullptr;          ///< @ref Body that owns the collider.
  Collider* collider = nullptr;  ///< @ref Collider that touches the other one.
};

/// One point of a contact manifold: where the shapes touch, and how deep.
struct ContactPoint {
  /// World-space position of the point, in meters.
  Vec2f position{};
  /// Overlap depth at this point along @ref Contact::contact_normal, in
  /// meters: 0 when the shapes just touch there.
  Scalar penetration{};
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
///
/// The contact manifold, `points`, is where the shapes touch: one point for a
/// circle, up to two for two boxes. A box lying flat on another touches along
/// a whole edge, and two points (its ends) describe that edge; a box landing on
/// a corner touches at one point. Box manifolds come from clipping: the
/// reference face is the face of the separating axis, the incident face is
/// the most opposed face of the other box, and the incident face's corners
/// are clipped to the reference face's width, keeping those below the
/// reference face.
struct Contact {
  /// The two touching bodies, with their colliders (see @ref CollidingBody).
  std::array<CollidingBody, 2> bodies{};
  /// Unit vector from `bodies[1]` towards `bodies[0]`: the direction in which
  /// `bodies[0]` is pushed out.
  Vec2f contact_normal{};
  /// Approximate world-space point where the shapes touch (informative, the
  /// solver doesn't use it).
  ///
  /// The deepest corner for boxes, the middle of the overlap for circles; see
  /// `points` for the full manifold.
  Vec2f contact_point{};
  /// The contact manifold: the first `point_count` entries are the points
  /// where the shapes touch, each with its own penetration.
  ///
  /// Informative for now: @ref Resolve pushes the bodies through their
  /// centers of mass with `penetration`. Contacts that make bodies spin will
  /// apply their impulses at these points.
  std::array<ContactPoint, 2> points{};
  /// Number of valid entries in `points`: 1 or 2 for a solid contact, 0 for a
  /// trigger pair (only the overlap is computed).
  std::uint8_t point_count = 0;
  /// Bounciness of the contact, the larger @ref Collider::restitution of the
  /// two colliders.
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
