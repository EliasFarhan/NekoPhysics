#ifndef NEKO_PHYSICS_PHYSICS_H_
#define NEKO_PHYSICS_PHYSICS_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/bsh/bsh.h"
#include "neko/bsh/sweep_and_prune.h"
#include "neko/core/allocator.h"
#include "neko/math/rotation.h"
#include "neko/math/shape.h"
#include "neko/math/vec2.h"
#include "neko/physics/contact_listener.h"
#include "neko/physics/contact_solver.h"
#include "neko/physics/joint.h"
#include "neko/physics/physics_type.h"
#include "neko/physics/query.h"

/// NekoPhysics: a small deterministic 2D physics library for games.
///
/// NekoPhysics moves bodies under gravity and forces, detects when their
/// shapes overlap, pushes solid shapes apart and reports contact events to
/// your game. It is built for the Nintendo DS (no floating-point unit) and for
/// rollback netcode, which shapes the whole design:
///
/// - **Deterministic.** The same inputs give bit-identical results on every
///   machine. Numbers are @ref Scalar, by default @ref Fixed16 (16.16 fixed
///   point: integer math, identical everywhere, fast without an FPU) instead
///   of `float`, whose results can differ between compilers and CPUs.
/// - **Copyable.** Rollback netcode saves the world every frame and, when a
///   late input arrives, restores an old copy and resimulates. Objects are
///   referred to by indices (@ref BodyIndex, @ref ColliderIndex), not
///   pointers, so a copy made with @ref PhysicsWorld::CopyFrom is
///   self-contained. Comparing @ref PhysicsWorld::GenerateChecksum between
///   peers detects desyncs.
/// - **Fixed capacities.** Every table is allocated once, when the world is
///   built (see @ref PhysicsWorldConfig), so @ref PhysicsWorld::Step never
///   allocates memory.
///
/// Main types and how they fit together:
///
/// - @ref PhysicsWorld owns everything and advances the simulation with
///   @ref PhysicsWorld::Step, using the fixed time step set in its
///   configuration.
/// - @ref Body is a rigid body: position, velocity, force and inverse mass
///   for its motion, a @ref Rotation, angular velocity, torque and inverse
///   inertia for its spin, and a @ref BodyType (dynamic, kinematic or
///   static).
/// - @ref Collider attaches a shape to a body, with an offset, a restitution
///   (bounciness) and a trigger flag. A body can have several colliders.
/// - Shapes are @ref CircleCollider (a radius), @ref AabbCollider (an
///   axis-aligned box, by half size) and @ref OrientedBoxCollider (a box
///   that turns with its body). Collider offsets turn with the body; an
///   @ref AabbCollider stays axis-aligned. Contacts find where shapes touch
///   but don't make bodies spin yet: the contact impulse goes through the
///   center of mass.
/// - @ref DistanceJoint links two bodies: a rod, a rope or a spring (see
///   @ref PhysicsWorld::AddDistanceJoint).
/// - @ref ContactListener is the interface your game implements to receive
///   trigger and collision enter/exit events.
/// - @ref Collider::category and @ref Collider::mask choose which colliders
///   meet at all (@ref ShouldCollide), and world queries ask what is at a
///   place without stepping: @ref PhysicsWorld::OverlapCircle,
///   @ref PhysicsWorld::OverlapAabb and @ref PhysicsWorld::Raycast, seen
///   through a @ref QueryFilter.
/// - Collision detection runs in two phases. The broadphase, a
///   @ref BoundingSurfaceHierarchy (@ref SweepAndPrune by default, or a
///   @ref QuadTree), cheaply finds pairs whose bounding boxes overlap. The
///   narrowphase (@ref PhysicsWorld::DetectContact) tests the exact shapes of
///   those pairs only and builds a @ref Contact. @ref SolveContacts then
///   pushes solid shapes apart with impulses, all the step's contacts
///   together, in several passes.
/// - Math types: @ref Vec2f (2D vector of @ref Scalar), @ref Aabbf and
///   @ref Circlef.
///
/// Conventions, checked by `tests/conventions_test.cc`:
///
/// - **Units.** Meters, seconds and kilograms: velocities in m/s, forces in
///   newtons (kg m/s^2), gravity in m/s^2. Mass is given by its inverse,
///   @ref Body::inverse_mass, in 1/kg.
/// - **Axes and angles.** The world is y-up (@ref Vec2f::Up is (0, 1) and the
///   default gravity points down) and a positive angle turns
///   counter-clockwise (@ref Vec2f::Rotate), so a positive angular velocity
///   or torque does too. Screens are y-down: the sample hosts flip y when
///   drawing.
/// - **Rotation units.** Angles in radians, wrapped to at least -pi and less
///   than pi (@ref WrapAngle); angular velocity in rad/s; torque in N m
///   (@ref Vec2f::Cross of the lever arm and the force); inertia in kg m^2,
///   given by its inverse, @ref Body::inverse_inertia. A body turns around
///   its `position`, which is its center of mass.
/// - **Contact normal.** @ref Contact::contact_normal points from the second
///   body toward the first, and @ref Contact::penetration is the overlap depth
///   along it, positive when the shapes overlap. The normal has unit length
///   up to rounding (it is computed with one reciprocal).
/// - **Touching counts.** Shapes whose boundaries exactly touch overlap with
///   zero penetration: the pair is touching and gets a contact.
/// - **Tolerances.** Overlaps up to @ref PhysicsWorldConfig::penetration_slop
///   (5 mm) are left alone, deeper ones are pushed out by a fifth
///   (@ref PhysicsWorldConfig::baumgarte) per step, and impacts slower than
///   @ref PhysicsWorldConfig::restitution_threshold (1 m/s) don't bounce.
/// - **Fixed-point ranges.** @ref Fixed16 holds values from -32768 to about
///   32767.99998 in steps of 1/65536 (about 0.000015). A product outside that
///   range overflows: @ref Vec2f::Dot and @ref Vec2f::SquareLength do for
///   components above about 181 (use @ref Vec2f::Length or
///   @ref CompareLength). The time step 1/60 s is stored as 1092/65536 s, so
///   60 steps last 0.99976 s, and a position moves by `velocity * dt` rounded
///   to 1/65536 m: a body slower than about 0.47 mm/s doesn't move.
/// - **Fixed-point ranges of rotation.** The angle moves by
///   `angular_velocity * dt` rounded to 1/65536 rad: below 31/65536 rad/s
///   (about 0.00047 rad/s, one turn in 3.7 hours) a body doesn't turn. 2 pi
///   is stored 0.83 step low, so each wrap of the angle turns it by 1.3e-5
///   rad. Inertia is computed per kilogram first (`r^2 / 2` for a circle,
///   `(w^2 + h^2) / 12` for a box, plus the squared offset), then
///   `inverse_inertia = inverse_mass / that`. Keep the inertia between about
///   0.001 and 1000 kg m^2 (a 0.1 m to 100 m box of 1 kg): the inverse
///   inertia is then within 0.8 % of the exact value. Below 3e-5 kg m^2 (a
///   circle of radius 0.05 m and 10 g) the inverse inertia overflows, shapes
///   under 0.02 m are 9 % off, and inertias above 6000 kg m^2 have an
///   inverse of only a few steps. `torque * inverse_inertia` (the angular
///   acceleration) must stay below 32767 rad/s^2, and a torque below 32767
///   N m.
///
/// A ball falling under the default gravity for one second:
///
/// @code
/// neko::PhysicsWorld world;  // gravity (0, -9.81), dt = 1/60 s
///
/// const neko::BodyIndex ball = world.AddBody();
/// neko::Body& body = world.body(ball);
/// body.position = neko::Vec2f{neko::Scalar{0}, neko::Scalar{10}};
/// body.type = neko::BodyType::kDynamic;
/// body.inverse_mass = neko::Scalar{1};  // mass of 1
///
/// const neko::ColliderIndex collider_index = world.AddCircleCollider(ball);
/// neko::Collider& collider = world.collider(collider_index);
/// collider.is_trigger = false;  // solid: colliders are triggers by default
/// world.circle(collider.shape_index).radius = neko::Scalar{1};
///
/// for (int i = 0; i < 60; ++i) {
///   world.Step();  // call from your game's fixed update
/// }
/// // world.body(ball).position.y() is now about 5
/// @endcode
namespace neko {

/// How a @ref Body moves and reacts to contacts.
enum class BodyType : std::uint8_t {
  /// Moved by gravity, forces and contacts: players, crates, balls.
  kDynamic,
  /// Moves at its own `velocity`, ignoring gravity, forces and contacts:
  /// moving platforms, scripted objects. Pushes dynamic bodies as if it had
  /// infinite mass.
  kKinematic,
  /// Never moves: ground, walls. Static bodies don't collide with each other.
  kStatic,
  /// No body: marks a free slot in the world's body table.
  kNone
};

/// How the world sets the @ref Body::inverse_inertia of a dynamic body.
enum class InertiaMode : std::uint8_t {
  /// Computed by every @ref PhysicsWorld::Step from the body's mass and its
  /// solid colliders (the default); see
  /// @ref PhysicsWorld::ComputeInverseInertia.
  kFromColliders,
  /// Left as you set it: use it for a body whose mass isn't spread like its
  /// shapes (a wheel with a heavy rim, a hollow box).
  kOverride,
  /// Torques and impulses never turn the body: the world keeps its inverse
  /// inertia at 0 (infinite inertia). Use it for characters that must stay
  /// upright. The body still turns at its `angular_velocity` if you set one.
  kFixedRotation
};

/// A rigid body: a position, a velocity and a mass, plus an orientation, an
/// angular velocity and an inertia.
///
/// Get one with @ref PhysicsWorld::AddBody and edit it in place through
/// @ref PhysicsWorld::body. A body has no shape on its own: attach one or more
/// @ref Collider to it. The body's `position` is its center of mass, the
/// point it turns around; colliders are placed around it with their offset.
///
/// Each @ref PhysicsWorld::Step integrates dynamic bodies with semi-implicit
/// Euler, the same way for motion and for spin, with the contacts solved
/// between the velocities and the positions:
///
/// @code
/// velocity += (gravity * gravity_scale + force * inverse_mass) * dt;
/// angular_velocity += torque * inverse_inertia * dt;
/// force = 0;
/// torque = 0;
/// // ... the contact solver corrects velocity and angular_velocity ...
/// position += velocity * dt;
/// angle += angular_velocity * dt;  // then wrapped to one turn
/// @endcode
///
/// Moving the positions last, with the corrected velocities, is what lets a
/// resting body stay at rest: its velocity after the contacts is zero, so it
/// doesn't sink by one step of gravity and get pushed back every step.
///
/// Torque is to spin what force is to motion, and inertia is to spin what
/// mass is to motion: the angular acceleration is `torque / inertia`.
/// Kinematic bodies move at their `velocity` and turn at their
/// `angular_velocity`; static bodies do neither.
struct Body {
  /// World-space position, in world units (meters with the default gravity).
  Vec2f position{};
  /// Velocity, in world units per second.
  Vec2f velocity{};
  /// Force accumulated for the next step only; cleared after each step.
  ///
  /// Add to it to push a dynamic body (`body.force += push`, or
  /// @ref PhysicsWorld::ApplyForce): it is scaled by `inverse_mass`, unlike
  /// gravity, which accelerates every body equally.
  Vec2f force{};
  /// One over the mass: 1 for a mass of 1, 0.5 for a mass of 2.
  ///
  /// Storing the inverse avoids divisions (slow on the DS) and makes infinite
  /// mass easy to express: 0 means the body is never moved by forces or
  /// contacts. Use 0 for static bodies. Kinematic and static bodies are
  /// treated as infinite mass in contacts whatever this value is.
  Scalar inverse_mass{1};
  /// Orientation: the angle in radians (counter-clockwise, wrapped to one
  /// turn) with its cached sine and cosine.
  ///
  /// Set it with `body.rotation.set_angle(angle)`. Collider offsets turn with
  /// it.
  Rotation rotation{};
  /// Angular velocity in radians per second, positive counter-clockwise.
  ///
  /// 2 pi rad/s is one turn per second.
  Scalar angular_velocity{};
  /// Torque accumulated for the next step only, in N m; cleared after each
  /// step like `force`.
  ///
  /// Positive turns counter-clockwise. Add to it directly for a pure torque
  /// (a motor), or use @ref PhysicsWorld::ApplyForceAtPoint, which adds both
  /// a force and the torque of that force about `position`.
  Scalar torque{};
  /// One over the moment of inertia, in 1/(kg m^2).
  ///
  /// The moment of inertia measures how hard the body is to spin: mass far
  /// from the center counts more (it grows with the distance squared). With
  /// the default @ref InertiaMode::kFromColliders, every
  /// @ref PhysicsWorld::Step computes it from `inverse_mass` and the solid
  /// colliders; it is 0 until then. 0 means infinite inertia: the body
  /// doesn't spin from torques or impulses.
  Scalar inverse_inertia{};
  /// How the world sets `inverse_inertia`, see @ref InertiaMode.
  InertiaMode inertia_mode = InertiaMode::kFromColliders;
  /// Multiplies the world's gravity for this body only: 1 by default, 0 for
  /// a body that floats (a balloon, a hovering drone), 2 to fall twice as
  /// fast (a heavier-feeling jump), negative to fall up.
  ///
  /// Gravity is an acceleration, the same for every mass, so scaling it
  /// can't be done with a force without knowing the mass: a force
  /// `m g (s - 1)` would do it, and that is what this field saves you.
  /// Hashed by @ref GenerateChecksum only when it isn't 1.
  Scalar gravity_scale{1};
  /// How the body moves, see @ref BodyType.
  BodyType type = BodyType::kDynamic;
  /// When `false` the body is frozen: not integrated and its colliders are
  /// left out of collision detection, but it keeps its slot.
  bool is_active = true;
};

/// Returns a hash of every field of `body`.
///
/// Building block of @ref PhysicsWorld::GenerateChecksum.
///
/// @param body @ref Body to hash.
/// @return FNV-1a hash of the body's fields.
[[nodiscard]] std::uint32_t GenerateChecksum(const Body& body);

/// Attaches a shape to a @ref Body and sets how it reacts to contacts.
///
/// Created with @ref PhysicsWorld::AddCircleCollider,
/// @ref PhysicsWorld::AddAabbCollider or
/// @ref PhysicsWorld::AddOrientedBoxCollider, which fill in `body_index`,
/// `collider_index`, `shape_index` and `type`; you then set the other fields
/// and the shape size (@ref PhysicsWorld::circle, @ref PhysicsWorld::aabb or
/// @ref PhysicsWorld::oriented_box with `shape_index`).
///
/// A collider is either:
/// - a **trigger** (`is_trigger`, the default): overlaps are only reported
///   through @ref ContactListener::OnTriggerEnter and
///   @ref ContactListener::OnTriggerExit, bodies pass through each other.
///   Use it for pickups, damage zones, checkpoints.
/// - **solid**: overlapping solid colliders are pushed apart and bounce, and
///   are reported through the `OnCollision*` events.
///
/// A pair is a trigger pair as soon as one of its two colliders is a trigger.
///
/// `category` and `mask` filter pairs before any of that: two colliders meet
/// only when each one's category is in the other's mask (see
/// @ref ShouldCollide). The defaults (category 1, every bit of the mask) let
/// everything meet.
struct Collider {
  /// Free pointer for your game (e.g. the entity that owns the collider).
  ///
  /// The physics never reads it and it is not part of the checksum. Prefer an
  /// index-like value over a real pointer if the world is copied for rollback.
  const void* user_data = nullptr;
  /// Position of the shape's center relative to the body's position, in the
  /// body's frame.
  ///
  /// It turns with the body: the shape's center in the world is
  /// `position + rotation.Rotate(offset)` (see
  /// @ref PhysicsWorld::ColliderCenter).
  Vec2f offset{};
  /// Bounciness, from 0 (no bounce) to 1 (perfectly elastic bounce).
  ///
  /// A contact uses the larger restitution of its two colliders.
  Scalar restitution{1};
  /// Friction coefficient: how hard the surfaces grip, from 0 (ice) to about
  /// 1 (rubber).
  ///
  /// Coulomb friction: sliding along a contact is resisted by a force up to
  /// `friction` times the force pressing the two shapes together. A contact
  /// uses `sqrt(friction1 * friction2)`, so a slippery surface stays
  /// slippery whatever touches it. 0.5 by default.
  Scalar friction{0.5f};
  /// @ref Body the collider is attached to.
  BodyIndex body_index{};
  /// Index of this collider (its own slot in the world).
  ColliderIndex collider_index{};
  /// Slot of the shape in the circle, box or oriented box table, depending
  /// on `type`.
  ShapeIndex shape_index{};
  /// Kind of shape; @ref ShapeType::kNone marks a free slot.
  ShapeType type = ShapeType::kNone;
  /// What the collider is, as one bit (or several) of 16: player, enemy,
  /// wall, bullet... Your game picks the meaning of each bit. 1 by default.
  ///
  /// World queries see it through @ref QueryFilter::mask.
  std::uint16_t category = 1;
  /// The categories this collider meets, one bit each. Every bit by default:
  /// it meets everything.
  ///
  /// The test is symmetric (see @ref ShouldCollide): a bullet whose mask
  /// leaves out the bullet bit ignores other bullets, whatever their own
  /// mask says.
  std::uint16_t mask = 0xFFFF;
  /// `true` for a trigger (detects overlaps only), `false` for a solid
  /// collider. Colliders are triggers by default.
  bool is_trigger = true;
};

/// Returns whether two colliders meet: each one's category is in the other's
/// mask.
///
/// The test runs on every broadphase pair before the shapes are compared. A
/// pair that fails it does nothing at all: no contact, no push, no trigger or
/// collision event. Because both directions must agree, either collider can
/// refuse the other.
///
/// @code
/// constexpr std::uint16_t kPlayer = 1 << 1;
/// constexpr std::uint16_t kBullet = 1 << 2;
/// // Players' bullets fly through players and through each other
/// bullet.category = kBullet;
/// bullet.mask = 0xFFFF & ~(kPlayer | kBullet);
/// @endcode
///
/// @param a First collider.
/// @param b Second collider.
/// @return `true` when `(a.category & b.mask) != 0` and
/// `(b.category & a.mask) != 0`.
[[nodiscard]] constexpr bool ShouldCollide(const Collider& a,
                                           const Collider& b) {
  return (a.category & b.mask) != 0 && (b.category & a.mask) != 0;
}
/// Returns a hash of the simulation fields of `collider`.
///
/// `user_data` is left out: it is game data, not simulation state.
/// `category` and `mask` are hashed only when they differ from their
/// defaults, so worlds that don't filter keep their checksums.
///
/// @param collider @ref Collider to hash.
/// @return FNV-1a hash of the collider's fields.
[[nodiscard]] std::uint32_t GenerateChecksum(const Collider& collider);

/// Circle shape of a @ref Collider, centered on the body position plus the
/// collider offset.
///
/// A negative radius marks a free slot.
struct CircleCollider {
  /// Radius in world units. @ref PhysicsWorld::AddCircleCollider sets it to 0:
  /// set the real size afterwards.
  Scalar radius{-1};
};

/// Returns a hash of `collider`'s radius.
///
/// @param collider @ref CircleCollider to hash.
/// @return FNV-1a hash of the radius.
[[nodiscard]] std::uint32_t GenerateChecksum(const CircleCollider& collider);

/// Axis-aligned box shape of a @ref Collider, centered on the body position
/// plus the collider offset.
///
/// AABB stands for axis-aligned bounding box: its sides stay parallel to the
/// x and y axes, which makes overlap tests very cheap. A negative half size
/// marks a free slot.
///
/// On a rotating body the box doesn't turn: its center follows the rotated
/// offset, but its sides stay parallel to the world axes, for collisions and
/// for the broadphase. For the inertia it counts as a box of the same size
/// (a box's inertia about its center doesn't depend on its angle). For a box
/// that turns with its body, use @ref OrientedBoxCollider.
struct AabbCollider {
  /// Half the width and half the height of the box, in world units.
  ///
  /// @ref PhysicsWorld::AddAabbCollider sets it to zero: set the real size
  /// afterwards. Each component is a @ref Scalar; the default of -1 marks a
  /// free slot.
  Vec2f half_size{Scalar{-1}, Scalar{-1}};
};

/// Returns a hash of `collider`'s half size.
///
/// @param collider @ref AabbCollider to hash.
/// @return FNV-1a hash of the half size.
[[nodiscard]] std::uint32_t GenerateChecksum(const AabbCollider& collider);

/// Box shape of a @ref Collider that turns with its body.
///
/// Its center is the body position plus the rotated collider offset, and its
/// sides follow the body's local axes (@ref Rotation::x_axis and
/// @ref Rotation::y_axis): a crate that tips over, a plank, a tilted ramp
/// (a static body whose rotation is set once).
///
/// Collisions with other boxes use the separating axis theorem (SAT): two
/// convex shapes don't overlap if and only if some axis separates their
/// projections, and for two boxes it is enough to test the four face
/// normals. When every axis overlaps, the one with the smallest overlap gives
/// the contact normal and the penetration. Contact points come from the
/// deepest corner (one point per pair); resting face to face needs two points,
/// which comes later.
///
/// It costs more than an @ref AabbCollider (four projections instead of four
/// comparisons), so keep @ref AabbCollider for boxes that never turn.
///
/// A negative half size marks a free slot.
struct OrientedBoxCollider {
  /// Half the width and half the height of the box along the body's local x
  /// and y axes, in world units.
  ///
  /// @ref PhysicsWorld::AddOrientedBoxCollider sets it to zero: set the real
  /// size afterwards. Each component is a @ref Scalar; the default of -1
  /// marks a free slot.
  Vec2f half_size{Scalar{-1}, Scalar{-1}};
};

/// Returns a hash of `collider`'s half size.
///
/// @param collider @ref OrientedBoxCollider to hash.
/// @return FNV-1a hash of the half size.
[[nodiscard]] std::uint32_t GenerateChecksum(
    const OrientedBoxCollider& collider);

/// Pair of colliders that touched at the end of the last step.
///
/// The world compares the touching pairs of one step with those of the
/// previous step to send enter and exit events to the @ref ContactListener.
struct TouchingPair {
  /// The two colliders.
  ColliderPair pair{};
  /// `true` when at least one of the two colliders is a trigger.
  bool is_trigger = false;
};

/// Settings of a @ref PhysicsWorld, fixed for the world's lifetime.
///
/// All tables are allocated once, in the @ref PhysicsWorld constructor, with
/// these capacities: @ref PhysicsWorld::Step and @ref PhysicsWorld::CopyFrom
/// never allocate, which keeps frame times stable and fits the DS's small
/// memory. Pick capacities for the largest scene you need.
///
/// @code
/// neko::PhysicsWorldConfig config;
/// config.gravity = neko::Vec2f{neko::Scalar{0}, neko::Scalar{-20}};
/// config.max_bodies = 64;
/// config.max_colliders = 64;
/// neko::PhysicsWorld world{config};
/// @endcode
struct PhysicsWorldConfig {
  /// Acceleration applied to every dynamic body, in world units per second
  /// squared.
  ///
  /// Each component is a @ref Scalar. The default is (0, -9.81).
  Vec2f gravity{Scalar{0.0f}, Scalar{-9.81f}};
  /// Duration of one @ref PhysicsWorld::Step, in seconds.
  ///
  /// Fixed, so that every peer and every rollback resimulation uses the same
  /// value: a variable time step would make results depend on the frame rate.
  /// Call @ref PhysicsWorld::Step from a fixed-step loop (an accumulator) in
  /// your game.
  Scalar dt{1.0f / 60.0f};
  /// Maximum number of bodies alive at once.
  std::size_t max_bodies = 256;
  /// Maximum number of colliders alive at once (circles and boxes together).
  std::size_t max_colliders = 256;
  /// Maximum number of touching pairs tracked for enter/exit events.
  ///
  /// Exceeding it asserts in debug builds; extra pairs are still resolved but
  /// not reported.
  std::size_t max_touching_pairs = 1024;
  /// Maximum number of pairs whose bounding boxes overlap, for the default
  /// broadphase (@ref SweepAndPrune).
  std::size_t max_broadphase_pairs = 2048;
  /// Maximum number of joints alive at once (see @ref DistanceJoint).
  ///
  /// A joint and its solver data take about 100 bytes; 0 for a world
  /// without joints.
  std::size_t max_joints = 64;
  /// Passes of the contact solver over all the contacts of a step, see
  /// @ref ContactSolverSettings::velocity_iterations.
  ///
  /// The solver's cost grows linearly with it.
  int velocity_iterations = 8;
  /// Fraction of a penetration removed per step, see
  /// @ref ContactSolverSettings::baumgarte. The default is 0.2.
  Scalar baumgarte{0.2f};
  /// Overlap left alone, in meters, see
  /// @ref ContactSolverSettings::penetration_slop. The default is 0.005.
  Scalar penetration_slop{0.005f};
  /// Slowest approach speed that bounces, in m/s, see
  /// @ref ContactSolverSettings::restitution_threshold. The default is 1.
  Scalar restitution_threshold{1};
  /// Whether the contact and joint solver starts from last step's impulses,
  /// see @ref ContactSolverSettings::warm_starting.
  ///
  /// The world keeps the impulses of every touching solid pair in a contact
  /// cache, matched by @ref ContactPoint::id, and each joint keeps its own in
  /// @ref DistanceJoint::impulse. Both are simulation state:
  /// @ref PhysicsWorld::CopyFrom copies it and
  /// @ref PhysicsWorld::GenerateChecksum hashes it, so rollback replays stay
  /// exact.
  bool warm_starting = true;
  /// Frequency of the length correction of rigid and rope joints, in Hz, see
  /// @ref ContactSolverSettings::joint_hertz. The default is 15.
  Scalar joint_hertz{15};
  /// Damping ratio of the length correction of rigid and rope joints, see
  /// @ref ContactSolverSettings::joint_damping_ratio. The default is 2.
  Scalar joint_damping_ratio{2};
  /// @ref Allocator for all tables; `nullptr` means @ref DefaultAllocator.
  ///
  /// @warning Must outlive the world.
  Allocator* allocator = nullptr;
};

/// The physics simulation: owns bodies, colliders and shapes, and advances
/// them in time.
///
/// Typical use: build the world once, add bodies and colliders, then call
/// @ref Step at a fixed rate (e.g. 60 times per second) and read the bodies'
/// positions to draw them. See the namespace documentation for a full
/// example.
///
/// Each @ref Step runs four phases:
///
/// - **Velocities**: the inertia of dynamic bodies is computed from their
///   colliders, springs add their forces, then dynamic bodies get gravity,
///   forces and torques.
/// - **Broadphase**: a fast, approximate pass that finds the pairs of
///   colliders whose bounding boxes overlap, to avoid testing every collider
///   against every other one.
/// - **Narrowphase**: the exact shape test of each candidate pair, the
///   solver for the solid pairs and the rigid and rope joints
///   (@ref SolveConstraints), and the enter/exit events.
/// - **Positions**: dynamic and kinematic bodies move and turn at their
///   (corrected) velocities.
///
/// See @ref Body for the integration formula, @ref ResolveBroadphase and
/// @ref ResolveNarrowphase for the two collision phases, and
/// @ref SolveContacts for the contact solver.
///
/// Bodies, colliders and shapes live in fixed-size tables and are referred to
/// by index (@ref BodyIndex, @ref ColliderIndex, @ref ShapeIndex); freed slots
/// are reused by later additions. For rollback netcode, keep snapshot worlds
/// built with the same config and use @ref CopyFrom to save and restore.
///
/// @note References returned by @ref body, @ref collider, @ref circle,
/// @ref aabb and @ref oriented_box point into the world's tables: don't keep
/// them across a
/// @ref CopyFrom or a move of the world; keep indices instead.
class PhysicsWorld {
 public:
  /// Builds an empty world and allocates all its tables.
  ///
  /// @param config Gravity, time step, capacities and allocator.
  /// @pre `config.max_bodies`, `config.max_colliders` and
  /// `config.max_joints` fit in an @ref Index.
  explicit PhysicsWorld(const PhysicsWorldConfig& config = {});
  /// Deleted: a copy would allocate. Rollback snapshots use @ref CopyFrom
  /// into an existing world instead.
  PhysicsWorld(const PhysicsWorld&) = delete;
  /// Deleted: a copy would allocate. Use @ref CopyFrom instead.
  PhysicsWorld& operator=(const PhysicsWorld&) = delete;
  /// Moves the world, its tables and its listener and BSH pointers.
  PhysicsWorld(PhysicsWorld&&) = default;
  /// Moves the world, its tables and its listener and BSH pointers.
  PhysicsWorld& operator=(PhysicsWorld&&) = default;
  /// Destroys the world and frees its tables.
  ~PhysicsWorld() = default;

  /// Adds a dynamic body at the origin, with default @ref Body values.
  ///
  /// Reuses the first free slot, so the indices of removed bodies come back.
  ///
  /// @return The new body's index, or @ref kInvalidBodyIndex when
  /// `max_bodies` bodies are in use.
  BodyIndex AddBody();
  /// Removes the body, all its colliders and all its joints, freeing their
  /// slots.
  ///
  /// @param index @ref Body to remove.
  void RemoveBody(BodyIndex index);
  /// Narrowphase test of two colliders: checks whether their exact shapes
  /// overlap and fills in `contact`.
  ///
  /// Handles every pair of circles, boxes and oriented boxes. The contact
  /// normal,
  /// point and penetration are only computed when both colliders are solid;
  /// for a trigger pair only the overlap result matters. Called by
  /// @ref ResolveNarrowphase; it doesn't change the bodies.
  ///
  /// @param body1 @ref Body of `collider1`.
  /// @param collider1 First collider.
  /// @param body2 @ref Body of `collider2`.
  /// @param collider2 Second collider.
  /// @param contact Output; its normal points from @p body2 towards
  /// `body1`.
  /// @return `true` if the shapes overlap.
  bool DetectContact(Body& body1, Collider& collider1, Body& body2,
                     Collider& collider2, Contact* contact);
  /// Advances the world by one time step of `config.dt` seconds.
  ///
  /// Integrates the bodies, then runs @ref ResolveBroadphase and
  /// @ref ResolveNarrowphase, which resolves contacts and calls the
  /// @ref ContactListener. Never allocates.
  void Step();
  /// Removes every body, collider and shape and forgets touching pairs.
  ///
  /// Keeps the allocated capacity, the listener and the BSH. No exit events
  /// are sent.
  void Clear();
  /// Runs the broadphase: inserts the bounding box of every active collider
  /// in the BSH and computes the pairs whose boxes overlap.
  ///
  /// Called by @ref Step; public for tests and profiling.
  void ResolveBroadphase();
  /// Runs the narrowphase on the broadphase pairs: exact tests, contact
  /// resolution for solid pairs and enter/exit events.
  ///
  /// Skips pairs where a body is inactive, both bodies are static, both
  /// colliders belong to the same body, @ref ShouldCollide refuses the pair,
  /// or a joint links two solid colliders' bodies (see
  /// @ref DistanceJoint::collide_connected). Called by @ref Step after
  /// @ref ResolveBroadphase; public for tests and profiling.
  void ResolveNarrowphase();
  /// Sorts this step's touching pairs by @ref ColliderPair (c1, then c2).
  ///
  /// Gives the same total order as `std::sort` on `pair`, but in
  /// O(n + max_colliders): a counting sort on c1 (colliders are indices into
  /// a small, fixed-size table) followed by an insertion sort of each c1
  /// bucket, which is short (a body only touches a handful of others). The
  /// sorted order makes event delivery deterministic. Called by
  /// @ref ResolveNarrowphase; public for tests.
  void SortNewTouchingPairs();

  /// Returns the body at `index`.
  ///
  /// @param index A valid body index.
  /// @return The body, which can be edited in place.
  [[nodiscard]] Body& body(BodyIndex index) { return bodies_[index.index()]; }
  /// Returns the body at @p index (read-only).
  ///
  /// @param index A valid body index.
  /// @return The body.
  [[nodiscard]] const Body& body(BodyIndex index) const {
    return bodies_[index.index()];
  }

  /// Applies a force through the center of mass of a body, for the next
  /// step: thrust, wind, a magnet.
  ///
  /// A force acts over time: it is added to @ref Body::force, and the next
  /// @ref Step turns it into `velocity += force * inverse_mass * dt`, then
  /// clears it. To push for a while, apply it every step: the velocity
  /// change over one second is `force / mass` whatever the time step, which
  /// is what makes forces the right tool for anything continuous.
  ///
  /// @code
  /// // Jetpack: 30 N up while the button is held, applied every step
  /// if (thrust_held) {
  ///   world.ApplyForce(player, {neko::Scalar{0}, neko::Scalar{30}});
  /// }
  /// @endcode
  ///
  /// @param index @ref Body to push. Only dynamic bodies use forces.
  /// @param force Force in newtons.
  /// @see @ref ApplyImpulse for a push all at once.
  void ApplyForce(BodyIndex index, Vec2f force);
  /// Applies an impulse through the center of mass of a body, right now:
  /// a kick, a jump, a knockback.
  ///
  /// An impulse is a force times the (short) time it acts, in N s: it
  /// changes the velocity at once, `velocity += impulse * inverse_mass`, so
  /// a heavier body gets a smaller kick. Apply it once per event, not every
  /// step: applied every step its effect would grow with the step rate,
  /// which is the frame-dependent behavior forces avoid.
  ///
  /// @code
  /// // Knockback from a hit: 4 N s away from the attacker
  /// world.ApplyImpulse(player, away_from_attacker * neko::Scalar{4});
  /// @endcode
  ///
  /// @param index @ref Body to push. Kinematic and static bodies are left
  /// unchanged.
  /// @param impulse Impulse in N s (kg m/s).
  /// @see @ref ApplyImpulseAtPoint to also make the body spin.
  void ApplyImpulse(BodyIndex index, Vec2f impulse);
  /// Applies a force at a point of the world to a body, for the next step.
  ///
  /// A force through the center of mass only pushes the body; the same
  /// force applied off-center also turns it. The force is added to
  /// @ref Body::force (the whole force moves the center of mass, wherever it
  /// is applied) and the torque `Vec2f::Cross(point - position, force)` to
  /// @ref Body::torque. `point - position` is the lever arm: only the part
  /// of the force perpendicular to it turns the body, and the farther the
  /// point, the larger the torque.
  ///
  /// @code
  /// // Push a crate up at its right edge, 0.5 m from its center: it rises
  /// // and turns counter-clockwise (torque 0.5 * 10 = 5 N m)
  /// const neko::Scalar half = neko::Scalar{1} / neko::Scalar{2};
  /// const neko::Vec2f edge =
  ///     world.body(crate).position + neko::Vec2f{half, neko::Scalar{0}};
  /// world.ApplyForceAtPoint(crate, {neko::Scalar{0}, neko::Scalar{10}}, edge);
  /// @endcode
  ///
  /// Two opposite forces at opposite points (a couple) cancel out on the
  /// center of mass but add up their torques: the body spins in place.
  ///
  /// @param index @ref Body to push. Only dynamic bodies use forces.
  /// @param force Force in newtons.
  /// @param point World-space point where the force is applied, in meters.
  void ApplyForceAtPoint(BodyIndex index, Vec2f force, Vec2f point);
  /// Applies an impulse at a point of the world to a body, right now.
  ///
  /// An impulse is a force times a (short) duration, in N s: it changes the
  /// velocities at once instead of over the next step, which is how contacts
  /// push bodies. `velocity += impulse * inverse_mass` and
  /// `angular_velocity += Vec2f::Cross(point - position, impulse) *
  /// inverse_inertia`. With @ref InertiaMode::kFromColliders the body's
  /// inverse inertia is computed first, so this works before the first
  /// @ref Step too.
  ///
  /// @param index @ref Body to push. Kinematic and static bodies are left
  /// unchanged.
  /// @param impulse Impulse in N s (kg m/s).
  /// @param point World-space point where the impulse is applied, in meters.
  void ApplyImpulseAtPoint(BodyIndex index, Vec2f impulse, Vec2f point);
  /// Computes the inverse inertia of a body from its mass and solid
  /// colliders, as @ref InertiaMode::kFromColliders does every step.
  ///
  /// The mass is spread over the solid colliders in proportion to their area
  /// (a uniform density); triggers have no mass. Each collider adds its own
  /// inertia about its center, `m r^2 / 2` for a circle of radius `r` and
  /// `m (w^2 + h^2) / 12` for a box of width `w` and height `h`, plus
  /// `m d^2` for its offset `d` from the body's position (the parallel axis
  /// theorem: the same mass turning farther from the center is harder to
  /// spin). The sum is computed per kilogram, then
  /// `inverse_inertia = inverse_mass / sum`.
  ///
  /// @note With @ref Fixed16 each term is rounded to 1/65536 m^2 per
  /// kilogram, so small shapes lose relative precision (measured in
  /// `tests/conventions_test.cc`).
  /// @param index A valid body index.
  /// @return The inverse inertia in 1/(kg m^2), or 0 when the body has no
  /// solid collider of non-zero size (or an infinite mass).
  [[nodiscard]] Scalar ComputeInverseInertia(BodyIndex index) const;
  /// Returns the world-space center of a collider's shape:
  /// `position + rotation.Rotate(offset)` of its body.
  ///
  /// @param index A valid collider index.
  /// @return The center of the circle or box, in meters.
  [[nodiscard]] Vec2f ColliderCenter(ColliderIndex index) const;

  /// Adds a circle collider of radius 0 to `body`.
  ///
  /// The collider is a trigger with restitution 1 and no offset: set
  /// `is_trigger`, `restitution` and the radius
  /// (`circle(collider(index).shape_index).radius`) afterwards.
  ///
  /// @param body @ref Body to attach the collider to.
  /// @return The new collider's index, or @ref kInvalidColliderIndex when
  /// `max_colliders` colliders are in use.
  ColliderIndex AddCircleCollider(BodyIndex body);
  /// Adds a box collider of half size zero to `body`.
  ///
  /// The collider is a trigger with restitution 1 and no offset: set
  /// `is_trigger`, `restitution` and the half size
  /// (`aabb(collider(index).shape_index).half_size`) afterwards.
  ///
  /// @param body @ref Body to attach the collider to.
  /// @return The new collider's index, or @ref kInvalidColliderIndex when
  /// `max_colliders` colliders are in use.
  ColliderIndex AddAabbCollider(BodyIndex body);
  /// Adds an oriented box collider of half size zero to `body`.
  ///
  /// The box turns with the body (see @ref OrientedBoxCollider). The
  /// collider is a trigger with restitution 1 and no offset: set
  /// `is_trigger`, `restitution` and the half size
  /// (`oriented_box(collider(index).shape_index).half_size`) afterwards.
  ///
  /// @param body @ref Body to attach the collider to.
  /// @return The new collider's index, or @ref kInvalidColliderIndex when
  /// `max_colliders` colliders are in use.
  ColliderIndex AddOrientedBoxCollider(BodyIndex body);
  /// Removes a collider and its shape, freeing their slots.
  ///
  /// @param index @ref Collider to remove.
  void RemoveCollider(ColliderIndex index);

  /// Adds a rigid distance joint between the positions of two bodies.
  ///
  /// The joint is a @ref DistanceJointType::kRigid rod with both anchors at
  /// the bodies' positions and a `length` equal to their current distance:
  /// it holds them where they are. Change its type, anchors, length or
  /// spring constants through @ref joint afterwards. Reuses the first free
  /// slot.
  ///
  /// @code
  /// // A pendulum: a bob hanging 2 m below a static pivot
  /// const neko::JointIndex rod = world.AddDistanceJoint(pivot, bob);
  /// // world.joint(rod).length is 2: the bob swings on a circle of radius 2
  /// world.joint(rod).type = neko::DistanceJointType::kRope;  // or a rope
  /// @endcode
  ///
  /// @param body_a First body.
  /// @param body_b Second body, different from `body_a`.
  /// @return The new joint's index, or @ref kInvalidJointIndex when
  /// `max_joints` joints are in use.
  JointIndex AddDistanceJoint(BodyIndex body_a, BodyIndex body_b);
  /// Removes a joint, freeing its slot.
  ///
  /// @param index @ref DistanceJoint to remove.
  void RemoveJoint(JointIndex index);
  /// Returns the joint at `index`.
  ///
  /// @param index A valid joint index.
  /// @return The joint, which can be edited in place.
  [[nodiscard]] DistanceJoint& joint(JointIndex index) {
    return joints_[index.index()];
  }
  /// Returns the joint at @p index (read-only).
  ///
  /// @param index A valid joint index.
  /// @return The joint.
  [[nodiscard]] const DistanceJoint& joint(JointIndex index) const {
    return joints_[index.index()];
  }
  /// Returns the world-space positions of a joint's two anchors:
  /// `position + rotation.Rotate(local_anchor)` of each body.
  ///
  /// @param index A valid joint index.
  /// @return Anchor A then anchor B, in meters.
  [[nodiscard]] std::array<Vec2f, 2> JointAnchors(JointIndex index) const;
  /// Returns the current distance between a joint's two anchors, in meters.
  ///
  /// Compare it with @ref DistanceJoint::length to measure how well a rigid
  /// joint holds (the length error) or how far a spring is stretched.
  ///
  /// @param index A valid joint index.
  /// @return The distance between the anchors.
  [[nodiscard]] Scalar JointDistance(JointIndex index) const;

  /// Finds the colliders that overlap a circle: an explosion radius, an
  /// area of effect, "is anything here?".
  ///
  /// Queries read the bodies as they are now (positions you just set
  /// included), not the last step's broadphase, and change nothing. They
  /// test every collider in table order (a scan, simple and exact; fine for
  /// a few hundred colliders and a few queries per step). Touching counts as
  /// overlapping, like contacts.
  ///
  /// Results go in a buffer you own, in collider index order; the return
  /// value counts every overlap, so a count above `hits.size()` means the
  /// buffer was too small and only the first `hits.size()` were written.
  ///
  /// @code
  /// std::array<neko::ColliderIndex, 16> hits;
  /// const std::size_t count = world.OverlapCircle(center, radius, hits);
  /// for (std::size_t i = 0; i < std::min(count, hits.size()); ++i) {
  ///   const neko::BodyIndex body = world.collider(hits[i]).body_index;
  ///   // ...
  /// }
  /// @endcode
  ///
  /// @note A body with several colliders can appear several times: your game
  /// decides whether that is one hit or several (one damage event per body,
  /// usually).
  /// @param center Center of the circle, in meters.
  /// @param radius Radius, in meters.
  /// @param hits Output buffer, can be empty to only count.
  /// @param filter Which colliders count, see @ref QueryFilter.
  /// @return The number of overlapping colliders, which can exceed
  /// `hits.size()`.
  std::size_t OverlapCircle(Vec2f center, Scalar radius,
                            std::span<ColliderIndex> hits,
                            QueryFilter filter = {}) const;
  /// Finds the colliders that overlap an axis-aligned box.
  ///
  /// Same rules as @ref OverlapCircle: current positions, table order,
  /// touching counts, total count returned.
  ///
  /// @param box The box, in world coordinates.
  /// @param hits Output buffer, can be empty to only count.
  /// @param filter Which colliders count, see @ref QueryFilter.
  /// @return The number of overlapping colliders, which can exceed
  /// `hits.size()`.
  std::size_t OverlapAabb(const Aabbf& box, std::span<ColliderIndex> hits,
                          QueryFilter filter = {}) const;
  /// Casts a segment from `start` to `end` and finds the colliders it
  /// touches, nearest first: line of sight, bullets, lasers, ground probes.
  ///
  /// For each shape the hit is where the segment first enters it (see
  /// @ref RaycastHit). A circle is solved as a quadratic: the segment's
  /// closest approach to the center, then back by
  /// `sqrt(radius^2 - distance^2)`. A box is clipped against its two slabs
  /// (between its left and right sides, between its bottom and top): the
  /// segment is inside the box between the latest entry and the earliest
  /// exit. An oriented box is the same in its own frame.
  ///
  /// The rules:
  /// - Hits are sorted by @ref RaycastHit::fraction, and equal fractions by
  ///   collider index, so the order never depends on the platform.
  /// - A segment that starts inside a shape (or on its boundary) hits it at
  ///   fraction 0, with a zero normal.
  /// - A segment that only grazes a shape (tangent to a circle, along a box
  ///   side) hits it: touching counts.
  /// - A segment of length zero hits the shapes that contain its start.
  /// - When more colliders are hit than `hits` holds, the nearest
  ///   `hits.size()` are kept, and the return value still counts them all.
  ///
  /// @note With @ref Fixed16 keep the segment under about 100 m: the circle
  /// test squares distances of the order of its length.
  /// @param start Start of the segment, in meters.
  /// @param end End of the segment, in meters.
  /// @param hits Output buffer, nearest hits first; a buffer of one keeps
  /// the nearest (see @ref RaycastClosest).
  /// @param filter Which colliders count, see @ref QueryFilter.
  /// @return The number of colliders hit, which can exceed `hits.size()`.
  std::size_t Raycast(Vec2f start, Vec2f end, std::span<RaycastHit> hits,
                      QueryFilter filter = {}) const;
  /// Returns the nearest hit of a segment, if any: @ref Raycast with a
  /// buffer of one.
  ///
  /// @code
  /// // Can the guard see the player? Only walls block the view
  /// const auto hit = world.RaycastClosest(guard_eye, player_center,
  ///                                       {.mask = kWall | kPlayer});
  /// const bool sees = hit.has_value() &&
  ///                   world.collider(hit->collider).category == kPlayer;
  /// @endcode
  ///
  /// @param start Start of the segment, in meters.
  /// @param end End of the segment, in meters.
  /// @param filter Which colliders count, see @ref QueryFilter.
  /// @return The nearest @ref RaycastHit, or no value when nothing is hit.
  [[nodiscard]] std::optional<RaycastHit> RaycastClosest(
      Vec2f start, Vec2f end, QueryFilter filter = {}) const;

  /// Returns the collider at `collider_index`.
  ///
  /// @param collider_index A valid collider index.
  /// @return The collider, which can be edited in place.
  [[nodiscard]] Collider& collider(ColliderIndex collider_index) {
    return colliders_[collider_index.index()];
  }
  /// Returns the collider at @p collider_index (read-only).
  ///
  /// @param collider_index A valid collider index.
  /// @return The collider.
  [[nodiscard]] const Collider& collider(ColliderIndex collider_index) const {
    return colliders_[collider_index.index()];
  }

  /// Returns the box shape at `shape_index`.
  ///
  /// @param shape_index `shape_index` of a collider of type
  /// @ref ShapeType::kAabb.
  /// @return The box shape, which can be edited in place.
  [[nodiscard]] AabbCollider& aabb(ShapeIndex shape_index) {
    return aabbs_[shape_index.index()];
  }
  /// Returns the box shape at @p shape_index (read-only).
  ///
  /// @param shape_index `shape_index` of a collider of type
  /// @ref ShapeType::kAabb.
  /// @return The box shape.
  [[nodiscard]] const AabbCollider& aabb(ShapeIndex shape_index) const {
    return aabbs_[shape_index.index()];
  }
  /// Returns the circle shape at `shape_index`.
  ///
  /// @param shape_index `shape_index` of a collider of type
  /// @ref ShapeType::kCircle.
  /// @return The circle shape, which can be edited in place.
  [[nodiscard]] CircleCollider& circle(ShapeIndex shape_index) {
    return circles_[shape_index.index()];
  }
  /// Returns the circle shape at @p shape_index (read-only).
  ///
  /// @param shape_index `shape_index` of a collider of type
  /// @ref ShapeType::kCircle.
  /// @return The circle shape.
  [[nodiscard]] const CircleCollider& circle(ShapeIndex shape_index) const {
    return circles_[shape_index.index()];
  }
  /// Returns the oriented box shape at `shape_index`.
  ///
  /// @param shape_index `shape_index` of a collider of type
  /// @ref ShapeType::kOrientedBox.
  /// @return The oriented box shape, which can be edited in place.
  [[nodiscard]] OrientedBoxCollider& oriented_box(ShapeIndex shape_index) {
    return oriented_boxes_[shape_index.index()];
  }
  /// Returns the oriented box shape at @p shape_index (read-only).
  ///
  /// @param shape_index `shape_index` of a collider of type
  /// @ref ShapeType::kOrientedBox.
  /// @return The oriented box shape.
  [[nodiscard]] const OrientedBoxCollider& oriented_box(
      ShapeIndex shape_index) const {
    return oriented_boxes_[shape_index.index()];
  }

  /// Replaces the broadphase.
  ///
  /// By default the world uses its own @ref SweepAndPrune; pass a
  /// @ref QuadTree (or any @ref BoundingSurfaceHierarchy) to use it instead.
  /// The world rebuilds it every step, so it holds no state to save for
  /// rollback.
  ///
  /// @param bounding_surface_hierarchy The broadphase to use, which must
  /// outlive its use by the world, or `nullptr` to restore the default one.
  void SetBSH(BoundingSurfaceHierarchy* bounding_surface_hierarchy) {
    bsh_ = bounding_surface_hierarchy;
  }
  /// Registers the object that receives contact events.
  ///
  /// @param contact_listener Listener, which must outlive its use by the
  /// world, or `nullptr` for no events.
  void SetContactListener(ContactListener* contact_listener) {
    contact_listener_ = contact_listener;
  }

  /// Returns the gravity from the config.
  [[nodiscard]] Vec2f gravity() const { return config_.gravity; }
  /// Returns the time step from the config, in seconds.
  [[nodiscard]] Scalar dt() const { return config_.dt; }
  /// Returns the pairs of colliders touching at the end of the last step.
  ///
  /// Use it to poll the current contacts instead of tracking events.
  ///
  /// @return The touching pairs, sorted by @ref ColliderPair.
  [[nodiscard]] const ArrayList<TouchingPair>& touching_pairs() const {
    return touching_pairs_;
  }
  /// Returns the number of pairs whose bounding boxes overlap, found by the
  /// broadphase during the last @ref ResolveBroadphase.
  ///
  /// Useful to compare broadphases: the narrowphase tests exactly these
  /// pairs.
  ///
  /// @return The number of candidate pairs.
  [[nodiscard]] std::size_t BroadphasePairCount() const {
    return Broadphase().GetPossiblePairs().size();
  }

  /// Copies the simulation state of @p physics_world into this world.
  ///
  /// This is how rollback netcode saves and restores the world: keep a
  /// snapshot world, `snapshot.CopyFrom(world)` to save and
  /// `world.CopyFrom(snapshot)` to roll back, then resimulate. Copies bodies,
  /// colliders, shapes, joints, touching pairs (so enter/exit events stay
  /// consistent after a rollback) and the contact cache, not the listener nor
  /// the BSH.
  ///
  /// @param physics_world World to copy from.
  /// @note Doesn't allocate when both worlds were built with the same
  /// capacities.
  void CopyFrom(const PhysicsWorld& physics_world);
  /// Returns a hash of the whole simulation state.
  ///
  /// Peers of a networked game exchange checksums of the same frame: a
  /// mismatch means their simulations diverged (a desync). Covers bodies,
  /// colliders, shapes, touching pairs, the contact cache and joints, but not
  /// @ref Collider::user_data.
  ///
  /// @return FNV-1a hash of the world state.
  [[nodiscard]] std::uint32_t GenerateChecksum() const;

 private:
  void NotifyContactChanges() const;
  [[nodiscard]] BoundingSurfaceHierarchy& Broadphase() {
    return bsh_ != nullptr ? *bsh_ : default_broadphase_;
  }
  [[nodiscard]] const BoundingSurfaceHierarchy& Broadphase() const {
    return bsh_ != nullptr ? static_cast<const BoundingSurfaceHierarchy&>(*bsh_)
                           : default_broadphase_;
  }
  [[nodiscard]] Aabbf GetAabb(const Collider& collider) const;
  // Area and inertia per kilogram about the body's position of a solid
  // collider, both 0 for a trigger or a free slot
  struct ColliderMass {
    Scalar area{};
    Scalar unit_inertia{};
  };
  [[nodiscard]] ColliderMass GetColliderMass(const Collider& collider) const;
  // Sets inverse_inertia from inertia_mode for every active dynamic body
  void UpdateInertia();
  // Copies the cached impulses into this step's contacts, and back
  void WarmStartContacts();
  void CacheContactImpulses();
  // Adds the Hooke and damping forces of the springs to their bodies
  void ApplySpringForces();
  // Marks the bodies of joints that filter their collisions, returns whether
  // there is any
  [[nodiscard]] bool UpdateJointFilter();
  // Whether a joint without collide_connected links the two bodies
  [[nodiscard]] bool AreJointFiltered(BodyIndex body1, BodyIndex body2) const;
  // Whether a query sees the collider: a live shape on an active body, in the
  // filter's mask, and solid unless the filter takes triggers
  [[nodiscard]] bool IsQueried(const Collider& collider,
                               QueryFilter filter) const;
  // Where the segment from start by delta (end - start) enters the collider
  [[nodiscard]] std::optional<RaycastHit> RaycastCollider(
      const Collider& collider, Vec2f start, Vec2f delta) const;

  PhysicsWorldConfig config_;
  SweepAndPrune default_broadphase_;
  ArrayList<Body> bodies_;
  ArrayList<AabbCollider> aabbs_;
  ArrayList<CircleCollider> circles_;
  ArrayList<OrientedBoxCollider> oriented_boxes_;
  ArrayList<Collider> colliders_;
  ArrayList<TouchingPair> touching_pairs_;
  ArrayList<TouchingPair> new_touching_pairs_;
  // Scratch buffer for ResolveBroadphase: each active collider's Aabbf,
  // computed once per Step and reused for both the world box pass and the
  // insert pass (GetAabb isn't free: a body lookup plus a shape switch)
  ArrayList<Aabbf> collider_aabb_cache_;
  // Scratch for UpdateInertia, one entry per body: the total area of its solid
  // colliders, then its inertia per kilogram
  ArrayList<Scalar> body_area_;
  // Scratch for UpdateInertia, one entry per collider: its mass, or zero when
  // it doesn't count (trigger, free slot, body not computed)
  ArrayList<ColliderMass> collider_mass_cache_;
  ArrayList<Scalar> body_unit_inertia_;
  // Scratch for SortNewTouchingPairs: bucketCount_[k] then bucketStart_[k]
  // hold, then temporarily the write cursor for, colliders whose
  // ColliderPair::c1.index == k; sorted_ is the counting sort's output buffer.
  ArrayList<std::uint32_t> touching_pair_bucket_count_;
  ArrayList<std::uint32_t> touching_pair_bucket_start_;
  ArrayList<TouchingPair> sorted_touching_pairs_;
  // The step's solid contacts, solved together after the narrowphase, and
  // the solver's working memory (two points per contact)
  ArrayList<Contact> contacts_;
  ArrayList<internal::ContactPointScratch> contact_scratch_;
  // The broadphase pair of each contact (DetectContact may swap the colliders)
  ArrayList<ColliderPair> contact_pairs_;
  // Impulses of the touching solid pairs at the end of the last step, sorted
  // by pair, to warm start the next one; copied and hashed (simulation state)
  struct CachedPoint {
    std::uint8_t id = 0;
    Scalar normal_impulse{};
    Scalar tangent_impulse{};
  };
  struct CachedContact {
    ColliderPair pair{};
    std::array<CachedPoint, 2> points{};
    std::uint8_t point_count = 0;
  };
  ArrayList<CachedContact> contact_cache_;
  ArrayList<CachedContact> new_contact_cache_;
  // Joints (simulation state, copied and hashed) and the solver's per-joint
  // working memory
  ArrayList<DistanceJoint> joints_;
  ArrayList<internal::JointScratch> joint_scratch_;
  // Scratch, one entry per body: 1 when it belongs to a joint that filters
  // its collisions
  ArrayList<std::uint8_t> body_joint_filter_;

  ContactListener* contact_listener_ = nullptr;
  BoundingSurfaceHierarchy* bsh_ = nullptr;
};

}  // namespace neko

#endif  // NEKO_PHYSICS_PHYSICS_H_
