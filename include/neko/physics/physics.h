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
#include "neko/math/angle.h"
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
///   @ref PhysicsWorld::AddDistanceJoint). @ref RevoluteJoint pins them
///   together at a point, a hinge with optional angle limits (see
///   @ref PhysicsWorld::AddRevoluteJoint).
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
///   those pairs only and builds a @ref Contact. The contact solver (see
///   @ref ContactSolverSettings) then pushes solid shapes apart with impulses,
///   all the step's contacts together, in several passes.
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
  /// `true` for a fast dynamic body that must not pass through thin walls: a
  /// projectile, a water shot, a dashing player. `false` by default.
  ///
  /// A step only looks at where bodies are, not at the way between: a body
  /// that moves farther in one step than the wall is thick can jump over it
  /// without ever overlapping it (tunneling: at 60 Hz a shot at 50 m/s moves
  /// 0.83 m per step, more than a 0.35 m platform). For a bullet, each
  /// @ref PhysicsWorld::Step adds a continuous collision pass after the
  /// positions moved: the translation of each of its colliders from the
  /// start to the end of the step (with the rotation of the end) is swept
  /// against the other colliders, exactly for every pair of circles, boxes
  /// and oriented boxes. Bullets are swept in body index order.
  ///
  /// - **Solid colliders** of the bullet are swept against the solid
  ///   colliders of static and kinematic bodies, one-way ones included (they
  ///   block only a bullet that comes from their solid side, the rule of
  ///   @ref Collider::is_one_way). Only an obstacle that the bullet's core
  ///   meets stops it: its shape shrunk on every side by its smallest half
  ///   size (the center of a circle, the middle segment of a box), in the
  ///   spirit of Box2D v3.1's centroid test. A bullet sliding on a floor
  ///   rests a little into it (a few millimeters, a few centimeters just
  ///   after a landing) and touches the corner of the next tile at every
  ///   seam; its core passes above as long as its middle stays above the
  ///   floor's top, so it slides exactly like the same body without the
  ///   flag. The price: an obstacle that grazes the bullet by less than its
  ///   smallest half size is left to the next step's contacts, so a fast
  ///   bullet can clip the corner of a thin ledge by up to that much. At the
  ///   earliest time of impact (equal times: the lower
  ///   collider index) the body is moved back to where its whole shape
  ///   touches, then half of @ref PhysicsWorldConfig::penetration_slop into
  ///   the shape (at least 1/1024 m), so the next step's contact finds the
  ///   pair and makes it bounce or stop. The velocity is kept and the rest
  ///   of the step's motion is dropped (as in Box2D): at most one time of
  ///   impact per bullet per step.
  /// - **Trigger pairs** (the bullet's collider or the other one is a
  ///   trigger) are swept against every collider, dynamic ones too: every
  ///   collider the path crosses entirely up to the time of impact gets a
  ///   @ref ContactListener::OnTriggerEnter at the end of this step, in the
  ///   order of the crossings (equal times: the lower collider index), and
  ///   its exit at the next step. A water shot that flies through a player
  ///   in one step still hits it. A pair that still overlaps where the step
  ///   ends is left to the next step's narrowphase, so its enter comes at the
  ///   same step as for the body without the flag.
  ///
  /// The sweep follows the motion of both bodies over the step (the other
  /// body moves from where it was to where it is now), so a bullet also hits
  /// a kinematic platform or a player that moves (whether to sweep at all
  /// depends on the bullet's own motion, see below: a slow bullet hit by a
  /// fast platform is not swept). It ignores what the bullet
  /// already overlaps at the start of the step (the step's contacts handle
  /// it), and each collider of the bullet is skipped when the bullet moves
  /// less than a quarter of that collider's smallest size in the step: steps
  /// that short overlap anything on the way (Box2D's rule). A thin sensor on
  /// the body doesn't make its larger colliders swept. Solid bullets against
  /// solid dynamic bodies are not swept: two fast dynamic bodies can still pass
  /// through each other. The motion of another bullet is taken from its
  /// velocity, even when its own sweep stopped it earlier in the same step: a
  /// trigger bullet crossing a bullet that hit a wall in that step sees it a
  /// little off.
  ///
  /// Each bullet costs one scan of the colliders per step, with a bounding
  /// box test before each exact sweep: mark only the few bodies that need
  /// it. Hashed by @ref GenerateChecksum only when `true`.
  bool is_bullet = false;
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
/// A solid collider can also be one-way (`is_one_way`): a platform that
/// blocks shapes coming from one side only. `is_enabled` takes a collider
/// out of the simulation without removing it.
///
/// `category` and `mask` filter pairs before any of that: two colliders meet
/// only when each one's category is in the other's mask (see
/// @ref ShouldCollide). The defaults (category 1, every bit of the mask) let
/// everything meet.
struct Collider {
  /// Integer id for your game, 0 by default: typically the index of the
  /// entity that owns the collider in one of your game's tables.
  ///
  /// The physics never reads it and it is not part of the checksum, but
  /// @ref PhysicsWorld::CopyFrom copies it with the collider. That is why it
  /// is an integer and not a pointer: an index into a game-side table stays
  /// valid in a rollback snapshot (the game rolls its own tables back with
  /// the world), while a pointer would still point into the state it was
  /// taken from. Read it in a @ref ContactListener to find which entities
  /// touched.
  std::uint32_t user_id = 0;
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
  /// Direction of the solid side of a one-way collider, in the body's frame:
  /// (0, 1) by default, up while the body is not turned.
  ///
  /// It turns with the body like `offset`: the world direction is
  /// `rotation.Rotate(one_way_normal)`. Keep it of unit length. Only read
  /// when `is_one_way` is set.
  Vec2f one_way_normal{Scalar{0}, Scalar{1}};
  /// Opening of the cone of contact normals that a one-way collider blocks,
  /// centered on `one_way_normal`: pi (a half plane) by default.
  ///
  /// A shape that starts touching the collider from a direction within half
  /// this angle of the normal lands on it; from anywhere else it passes
  /// through. pi blocks everything that comes from the normal's side; a
  /// narrower arc (150 degrees, 5 pi / 6) also lets shapes that hit the
  /// collider at a grazing angle, such as the side of a tilted platform, pass
  /// through. The test is strict: a contact normal exactly at half the arc
  /// (a side hit on a half plane) passes. Only read when `is_one_way` is set.
  Radian one_way_arc{Pi<Scalar>()};
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
  /// `true` for a one-way collider: a platform that shapes can jump through
  /// from below and land on from above. `false` by default.
  ///
  /// A solid pair where one collider is one-way is decided when it starts
  /// touching. It is solid when the contact normal, from the one-way
  /// collider toward the other, is within half of `one_way_arc` of the
  /// world `one_way_normal`, and the other shape's velocity relative to the
  /// one-way collider (at the contact point) doesn't point along the normal:
  /// it comes from the solid side and isn't already leaving. Otherwise the
  /// pair is *passing*: it is not solved, gets no contact and sends no
  /// collision events, and it stays passing until the two shapes stop
  /// overlapping (see @ref PhysicsWorld::passing_pairs). When both colliders
  /// are one-way, both rules must say solid. Triggers ignore the flag.
  ///
  /// @see @ref PhysicsWorld::DropThrough to fall through a platform the body
  /// stands on.
  bool is_one_way = false;
  /// `false` takes the collider out of the simulation without removing it:
  /// `true` by default.
  ///
  /// A disabled collider is skipped by the broadphase, the narrowphase, the
  /// world queries and the inertia of its body; it keeps its slot, shape and
  /// settings. Its touching pairs end with exit events at the next
  /// @ref PhysicsWorld::Step. Use it for a platform that comes and goes, a
  /// door, a hitbox active on some frames of an attack. Hashed by
  /// @ref GenerateChecksum only when `false`.
  bool is_enabled = true;
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
/// `user_id` is left out: it is game data, not simulation state.
/// `category` and `mask` are hashed only when they differ from their
/// defaults, the one-way fields only when `is_one_way` is set and
/// `is_enabled` only when `false`, so worlds that use none of them keep their
/// checksums.
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
  /// `true` for a solid pair with a one-way collider
  /// (@ref Collider::is_one_way) when @ref PhysicsWorld::Teleport moved one
  /// of the bodies or @ref PhysicsWorld::RemoveCollider removed one of the
  /// colliders since the last step: the world no longer counts the pair as
  /// solid, and the one-way rule decides it again at the next step, as if it
  /// started touching. Events don't change. Hashed by @ref
  /// PhysicsWorld::GenerateChecksum only when `true`.
  bool decide_again = false;
};

/// How many items a capacity of @ref PhysicsWorldConfig turned away, read
/// with @ref PhysicsWorld::step_overflow.
///
/// The world never allocates during a step, so when a table is full the
/// extra items are dropped. Before, that was only an `assert` in debug
/// builds; a release build lost contacts or events silently. The world now
/// counts every item it drops, and keeps the first ones in its usual stable
/// order (broadphase order for pairs), so a full table still gives the same
/// result on every machine. A non-zero counter in a test or a debug overlay
/// says which capacity to raise.
///
/// @ref PhysicsWorld::Step resets the counters when it starts, then adds what
/// that step drops. Calls made between two steps that hit a limit
/// (@ref PhysicsWorld::AddDistanceJoint, @ref PhysicsWorld::AddRevoluteJoint,
/// @ref PhysicsWorld::DropThrough) are added to the next step's counters.
/// The counters are diagnostics: not copied by @ref PhysicsWorld::CopyFrom
/// nor hashed.
///
/// @code
/// world.Step();
/// if (world.step_overflow().IsAnyCounted()) {
///   // raise the capacities of PhysicsWorldConfig
/// }
/// @endcode
struct StepOverflow {
  /// Colliders the broadphase couldn't insert (only a custom
  /// @ref BoundingSurfaceHierarchy built for fewer colliders than the world
  /// can overflow).
  std::uint32_t broadphase_colliders = 0;
  /// Broadphase pairs dropped past
  /// @ref PhysicsWorldConfig::max_broadphase_pairs (or the capacity of a
  /// custom broadphase): these pairs are never tested.
  std::uint32_t broadphase_pairs = 0;
  /// Touching pairs left out of @ref PhysicsWorld::touching_pairs past
  /// @ref PhysicsWorldConfig::max_touching_pairs: no enter or exit events for
  /// them.
  std::uint32_t touching_pairs = 0;
  /// Solid contacts left unsolved past
  /// @ref PhysicsWorldConfig::max_touching_pairs: these shapes overlap
  /// without being pushed apart.
  std::uint32_t contacts = 0;
  /// Pairs that should have been passing past
  /// @ref PhysicsWorldConfig::max_passing_pairs.
  std::uint32_t passing_pairs = 0;
  /// @ref PhysicsWorld::AddDistanceJoint calls refused past
  /// @ref PhysicsWorldConfig::max_joints.
  std::uint32_t joints = 0;
  /// @ref PhysicsWorld::AddRevoluteJoint calls refused past
  /// @ref PhysicsWorldConfig::max_revolute_joints.
  std::uint32_t revolute_joints = 0;
  /// Trigger crossings of a bullet (see @ref Body::is_bullet) dropped past
  /// @ref PhysicsWorldConfig::max_bullet_trigger_hits: no enter event for
  /// them.
  std::uint32_t bullet_trigger_hits = 0;

  /// Returns whether any counter is non-zero.
  ///
  /// @return `true` when the step dropped at least one item.
  [[nodiscard]] constexpr bool IsAnyCounted() const noexcept {
    return broadphase_colliders != 0 || broadphase_pairs != 0 ||
           touching_pairs != 0 || contacts != 0 || passing_pairs != 0 ||
           joints != 0 || revolute_joints != 0 || bullet_trigger_hits != 0;
  }
  /// Compares every counter.
  constexpr bool operator==(const StepOverflow& rhs) const noexcept = default;
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
  /// Maximum number of touching pairs tracked for enter/exit events, and of
  /// solid contacts solved per step.
  ///
  /// Pairs past it are counted in @ref StepOverflow::touching_pairs (and
  /// @ref StepOverflow::contacts for the solid ones that aren't solved
  /// either); the first ones, in broadphase order, are kept.
  std::size_t max_touching_pairs = 1024;
  /// Maximum number of pairs whose bounding boxes overlap, for the default
  /// broadphase (@ref SweepAndPrune).
  ///
  /// Pairs past it are dropped and counted in
  /// @ref StepOverflow::broadphase_pairs.
  std::size_t max_broadphase_pairs = 2048;
  /// Maximum number of joints alive at once (see @ref DistanceJoint).
  ///
  /// A joint and its solver data take about 100 bytes; 0 for a world
  /// without joints. Refused additions are counted in
  /// @ref StepOverflow::joints.
  std::size_t max_joints = 64;
  /// Maximum number of revolute joints alive at once (see
  /// @ref RevoluteJoint), in their own table next to the distance joints.
  ///
  /// A revolute joint and its solver data take about 150 bytes with Fixed16;
  /// 0 for a world without hinges. Refused additions are counted in
  /// @ref StepOverflow::revolute_joints.
  std::size_t max_revolute_joints = 16;
  /// Maximum number of passing pairs (see @ref Collider::is_one_way), 4 bytes
  /// each.
  ///
  /// A pair that should become passing when the table is full is still not
  /// solved for that step, and is counted in
  /// @ref StepOverflow::passing_pairs; the next step decides it again as if
  /// it had just started touching.
  std::size_t max_passing_pairs = 64;
  /// Maximum number of colliders one bullet's trigger pairs can cross in a
  /// step (see @ref Body::is_bullet), 8 bytes each.
  ///
  /// Crossings past it, in collider order, get no enter event and are
  /// counted in @ref StepOverflow::bullet_trigger_hits.
  std::size_t max_bullet_trigger_hits = 16;
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
  /// @ref DistanceJoint::impulse (@ref RevoluteJoint::impulse and the limit
  /// impulses for a hinge). Both are simulation state:
  /// @ref PhysicsWorld::CopyFrom copies it and
  /// @ref PhysicsWorld::GenerateChecksum hashes it, so rollback replays stay
  /// exact.
  bool warm_starting = true;
  /// Frequency of the length correction of rigid and rope joints (and of the
  /// point and limit corrections of revolute joints), in Hz, see
  /// @ref ContactSolverSettings::joint_hertz. The default is 15.
  Scalar joint_hertz{15};
  /// Damping ratio of the joint corrections (distance and revolute), see
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
/// Each @ref Step runs four phases, plus a fifth when there are bullets:
///
/// - **Velocities**: the inertia of dynamic bodies is computed from their
///   colliders, springs add their forces, then dynamic bodies get gravity,
///   forces and torques.
/// - **Broadphase**: a fast, approximate pass that finds the pairs of
///   colliders whose bounding boxes overlap, to avoid testing every collider
///   against every other one.
/// - **Narrowphase**: the exact shape test of each candidate pair, the
///   solver for the solid pairs, the rigid and rope joints and the revolute
///   joints (all in the same passes, see @ref ContactSolverSettings), and
///   the enter/exit events.
/// - **Positions**: dynamic and kinematic bodies move and turn at their
///   (corrected) velocities, and kinematic bodies with a target (see
///   @ref SetKinematicTarget) are put exactly on it.
/// - **Continuous collision**: each bullet (@ref Body::is_bullet) is swept
///   from its start to its end position, stopped at its first time of
///   impact and given the trigger events of what it crossed.
///
/// See @ref Body for the integration formula, @ref ResolveBroadphase and
/// @ref ResolveNarrowphase for the two collision phases, and
/// @ref ContactSolverSettings for the contact and joint solver.
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
  /// Removes the body, all its colliders and all its joints (distance and
  /// revolute), freeing their slots.
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
  /// Removes every body, collider and shape and forgets touching and passing
  /// pairs and pending kinematic targets.
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
  /// @ref DistanceJoint::collide_connected). Solid pairs with a one-way
  /// collider are decided by the one-way rule (see @ref Collider::is_one_way)
  /// and the passing ones are left out. Called by @ref Step after
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
  /// Moves a distance joint to two other bodies, at runtime: a chain hooked
  /// onto whatever trapped a player, a rope cut from one body and tied to
  /// another.
  ///
  /// Keeps the joint's type, anchors (in the new bodies' frames), length and
  /// spring constants, and resets its accumulated impulse
  /// (@ref DistanceJoint::impulse): last step's pull belonged to the old
  /// bodies, and the next warm start would start from that wrong guess (the
  /// velocity passes then remove most of it). Set the anchors and the length
  /// afterwards if the new bodies need others: a rope that keeps the old
  /// length pulls the new body in at the next step, as hard as the soft step
  /// allows.
  ///
  /// The collision filter follows the joint (see
  /// @ref DistanceJoint::collide_connected): at the next @ref Step the old
  /// pair may collide again and gets its enter events, the new pair stops
  /// colliding and gets its exit events.
  ///
  /// @param index A valid distance joint index.
  /// @param body_a New first body.
  /// @param body_b New second body, different from `body_a`.
  void SetJointBodies(JointIndex index, BodyIndex body_a, BodyIndex body_b);

  /// Adds a revolute joint (a hinge) that pins two bodies together at a
  /// world point.
  ///
  /// Each body gets the point as an anchor in its own frame, so the anchors
  /// are on top of each other now and the joint keeps them there. The
  /// reference angle is the bodies' current relative angle (the joint angle
  /// is 0 now, see @ref RevoluteJointAngle). No limits, no collision
  /// between the two bodies: set @ref RevoluteJoint::enable_limit,
  /// @ref RevoluteJoint::lower_angle, @ref RevoluteJoint::upper_angle or
  /// @ref RevoluteJoint::collide_connected through @ref revolute_joint
  /// afterwards. Reuses the first free slot.
  ///
  /// @code
  /// // A seesaw: a plank turning about a static axle, 45 degrees each way
  /// const neko::RevoluteJointIndex hinge =
  ///     world.AddRevoluteJoint(axle, plank, world.body(axle).position);
  /// world.revolute_joint(hinge).enable_limit = true;
  /// world.revolute_joint(hinge).lower_angle =
  ///     neko::Radian{neko::Degree{neko::Scalar{-45}}};
  /// world.revolute_joint(hinge).upper_angle =
  ///     neko::Radian{neko::Degree{neko::Scalar{45}}};
  /// @endcode
  ///
  /// @param body_a First body.
  /// @param body_b Second body, different from `body_a`.
  /// @param world_anchor The pin, in world coordinates (meters).
  /// @return The new joint's index, or @ref kInvalidRevoluteJointIndex when
  /// `max_revolute_joints` revolute joints are in use.
  RevoluteJointIndex AddRevoluteJoint(BodyIndex body_a, BodyIndex body_b,
                                      Vec2f world_anchor);
  /// Removes a revolute joint, freeing its slot.
  ///
  /// @param index @ref RevoluteJoint to remove.
  void RemoveRevoluteJoint(RevoluteJointIndex index);
  /// Returns the revolute joint at `index`.
  ///
  /// @param index A valid revolute joint index.
  /// @return The joint, which can be edited in place.
  [[nodiscard]] RevoluteJoint& revolute_joint(RevoluteJointIndex index) {
    return revolute_joints_[index.index()];
  }
  /// Returns the revolute joint at `index` (read-only).
  ///
  /// @param index A valid revolute joint index.
  /// @return The joint.
  [[nodiscard]] const RevoluteJoint& revolute_joint(
      RevoluteJointIndex index) const {
    return revolute_joints_[index.index()];
  }
  /// Returns the world-space positions of a revolute joint's two anchors.
  ///
  /// They are on top of each other up to the joint's softness: their
  /// distance is how far the pin gives under load.
  ///
  /// @param index A valid revolute joint index.
  /// @return Anchor A then anchor B, in meters.
  [[nodiscard]] std::array<Vec2f, 2> RevoluteJointAnchors(
      RevoluteJointIndex index) const;
  /// Returns the joint angle of a revolute joint: body B's angle relative to
  /// body A, minus the reference angle.
  ///
  /// `WrapAngle(angle_b - angle_a - reference_angle)`, between -pi and pi,
  /// the angle the limits compare with @ref RevoluteJoint::lower_angle and
  /// @ref RevoluteJoint::upper_angle. Counterclockwise is positive (y-up).
  ///
  /// @param index A valid revolute joint index.
  /// @return The joint angle, in radians.
  [[nodiscard]] Radian RevoluteJointAngle(RevoluteJointIndex index) const;
  /// Moves a revolute joint to two other bodies, at runtime.
  ///
  /// Keeps the anchors (in the new bodies' frames), the reference angle,
  /// the limits and the flags, and resets the accumulated impulses
  /// (@ref RevoluteJoint::impulse and the limit impulses). The collision
  /// filter follows the joint, as for @ref SetJointBodies. Set the anchors
  /// and the reference angle afterwards if the new bodies aren't pinned at
  /// the same places.
  ///
  /// @param index A valid revolute joint index.
  /// @param body_a New first body.
  /// @param body_b New second body, different from `body_a`.
  void SetRevoluteJointBodies(RevoluteJointIndex index, BodyIndex body_a,
                              BodyIndex body_b);

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
  /// @note With @ref Fixed16 a box (@ref AabbCollider or
  /// @ref OrientedBoxCollider) is missed when the segment spans 16384 m or
  /// more along one of the box's axes: the slab test doubles that span, which
  /// overflows past 32767. A circle is found for any segment shorter than
  /// 32768 m. Below these limits only precision changes with the length.
  /// @ref RaycastHit::fraction is a multiple of 1/65536 rounded down, so a
  /// box's hit point, computed from it, can stop up to `length / 65536`
  /// before the surface: 1.5 mm for a 100 m segment, 15 mm for 1 km, 0.25 m
  /// near 16 km. A circle's hit point comes from the distance along the
  /// segment instead: exact along an axis, and on a diagonal off by an amount
  /// that grows with the circle's distance from `start` (measured 2.7 mm at
  /// 100 m, 23 mm at 1 km, 0.23 m at 10 km). Up to about 300 m both stay
  /// under a centimeter.
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
  /// Use it to poll the current contacts instead of tracking events. It
  /// also lists the trigger pairs a bullet crossed entirely during the last
  /// step (see @ref Body::is_bullet), which no longer overlap: the next step
  /// sends their exit events.
  ///
  /// @return The touching pairs, sorted by @ref ColliderPair.
  [[nodiscard]] const ArrayList<TouchingPair>& touching_pairs() const {
    return touching_pairs_;
  }
  /// Returns the passing pairs: solid pairs with a one-way collider (see
  /// @ref Collider::is_one_way) whose shapes overlap but are not solved.
  ///
  /// A pair enters the table when it starts touching from the open side of
  /// a one-way collider, or through @ref DropThrough, and leaves it at the
  /// first step its shapes no longer overlap. The table is simulation state:
  /// @ref CopyFrom copies it and @ref GenerateChecksum hashes it.
  ///
  /// @return The passing pairs, sorted by @ref ColliderPair.
  [[nodiscard]] const ArrayList<ColliderPair>& passing_pairs() const {
    return passing_pairs_;
  }
  /// Returns whether a pair is passing: overlapping a one-way collider
  /// without being solved.
  ///
  /// @param pair The two colliders, in any order.
  /// @return `true` when @p pair is in @ref passing_pairs.
  [[nodiscard]] bool IsPassing(ColliderPair pair) const;
  /// Lets a body fall through the one-way colliders it stands on: every
  /// one-way pair of its colliders that touches at the end of the last step
  /// becomes passing.
  ///
  /// Call it on the drop-through input (down plus jump in most platformers).
  /// At the next @ref Step these pairs are not solved and send a collision
  /// exit event; they stay passing, without any warm-start impulse, until
  /// the shapes stop overlapping. Then the body can land on the platform
  /// again. Pairs the body only reaches during the next step are not
  /// affected: they are decided by the usual one-way rule.
  ///
  /// @code
  /// if (input.down && input.jump_pressed) {
  ///   world.DropThrough(player);
  /// }
  /// @endcode
  ///
  /// @param index @ref Body that drops.
  /// @return The number of pairs that became passing. Pairs that didn't fit
  /// in @ref PhysicsWorldConfig::max_passing_pairs are not counted here but in
  /// the next step's @ref StepOverflow::passing_pairs.
  std::size_t DropThrough(BodyIndex index);
  /// Moves a kinematic body to a target position and angle over the next
  /// @ref Step: a moving platform, a lift, a door that follows a path.
  ///
  /// Instead of setting the position directly (a jump that contacts can't
  /// follow), this sets the velocities that reach the target in exactly one
  /// step: `velocity = (position - body.position) / dt` and
  /// `angular_velocity = WrapAngle(angle - body angle) / dt` (the short way
  /// around). The contact solver sees these velocities, so a crate on a lift
  /// is carried by friction and a box in the way is pushed, as by any
  /// kinematic body. At the end of that step the body is put exactly on the
  /// target: `velocity * dt` is rounded (to 1/65536 m with @ref Fixed16), and
  /// without this snap a platform driven every step would drift away from
  /// its path.
  ///
  /// Call it every step with the next point of the path, before
  /// @ref Step. The velocity stays after the step: a platform that gets no
  /// new target keeps moving at it (set `velocity` to zero to stop it). The
  /// pending target is simulation state: @ref CopyFrom copies it and
  /// @ref GenerateChecksum hashes it. Calling it twice before a step keeps
  /// the last target.
  ///
  /// @code
  /// // A lift going up and down 2 m around y = 3, one cycle every 4 s
  /// const neko::Scalar phase = time * (two_pi / neko::Scalar{4});
  /// world.SetKinematicTarget(
  ///     lift, {x, neko::Scalar{3} + neko::Scalar{2} * neko::Sin(phase)},
  ///     neko::Radian{});
  /// world.Step();
  /// @endcode
  ///
  /// @note With @ref Fixed16 the velocity is `distance / dt`: keep the move
  /// per step under about 500 m (32767 m/s at 60 Hz).
  /// @param index A kinematic @ref Body; does nothing for other body types.
  /// @param position Where the body's position must be after the next step,
  /// in meters.
  /// @param angle The body's angle after the next step, in radians.
  /// @see @ref Teleport to move a body without crossing the space between.
  void SetKinematicTarget(BodyIndex index, Vec2f position, Radian angle);
  /// Moves a body to a new position and angle at once, without crossing the
  /// space between: a respawn, a portal, a platform that wraps around the
  /// screen.
  ///
  /// The velocity and angular velocity are kept (set them yourself if the
  /// body must stop or turn around). Everything the world remembers about
  /// where the body was is dropped, since it would be wrong at the new
  /// place:
  /// - its cached contact impulses, so the next step doesn't warm start from
  ///   a contact of the old place (see @ref PhysicsWorldConfig::warm_starting);
  /// - its passing pairs (see @ref passing_pairs);
  /// - its pending @ref SetKinematicTarget (call that after the teleport);
  /// - the accumulated impulses of its joints (@ref DistanceJoint::impulse,
  ///   and @ref RevoluteJoint::impulse with the limit impulses).
  ///
  /// The touching pairs stay until the next @ref Step, which finds the pairs
  /// at the new place: pairs of the old place get their exit events, pairs
  /// of the new place their enter events, as for any move. World queries
  /// made right after the call already see the body at its new place.
  ///
  /// @param index @ref Body to move, of any type.
  /// @param position The new position, in meters.
  /// @param angle The new angle, in radians (wrapped to one turn).
  void Teleport(BodyIndex index, Vec2f position, Radian angle);
  /// Returns what the last @ref Step dropped because a table was full.
  ///
  /// @return The counters, all zero when nothing overflowed.
  [[nodiscard]] const StepOverflow& step_overflow() const {
    return step_overflow_;
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
  /// colliders, shapes, joints of both kinds, touching pairs (so enter/exit
  /// events stay consistent after a rollback), the contact cache, the passing
  /// pairs and the pending kinematic targets, not the listener, the BSH nor the
  /// @ref StepOverflow counters.
  ///
  /// @param physics_world World to copy from.
  /// @note Doesn't allocate when both worlds were built with the same
  /// capacities.
  void CopyFrom(const PhysicsWorld& physics_world);
  /// Returns a hash of the whole simulation state.
  ///
  /// Peers of a networked game exchange checksums of the same frame: a
  /// mismatch means their simulations diverged (a desync). Covers bodies,
  /// colliders, shapes, touching pairs, the contact cache, joints of both
  /// kinds, passing pairs and pending kinematic targets, but not
  /// @ref Collider::user_id nor @ref step_overflow.
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
  // Whether a solid pair with a one-way collider is solved this step: it was
  // solid last step, or it starts touching from the solid side. Records the
  // pair in new_passing_pairs_ when it isn't.
  [[nodiscard]] bool IsOneWaySolid(ColliderPair pair, const Contact& contact);
  // The continuous collision pass of the bullets (Body::is_bullet), after
  // the positions moved
  void SolveBullets();
  // Copies the cached impulses into this step's contacts, and back
  void WarmStartContacts();
  void CacheContactImpulses();
  // Adds the Hooke and damping forces of the springs to their bodies
  void ApplySpringForces();
  // Marks the bodies of joints (distance and revolute) that filter their
  // collisions, returns whether there is any
  [[nodiscard]] bool UpdateJointFilter();
  // Whether a joint without collide_connected links the two bodies
  [[nodiscard]] bool AreJointFiltered(BodyIndex body1, BodyIndex body2) const;
  // Whether the index names a body in the table (not a free slot)
  [[nodiscard]] bool IsLiveBody(BodyIndex index) const;
  // Whether a touching pair is solid with a one-way collider (the pairs the
  // one-way rule decides)
  [[nodiscard]] bool IsOneWayPair(const TouchingPair& touching) const;
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
  // Passing pairs of one-way colliders, sorted by pair (simulation state,
  // copied and hashed), and the next step's, built by the narrowphase
  ArrayList<ColliderPair> passing_pairs_;
  ArrayList<ColliderPair> new_passing_pairs_;
  // Targets of SetKinematicTarget for the next step, sorted by body, at most
  // one per body (simulation state, copied and hashed); emptied by Step
  struct KinematicTarget {
    BodyIndex body{};
    Vec2f position{};
    Radian angle{};
  };
  ArrayList<KinematicTarget> kinematic_targets_;
  // Scratch for SolveBullets: the trigger crossings of one bullet, sorted by
  // time before their enter events
  struct BulletTriggerHit {
    Scalar fraction{};
    ColliderIndex bullet{};
    ColliderIndex other{};
  };
  ArrayList<BulletTriggerHit> bullet_trigger_hits_;
  // What the last step dropped, and what calls since then dropped (added to
  // the next step's counters)
  StepOverflow step_overflow_{};
  StepOverflow pending_overflow_{};
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
  // Revolute joints, in their own table (simulation state, copied and
  // hashed), and their solver working memory
  ArrayList<RevoluteJoint> revolute_joints_;
  ArrayList<internal::RevoluteJointScratch> revolute_scratch_;
  // Scratch, one entry per body: 1 when it belongs to a joint that filters
  // its collisions
  ArrayList<std::uint8_t> body_joint_filter_;

  ContactListener* contact_listener_ = nullptr;
  BoundingSurfaceHierarchy* bsh_ = nullptr;
};

}  // namespace neko

#endif  // NEKO_PHYSICS_PHYSICS_H_
