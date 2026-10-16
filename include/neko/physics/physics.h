#ifndef NEKO_PHYSICS_PHYSICS_H_
#define NEKO_PHYSICS_PHYSICS_H_

#include <cstddef>
#include <cstdint>

#include "neko/bsh/bsh.h"
#include "neko/bsh/sweep_and_prune.h"
#include "neko/core/allocator.h"
#include "neko/math/rotation.h"
#include "neko/math/shape.h"
#include "neko/math/vec2.h"
#include "neko/physics/contact_listener.h"
#include "neko/physics/contact_solver.h"
#include "neko/physics/physics_type.h"

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
/// - Shapes are @ref CircleCollider (a radius) and @ref AabbCollider (an
///   axis-aligned box, by half size). Collider offsets turn with the body,
///   but contacts don't use the rotation yet: a box stays axis-aligned (see
///   @ref AabbCollider).
/// - @ref ContactListener is the interface your game implements to receive
///   trigger and collision enter/exit events.
/// - Collision detection runs in two phases. The broadphase, a
///   @ref BoundingSurfaceHierarchy (@ref SweepAndPrune by default, or a
///   @ref QuadTree), cheaply finds pairs whose bounding boxes overlap. The
///   narrowphase (@ref PhysicsWorld::DetectContact) tests the exact shapes of
///   those pairs only and builds a @ref Contact. @ref Contact::Resolve then
///   pushes solid shapes apart with an impulse.
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
/// - **No tolerances yet.** There is no penetration slop and no restitution
///   threshold: a contact removes its whole penetration in the step that
///   finds it, and bounces at any speed.
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
/// Euler, the same way for motion and for spin:
///
/// @code
/// velocity += (gravity + force * inverse_mass) * dt;
/// position += velocity * dt;
/// angular_velocity += torque * inverse_inertia * dt;
/// angle += angular_velocity * dt;  // then wrapped to one turn
/// force = 0;
/// torque = 0;
/// @endcode
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
  /// Add to it to push a dynamic body (`body.force += push`): it is scaled by
  /// `inverse_mass`, unlike gravity, which accelerates every body equally.
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
/// Created with @ref PhysicsWorld::AddCircleCollider or
/// @ref PhysicsWorld::AddAabbCollider, which fill in `body_index`,
/// `collider_index`, `shape_index` and `type`; you then set the other fields
/// and the shape size (@ref PhysicsWorld::circle or @ref PhysicsWorld::aabb
/// with `shape_index`).
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
  /// @ref Body the collider is attached to.
  BodyIndex body_index{};
  /// Index of this collider (its own slot in the world).
  ColliderIndex collider_index{};
  /// Slot of the shape in the circle or box table, depending on `type`.
  ShapeIndex shape_index{};
  /// Kind of shape; @ref ShapeType::kNone marks a free slot.
  ShapeType type = ShapeType::kNone;
  /// `true` for a trigger (detects overlaps only), `false` for a solid
  /// collider. Colliders are triggers by default.
  bool is_trigger = true;
};
/// Returns a hash of the simulation fields of `collider`.
///
/// `user_data` is left out: it is game data, not simulation state.
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
/// (a box's inertia about its center doesn't depend on its angle). Give
/// rotating bodies circles, or set @ref InertiaMode::kFixedRotation on
/// bodies whose boxes must line up with their contacts.
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
/// Each @ref Step runs three phases:
///
/// - **Integration**: the inertia of dynamic bodies is computed from their
///   colliders, then dynamic bodies get gravity, forces and torques, and
///   kinematic bodies move and turn at their velocities.
/// - **Broadphase**: a fast, approximate pass that finds the pairs of
///   colliders whose bounding boxes overlap, to avoid testing every collider
///   against every other one.
/// - **Narrowphase**: the exact shape test of each candidate pair, contact
///   resolution for solid pairs and the enter/exit events.
///
/// See @ref Body for the integration formula, @ref ResolveBroadphase and
/// @ref ResolveNarrowphase for the two collision phases, and
/// @ref Contact::Resolve for contact resolution.
///
/// Bodies, colliders and shapes live in fixed-size tables and are referred to
/// by index (@ref BodyIndex, @ref ColliderIndex, @ref ShapeIndex); freed slots
/// are reused by later additions. For rollback netcode, keep snapshot worlds
/// built with the same config and use @ref CopyFrom to save and restore.
///
/// @note References returned by @ref body, @ref collider, @ref circle and
/// @ref aabb point into the world's tables: don't keep them across a
/// @ref CopyFrom or a move of the world; keep indices instead.
class PhysicsWorld {
 public:
  /// Builds an empty world and allocates all its tables.
  ///
  /// @param config Gravity, time step, capacities and allocator.
  /// @pre `config.max_bodies` and `config.max_colliders` fit in an
  /// @ref Index.
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
  /// Removes the body and all its colliders, freeing their slots.
  ///
  /// @param index @ref Body to remove.
  void RemoveBody(BodyIndex index);
  /// Narrowphase test of two colliders: checks whether their exact shapes
  /// overlap and fills in `contact`.
  ///
  /// Handles circle-circle, box-box and circle-box. The contact normal,
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
  /// Skips pairs where a body is inactive, both bodies are static, or both
  /// colliders belong to the same body. Called by @ref Step after
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
  /// Removes a collider and its shape, freeing their slots.
  ///
  /// @param index @ref Collider to remove.
  void RemoveCollider(ColliderIndex index);

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
  /// colliders, shapes and touching pairs (so enter/exit events stay
  /// consistent after a rollback), not the listener nor the BSH.
  ///
  /// @param physics_world World to copy from.
  /// @note Doesn't allocate when both worlds were built with the same
  /// capacities.
  void CopyFrom(const PhysicsWorld& physics_world);
  /// Returns a hash of the whole simulation state.
  ///
  /// Peers of a networked game exchange checksums of the same frame: a
  /// mismatch means their simulations diverged (a desync). Covers bodies,
  /// colliders, shapes and touching pairs, but not @ref Collider::user_data.
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

  PhysicsWorldConfig config_;
  SweepAndPrune default_broadphase_;
  ArrayList<Body> bodies_;
  ArrayList<AabbCollider> aabbs_;
  ArrayList<CircleCollider> circles_;
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

  ContactListener* contact_listener_ = nullptr;
  BoundingSurfaceHierarchy* bsh_ = nullptr;
};

}  // namespace neko

#endif  // NEKO_PHYSICS_PHYSICS_H_
