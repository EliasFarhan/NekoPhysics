#ifndef NEKO_PHYSICS_CONTACT_SOLVER_H_
#define NEKO_PHYSICS_CONTACT_SOLVER_H_
#include <array>
#include <cstdint>
#include <span>

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

/// One point of a contact manifold: where the shapes touch, how deep, and the
/// impulse the solver pushed there.
struct ContactPoint {
  /// World-space position of the point, in meters.
  Vec2f position{};
  /// Overlap depth at this point along @ref Contact::contact_normal, in
  /// meters: 0 when the shapes just touch there.
  Scalar penetration{};
  /// Total impulse pushed along the normal at this point during the step, in
  /// N s; never negative (contacts push, they don't pull).
  ///
  /// Divided by the time step it is the contact force: a 1 kg box resting on
  /// two points carries about `9.81 * dt / 2` at each.
  Scalar normal_impulse{};
  /// Total friction impulse along the tangent at this point during the step,
  /// in N s.
  ///
  /// The tangent is the normal turned a quarter turn clockwise,
  /// `(n.y, -n.x)`. Its size never exceeds `Contact::friction` times
  /// `normal_impulse` (Coulomb's law).
  Scalar tangent_impulse{};
  /// Feature id: which faces and corner of the two shapes made this point.
  ///
  /// The same pair of shapes touching the same way gives the same id from
  /// one step to the next, which is how the world recognizes a point to warm
  /// start it with last step's impulses (see
  /// @ref PhysicsWorldConfig::warm_starting). Circles use 0.
  std::uint8_t id = 0;
};

/// Contact between two solid colliders, found by the narrowphase and solved
/// with impulses.
///
/// @ref PhysicsWorld::DetectContact fills it in when two colliders overlap;
/// @ref SolveContacts then fixes the velocities of all the step's contacts
/// together. A `Contact` lives only for the step that created it: games
/// normally never build one, they receive @ref ContactListener events
/// instead.
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
  /// where the shapes touch, each with its own penetration. The solver pushes
  /// at each of them.
  std::array<ContactPoint, 2> points{};
  /// Number of valid entries in `points`: 1 or 2 for a solid contact, 0 for a
  /// trigger pair (only the overlap is computed).
  std::uint8_t point_count = 0;
  /// Bounciness of the contact, the larger @ref Collider::restitution of the
  /// two colliders.
  ///
  /// 0 stops the bodies along the normal (no bounce), 1 is a perfectly elastic
  /// bounce that keeps the speed along the normal. Slow impacts don't bounce
  /// (see @ref ContactSolverSettings::restitution_threshold).
  Scalar restitution{1};
  /// Overlap depth along `contact_normal`, in world units: the deepest point's.
  Scalar penetration{};
  /// Friction coefficient of the contact, `sqrt(friction1 * friction2)` of
  /// the two colliders (see @ref Collider::friction).
  Scalar friction{};

  /// Returns the relative velocity of the two bodies' centers along the
  /// normal.
  ///
  /// Computed as the dot product of `bodies[0].velocity - bodies[1].velocity`
  /// with `contact_normal`. Negative when the bodies are moving towards each
  /// other, positive when they are already separating. The solver uses the
  /// velocity at each point instead, which includes the spin.
  ///
  /// @pre Both `bodies[i].body` pointers are valid.
  /// @return The separating velocity, in world units per second.
  [[nodiscard]] Scalar CalculateSeparateVelocity() const;
};

/// Settings of @ref SolveContacts, taken from the @ref PhysicsWorldConfig.
struct ContactSolverSettings {
  /// Time step, in seconds: turns a penetration into a correction speed.
  Scalar dt{1.0f / 60.0f};
  /// Number of passes over all the contacts.
  ///
  /// Each pass corrects every point a little, and the corrections disturb
  /// each other (a box's second point undoes part of its first point's
  /// push), so the solver repeats them: more passes, more accurate stacks,
  /// more CPU time. 1 is the single pass of a one-shot solver.
  int velocity_iterations = 8;
  /// Fraction of the penetration (beyond the slop) removed per step, the
  /// Baumgarte factor, from 0 to 1.
  ///
  /// Overlapping shapes are pushed apart by a target separating speed of
  /// `baumgarte / dt * (penetration - penetration_slop)` instead of being
  /// moved: 0.2 removes about a fifth of the overlap each step. Larger values
  /// correct faster but make resting objects pop and jitter.
  Scalar baumgarte{0.2f};
  /// Overlap left alone, in meters.
  ///
  /// Shapes resting on each other keep overlapping by about this much, so
  /// the contact stays alive from one step to the next instead of flickering
  /// between touching and separated.
  Scalar penetration_slop{0.005f};
  /// Slowest approach speed that bounces, in m/s.
  ///
  /// Below it a contact doesn't bounce whatever its restitution: without it,
  /// a resting body would keep bouncing on the tiny speeds that gravity adds
  /// every step.
  Scalar restitution_threshold{1};
  /// Whether to start from the points' `normal_impulse` and
  /// `tangent_impulse` instead of zero (warm starting).
  ///
  /// The impulses that held a pile last step are almost the ones that hold it
  /// now: applying them first leaves the passes only the small difference to
  /// find, so a stack settles with fewer passes. The world fills them in from
  /// its contact cache; a contact built by hand has zero impulses either way.
  bool warm_starting = true;
  /// Frequency of the length correction of rigid and rope joints, in Hz; 0
  /// for none (the length then drifts).
  ///
  /// A joint pulls its length error back like a spring of this frequency
  /// with `joint_damping_ratio`, solved implicitly (the soft step): stable
  /// at any number of passes and with warm starting, where a Baumgarte
  /// correction would feed last step's push back every step. Keep it at
  /// most a quarter of the step rate (15 Hz at 60 steps per second).
  Scalar joint_hertz{15};
  /// Damping ratio of the length correction of rigid and rope joints: 1 is
  /// critical damping (the fastest return without overshoot), more returns
  /// slower but never rings.
  Scalar joint_damping_ratio{2};
};

namespace internal {
// Per-point data computed once per step by SolveContacts and reused by every
// pass
struct ContactPointScratch {
  // The contact's inverse masses and inertias (0 for bodies that don't
  // move or turn), the same for both points, read once instead of through the
  // body and collider pointers in every pass
  Scalar inverse_mass1{};
  Scalar inverse_mass2{};
  Scalar inverse_inertia1{};
  Scalar inverse_inertia2{};
  Vec2f r1{};
  Vec2f r2{};
  Scalar normal_mass{};
  Scalar tangent_mass{};
  Scalar target_velocity{};
};

// The three parts of SolveContacts, so SolveConstraints can interleave the
// joints with them. Same arithmetic in the same order: a step without joints
// is bit-identical to SolveContacts.
//
// Once per step: lever arms, effective masses and target speeds
void PrepareContacts(std::span<Contact> contacts,
                     std::span<ContactPointScratch> scratch,
                     const ContactSolverSettings& settings);
// Applies the impulses already in the points (last step's) once
void WarmStartContacts(std::span<Contact> contacts,
                       std::span<const ContactPointScratch> scratch);
// One pass over every point: friction, then the normal
void SolveContactPass(std::span<Contact> contacts,
                      std::span<const ContactPointScratch> scratch);
}  // namespace internal

/// Solves all the contacts of a step together with accumulated impulses,
/// along the normals (contacts push) and the tangents (friction).
///
/// For each point, the solver wants the bodies' velocities at the point (each
/// `v + w x r`) to separate at a target speed along the normal: the bounce
/// `-restitution * vn` for an impact faster than the threshold, or the
/// Baumgarte push-out of the penetration, whichever is larger (0 when the
/// shapes just touch). It then runs `velocity_iterations` passes over every
/// point of every contact, in order:
///
/// @code
/// vn = Dot(v1 + w1 x r1 - v2 - w2 x r2, n)       // current approach speed
/// k = 1/m1 + 1/m2 + Cross(r1, n)^2 / I1 + Cross(r2, n)^2 / I2
/// dj = (target - vn) / k                         // impulse to reach target
/// total = max(normal_impulse + dj, 0)            // contacts never pull
/// dj = total - normal_impulse;  normal_impulse = total
/// apply dj * n at the point to body 1, -dj * n to body 2
/// @endcode
///
/// Clamping the accumulated impulse, not each pass's increment, lets a later
/// pass take back part of an earlier push (when a neighbor already did the
/// work) without ever pulling the bodies together. Repeating a one-shot
/// impulse instead would add a new bounce every pass. The passes converge
/// toward impulses that satisfy every contact at once, which is what lets a
/// box rest flat on two points and a pile settle.
///
/// Friction works the same way along the tangent `t = (n.y, -n.x)`, before
/// the normal in each pass: the target is no sliding (tangent speed 0), and
/// the accumulated friction impulse is clamped to
/// `[-friction * normal_impulse, friction * normal_impulse]`. Inside the
/// range the surfaces stick (static friction); at the limit they slide and
/// friction resists with its maximum, `friction` times the push (kinetic
/// friction). The friction impulse acts at the point, so it also spins the
/// bodies: a sliding ball starts to roll.
///
/// Only @ref BodyType::kDynamic bodies move; static and kinematic bodies have
/// infinite mass and inertia. A body whose collider in the contact is an
/// @ref AabbCollider doesn't spin (its shape can't turn): its inertia is
/// infinite in that contact.
///
/// @param contacts The step's solid contacts, filled by
/// @ref PhysicsWorld::DetectContact. Their impulses are the warm start (or
/// are reset to 0 without warm starting) and hold the step's totals when it
/// returns.
/// @param scratch Working memory, two entries per contact.
/// @param settings Time step, passes and tolerances.
/// @pre `scratch.size() >= 2 * contacts.size()`, and every contact's body and
/// collider pointers are valid.
void SolveContacts(std::span<Contact> contacts,
                   std::span<internal::ContactPointScratch> scratch,
                   const ContactSolverSettings& settings);
}  // namespace neko

#endif  // NEKO_PHYSICS_CONTACT_SOLVER_H_
