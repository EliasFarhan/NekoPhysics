#ifndef NEKO_PHYSICS_JOINT_H_
#define NEKO_PHYSICS_JOINT_H_

#include <cstdint>
#include <span>

#include "neko/math/vec2.h"
#include "neko/physics/contact_solver.h"
#include "neko/physics/physics_type.h"

namespace neko {

/// How a @ref DistanceJoint holds its two anchors at its length.
enum class DistanceJointType : std::uint8_t {
  /// A massless rod: the distance stays at `length`, the joint pushes and
  /// pulls. An equality constraint, `C = distance - length = 0`.
  kRigid,
  /// A rope: the distance stays at most `length`, the joint only pulls. An
  /// inequality constraint, `C <= 0`: while the rope is slack the bodies move
  /// freely.
  kRope,
  /// A spring: a Hooke force `stiffness * (distance - length)` plus a damping
  /// force, added to the bodies' forces before each step. Not a constraint:
  /// the distance oscillates around `length`.
  kSpring,
  /// No joint: marks a free slot in the world's joint table.
  kNone
};

/// Links an anchor point of one @ref Body to an anchor point of another and
/// keeps them at a distance: a rod, a rope or a spring (see
/// @ref DistanceJointType).
///
/// Get one with @ref PhysicsWorld::AddDistanceJoint and edit it in place
/// through @ref PhysicsWorld::joint. The anchors are given in each body's
/// frame (like @ref Collider::offset) and turn with it, so a joint anchored
/// off-center also turns its bodies. A zero anchor is the body's position,
/// its center of mass: a joint between two centers never makes the bodies
/// spin, which turns a body into the point mass of a textbook pendulum.
///
/// **Rigid and rope joints** are constraints, solved with impulses by the
/// same passes as the contacts (see @ref SolveConstraints). With `u` the unit
/// vector from anchor A to anchor B, `rA` and `rB` the anchors' offsets from
/// the bodies' centers in the world, the solver wants the speed at which the
/// anchors separate along `u`,
///
/// @code
/// Cdot = Dot(vB + wB x rB - vA - wA x rA, u)
/// @endcode
///
/// to be 0, plus a correction that pulls the length error `C = distance -
/// length` back. Each pass applies `-lambda * u` at anchor A and
/// `+lambda * u` at anchor B, with the effective mass along `u`:
///
/// @code
/// k = 1/mA + 1/mB + Cross(rA, u)^2 / IA + Cross(rB, u)^2 / IB
/// lambda = mass_scale * (-bias_rate * C - Cdot) / k
///          - impulse_scale * accumulated_impulse
/// @endcode
///
/// This is the soft step: the correction behaves like a stiff, heavily
/// damped spring of frequency `f` (@ref PhysicsWorldConfig::joint_hertz,
/// 15 Hz) and damping ratio `zeta` (2), solved implicitly. With `w = 2 pi f`
/// and the time step `h`:
///
/// @code
/// a1 = 2 zeta + h w;   a2 = h w a1;   a3 = 1 / (1 + a2)
/// bias_rate = w / a1;  mass_scale = a2 a3;  impulse_scale = a3
/// @endcode
///
/// Without the leak `impulse_scale`, the warm start would apply last step's
/// correction again on top of this step's, and a chain solved with few passes
/// would gain energy every step and blow up. The cost is a little softness:
/// at rest the joint acts as a spring of stiffness `w^2 / k` (with `k` the
/// inverse effective mass above), so a load `F` stretches it by `F k / w^2`.
/// A 1 kg bob hanging from a static pivot: `9.81 / (2 pi 15)^2`, 1.1 mm.
///
/// A rope keeps its accumulated impulse at or below 0 (it pulls, never
/// pushes), and while slack its target lets the anchors approach up to taut
/// within the step (`-C / dt`, rigidly), so it catches a falling body in the
/// step it would overstretch.
///
/// The length also drifts outward on a fast swing: each step moves the body
/// along the tangent, `v dt`, which takes it `(v dt)^2 / (2 length)` off the
/// circle before the correction pulls it back.
///
/// **Springs** are explicit forces: before each step, with `x = distance -
/// length` and the separation speed `Cdot` above,
///
/// @code
/// F = stiffness * x + damping * Cdot
/// // pulls A toward B and B toward A by F (pushes them apart when F < 0)
/// @endcode
///
/// A mass `m` on a spring of stiffness `k` oscillates with the period `2 pi
/// sqrt(m / k)`, and the damping makes the amplitude decay as `exp(-damping
/// / (2 m) t)`. Semi-implicit Euler keeps the spring's energy bounded only
/// while `sqrt(k / m) dt < 2`: a stiffer spring (or a lighter body) explodes.
/// Past that, use a rigid joint.
///
/// Bodies linked by a joint don't collide with each other unless
/// `collide_connected` is set. Static and kinematic bodies have infinite
/// mass in joints, so a joint to a static body is a fixed pivot.
///
/// @note A joint is simulation state: @ref PhysicsWorld::CopyFrom copies the
/// joint table and @ref PhysicsWorld::GenerateChecksum hashes it, including
/// the accumulated `impulse` that warm starts the next step.
struct DistanceJoint {
  /// First body, A.
  BodyIndex body_a{};
  /// Second body, B.
  BodyIndex body_b{};
  /// Anchor on body A, in A's frame, in meters from its position.
  Vec2f local_anchor_a{};
  /// Anchor on body B, in B's frame, in meters from its position.
  Vec2f local_anchor_b{};
  /// Rest length in meters: the distance a rigid joint keeps, the maximum of
  /// a rope, the length at which a spring pulls with no force.
  ///
  /// @ref PhysicsWorld::AddDistanceJoint sets it to the distance between the
  /// two bodies' positions.
  Scalar length{};
  /// Spring stiffness `k` in N/m: the pull per meter of stretch. Springs
  /// only.
  Scalar stiffness{};
  /// Spring damping `c` in N s/m: the force per m/s of separation speed.
  /// Springs only.
  Scalar damping{};
  /// Accumulated impulse of the last step along `u` (from A toward B), in N
  /// s: negative when the joint pulls the anchors together, positive when a
  /// rigid joint pushes them apart.
  ///
  /// Divided by the time step it is the joint's force: a 1 kg pendulum bob
  /// hanging at rest pulls with about `-9.81 * dt`. It warm starts the next
  /// step (see @ref PhysicsWorldConfig::warm_starting). Always 0 for a
  /// spring.
  Scalar impulse{};
  /// Rod, rope or spring; @ref DistanceJointType::kNone marks a free slot.
  DistanceJointType type = DistanceJointType::kNone;
  /// Whether the colliders of body A and body B still collide with each
  /// other. `false` by default: the links of a chain overlap at their ends.
  ///
  /// Only solid pairs are filtered; triggers still report overlaps.
  bool collide_connected = false;
};

/// Returns a hash of every field of `joint`.
///
/// Building block of @ref PhysicsWorld::GenerateChecksum.
///
/// @param joint @ref DistanceJoint to hash.
/// @return FNV-1a hash of the joint's fields.
[[nodiscard]] std::uint32_t GenerateChecksum(const DistanceJoint& joint);

namespace internal {
// Per-joint data computed once per step by SolveConstraints and reused by
// every pass. body_a is null for a joint the solver skips (a spring, a free
// slot, an inactive body, or two bodies that can't move).
struct JointScratch {
  Body* body_a = nullptr;
  Body* body_b = nullptr;
  Scalar inverse_mass_a{};
  Scalar inverse_mass_b{};
  Scalar inverse_inertia_a{};
  Scalar inverse_inertia_b{};
  Vec2f r_a{};
  Vec2f r_b{};
  // Unit vector from anchor A to anchor B
  Vec2f axis{};
  Scalar mass{};
  Scalar target_velocity{};
  // Soft step coefficients (1 and 0 for a slack rope, solved rigidly)
  Scalar mass_scale{1};
  Scalar impulse_scale{};
  bool is_rope = false;
};
}  // namespace internal

/// Solves the contacts and the rigid and rope joints of a step together, in
/// the same passes.
///
/// Each pass solves every rigid and rope joint in table order (see
/// @ref DistanceJoint for the math), then every contact as
/// @ref SolveContacts does: the contacts come last, so a pass ends with no
/// body pushed into another. Solving them together is what lets a chain
/// resting on the ground settle as one system: each pass sees the others'
/// impulses. Before the passes the contact targets are computed from the
/// velocities before any impulse, then the joints' and the contacts'
/// impulses of the last step are applied (warm starting).
///
/// Without joints this is exactly @ref SolveContacts, bit for bit.
///
/// @param contacts The step's solid contacts, see @ref SolveContacts.
/// @param contact_scratch Working memory, two entries per contact.
/// @param joints The joint table; springs and free slots are skipped. Each
/// joint's `impulse` holds the warm start, and the step's total on return.
/// @param bodies The body table the joints' indices refer to.
/// @param joint_scratch Working memory, one entry per joint.
/// @param settings Time step, passes and tolerances; the joints use
/// `joint_hertz`, `joint_damping_ratio` and `warm_starting`.
/// @pre `contact_scratch.size() >= 2 * contacts.size()` and
/// `joint_scratch.size() >= joints.size()`.
void SolveConstraints(std::span<Contact> contacts,
                      std::span<internal::ContactPointScratch> contact_scratch,
                      std::span<DistanceJoint> joints, std::span<Body> bodies,
                      std::span<internal::JointScratch> joint_scratch,
                      const ContactSolverSettings& settings);

}  // namespace neko

#endif  // NEKO_PHYSICS_JOINT_H_
