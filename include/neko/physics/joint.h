#ifndef NEKO_PHYSICS_JOINT_H_
#define NEKO_PHYSICS_JOINT_H_

#include <cstdint>
#include <span>

#include "neko/math/angle.h"
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
/// same passes as the contacts (see @ref ContactSolverSettings). With `u` the
/// unit vector from anchor A to anchor B, `rA` and `rB` the anchors' offsets
/// from the bodies' centers in the world, the solver wants the speed at which
/// the anchors separate along `u`,
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

/// Pins an anchor point of one @ref Body to an anchor point of another, so
/// that the bodies can only turn about it: a hinge, the axle of a seesaw, a
/// door's pivot, the links of a jointed bridge.
///
/// Get one with @ref PhysicsWorld::AddRevoluteJoint and edit it in place
/// through @ref PhysicsWorld::revolute_joint. Revolute joints live in their
/// own table, next to the distance joints (@ref DistanceJoint), with their
/// own index type (@ref RevoluteJointIndex).
///
/// **The point constraint.** Where a distance joint holds one number, the
/// distance, a revolute joint holds a vector: the two anchors must stay on
/// top of each other, `C = (pB + rB) - (pA + rA) = 0` in x and in y. The
/// solver wants the anchors' relative velocity
///
/// @code
/// Cdot = vB + wB x rB - vA - wA x rA
/// @endcode
///
/// to be 0 (plus the soft correction of `C`), and applies `-P` at anchor A
/// and `+P` at anchor B, a 2D impulse found by solving a 2x2 system with the
/// effective mass matrix (the same `K` as Box2D's revolute joint):
///
/// @code
/// K = | mA + mB + iA rA.y^2 + iB rB.y^2    -iA rA.x rA.y - iB rB.x rB.y |
///     | -iA rA.x rA.y - iB rB.x rB.y       mA + mB + iA rA.x^2 + iB rB.x^2 |
/// P = -mass_scale K^-1 (Cdot + bias_rate C) - impulse_scale impulse
/// @endcode
///
/// with `mA`, `mB` the inverse masses and `iA`, `iB` the inverse inertias.
/// The soft step is the one of the distance joints (see @ref DistanceJoint,
/// @ref PhysicsWorldConfig::joint_hertz and
/// @ref PhysicsWorldConfig::joint_damping_ratio): at rest the pin acts as a
/// stiff spring, so a load sags it by about `F K / w^2` (1.1 mm for 1 kg
/// hanging from a static pivot at 15 Hz). Unlike a distance joint of length
/// 0, which only holds the anchors along the line between them and has no
/// line when they meet, the pin holds both directions at once.
///
/// **The limits.** With `enable_limit`, the joint angle (see
/// @ref PhysicsWorld::RevoluteJointAngle)
///
/// @code
/// angle = WrapAngle(angle_b - angle_a - reference_angle)
/// @endcode
///
/// stays between `lower_angle` and `upper_angle`. Each limit is a one-sided
/// angular constraint with the axial mass `1 / (iA + iB)`, like a rope for
/// rotation: its accumulated impulse only pushes away from the limit (it is
/// clamped at 0), soft when the angle is past the limit, and rigid but
/// speculative before it (the angle may close up to the limit within the
/// step, not past it). The limits are solved before the point in each pass.
///
/// There is no motor: to drive a hinge, add a torque to one of its bodies
/// before the step (@ref Body::torque), equal and opposite on the other one
/// if both turn.
///
/// Bodies linked by a revolute joint don't collide with each other unless
/// `collide_connected` is set. Static and kinematic bodies have infinite
/// mass in joints, so a revolute joint to a static body is a fixed axle.
///
/// @note A revolute joint is simulation state: @ref PhysicsWorld::CopyFrom
/// copies the table and @ref PhysicsWorld::GenerateChecksum hashes it,
/// including the accumulated impulses that warm start the next step.
struct RevoluteJoint {
  /// First body, A.
  BodyIndex body_a{};
  /// Second body, B.
  BodyIndex body_b{};
  /// Anchor on body A, in A's frame, in meters from its position.
  Vec2f local_anchor_a{};
  /// Anchor on body B, in B's frame, in meters from its position.
  Vec2f local_anchor_b{};
  /// Angle of B relative to A at which the joint angle is 0, in radians.
  ///
  /// @ref PhysicsWorld::AddRevoluteJoint sets it to the bodies' current
  /// relative angle, `angle_b - angle_a`, so a joint starts at angle 0 and
  /// the limits are measured from the pose it was built in.
  Radian reference_angle{};
  /// Lowest joint angle when `enable_limit` is set, in radians, between -pi
  /// and `upper_angle`.
  Radian lower_angle{};
  /// Highest joint angle when `enable_limit` is set, in radians, between
  /// `lower_angle` and pi.
  Radian upper_angle{};
  /// Accumulated impulse of the point constraint over the last step, in N s,
  /// applied at anchor B (and its opposite at anchor A).
  ///
  /// Divided by the time step it is the force the pin carries: a 1 kg bob
  /// hanging at rest from a static pivot pushes it up with about
  /// `(0, 9.81 * dt)`. It warm starts the next step.
  Vec2f impulse{};
  /// Accumulated angular impulse of the lower limit over the last step, in
  /// N m s, at least 0 (it only turns B up from the limit, relative to A).
  Scalar lower_impulse{};
  /// Accumulated angular impulse of the upper limit over the last step, in
  /// N m s, at least 0 (it only turns B down from the limit, relative to A).
  Scalar upper_impulse{};
  /// Whether `lower_angle` and `upper_angle` are enforced. `false` by
  /// default: the bodies turn freely about the pin.
  bool enable_limit = false;
  /// Whether the colliders of body A and body B still collide with each
  /// other. `false` by default: hinged parts usually overlap at the hinge.
  ///
  /// Only solid pairs are filtered; triggers still report overlaps.
  bool collide_connected = false;
  /// Whether the slot holds a joint; `false` marks a free slot in the
  /// world's revolute joint table.
  bool is_used = false;
};

/// Returns a hash of every field of `joint`.
///
/// Building block of @ref PhysicsWorld::GenerateChecksum.
///
/// @param joint @ref RevoluteJoint to hash.
/// @return FNV-1a hash of the joint's fields.
[[nodiscard]] std::uint32_t GenerateChecksum(const RevoluteJoint& joint);

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
// Per-revolute-joint data computed once per step by SolveConstraints. body_a
// is null for a joint the solver skips (a free slot, an inactive body, or two
// bodies that can't move).
struct RevoluteJointScratch {
  Body* body_a = nullptr;
  Body* body_b = nullptr;
  Scalar inverse_mass_a{};
  Scalar inverse_mass_b{};
  Scalar inverse_inertia_a{};
  Scalar inverse_inertia_b{};
  Vec2f r_a{};
  Vec2f r_b{};
  // Inverse of the effective mass matrix K, symmetric: (xx, xy; xy, yy)
  Scalar mass_xx{};
  Scalar mass_xy{};
  Scalar mass_yy{};
  // Soft correction of the anchor separation, bias_rate * C
  Vec2f bias{};
  // Soft step coefficients of the point constraint
  Scalar mass_scale{1};
  Scalar impulse_scale{};
  // 1 / (iA + iB), 0 when neither body turns (no limit then)
  Scalar axial_mass{};
  // Target angular speeds of the lower and upper limits, and their soft step
  // coefficients (rigid while the limit isn't reached)
  Scalar lower_bias{};
  Scalar lower_mass_scale{1};
  Scalar lower_impulse_scale{};
  Scalar upper_bias{};
  Scalar upper_mass_scale{1};
  Scalar upper_impulse_scale{};
  bool has_limit = false;
};
}  // namespace internal

namespace internal {
// Solves the contacts and the rigid and rope joints of a step together, in the
// same passes (order and math on ContactSolverSettings and DistanceJoint).
// Without joints this is exactly SolveContacts, bit for bit.
//
// joints: the joint table; springs and free slots are skipped. Each joint's
// impulse holds the warm start, and the step's total on return. bodies: the
// body table the joints' indices refer to. joint_scratch: one entry per
// joint. Requires contact_scratch.size() >= 2 * contacts.size() and
// joint_scratch.size() >= joints.size().
void SolveConstraints(std::span<Contact> contacts,
                      std::span<ContactPointScratch> contact_scratch,
                      std::span<DistanceJoint> joints, std::span<Body> bodies,
                      std::span<JointScratch> joint_scratch,
                      const ContactSolverSettings& settings);

// The same with the revolute joints, solved after the distance joints and
// before the contacts in each pass, in table order (limits first, then the
// point). Free slots are skipped; each joint's impulse, lower_impulse and
// upper_impulse hold the warm start, and the step's totals on return. With an
// empty revolute table this is exactly the overload above, bit for bit.
// Requires revolute_scratch.size() >= revolute_joints.size() too.
void SolveConstraints(std::span<Contact> contacts,
                      std::span<ContactPointScratch> contact_scratch,
                      std::span<DistanceJoint> joints,
                      std::span<RevoluteJoint> revolute_joints,
                      std::span<Body> bodies,
                      std::span<JointScratch> joint_scratch,
                      std::span<RevoluteJointScratch> revolute_scratch,
                      const ContactSolverSettings& settings);
}  // namespace internal

}  // namespace neko

#endif  // NEKO_PHYSICS_JOINT_H_
