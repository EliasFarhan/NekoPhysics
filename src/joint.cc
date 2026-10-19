#include "neko/physics/joint.h"

#include <cassert>
#include <cstddef>

#include "neko/core/platform.h"
#include "neko/math/const.h"
#include "neko/physics/physics.h"

namespace neko {
namespace {
// Only dynamic bodies respond to joints: static and kinematic bodies act as
// if their mass and inertia were infinite
Scalar InverseMass(const Body& body) {
  return body.type == BodyType::kDynamic ? body.inverse_mass : Scalar{0};
}
Scalar InverseInertia(const Body& body) {
  return body.type == BodyType::kDynamic ? body.inverse_inertia : Scalar{0};
}

// Velocity of the body's material point at r from its center, v + w x r
Vec2f PointVelocity(const Body& body, Vec2f r) {
  return body.velocity + r.Perpendicular2() * body.angular_velocity;
}

// Separation speed of the anchors along the axis, Cdot
Scalar SeparationVelocity(const internal::JointScratch& joint) {
  return Vec2f::Dot(PointVelocity(*joint.body_b, joint.r_b) -
                        PointVelocity(*joint.body_a, joint.r_a),
                    joint.axis);
}

// Applies -impulse at anchor A and +impulse at anchor B
NEKO_HOT void ApplyImpulse(const internal::JointScratch& joint, Vec2f impulse) {
  Body& body_a = *joint.body_a;
  Body& body_b = *joint.body_b;
  body_a.velocity -= impulse * joint.inverse_mass_a;
  body_a.angular_velocity -=
      Vec2f::Cross(joint.r_a, impulse) * joint.inverse_inertia_a;
  body_b.velocity += impulse * joint.inverse_mass_b;
  body_b.angular_velocity +=
      Vec2f::Cross(joint.r_b, impulse) * joint.inverse_inertia_b;
}

// The soft step coefficients of a joint frequency and damping ratio, the
// same for every joint: a spring of that frequency pulls the length error
// back, implicitly, so it stays stable whatever the passes and the warm start
struct Softness {
  Scalar bias_rate{};
  Scalar mass_scale{1};
  Scalar impulse_scale{};
};

Softness ComputeSoftness(Scalar hertz, Scalar damping_ratio, Scalar dt) {
  if (hertz <= Scalar{0}) {
    // Rigid: no position correction, the velocity only
    return {};
  }
  const Scalar omega = Scalar{2} * Pi<Scalar>() * hertz;
  const Scalar a1 = Scalar{2} * damping_ratio + dt * omega;
  const Scalar a2 = dt * omega * a1;
  const Scalar a3 = Scalar{1} / (Scalar{1} + a2);
  return {omega / a1, a2 * a3, a3};
}

// Lever arms, axis, effective mass and target of each rigid or rope joint.
// Two divisions for the axis and one for the mass per joint, none in the
// passes.
void PrepareJoints(std::span<DistanceJoint> joints, std::span<Body> bodies,
                   std::span<internal::JointScratch> scratch,
                   const ContactSolverSettings& settings) {
  assert(scratch.size() >= joints.size());
  const Softness softness = ComputeSoftness(
      settings.joint_hertz, settings.joint_damping_ratio, settings.dt);
  const Scalar inverse_dt = Scalar{1} / settings.dt;
  for (std::size_t j = 0; j < joints.size(); ++j) {
    DistanceJoint& joint = joints[j];
    internal::JointScratch& data = scratch[j];
    data = {};
    if (joint.type != DistanceJointType::kRigid &&
        joint.type != DistanceJointType::kRope) {
      continue;
    }
    Body& body_a = bodies[joint.body_a.index()];
    Body& body_b = bodies[joint.body_b.index()];
    if (!body_a.is_active || !body_b.is_active) {
      joint.impulse = Scalar{0};
      continue;
    }
    data.inverse_mass_a = InverseMass(body_a);
    data.inverse_mass_b = InverseMass(body_b);
    data.inverse_inertia_a = InverseInertia(body_a);
    data.inverse_inertia_b = InverseInertia(body_b);
    data.r_a = body_a.rotation.Rotate(joint.local_anchor_a);
    data.r_b = body_b.rotation.Rotate(joint.local_anchor_b);
    const Vec2f delta =
        (body_b.position + data.r_b) - (body_a.position + data.r_a);
    const Scalar distance = delta.Length();
    // Each component over the length is at most 1 in size: no overflow, even
    // for anchors a few 1/65536 m apart. Anchors on top of each other have
    // no direction: pick up, like two circles at the same position.
    data.axis = distance > Scalar{0}
                    ? Vec2f{delta.x() / distance, delta.y() / distance}
                    : Vec2f::Up();
    const Scalar lever_a = Vec2f::Cross(data.r_a, data.axis);
    const Scalar lever_b = Vec2f::Cross(data.r_b, data.axis);
    const Scalar inverse_effective_mass =
        data.inverse_mass_a + data.inverse_mass_b +
        lever_a * lever_a * data.inverse_inertia_a +
        lever_b * lever_b * data.inverse_inertia_b;
    if (inverse_effective_mass <= Scalar{0}) {
      // Neither body can move along the axis
      joint.impulse = Scalar{0};
      continue;
    }
    data.body_a = &body_a;
    data.body_b = &body_b;
    data.mass = Scalar{1} / inverse_effective_mass;
    data.is_rope = joint.type == DistanceJointType::kRope;
    const Scalar error = distance - joint.length;
    if (data.is_rope && error < Scalar{0}) {
      // A slack rope may close up to taut within the step: the speculative
      // target -C / dt lets the pull start in the step it would overstretch,
      // rigidly
      data.target_velocity = -error * inverse_dt;
      data.mass_scale = Scalar{1};
      data.impulse_scale = Scalar{0};
    } else {
      data.target_velocity = -softness.bias_rate * error;
      data.mass_scale = softness.mass_scale;
      data.impulse_scale = softness.impulse_scale;
    }
    if (!settings.warm_starting) {
      joint.impulse = Scalar{0};
    } else if (data.is_rope && joint.impulse > Scalar{0}) {
      joint.impulse = Scalar{0};
    }
  }
}

void WarmStartJoints(std::span<const DistanceJoint> joints,
                     std::span<const internal::JointScratch> scratch) {
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const internal::JointScratch& data = scratch[j];
    if (data.body_a == nullptr || joints[j].impulse == Scalar{0}) {
      continue;
    }
    ApplyImpulse(data, data.axis * joints[j].impulse);
  }
}

NEKO_HOT void SolveJointPass(std::span<DistanceJoint> joints,
                             std::span<const internal::JointScratch> scratch) {
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const internal::JointScratch& data = scratch[j];
    if (data.body_a == nullptr) {
      continue;
    }
    DistanceJoint& joint = joints[j];
    // The soft step: a little less than the rigid impulse (mass_scale), and
    // a leak of the accumulated impulse (impulse_scale), which is what keeps
    // the warm start from feeding last step's correction back every step
    const Scalar change = (data.target_velocity - SeparationVelocity(data)) *
                              data.mass * data.mass_scale -
                          joint.impulse * data.impulse_scale;
    // A rope only pulls: its accumulated impulse stays at or below 0, so a
    // pass can release an earlier pull but never push
    const Scalar total = data.is_rope ? Min(joint.impulse + change, Scalar{0})
                                      : joint.impulse + change;
    const Scalar impulse = total - joint.impulse;
    joint.impulse = total;
    if (impulse != Scalar{0}) {
      ApplyImpulse(data, data.axis * impulse);
    }
  }
}

// Applies -impulse at anchor A and +impulse at anchor B, plus the angular
// impulse of the limits (+axial on B, -axial on A)
NEKO_HOT void ApplyRevoluteImpulse(const internal::RevoluteJointScratch& joint,
                                   Vec2f impulse, Scalar axial) {
  Body& body_a = *joint.body_a;
  Body& body_b = *joint.body_b;
  body_a.velocity -= impulse * joint.inverse_mass_a;
  body_a.angular_velocity -=
      (Vec2f::Cross(joint.r_a, impulse) + axial) * joint.inverse_inertia_a;
  body_b.velocity += impulse * joint.inverse_mass_b;
  body_b.angular_velocity +=
      (Vec2f::Cross(joint.r_b, impulse) + axial) * joint.inverse_inertia_b;
}

void ResetImpulses(RevoluteJoint& joint) {
  joint.impulse = Vec2f::Zero();
  joint.lower_impulse = Scalar{0};
  joint.upper_impulse = Scalar{0};
}

// Target and soft step of one limit
struct LimitTarget {
  Scalar bias{};
  Scalar mass_scale{1};
  Scalar impulse_scale{};
};

// The limit target for a distance C to the limit (positive while the limit
// is not reached). Before the limit the angle may close up to it within the
// step, rigidly (speculative, like a slack rope); past it the soft step pulls
// it back.
LimitTarget PrepareLimit(Scalar distance, const Softness& softness,
                         Scalar inverse_dt) {
  if (distance > Scalar{0}) {
    return {distance * inverse_dt, Scalar{1}, Scalar{0}};
  }
  return {softness.bias_rate * distance, softness.mass_scale,
          softness.impulse_scale};
}

// Lever arms, inverse effective mass matrix, separation and limit targets of
// each revolute joint. The 2x2 matrix is normalized by its largest diagonal
// entry before the inversion, so its determinant keeps its precision in
// Fixed16 even for heavy bodies (inverse masses of a few 1/100).
void PrepareRevoluteJoints(std::span<RevoluteJoint> joints,
                           std::span<Body> bodies,
                           std::span<internal::RevoluteJointScratch> scratch,
                           const ContactSolverSettings& settings) {
  assert(scratch.size() >= joints.size());
  const Softness softness = ComputeSoftness(
      settings.joint_hertz, settings.joint_damping_ratio, settings.dt);
  const Scalar inverse_dt = Scalar{1} / settings.dt;
  for (std::size_t j = 0; j < joints.size(); ++j) {
    RevoluteJoint& joint = joints[j];
    internal::RevoluteJointScratch& data = scratch[j];
    data = {};
    if (!joint.is_used) {
      continue;
    }
    Body& body_a = bodies[joint.body_a.index()];
    Body& body_b = bodies[joint.body_b.index()];
    if (!body_a.is_active || !body_b.is_active) {
      ResetImpulses(joint);
      continue;
    }
    data.inverse_mass_a = InverseMass(body_a);
    data.inverse_mass_b = InverseMass(body_b);
    data.inverse_inertia_a = InverseInertia(body_a);
    data.inverse_inertia_b = InverseInertia(body_b);
    data.r_a = body_a.rotation.Rotate(joint.local_anchor_a);
    data.r_b = body_b.rotation.Rotate(joint.local_anchor_b);
    const Vec2f r_a = data.r_a;
    const Vec2f r_b = data.r_b;
    const Scalar i_a = data.inverse_inertia_a;
    const Scalar i_b = data.inverse_inertia_b;
    const Scalar mass_sum = data.inverse_mass_a + data.inverse_mass_b;
    const Scalar k_xx =
        mass_sum + i_a * r_a.y() * r_a.y() + i_b * r_b.y() * r_b.y();
    const Scalar k_xy = -i_a * r_a.x() * r_a.y() - i_b * r_b.x() * r_b.y();
    const Scalar k_yy =
        mass_sum + i_a * r_a.x() * r_a.x() + i_b * r_b.x() * r_b.x();
    const Scalar scale = Max(k_xx, k_yy);
    if (scale <= Scalar{0}) {
      // Neither body can move
      ResetImpulses(joint);
      continue;
    }
    const Scalar n_xx = k_xx / scale;
    const Scalar n_xy = k_xy / scale;
    const Scalar n_yy = k_yy / scale;
    const Scalar determinant = n_xx * n_yy - n_xy * n_xy;
    if (determinant <= Scalar{0}) {
      // Free in one direction only (a body that turns but has no mass): no
      // 2x2 inverse
      ResetImpulses(joint);
      continue;
    }
    const Scalar inverse_determinant = Scalar{1} / (determinant * scale);
    data.mass_xx = n_yy * inverse_determinant;
    data.mass_xy = -n_xy * inverse_determinant;
    data.mass_yy = n_xx * inverse_determinant;
    data.body_a = &body_a;
    data.body_b = &body_b;
    const Vec2f separation = (body_b.position + r_b) - (body_a.position + r_a);
    data.bias = separation * softness.bias_rate;
    data.mass_scale = softness.mass_scale;
    data.impulse_scale = softness.impulse_scale;

    const Scalar axial_inverse_mass = i_a + i_b;
    data.has_limit = joint.enable_limit && axial_inverse_mass > Scalar{0};
    if (data.has_limit) {
      data.axial_mass = Scalar{1} / axial_inverse_mass;
      const Scalar angle =
          WrapAngle(body_b.rotation.angle() - body_a.rotation.angle() -
                    joint.reference_angle)
              .value();
      const LimitTarget lower =
          PrepareLimit(angle - joint.lower_angle.value(), softness, inverse_dt);
      const LimitTarget upper =
          PrepareLimit(joint.upper_angle.value() - angle, softness, inverse_dt);
      data.lower_bias = lower.bias;
      data.lower_mass_scale = lower.mass_scale;
      data.lower_impulse_scale = lower.impulse_scale;
      data.upper_bias = upper.bias;
      data.upper_mass_scale = upper.mass_scale;
      data.upper_impulse_scale = upper.impulse_scale;
    } else {
      joint.lower_impulse = Scalar{0};
      joint.upper_impulse = Scalar{0};
    }
    if (!settings.warm_starting) {
      ResetImpulses(joint);
    }
  }
}

void WarmStartRevoluteJoints(
    std::span<const RevoluteJoint> joints,
    std::span<const internal::RevoluteJointScratch> scratch) {
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const internal::RevoluteJointScratch& data = scratch[j];
    if (data.body_a == nullptr) {
      continue;
    }
    const RevoluteJoint& joint = joints[j];
    ApplyRevoluteImpulse(data, joint.impulse,
                         joint.lower_impulse - joint.upper_impulse);
  }
}

// One limit: the accumulated angular impulse stays at or above 0, so a pass
// can release an earlier push but never pull. relative_speed is the speed at
// which the angle moves away from the limit.
NEKO_HOT Scalar SolveLimit(Scalar relative_speed, Scalar axial_mass,
                           Scalar bias, Scalar mass_scale, Scalar impulse_scale,
                           Scalar* accumulated) {
  const Scalar target = relative_speed + bias;
  const Scalar mass = axial_mass * mass_scale;
  // What the limit keeps of its push before this pass's correction
  const Scalar kept = *accumulated - impulse_scale * *accumulated;
  Scalar total{};
  if (target > Scalar{0} && target >= kept / mass) {
    // Moving away from the limit, or far from it: everything is released.
    // Decided without mass * target, which passes Fixed16's 32767 for a
    // heavy body far from its limit (an 8 m, 200 kg plank at 30 degrees).
    total = Scalar{0};
  } else {
    total = Max(kept - mass * target, Scalar{0});
  }
  const Scalar impulse = total - *accumulated;
  *accumulated = total;
  return impulse;
}

NEKO_HOT void SolveRevolutePass(
    std::span<RevoluteJoint> joints,
    std::span<const internal::RevoluteJointScratch> scratch) {
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const internal::RevoluteJointScratch& data = scratch[j];
    if (data.body_a == nullptr) {
      continue;
    }
    RevoluteJoint& joint = joints[j];
    if (data.has_limit) {
      // Lower limit: turns B up from it relative to A
      const Scalar lower = SolveLimit(
          data.body_b->angular_velocity - data.body_a->angular_velocity,
          data.axial_mass, data.lower_bias, data.lower_mass_scale,
          data.lower_impulse_scale, &joint.lower_impulse);
      if (lower != Scalar{0}) {
        ApplyRevoluteImpulse(data, Vec2f::Zero(), lower);
      }
      // Upper limit: the mirror image, turns B down from it
      const Scalar upper = SolveLimit(
          data.body_a->angular_velocity - data.body_b->angular_velocity,
          data.axial_mass, data.upper_bias, data.upper_mass_scale,
          data.upper_impulse_scale, &joint.upper_impulse);
      if (upper != Scalar{0}) {
        ApplyRevoluteImpulse(data, Vec2f::Zero(), -upper);
      }
    }
    // The point: P = -mass_scale K^-1 (Cdot + bias) - impulse_scale impulse
    const Vec2f rhs = PointVelocity(*data.body_b, data.r_b) -
                      PointVelocity(*data.body_a, data.r_a) + data.bias;
    const Vec2f solved{data.mass_xx * rhs.x() + data.mass_xy * rhs.y(),
                       data.mass_xy * rhs.x() + data.mass_yy * rhs.y()};
    const Vec2f impulse =
        solved * (-data.mass_scale) - joint.impulse * data.impulse_scale;
    joint.impulse += impulse;
    ApplyRevoluteImpulse(data, impulse, Scalar{0});
  }
}
}  // namespace

NEKO_HOT void SolveConstraints(
    std::span<Contact> contacts,
    std::span<internal::ContactPointScratch> contact_scratch,
    std::span<DistanceJoint> joints, std::span<Body> bodies,
    std::span<internal::JointScratch> joint_scratch,
    const ContactSolverSettings& settings) {
  SolveConstraints(contacts, contact_scratch, joints, {}, bodies, joint_scratch,
                   {}, settings);
}

NEKO_HOT void SolveConstraints(
    std::span<Contact> contacts,
    std::span<internal::ContactPointScratch> contact_scratch,
    std::span<DistanceJoint> joints, std::span<RevoluteJoint> revolute_joints,
    std::span<Body> bodies, std::span<internal::JointScratch> joint_scratch,
    std::span<internal::RevoluteJointScratch> revolute_scratch,
    const ContactSolverSettings& settings) {
  // The contact targets use the velocities before any impulse, joints' or
  // contacts': prepare them first, then warm start both
  internal::PrepareContacts(contacts, contact_scratch, settings);
  // Without joints this is SolveContacts: no joint call in the passes
  const bool has_joints = !joints.empty();
  const bool has_revolute_joints = !revolute_joints.empty();
  if (has_joints) {
    PrepareJoints(joints, bodies, joint_scratch, settings);
  }
  if (has_revolute_joints) {
    PrepareRevoluteJoints(revolute_joints, bodies, revolute_scratch, settings);
  }
  if (settings.warm_starting) {
    if (has_joints) {
      WarmStartJoints(joints, joint_scratch);
    }
    if (has_revolute_joints) {
      WarmStartRevoluteJoints(revolute_joints, revolute_scratch);
    }
    internal::WarmStartContacts(contacts, contact_scratch);
  }
  for (int pass = 0; pass < settings.velocity_iterations; ++pass) {
    if (has_joints) {
      SolveJointPass(joints, joint_scratch);
    }
    if (has_revolute_joints) {
      SolveRevolutePass(revolute_joints, revolute_scratch);
    }
    internal::SolveContactPass(contacts, contact_scratch);
  }
}

}  // namespace neko
