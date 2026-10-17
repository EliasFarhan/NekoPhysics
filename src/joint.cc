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
void ApplyImpulse(const internal::JointScratch& joint, Vec2f impulse) {
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
}  // namespace

NEKO_HOT void SolveConstraints(
    std::span<Contact> contacts,
    std::span<internal::ContactPointScratch> contact_scratch,
    std::span<DistanceJoint> joints, std::span<Body> bodies,
    std::span<internal::JointScratch> joint_scratch,
    const ContactSolverSettings& settings) {
  // The contact targets use the velocities before any impulse, joints' or
  // contacts': prepare them first, then warm start both
  internal::PrepareContacts(contacts, contact_scratch, settings);
  // Without joints this is SolveContacts: no joint call in the passes
  const bool has_joints = !joints.empty();
  if (has_joints) {
    PrepareJoints(joints, bodies, joint_scratch, settings);
  }
  if (settings.warm_starting) {
    if (has_joints) {
      WarmStartJoints(joints, joint_scratch);
    }
    internal::WarmStartContacts(contacts, contact_scratch);
  }
  for (int pass = 0; pass < settings.velocity_iterations; ++pass) {
    if (has_joints) {
      SolveJointPass(joints, joint_scratch);
    }
    internal::SolveContactPass(contacts, contact_scratch);
  }
}

}  // namespace neko
