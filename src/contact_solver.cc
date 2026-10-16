#include "neko/physics/contact_solver.h"

#include <array>
#include <cstdint>

#include "neko/core/platform.h"
#include "neko/physics/physics.h"

namespace neko {
namespace {
// Only dynamic bodies respond to contacts: static and kinematic bodies act as
// if their mass was infinite
Scalar InverseMass(const Body& body) {
  return body.type == BodyType::kDynamic ? body.inverse_mass : Scalar{0};
}
Scalar InverseInertia(const Body& body) {
  return body.type == BodyType::kDynamic ? body.inverse_inertia : Scalar{0};
}

// Velocity of the body's material point at r from its center: v + w x r, where
// the 2D cross product of an angular velocity and a vector is w (-r.y, r.x)
Vec2f PointVelocity(const Body& body, Vec2f r) {
  return body.velocity + r.Perpendicular2() * body.angular_velocity;
}

bool HasOrientedBox(const Contact& contact) {
  return contact.bodies[0].collider->type == ShapeType::kOrientedBox ||
         contact.bodies[1].collider->type == ShapeType::kOrientedBox;
}

// The velocity part of Resolve for contacts with an oriented box: one impulse
// per manifold point, with lever arms
void ResolveAtPoints(Contact& contact) {
  Body& body1 = *contact.bodies[0].body;
  Body& body2 = *contact.bodies[1].body;
  const auto& points = contact.points;
  const std::uint8_t point_count = contact.point_count;
  const Vec2f contact_normal = contact.contact_normal;
  const Scalar restitution = contact.restitution;
  const Scalar inverse_mass1 = InverseMass(body1);
  const Scalar inverse_mass2 = InverseMass(body2);
  const Scalar inverse_inertia1 = InverseInertia(body1);
  const Scalar inverse_inertia2 = InverseInertia(body2);

  // Approach speed at each point before any impulse: the restitution target
  std::array<Scalar, 2> approach_before{};
  for (std::uint8_t i = 0; i < point_count; ++i) {
    const Vec2f r1 = points[i].position - body1.position;
    const Vec2f r2 = points[i].position - body2.position;
    approach_before[i] = Vec2f::Dot(
        PointVelocity(body1, r1) - PointVelocity(body2, r2), contact_normal);
  }

  for (std::uint8_t i = 0; i < point_count; ++i) {
    if (approach_before[i] >= Scalar{0}) {
      continue;  // separating at this point
    }
    const Vec2f r1 = points[i].position - body1.position;
    const Vec2f r2 = points[i].position - body2.position;
    const Scalar normal_velocity = Vec2f::Dot(
        PointVelocity(body1, r1) - PointVelocity(body2, r2), contact_normal);
    const Scalar lever1 = Vec2f::Cross(r1, contact_normal);
    const Scalar lever2 = Vec2f::Cross(r2, contact_normal);
    const Scalar effective_inverse_mass = inverse_mass1 + inverse_mass2 +
                                          lever1 * lever1 * inverse_inertia1 +
                                          lever2 * lever2 * inverse_inertia2;
    if (effective_inverse_mass <= Scalar{0}) {
      continue;
    }
    const Scalar impulse =
        (-restitution * approach_before[i] - normal_velocity) /
        effective_inverse_mass;
    if (impulse <= Scalar{0}) {
      continue;  // an earlier point's impulse already separated this one
    }
    body1.velocity += contact_normal * (impulse * inverse_mass1);
    body1.angular_velocity += lever1 * impulse * inverse_inertia1;
    body2.velocity -= contact_normal * (impulse * inverse_mass2);
    body2.angular_velocity -= lever2 * impulse * inverse_inertia2;
  }
}
}  // namespace

Scalar Contact::CalculateSeparateVelocity() const {
  const auto relative_velocity =
      bodies[0].body->velocity - bodies[1].body->velocity;
  return Vec2f::Dot(relative_velocity, contact_normal);
}

// Velocity and interpenetration correction both divide by the same
// total_inverse_mass; sharing one reciprocal (1 division, 2 multiplies) instead
// of dividing twice is cheaper on the DS, where division goes through the
// memory-mapped hardware divider. Rounds differently from two separate
// divisions (Fixed16 division truncates, multiplication rounds to nearest), so
// results aren't bit-identical to the old code, only to themselves.
NEKO_HOT void Contact::Resolve() {
  const auto inverse_mass1 = InverseMass(*bodies[0].body);
  const auto inverse_mass2 = InverseMass(*bodies[1].body);
  const auto total_inverse_mass = inverse_mass1 + inverse_mass2;
  if (total_inverse_mass <= Scalar{0}) {
    // Both bodies are effectively infinite mass: neither velocity nor position
    // correction applies
    return;
  }
  const auto inv_total_inverse_mass = Scalar{1} / total_inverse_mass;

  if (HasOrientedBox(*this)) {
    ResolveAtPoints(*this);
  } else if (const auto separating_velocity = CalculateSeparateVelocity();
             separating_velocity <= Scalar{0}) {
    const auto new_sep_velocity = -separating_velocity * restitution;
    const auto delta_velocity = new_sep_velocity - separating_velocity;
    const auto impulse_per_i_mass =
        contact_normal * (delta_velocity * inv_total_inverse_mass);
    bodies[0].body->velocity += impulse_per_i_mass * inverse_mass1;
    bodies[1].body->velocity -= impulse_per_i_mass * inverse_mass2;
  }

  if (penetration > Scalar{0}) {
    const auto move_per_i_mass =
        contact_normal * (penetration * inv_total_inverse_mass);
    bodies[0].body->position += move_per_i_mass * inverse_mass1;
    bodies[1].body->position -= move_per_i_mass * inverse_mass2;
  }
}
}  // namespace neko
