#include "neko/physics/contact_solver.h"

#include "neko/core/platform.h"
#include "neko/physics/physics.h"

namespace neko {
namespace {
// Only dynamic bodies respond to contacts: static and kinematic bodies act as
// if their mass was infinite
Scalar InverseMass(const Body& body) {
  return body.type == BodyType::kDynamic ? body.inverse_mass : Scalar{0};
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

  const auto separating_velocity = CalculateSeparateVelocity();
  if (separating_velocity <= Scalar{0}) {
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
