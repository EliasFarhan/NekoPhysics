#include "neko/physics/contact_solver.h"

#include <cassert>
#include <cstddef>
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
// An AABB's shape doesn't turn, so its body doesn't spin from the contact
Scalar InverseInertia(const CollidingBody& side) {
  return side.body->type == BodyType::kDynamic &&
                 side.collider->type != ShapeType::kAabb
             ? side.body->inverse_inertia
             : Scalar{0};
}

// Velocity of the body's material point at r from its center: v + w x r, where
// the 2D cross product of an angular velocity and a vector is w (-r.y, r.x)
Vec2f PointVelocity(const Body& body, Vec2f r) {
  return body.velocity + r.Perpendicular2() * body.angular_velocity;
}

Vec2f RelativeVelocity(const Contact& contact,
                       const internal::ContactPointScratch& point) {
  return PointVelocity(*contact.bodies[0].body, point.r1) -
         PointVelocity(*contact.bodies[1].body, point.r2);
}

Scalar NormalVelocity(const Contact& contact,
                      const internal::ContactPointScratch& point) {
  return Vec2f::Dot(RelativeVelocity(contact, point), contact.contact_normal);
}

// One over the effective inverse mass along a direction at the point, 0 when
// neither body can move
Scalar EffectiveMass(Scalar inverse_mass_sum, Scalar inverse_inertia1,
                     Scalar inverse_inertia2,
                     const internal::ContactPointScratch& point,
                     Vec2f direction) {
  const Scalar lever1 = Vec2f::Cross(point.r1, direction);
  const Scalar lever2 = Vec2f::Cross(point.r2, direction);
  const Scalar inverse_effective_mass = inverse_mass_sum +
                                        lever1 * lever1 * inverse_inertia1 +
                                        lever2 * lever2 * inverse_inertia2;
  return inverse_effective_mass > Scalar{0} ? Scalar{1} / inverse_effective_mass
                                            : Scalar{0};
}

// Applies an impulse at the point: +impulse to body 1, -impulse to body 2
NEKO_HOT void ApplyImpulse(Contact& contact,
                           const internal::ContactPointScratch& point,
                           Vec2f impulse) {
  Body& body1 = *contact.bodies[0].body;
  Body& body2 = *contact.bodies[1].body;
  body1.velocity += impulse * point.inverse_mass1;
  body1.angular_velocity +=
      Vec2f::Cross(point.r1, impulse) * point.inverse_inertia1;
  body2.velocity -= impulse * point.inverse_mass2;
  body2.angular_velocity -=
      Vec2f::Cross(point.r2, impulse) * point.inverse_inertia2;
}
}  // namespace

Scalar Contact::CalculateSeparateVelocity() const {
  const auto relative_velocity =
      bodies[0].body->velocity - bodies[1].body->velocity;
  return Vec2f::Dot(relative_velocity, contact_normal);
}

namespace internal {
NEKO_HOT void PrepareContacts(std::span<Contact> contacts,
                              std::span<ContactPointScratch> scratch,
                              const ContactSolverSettings& settings) {
  assert(scratch.size() >= 2 * contacts.size());
  // Baumgarte: the push-out speed per meter of penetration beyond the slop
  const Scalar bias_factor = settings.baumgarte / settings.dt;

  // Once per step: lever arms, effective masses and target speeds. One
  // division per point here, none in the passes.
  for (std::size_t c = 0; c < contacts.size(); ++c) {
    Contact& contact = contacts[c];
    const Scalar inverse_mass1 = InverseMass(*contact.bodies[0].body);
    const Scalar inverse_mass2 = InverseMass(*contact.bodies[1].body);
    const Scalar inverse_inertia1 = InverseInertia(contact.bodies[0]);
    const Scalar inverse_inertia2 = InverseInertia(contact.bodies[1]);
    const Scalar inverse_mass_sum = inverse_mass1 + inverse_mass2;
    for (std::uint8_t i = 0; i < contact.point_count; ++i) {
      ContactPoint& point = contact.points[i];
      ContactPointScratch& data = scratch[2 * c + i];
      data.inverse_mass1 = inverse_mass1;
      data.inverse_mass2 = inverse_mass2;
      data.inverse_inertia1 = inverse_inertia1;
      data.inverse_inertia2 = inverse_inertia2;
      data.r1 = point.position - contact.bodies[0].body->position;
      data.r2 = point.position - contact.bodies[1].body->position;
      data.normal_mass =
          EffectiveMass(inverse_mass_sum, inverse_inertia1, inverse_inertia2,
                        data, contact.contact_normal);
      // Without friction the tangent impulse is clamped to 0 whatever its
      // mass: skip the second division
      data.tangent_mass =
          contact.friction > Scalar{0}
              ? EffectiveMass(inverse_mass_sum, inverse_inertia1,
                              inverse_inertia2, data,
                              contact.contact_normal.Perpendicular())
              : Scalar{0};
      const Scalar approach = NormalVelocity(contact, data);
      const Scalar bounce = approach < -settings.restitution_threshold
                                ? -contact.restitution * approach
                                : Scalar{0};
      const Scalar push_out =
          point.penetration > settings.penetration_slop
              ? bias_factor * (point.penetration - settings.penetration_slop)
              : Scalar{0};
      data.target_velocity = Max(bounce, push_out);
      if (!settings.warm_starting) {
        point.normal_impulse = Scalar{0};
        point.tangent_impulse = Scalar{0};
      }
    }
  }
}

NEKO_HOT void WarmStartContacts(std::span<Contact> contacts,
                                std::span<const ContactPointScratch> scratch) {
  for (std::size_t c = 0; c < contacts.size(); ++c) {
    Contact& contact = contacts[c];
    const Vec2f tangent = contact.contact_normal.Perpendicular();
    for (std::uint8_t i = 0; i < contact.point_count; ++i) {
      const ContactPoint& point = contact.points[i];
      if (point.normal_impulse == Scalar{0} &&
          point.tangent_impulse == Scalar{0}) {
        continue;
      }
      ApplyImpulse(contact, scratch[2 * c + i],
                   contact.contact_normal * point.normal_impulse +
                       tangent * point.tangent_impulse);
    }
  }
}

NEKO_HOT void SolveContactPass(std::span<Contact> contacts,
                               std::span<const ContactPointScratch> scratch) {
  for (std::size_t c = 0; c < contacts.size(); ++c) {
    Contact& contact = contacts[c];
    const Vec2f tangent = contact.contact_normal.Perpendicular();
    for (std::uint8_t i = 0; i < contact.point_count; ++i) {
      ContactPoint& point = contact.points[i];
      const ContactPointScratch& data = scratch[2 * c + i];

      // Friction first: stop the sliding, within the Coulomb limit of the
      // push accumulated so far
      const Scalar max_friction = contact.friction * point.normal_impulse;
      const Scalar tangent_change =
          -Vec2f::Dot(RelativeVelocity(contact, data), tangent) *
          data.tangent_mass;
      const Scalar tangent_total = Clamp(point.tangent_impulse + tangent_change,
                                         -max_friction, max_friction);
      const Scalar friction_impulse = tangent_total - point.tangent_impulse;
      point.tangent_impulse = tangent_total;
      if (friction_impulse != Scalar{0}) {
        ApplyImpulse(contact, data, tangent * friction_impulse);
      }

      const Scalar change =
          (data.target_velocity - NormalVelocity(contact, data)) *
          data.normal_mass;
      // Clamp the total, not the change: a pass can take back part of an
      // earlier push, but the contact never pulls
      const Scalar total = Max(point.normal_impulse + change, Scalar{0});
      const Scalar impulse = total - point.normal_impulse;
      point.normal_impulse = total;
      if (impulse != Scalar{0}) {
        ApplyImpulse(contact, data, contact.contact_normal * impulse);
      }
    }
  }
}

void SolveContacts(std::span<Contact> contacts,
                   std::span<ContactPointScratch> scratch,
                   const ContactSolverSettings& settings) {
  PrepareContacts(contacts, scratch, settings);
  // Warm start: apply last step's impulses before the passes, once every
  // target is computed (the targets use the velocities before any impulse)
  if (settings.warm_starting) {
    WarmStartContacts(contacts, scratch);
  }
  for (int pass = 0; pass < settings.velocity_iterations; ++pass) {
    SolveContactPass(contacts, scratch);
  }
}
}  // namespace internal
}  // namespace neko
