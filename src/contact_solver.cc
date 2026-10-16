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

Scalar NormalVelocity(const Contact& contact,
                      const internal::ContactPointScratch& point) {
  return Vec2f::Dot(PointVelocity(*contact.bodies[0].body, point.r1) -
                        PointVelocity(*contact.bodies[1].body, point.r2),
                    contact.contact_normal);
}
}  // namespace

Scalar Contact::CalculateSeparateVelocity() const {
  const auto relative_velocity =
      bodies[0].body->velocity - bodies[1].body->velocity;
  return Vec2f::Dot(relative_velocity, contact_normal);
}

NEKO_HOT void SolveContacts(std::span<Contact> contacts,
                            std::span<internal::ContactPointScratch> scratch,
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
    for (std::uint8_t i = 0; i < contact.point_count; ++i) {
      ContactPoint& point = contact.points[i];
      internal::ContactPointScratch& data = scratch[2 * c + i];
      data.r1 = point.position - contact.bodies[0].body->position;
      data.r2 = point.position - contact.bodies[1].body->position;
      const Scalar lever1 = Vec2f::Cross(data.r1, contact.contact_normal);
      const Scalar lever2 = Vec2f::Cross(data.r2, contact.contact_normal);
      const Scalar inverse_effective_mass =
          inverse_mass1 + inverse_mass2 + lever1 * lever1 * inverse_inertia1 +
          lever2 * lever2 * inverse_inertia2;
      data.normal_mass = inverse_effective_mass > Scalar{0}
                             ? Scalar{1} / inverse_effective_mass
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
      point.normal_impulse = Scalar{0};
    }
  }

  for (int pass = 0; pass < settings.velocity_iterations; ++pass) {
    for (std::size_t c = 0; c < contacts.size(); ++c) {
      Contact& contact = contacts[c];
      Body& body1 = *contact.bodies[0].body;
      Body& body2 = *contact.bodies[1].body;
      const Scalar inverse_mass1 = InverseMass(body1);
      const Scalar inverse_mass2 = InverseMass(body2);
      const Scalar inverse_inertia1 = InverseInertia(contact.bodies[0]);
      const Scalar inverse_inertia2 = InverseInertia(contact.bodies[1]);
      for (std::uint8_t i = 0; i < contact.point_count; ++i) {
        ContactPoint& point = contact.points[i];
        const internal::ContactPointScratch& data = scratch[2 * c + i];
        const Scalar change =
            (data.target_velocity - NormalVelocity(contact, data)) *
            data.normal_mass;
        // Clamp the total, not the change: a pass can take back part of an
        // earlier push, but the contact never pulls
        const Scalar total = Max(point.normal_impulse + change, Scalar{0});
        const Scalar impulse = total - point.normal_impulse;
        point.normal_impulse = total;
        if (impulse == Scalar{0}) {
          continue;
        }
        const Vec2f push = contact.contact_normal * impulse;
        body1.velocity += push * inverse_mass1;
        body1.angular_velocity +=
            Vec2f::Cross(data.r1, push) * inverse_inertia1;
        body2.velocity -= push * inverse_mass2;
        body2.angular_velocity -=
            Vec2f::Cross(data.r2, push) * inverse_inertia2;
      }
    }
  }
}
}  // namespace neko
