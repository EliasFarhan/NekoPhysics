#include "physics/contact_solver.h"

#include "physics/physics.h"

#include "core/platform.h"

namespace neko
{
namespace
{
// Only dynamic bodies respond to contacts: static and kinematic bodies act as if their mass was infinite
Scalar InverseMass(const Body& body)
{
    return body.type == BodyType::DYNAMIC ? body.inverseMass : Scalar{0};
}
} // namespace

Scalar Contact::CalculateSeparateVelocity() const
{
    const auto relativeVelocity = bodies[0].body->velocity - bodies[1].body->velocity;
    return Vec2f::Dot(relativeVelocity, contactNormal);
}

// Velocity and interpenetration correction both divide by the same totalInverseMass; sharing one reciprocal
// (1 division, 2 multiplies) instead of dividing twice is cheaper on the DS, where division goes through the
// memory-mapped hardware divider. Rounds differently from two separate divisions (Fixed16 division truncates,
// multiplication rounds to nearest), so results aren't bit-identical to the old code, only to themselves.
NEKO_HOT void Contact::Resolve()
{
    const auto inverseMass1 = InverseMass(*bodies[0].body);
    const auto inverseMass2 = InverseMass(*bodies[1].body);
    const auto totalInverseMass = inverseMass1 + inverseMass2;
    if (totalInverseMass <= Scalar{0})
    {
        // Both bodies are effectively infinite mass: neither velocity nor position correction applies
        return;
    }
    const auto invTotalInverseMass = Scalar{1} / totalInverseMass;

    const auto separatingVelocity = CalculateSeparateVelocity();
    if (separatingVelocity <= Scalar{0})
    {
        const auto newSepVelocity = -separatingVelocity * restitution;
        const auto deltaVelocity = newSepVelocity - separatingVelocity;
        const auto impulsePerIMass = contactNormal * (deltaVelocity * invTotalInverseMass);
        bodies[0].body->velocity += impulsePerIMass * inverseMass1;
        bodies[1].body->velocity -= impulsePerIMass * inverseMass2;
    }

    if (penetration > Scalar{0})
    {
        const auto movePerIMass = contactNormal * (penetration * invTotalInverseMass);
        bodies[0].body->position += movePerIMass * inverseMass1;
        bodies[1].body->position -= movePerIMass * inverseMass2;
    }
}
} // namespace neko
