#pragma once
#include <array>

#include "math/vec2.h"

namespace neko
{
struct Collider;
struct Body;

struct CollidingBody
{
    Body* body = nullptr;
    const Collider* collider = nullptr;
};

struct Contact
{
    std::array<CollidingBody, 2> bodies{};
    Vec2f contactNormal{};
    Scalar restitution{};
    Scalar penetration{};
    void ResolveInterpenetration(Scalar dt) const;
    void Resolve(Scalar dt) const;
    Scalar CalculateSeparateVelocity() const;
    void ResolveVelocity(Scalar dt) const;
};
} // namespace neko
