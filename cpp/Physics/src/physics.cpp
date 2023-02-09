#include "physics.h"


namespace neko
{
PhysicsWorld::PhysicsWorld(Vec2f gravity): gravity_(gravity)
{
}

BodyIndex PhysicsWorld::AddBody()
{
    const BodyIndex index{ bodies_.size() };
    bodies_.push_back({});
    return index;
}

void PhysicsWorld::Step(Scalar dt)
{
    for(auto& body: bodies_)
    {
        body.force += body.mass * gravity_;

        body.velocity += body.force / body.mass * dt;
        body.position += body.velocity * dt;
        body.force = Vec2f::zero();
    }
}

void PhysicsWorld::Clear()
{
    bodies_.clear();
}
} // namespace neko
