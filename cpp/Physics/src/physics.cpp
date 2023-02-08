#include "physics.h"


namespace neko
{
void PhysicsWorld::Step(Scalar dt)
{
    for(auto& body: bodies_)
    {
        body.force += body.mass * gravity;

        body.velocity += body.force / body.mass * dt;
        body.position += body.velocity * dt;
        body.force = Vec2f::zero();
    }
}
} // namespace neko
