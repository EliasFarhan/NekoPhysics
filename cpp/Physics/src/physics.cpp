#include "physics.h"


namespace neko
{
PhysicsWorld::PhysicsWorld(Vec2f gravity): gravity_(gravity)
{
}

BodyIndex PhysicsWorld::AddBody()
{
    const auto it = std::ranges::find_if(bodies_, [](const auto& body)
    {
        return body.type == BodyType::NONE;
    });
    if(it != bodies_.end())
    {
        it->type = BodyType::DYNAMIC;
        return { static_cast<std::size_t>(std::distance(bodies_.begin(), it)) };
    }
    const BodyIndex index{ bodies_.size() };
    bodies_.push_back({});
    return index;
}

void PhysicsWorld::RemoveBody(BodyIndex index)
{
    bodies_[index.index].type = BodyType::NONE;
}

void PhysicsWorld::Step(Scalar dt)
{
    for(auto& body: bodies_)
    {
        switch (body.type)
        {
        case BodyType::DYNAMIC:
        {
            body.force += body.mass * gravity_;

            body.velocity += body.force / body.mass * dt;

            body.position += body.velocity * dt;
            body.force = Vec2f::zero();
            break;
        }
        case BodyType::KINEMATIC:
        {
            body.position += body.velocity * dt;
            break;
        }
        default: break;
        }

    }
}

void PhysicsWorld::Clear()
{
    bodies_.clear();
}
} // namespace neko
