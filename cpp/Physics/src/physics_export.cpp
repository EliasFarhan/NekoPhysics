#include "physics_export.h"


EXPORT neko::PhysicsWorld* CreateWorld()
{
    return new neko::PhysicsWorld();
}

EXPORT void DestroyWorld(const neko::PhysicsWorld * instance)
{
    delete instance;
}

EXPORT int CreateBody(neko::PhysicsWorld * instance)
{
    return static_cast<int>(instance->AddBody().index);
}

EXPORT neko::Body * GetBody(neko::PhysicsWorld* instance, int index)
{
    return &instance->body({ static_cast<std::size_t>(index) });
}

EXPORT void SetPosition(neko::Body * instance, Vector2f newPos)
{
    instance->position = { newPos.x, newPos.y };
}
EXPORT void SetVelocity(neko::Body * instance, Vector2f newVel)
{
    instance->velocity = { newVel.x, newVel.y };
}
EXPORT void AddForce(neko::Body * instance, Vector2f force)
{
    instance->force += neko::Vec2f{ force.x, force.y };
}
EXPORT void SetMass(neko::Body * instance, float mass)
{
    instance->mass = mass;
}
EXPORT Vector2f GetPosition(const neko::Body * instance)
{
    return { instance->position.x, instance->position.y };
}
EXPORT Vector2f GetVelocity(const neko::Body * instance)
{
    return { instance->velocity.x, instance->velocity.y };
}
EXPORT Vector2f GetForce(const neko::Body * instance)
{
    return { instance->force.x, instance->force.y };
}
EXPORT float GetMass(const neko::Body * instance)
{
    return instance->mass;
}
EXPORT void Step(neko::PhysicsWorld* instance, float dt)
{
    instance->Step(dt);
}