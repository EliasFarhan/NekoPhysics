#include "physics_export.h"


EXPORT neko::PhysicsWorld* CreateWorld()
{
    return new neko::PhysicsWorld();
}

EXPORT void DestroyWorld(const neko::PhysicsWorld * instance)
{
    delete instance;
}