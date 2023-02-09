#pragma once

#ifdef DLLPROJECT_EXPORTS
#   define EXPORT __declspec(dllexport)
#else
#   define EXPORT __declspec(dllimport)
#endif
#include "physics.h"

typedef struct _Vector2f
{
    float x;
    float y;
} Vector2f;

extern "C" EXPORT neko::PhysicsWorld* CreateWorld();
extern "C" EXPORT void DestroyWorld(const neko::PhysicsWorld * instance);
extern "C" EXPORT int CreateBody(neko::PhysicsWorld * instance);
extern "C" EXPORT neko::Body* GetBody(int index);