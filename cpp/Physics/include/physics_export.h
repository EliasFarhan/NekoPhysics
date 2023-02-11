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
extern "C" EXPORT neko::Body* GetBody(neko::PhysicsWorld * instance, int index);
extern "C" EXPORT void SetPosition(neko::Body * instance, Vector2f newPos);
extern "C" EXPORT void SetVelocity(neko::Body * instance, Vector2f newVel);
extern "C" EXPORT void AddForce(neko::Body * instance, Vector2f force);
extern "C" EXPORT void SetMass(neko::Body * instance, float mass);
extern "C" EXPORT Vector2f GetPosition(const neko::Body * instance);
extern "C" EXPORT Vector2f GetVelocity(const neko::Body * instance);
extern "C" EXPORT Vector2f GetForce(const neko::Body * instance);
extern "C" EXPORT float GetMass(const neko::Body * instance);
extern "C" EXPORT void Step(neko::PhysicsWorld * instance, float dt);
