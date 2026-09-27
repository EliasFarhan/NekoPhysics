#pragma once

#include "physics/physics.h"

namespace neko::scenes
{

inline constexpr int FALLING_PILE_DYNAMIC_COUNT = 64;

/**
 * @brief num / den computed in Scalar, so the scene is built without any float arithmetic
 */
constexpr Scalar Ratio(int num, int den)
{
    return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

/**
 * @brief Deterministic scene shared by the golden checksum test and the DS test ROM: a static floor and walls, a
 * kinematic paddle, and a pile of dynamic circles and boxes, some of them triggers.
 * Fits PhysicsWorldConfig's default capacities.
 */
inline void BuildFallingPile(PhysicsWorld& world)
{
    const auto addStaticBox = [&world](Vec2f position, Vec2f halfSize)
    {
        const auto body = world.AddBody();
        world.body(body).type = BodyType::STATIC;
        world.body(body).position = position;
        const auto collider = world.AddAabbCollider(body);
        world.collider(collider).isTrigger = false;
        world.collider(collider).restitution = Ratio(1, 4);
        world.aabb(world.collider(collider).shapeIndex).halfSize = halfSize;
    };
    addStaticBox({ Scalar{0}, Ratio(-1, 2) }, { Scalar{8}, Ratio(1, 2) });
    addStaticBox({ Ratio(-17, 2), Scalar{6} }, { Ratio(1, 2), Scalar{7} });
    addStaticBox({ Ratio(17, 2), Scalar{6} }, { Ratio(1, 2), Scalar{7} });

    for (int i = 0; i < FALLING_PILE_DYNAMIC_COUNT; ++i)
    {
        const auto body = world.AddBody();
        const int column = i % 16;
        const int row = i / 16;
        world.body(body).position = { Ratio(-70 + 9 * column + 2 * (row % 2), 10), Ratio(20 + 11 * row, 10) };
        world.body(body).velocity = { Ratio(7 * (i % 5 - 2), 10), static_cast<Scalar>(i % 3) };
        world.body(body).inverseMass = i % 4 == 0 ? Ratio(1, 2) : Scalar{ 1 };
        const bool isCircle = i % 3 != 0;
        const auto collider = isCircle ? world.AddCircleCollider(body) : world.AddAabbCollider(body);
        world.collider(collider).isTrigger = i % 11 == 5;
        world.collider(collider).restitution = Ratio(i % 4, 10);
        if (isCircle)
        {
            world.circle(world.collider(collider).shapeIndex).radius = Ratio(6 + i % 3, 20);
        }
        else
        {
            world.aabb(world.collider(collider).shapeIndex).halfSize = { Ratio(7, 20), Ratio(5 + i % 2, 20) };
        }
    }

    const auto paddle = world.AddBody();
    world.body(paddle).type = BodyType::KINEMATIC;
    world.body(paddle).position = { Scalar{-7}, Ratio(1, 2) };
    world.body(paddle).velocity = { Ratio(3, 2), Scalar{0} };
    const auto paddleCollider = world.AddAabbCollider(paddle);
    world.collider(paddleCollider).isTrigger = false;
    world.aabb(world.collider(paddleCollider).shapeIndex).halfSize = { Ratio(1, 4), Ratio(1, 2) };
}

} // namespace neko::scenes
