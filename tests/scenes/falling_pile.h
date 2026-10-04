#ifndef TESTS_SCENES_FALLING_PILE_H_
#define TESTS_SCENES_FALLING_PILE_H_

#include "neko/physics/physics.h"

#include <cstdint>
#include <utility>

namespace neko::scenes
{

inline constexpr int kFallingPileDynamicCount = 64;

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
    const auto add_static_box = [&world](Vec2f position, Vec2f half_size)
    {
        const auto body = world.AddBody();
        world.body(body).type = BodyType::kStatic;
        world.body(body).position = position;
        const auto collider = world.AddAabbCollider(body);
        world.collider(collider).is_trigger = false;
        world.collider(collider).restitution = Ratio(1, 4);
        world.aabb(world.collider(collider).shape_index).half_size = half_size;
    };
    add_static_box({ Scalar{0}, Ratio(-1, 2) }, { Scalar{8}, Ratio(1, 2) });
    add_static_box({ Ratio(-17, 2), Scalar{6} }, { Ratio(1, 2), Scalar{7} });
    add_static_box({ Ratio(17, 2), Scalar{6} }, { Ratio(1, 2), Scalar{7} });

    for (int i = 0; i < kFallingPileDynamicCount; ++i)
    {
        const auto body = world.AddBody();
        const int column = i % 16;
        const int row = i / 16;
        world.body(body).position = { Ratio(-70 + 9 * column + 2 * (row % 2), 10), Ratio(20 + 11 * row, 10) };
        world.body(body).velocity = { Ratio(7 * (i % 5 - 2), 10), static_cast<Scalar>(i % 3) };
        world.body(body).inverse_mass = i % 4 == 0 ? Ratio(1, 2) : Scalar{ 1 };
        const bool is_circle = i % 3 != 0;
        const auto collider = is_circle ? world.AddCircleCollider(body) : world.AddAabbCollider(body);
        world.collider(collider).is_trigger = i % 11 == 5;
        world.collider(collider).restitution = Ratio(i % 4, 10);
        if (is_circle)
        {
            world.circle(world.collider(collider).shape_index).radius = Ratio(6 + i % 3, 20);
        }
        else
        {
            world.aabb(world.collider(collider).shape_index).half_size = { Ratio(7, 20), Ratio(5 + i % 2, 20) };
        }
    }

    const auto paddle = world.AddBody();
    world.body(paddle).type = BodyType::kKinematic;
    world.body(paddle).position = { Scalar{-7}, Ratio(1, 2) };
    world.body(paddle).velocity = { Ratio(3, 2), Scalar{0} };
    const auto paddle_collider = world.AddAabbCollider(paddle);
    world.collider(paddle_collider).is_trigger = false;
    world.aabb(world.collider(paddle_collider).shape_index).half_size = { Ratio(1, 4), Ratio(1, 2) };
}

/**
 * @brief Golden checksums of BuildFallingPile with Scalar = Fixed16, shared by tests/scene_test.cc and the DS test
 * ROM (nds/main.cc) so there is one source to re-pin when the simulation changes on purpose.
 * Re-pinned on 2026-09-27, most recently for the kDetectContact reciprocal change (one division instead of two
 * for the circle-circle and circle-AABB contact normal): verified with GCC 15.2 (-O2, -fno-exceptions -fno-rtti)
 * and the DS test ROM on melonDS; not re-checked against MSVC.
 * The DS test ROM must print the same values.
 */
inline constexpr std::pair<int, std::uint32_t> kFallingPileGoldenChecksums[] = {
    { 0, 0x61c2e799u },
    { 1, 0xfa00ce4cu },
    { 60, 0x1846fbd1u },
    { 120, 0x2e3c79a0u },
    { 300, 0x4ab151e4u },
    { 600, 0x3e890176u },
};

} // namespace neko::scenes

#endif  // TESTS_SCENES_FALLING_PILE_H_
