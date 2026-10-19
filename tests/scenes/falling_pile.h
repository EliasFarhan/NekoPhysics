#ifndef TESTS_SCENES_FALLING_PILE_H_
#define TESTS_SCENES_FALLING_PILE_H_

#include <bit>
#include <cstdint>
#include <utility>

#include "neko/math/scalar_traits.h"
#include "neko/physics/physics.h"

namespace neko::scenes {

inline constexpr int kFallingPileDynamicCount = 64;
// 3 static boxes, the dynamic bodies and the kinematic paddle, in AddBody order
inline constexpr Index kFallingPileBodyCount = kFallingPileDynamicCount + 4;

// num / den computed in Scalar, so the scene is built without any float
// arithmetic
constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

// Deterministic scene shared by the golden checksum test and the DS test ROM: a
// static floor and walls, a kinematic paddle, and a pile of dynamic circles and
// boxes, some of them triggers. Fits PhysicsWorldConfig's default capacities.
inline void BuildFallingPile(PhysicsWorld& world) {
  const auto add_static_box = [&world](Vec2f position, Vec2f half_size) {
    const auto body = world.AddBody();
    world.body(body).type = BodyType::kStatic;
    world.body(body).position = position;
    const auto collider = world.AddAabbCollider(body);
    world.collider(collider).is_trigger = false;
    world.collider(collider).restitution = Ratio(1, 4);
    world.aabb(world.collider(collider).shape_index).half_size = half_size;
  };
  add_static_box({Scalar{0}, Ratio(-1, 2)}, {Scalar{8}, Ratio(1, 2)});
  add_static_box({Ratio(-17, 2), Scalar{6}}, {Ratio(1, 2), Scalar{7}});
  add_static_box({Ratio(17, 2), Scalar{6}}, {Ratio(1, 2), Scalar{7}});

  for (int i = 0; i < kFallingPileDynamicCount; ++i) {
    const auto body = world.AddBody();
    const int column = i % 16;
    const int row = i / 16;
    world.body(body).position = {Ratio(-70 + 9 * column + 2 * (row % 2), 10),
                                 Ratio(20 + 11 * row, 10)};
    world.body(body).velocity = {Ratio(7 * (i % 5 - 2), 10),
                                 static_cast<Scalar>(i % 3)};
    world.body(body).inverse_mass = i % 4 == 0 ? Ratio(1, 2) : Scalar{1};
    const bool is_circle = i % 3 != 0;
    const auto collider =
        is_circle ? world.AddCircleCollider(body) : world.AddAabbCollider(body);
    world.collider(collider).is_trigger = i % 11 == 5;
    world.collider(collider).restitution = Ratio(i % 4, 10);
    if (is_circle) {
      world.circle(world.collider(collider).shape_index).radius =
          Ratio(6 + i % 3, 20);
    } else {
      world.aabb(world.collider(collider).shape_index).half_size = {
          Ratio(7, 20), Ratio(5 + i % 2, 20)};
    }
  }

  const auto paddle = world.AddBody();
  world.body(paddle).type = BodyType::kKinematic;
  world.body(paddle).position = {Scalar{-7}, Ratio(1, 2)};
  world.body(paddle).velocity = {Ratio(3, 2), Scalar{0}};
  const auto paddle_collider = world.AddAabbCollider(paddle);
  world.collider(paddle_collider).is_trigger = false;
  world.aabb(world.collider(paddle_collider).shape_index).half_size = {
      Ratio(1, 4), Ratio(1, 2)};
}

// Golden checksums of BuildFallingPile with Scalar = Fixed16, shared by
// tests/scene_test.cc and the DS test ROM (nds/main.cc) so there is one source
// to re-pin when the simulation changes on purpose. Re-pinned on 2026-10-17
// with MSVC 14.51 for the iterative contact solver (M3 stage 1): contacts are
// solved together in 8 passes with accumulated impulses, Baumgarte push-out
// and slop instead of an instant position correction, and Step integrates
// positions after the contacts. Re-pinned again the same day for friction
// (M3 stage 2): Collider::friction (0.5 by default) is hashed, and the
// contacts grip. And again for warm starting (M3 stage 3): the solver starts
// from last step's impulses, and the contact cache is hashed, so step 1
// changes too (the pile has contacts from its first step).
// The DS test ROM must print the same values.
inline constexpr std::pair<int, std::uint32_t> kFallingPileGoldenChecksums[] = {
    {0, 0x84d0a2d9u},   {1, 0xf1757b85u},   {60, 0x26a9ff73u},
    {120, 0x70f4dddfu}, {300, 0xc498aeaeu}, {600, 0x23abe78du},
};

// FNV-1a over the raw bits of every body's position and velocity, and nothing
// else. Unlike PhysicsWorld::GenerateChecksum it ignores the other body fields
// and the colliders, so it stays the same when Body gains fields (rotation)
// as long as the bodies move exactly as before.
inline std::uint32_t TrajectoryChecksum(const PhysicsWorld& world,
                                        Index body_count) {
  std::uint32_t hash = 2166136261u;
  const auto add = [&hash](Scalar value) {
    const auto word = ToBitsWord(value);
    for (int shift = 0; shift < 32; shift += 8) {
      hash ^= (word >> shift) & 0xFFu;
      hash *= 16777619u;
    }
  };
  for (Index i = 0; i < body_count; ++i) {
    const Body& body = world.body(BodyIndex{i});
    add(body.position.x());
    add(body.position.y());
    add(body.velocity.x());
    add(body.velocity.y());
  }
  return hash;
}

// Golden TrajectoryChecksum of BuildFallingPile with Scalar = Fixed16, at the
// steps of kFallingPileGoldenChecksums. Pinned on 2026-10-16 before rotation
// (unchanged by M5, whose stages didn't move non-rotating scenes), re-pinned
// on 2026-10-17 with MSVC 14.51 for the iterative contact solver (M3 stage 1),
// which changes how every contact moves the bodies, and again for friction
// (M3 stage 2) and warm starting (M3 stage 3).
inline constexpr std::pair<int, std::uint32_t>
    kFallingPileTrajectoryChecksums[] = {
        {0, 0x239c5dc0u},   {1, 0xbc032ed3u},   {60, 0xb803c72du},
        {120, 0xac5dd0a0u}, {300, 0x8e5dd1c9u}, {600, 0x94e1db39u},
};

}  // namespace neko::scenes

#endif  // TESTS_SCENES_FALLING_PILE_H_
