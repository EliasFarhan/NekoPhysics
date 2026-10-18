// Warm starting (M3 stage 3): the contact solver starts from last step's
// impulses, matched by feature id, kept in a contact cache that rollback
// copies and hashes.
#include <algorithm>
#include <array>
#include <cmath>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "neko/math/scalar_traits.h"
#include "neko/physics/physics.h"

namespace {

using neko::Scalar;
using neko::Vec2f;

template <typename T>
float ToFloat(T value) {
  return neko::ScalarTraits<T>::ToFloat(value);
}

struct Stack {
  float sunk = 0.0f;   // how far the top box sank, in meters
  float drift = 0.0f;  // largest sideways drift of a box, in meters
};

// Six 1 m boxes stacked on a static ground, run for 5 s
Stack RunStack(bool warm_starting, int iterations, bool oriented) {
  neko::PhysicsWorld world{
      {.velocity_iterations = iterations, .warm_starting = warm_starting}};
  const auto ground = world.AddBody();
  world.body(ground).type = neko::BodyType::kStatic;
  world.body(ground).position = {Scalar{0}, Scalar{-1}};
  const auto ground_collider = world.AddAabbCollider(ground);
  world.collider(ground_collider).is_trigger = false;
  world.collider(ground_collider).restitution = Scalar{0};
  world.aabb(world.collider(ground_collider).shape_index).half_size = {
      Scalar{10}, Scalar{1}};
  const Vec2f half{Scalar{1} / Scalar{2}, Scalar{1} / Scalar{2}};
  std::vector<neko::BodyIndex> boxes;
  for (int i = 0; i < 6; ++i) {
    const auto box = world.AddBody();
    boxes.push_back(box);
    world.body(box).position = {Scalar{0}, half.y() + static_cast<Scalar>(i)};
    const auto collider = oriented ? world.AddOrientedBoxCollider(box)
                                   : world.AddAabbCollider(box);
    world.collider(collider).is_trigger = false;
    world.collider(collider).restitution = Scalar{0};
    if (oriented) {
      world.oriented_box(world.collider(collider).shape_index).half_size = half;
    } else {
      world.aabb(world.collider(collider).shape_index).half_size = half;
    }
  }
  for (int step = 0; step < 300; ++step) {
    world.Step();
  }
  Stack stack;
  stack.sunk = 5.5f - ToFloat(world.body(boxes.back()).position.y());
  for (const auto box : boxes) {
    stack.drift =
        std::max(stack.drift, std::abs(ToFloat(world.body(box).position.x())));
  }
  return stack;
}

}  // namespace

TEST(WarmStarting, StackSinksLessWithTheSamePasses) {
  // Six AABBs, 8 passes. Each contact keeps up to the 5 mm slop, so the top
  // may sink 6 * 5 mm = 3 cm. Starting from zero every step, 8 passes don't
  // build up the impulses that hold the stack (4.5 cm); starting from last
  // step's, they do (4 mm).
  const auto cold = RunStack(false, 8, false);
  const auto warm = RunStack(true, 8, false);
  EXPECT_GT(cold.sunk, 0.03f);
  EXPECT_LT(warm.sunk, 0.01f);
}

TEST(WarmStarting, TurningBoxesStackWithFourPasses) {
  // Six oriented boxes, 4 passes: without warm starting the stack collapses
  // (the top falls 5 m); with it, it stands
  const auto cold = RunStack(false, 4, true);
  const auto warm = RunStack(true, 4, true);
  EXPECT_GT(cold.sunk, 1.0f) << "collapsed";
  EXPECT_LT(warm.sunk, 0.03f);
  EXPECT_LT(warm.drift, 0.05f);
}

TEST(WarmStarting, CacheIsPartOfTheReplayedState) {
  // A copy of a world whose boxes rest on the ground carries the contact
  // cache: stepping it gives exactly the original's states, warm start
  // included (the checksum hashes the cache)
  const auto build = [](neko::PhysicsWorld& world) {
    const auto ground = world.AddBody();
    world.body(ground).type = neko::BodyType::kStatic;
    const auto ground_collider = world.AddAabbCollider(ground);
    world.collider(ground_collider).is_trigger = false;
    world.collider(ground_collider).restitution = Scalar{0};
    world.aabb(world.collider(ground_collider).shape_index).half_size = {
        Scalar{10}, Scalar{1}};
    for (int i = 0; i < 4; ++i) {
      const auto box = world.AddBody();
      world.body(box).position = {static_cast<Scalar>(i - 2),
                                  Scalar{2} + static_cast<Scalar>(i)};
      const auto collider = world.AddOrientedBoxCollider(box);
      world.collider(collider).is_trigger = false;
      world.collider(collider).restitution = Scalar{0};
      world.oriented_box(world.collider(collider).shape_index).half_size = {
          Scalar{1} / Scalar{2}, Scalar{1} / Scalar{4}};
    }
  };
  neko::PhysicsWorld world;
  build(world);
  for (int step = 0; step < 120; ++step) {
    world.Step();
  }
  ASSERT_FALSE(world.touching_pairs().empty());
  neko::PhysicsWorld copy;
  copy.CopyFrom(world);
  EXPECT_EQ(copy.GenerateChecksum(), world.GenerateChecksum());
  for (int step = 0; step < 60; ++step) {
    world.Step();
    copy.Step();
    ASSERT_EQ(copy.GenerateChecksum(), world.GenerateChecksum()) << step;
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
