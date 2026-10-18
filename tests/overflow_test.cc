// StepOverflow: every capacity limit hit during a step is counted, and the
// kept items are the first ones in the usual order (slice S2 of the night run
// of 2026-10-18).
#include <cstddef>

#include <gtest/gtest.h>

#include "neko/bsh/quadtree.h"
#include "neko/physics/physics.h"

namespace {
using neko::BodyIndex;
using neko::ColliderIndex;
using neko::Scalar;
using neko::Vec2f;

// `count` dynamic circles of radius 1 at the origin, all
// overlapping each other, without gravity
void AddOverlappingCircles(neko::PhysicsWorld& world, int count,
                           bool is_trigger) {
  for (int i = 0; i < count; ++i) {
    const BodyIndex body = world.AddBody();
    world.body(body).position = {static_cast<Scalar>(i) / Scalar{10},
                                 Scalar{0}};
    const ColliderIndex collider = world.AddCircleCollider(body);
    world.collider(collider).is_trigger = is_trigger;
    world.circle(world.collider(collider).shape_index).radius = Scalar{1};
  }
}

constexpr neko::PhysicsWorldConfig Config(std::size_t touching,
                                          std::size_t broadphase) {
  return {.gravity = Vec2f::Zero(),
          .max_bodies = 8,
          .max_colliders = 8,
          .max_touching_pairs = touching,
          .max_broadphase_pairs = broadphase,
          .max_joints = 1,
          .max_passing_pairs = 1};
}
}  // namespace

// A world within its capacities counts nothing
TEST(StepOverflow, NothingCountedWithinCapacity) {
  neko::PhysicsWorld world{Config(8, 8)};
  AddOverlappingCircles(world, 3, false);
  world.Step();
  EXPECT_FALSE(world.step_overflow().IsAnyCounted());
  EXPECT_EQ(world.step_overflow(), neko::StepOverflow{});
  EXPECT_EQ(world.touching_pairs().size(), 3u);
}

// Three overlapping colliders make three pairs: two are dropped by a
// broadphase of one pair
TEST(StepOverflow, BroadphasePairs) {
  neko::PhysicsWorld world{Config(8, 1)};
  AddOverlappingCircles(world, 3, true);
  world.Step();
  EXPECT_EQ(world.step_overflow().broadphase_pairs, 2u);
  EXPECT_EQ(world.BroadphasePairCount(), 1u);
  EXPECT_EQ(world.touching_pairs().size(), 1u);
  EXPECT_TRUE(world.step_overflow().IsAnyCounted());
}

// A custom broadphase built for fewer colliders than the world has
TEST(StepOverflow, BroadphaseColliders) {
  neko::PhysicsWorld world{Config(8, 8)};
  neko::QuadTree quad_tree{/*max_colliders=*/2, /*max_pairs=*/8};
  world.SetBSH(&quad_tree);
  AddOverlappingCircles(world, 3, true);
  world.Step();
  EXPECT_EQ(world.step_overflow().broadphase_colliders, 1u);
  EXPECT_EQ(world.step_overflow().broadphase_pairs, 0u);
  EXPECT_EQ(world.touching_pairs().size(), 1u);
  // And the quadtree's own pair capacity
  neko::QuadTree small_pairs{/*max_colliders=*/8, /*max_pairs=*/1};
  world.SetBSH(&small_pairs);
  world.Step();
  EXPECT_EQ(world.step_overflow().broadphase_colliders, 0u);
  EXPECT_EQ(world.step_overflow().broadphase_pairs, 2u);
}

// Trigger pairs past max_touching_pairs: no events for them, no contacts
// to drop
TEST(StepOverflow, TouchingPairs) {
  neko::PhysicsWorld world{Config(1, 8)};
  AddOverlappingCircles(world, 3, true);
  world.Step();
  EXPECT_EQ(world.step_overflow().touching_pairs, 2u);
  EXPECT_EQ(world.step_overflow().contacts, 0u);
  EXPECT_EQ(world.touching_pairs().size(), 1u);
}

// Solid pairs past max_touching_pairs: neither solved nor reported
TEST(StepOverflow, Contacts) {
  neko::PhysicsWorld world{Config(1, 8)};
  AddOverlappingCircles(world, 3, false);
  world.Step();
  EXPECT_EQ(world.step_overflow().contacts, 2u);
  EXPECT_EQ(world.step_overflow().touching_pairs, 2u);
  EXPECT_EQ(world.touching_pairs().size(), 1u);
}

// The kept pairs are the first ones in broadphase order, on every machine
TEST(StepOverflow, KeepsTheFirstPairs) {
  neko::PhysicsWorld small{Config(1, 8)};
  AddOverlappingCircles(small, 3, true);
  small.Step();
  ASSERT_EQ(small.touching_pairs().size(), 1u);
  // SweepAndPrune sorts by the left side of the boxes: circle 0 is leftmost
  // and its pair with circle 1 comes first
  EXPECT_EQ(small.touching_pairs()[0].pair,
            (neko::ColliderPair{ColliderIndex{0}, ColliderIndex{1}}));
}

// Two boxes rising through a one-way platform at once, with room for one
// passing pair: the second is not solved this step either, and counted
TEST(StepOverflow, PassingPairs) {
  neko::PhysicsWorld world{Config(8, 8)};
  const BodyIndex platform = world.AddBody();
  world.body(platform).type = neko::BodyType::kStatic;
  const ColliderIndex platform_collider = world.AddAabbCollider(platform);
  world.collider(platform_collider).is_trigger = false;
  world.collider(platform_collider).is_one_way = true;
  world.aabb(world.collider(platform_collider).shape_index).half_size = {
      Scalar{4}, Scalar{0.175f}};
  for (int i = 0; i < 2; ++i) {
    const BodyIndex box = world.AddBody();
    world.body(box).position = {static_cast<Scalar>(2 * i - 1), Scalar{-0.4f}};
    world.body(box).velocity = {Scalar{0}, Scalar{2}};
    world.body(box).inertia_mode = neko::InertiaMode::kFixedRotation;
    const ColliderIndex collider = world.AddAabbCollider(box);
    world.collider(collider).is_trigger = false;
    world.aabb(world.collider(collider).shape_index).half_size = {
        Scalar{0.25f}, Scalar{0.25f}};
  }
  world.Step();
  EXPECT_EQ(world.step_overflow().passing_pairs, 1u);
  EXPECT_EQ(world.passing_pairs().size(), 1u);
  EXPECT_TRUE(world.touching_pairs().empty());
}

// DropThrough past the capacity counts into the next step
TEST(StepOverflow, DropThroughPassingPairs) {
  neko::PhysicsWorld world{
      {.max_bodies = 4, .max_colliders = 4, .max_passing_pairs = 1}};
  for (int i = 0; i < 2; ++i) {
    const BodyIndex platform = world.AddBody();
    world.body(platform).type = neko::BodyType::kStatic;
    world.body(platform).position = {Scalar{0}, Scalar{-0.5f * i}};
    const ColliderIndex collider = world.AddAabbCollider(platform);
    world.collider(collider).is_trigger = false;
    world.collider(collider).is_one_way = true;
    world.aabb(world.collider(collider).shape_index).half_size = {Scalar{2},
                                                                  Scalar{0.4f}};
  }
  // A tall box standing on both platforms' overlap
  const BodyIndex box = world.AddBody();
  world.body(box).position = {Scalar{0}, Scalar{0.6f}};
  world.body(box).inertia_mode = neko::InertiaMode::kFixedRotation;
  const ColliderIndex collider = world.AddAabbCollider(box);
  world.collider(collider).is_trigger = false;
  world.collider(collider).restitution = Scalar{0};
  world.aabb(world.collider(collider).shape_index).half_size = {Scalar{0.25f},
                                                                Scalar{1}};
  world.Step();
  ASSERT_EQ(world.touching_pairs().size(), 2u);
  EXPECT_EQ(world.DropThrough(box), 1u);
  EXPECT_EQ(world.step_overflow().passing_pairs, 0u);
  world.Step();
  EXPECT_EQ(world.step_overflow().passing_pairs, 1u);
}

// A refused joint is counted in the next step's counters, then reset
TEST(StepOverflow, JointsAndResetEachStep) {
  neko::PhysicsWorld world{Config(8, 8)};
  const BodyIndex a = world.AddBody();
  const BodyIndex b = world.AddBody();
  world.body(b).position = {Scalar{1}, Scalar{0}};
  EXPECT_NE(world.AddDistanceJoint(a, b), neko::kInvalidJointIndex);
  EXPECT_EQ(world.AddDistanceJoint(a, b), neko::kInvalidJointIndex);
  EXPECT_EQ(world.step_overflow().joints, 0u);
  world.Step();
  EXPECT_EQ(world.step_overflow().joints, 1u);
  world.Step();
  EXPECT_EQ(world.step_overflow().joints, 0u);
  EXPECT_FALSE(world.step_overflow().IsAnyCounted());
}

// Counters follow the current step: they fall back to zero once the scene
// fits again
TEST(StepOverflow, CountersFollowTheScene) {
  neko::PhysicsWorld world{Config(1, 8)};
  AddOverlappingCircles(world, 3, true);
  world.Step();
  EXPECT_EQ(world.step_overflow().touching_pairs, 2u);
  world.collider(ColliderIndex{2}).is_enabled = false;
  world.Step();
  EXPECT_EQ(world.step_overflow().touching_pairs, 0u);
  world.Clear();
  EXPECT_FALSE(world.step_overflow().IsAnyCounted());
}

// The counters are diagnostics: not copied, not hashed
TEST(StepOverflow, NotCopiedNorHashed) {
  neko::PhysicsWorld world{Config(1, 8)};
  neko::PhysicsWorld copy{Config(1, 8)};
  AddOverlappingCircles(world, 3, true);
  world.Step();
  copy.CopyFrom(world);
  EXPECT_TRUE(world.step_overflow().IsAnyCounted());
  EXPECT_FALSE(copy.step_overflow().IsAnyCounted());
  EXPECT_EQ(copy.GenerateChecksum(), world.GenerateChecksum());
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
