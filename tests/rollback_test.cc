#include <cstdint>
#include <cstdlib>
#include <vector>

#include <gtest/gtest.h>

#include "neko/bsh/quadtree.h"
#include "neko/math/angle.h"
#include "neko/physics/physics.h"

namespace {
using neko::Index;
using neko::Scalar;
using neko::Vec2f;

class CountingAllocator final : public neko::Allocator {
 public:
  void* Allocate(std::size_t count, std::size_t type_size) override {
    ++allocation_count;
    return std::malloc(count * type_size);
  }
  void Deallocate(void* ptr) override { std::free(ptr); }
  int allocation_count = 0;
};

class CountingListener final : public neko::ContactListener {
 public:
  void OnTriggerEnter(const neko::ColliderPair&) override { ++events; }
  void OnTriggerExit(const neko::ColliderPair&) override { ++events; }
  void OnCollisionEnter(const neko::ColliderPair&) override { ++events; }
  void OnCollisionExit(const neko::ColliderPair&) override { ++events; }
  int events = 0;
};

constexpr neko::PhysicsWorldConfig kConfig{
    .max_bodies = 32, .max_colliders = 32, .max_touching_pairs = 128};

// A static ground, a pile of falling boxes and circles (some of them triggers)
// and a kinematic paddle
void BuildScene(neko::PhysicsWorld& world) {
  const auto ground = world.AddBody();
  world.body(ground).type = neko::BodyType::kStatic;
  world.body(ground).position = {Scalar{0}, Scalar{-1}};
  const auto ground_collider = world.AddAabbCollider(ground);
  world.collider(ground_collider).is_trigger = false;
  world.collider(ground_collider).restitution = Scalar{0.5f};
  world.aabb(world.collider(ground_collider).shape_index).half_size = {
      Scalar{10}, Scalar{1}};

  for (int i = 0; i < 16; ++i) {
    const auto body = world.AddBody();
    world.body(body).position = {
        Scalar{-4.0f + 0.55f * static_cast<float>(i)},
        Scalar{1.0f + 0.3f * static_cast<float>(i % 5)}};
    world.body(body).velocity = {Scalar{static_cast<float>(i % 3) - 1.0f},
                                 Scalar{0}};
    const auto collider = i % 2 == 0 ? world.AddCircleCollider(body)
                                     : world.AddAabbCollider(body);
    world.collider(collider).is_trigger = i % 7 == 3;
    world.collider(collider).restitution = Scalar{0.2f};
    if (i % 2 == 0) {
      world.circle(world.collider(collider).shape_index).radius = Scalar{0.3f};
    } else {
      world.aabb(world.collider(collider).shape_index).half_size = {
          Scalar{0.3f}, Scalar{0.25f}};
    }
  }

  const auto paddle = world.AddBody();
  world.body(paddle).type = neko::BodyType::kKinematic;
  world.body(paddle).position = {Scalar{-6}, Scalar{0.5f}};
  world.body(paddle).velocity = {Scalar{2}, Scalar{0}};
  const auto paddle_collider = world.AddAabbCollider(paddle);
  world.collider(paddle_collider).is_trigger = false;
  world.aabb(world.collider(paddle_collider).shape_index).half_size = {
      Scalar{0.2f}, Scalar{0.5f}};
}
}  // namespace

TEST(Checksum, WorldChecksumFollowsState) {
  neko::PhysicsWorld a{kConfig};
  neko::PhysicsWorld b{kConfig};
  BuildScene(a);
  BuildScene(b);
  EXPECT_EQ(a.GenerateChecksum(), b.GenerateChecksum());
  auto& tweaked = b.body(neko::BodyIndex{3}).position;
  tweaked.set_x(tweaked.x() + Scalar{0.001f});
  EXPECT_NE(a.GenerateChecksum(), b.GenerateChecksum());
}

// Rollback: restore a snapshot, resimulate, and get exactly the same states and
// events
TEST(Rollback, ResimulationIsIdentical) {
  neko::QuadTree quad_tree;
  CountingListener listener;
  neko::PhysicsWorld world{kConfig};
  world.SetBSH(&quad_tree);
  world.SetContactListener(&listener);
  BuildScene(world);
  for (int i = 0; i < 30; ++i) {
    world.Step();
  }
  ASSERT_FALSE(world.touching_pairs().empty());

  neko::PhysicsWorld snapshot{kConfig};
  snapshot.CopyFrom(world);
  EXPECT_EQ(snapshot.GenerateChecksum(), world.GenerateChecksum());

  std::vector<std::uint32_t> checksums;
  listener.events = 0;
  for (int i = 0; i < 90; ++i) {
    world.Step();
    checksums.push_back(world.GenerateChecksum());
  }
  const int events = listener.events;
  EXPECT_GT(events, 0);

  world.CopyFrom(snapshot);
  listener.events = 0;
  for (int i = 0; i < 90; ++i) {
    world.Step();
    EXPECT_EQ(world.GenerateChecksum(), checksums[static_cast<std::size_t>(i)])
        << "step " << i;
  }
  EXPECT_EQ(listener.events, events);
}

// The same scene spinning: off-center colliders, angular velocities, inertia
// overrides and a force at a point every step (like a player input)
void BuildSpinningScene(neko::PhysicsWorld& world) {
  BuildScene(world);
  for (Index i = 1; i <= 16; ++i) {
    neko::Body& body = world.body(neko::BodyIndex{i});
    body.angular_velocity = static_cast<Scalar>(i % 5) - Scalar{2};
    body.rotation.set_angle(neko::Radian{static_cast<Scalar>(i)});
    if (i % 4 == 0) {
      body.inertia_mode = neko::InertiaMode::kOverride;
      body.inverse_inertia = Scalar{3};
    }
    world.collider(neko::ColliderIndex{i}).offset = {Scalar{0.1f}, Scalar{0}};
  }
}

void PushSpinningScene(neko::PhysicsWorld& world, int step) {
  const neko::BodyIndex pushed{static_cast<Index>(1 + step % 16)};
  world.ApplyForceAtPoint(
      pushed, {Scalar{0}, Scalar{5}},
      world.body(pushed).position + Vec2f{Scalar{0.25f}, Scalar{0}});
}

TEST(Checksum, WorldChecksumFollowsRotationState) {
  neko::PhysicsWorld a{kConfig};
  neko::PhysicsWorld b{kConfig};
  BuildSpinningScene(a);
  BuildSpinningScene(b);
  a.Step();
  b.Step();
  ASSERT_EQ(a.GenerateChecksum(), b.GenerateChecksum());
  const neko::BodyIndex index{3};
  const auto changed = [&a, &b] {
    return a.GenerateChecksum() != b.GenerateChecksum();
  };
  b.body(index).angular_velocity += Scalar{0.001f};
  EXPECT_TRUE(changed());
  b.CopyFrom(a);
  b.body(index).rotation.set_angle(a.body(index).rotation.angle() +
                                   neko::Radian{Scalar{0.001f}});
  EXPECT_TRUE(changed());
  b.CopyFrom(a);
  b.body(index).torque = Scalar{1};
  EXPECT_TRUE(changed());
  b.CopyFrom(a);
  b.body(index).inverse_inertia += Scalar{0.001f};
  EXPECT_TRUE(changed());
  b.CopyFrom(a);
  b.body(index).inertia_mode = neko::InertiaMode::kFixedRotation;
  EXPECT_TRUE(changed());
  b.CopyFrom(a);
  EXPECT_FALSE(changed());
}

TEST(Rollback, SpinningResimulationIsIdentical) {
  CountingListener listener;
  neko::PhysicsWorld world{kConfig};
  world.SetContactListener(&listener);
  BuildSpinningScene(world);
  for (int i = 0; i < 30; ++i) {
    PushSpinningScene(world, i);
    world.Step();
  }
  neko::PhysicsWorld snapshot{kConfig};
  snapshot.CopyFrom(world);

  std::vector<std::uint32_t> checksums;
  listener.events = 0;
  for (int i = 30; i < 120; ++i) {
    PushSpinningScene(world, i);
    world.Step();
    checksums.push_back(world.GenerateChecksum());
  }
  const int events = listener.events;
  EXPECT_GT(events, 0);
  // The scene really spins
  EXPECT_NE(world.body(neko::BodyIndex{1}).rotation.angle().value(),
            snapshot.body(neko::BodyIndex{1}).rotation.angle().value());

  world.CopyFrom(snapshot);
  listener.events = 0;
  for (int i = 30; i < 120; ++i) {
    PushSpinningScene(world, i);
    world.Step();
    EXPECT_EQ(world.GenerateChecksum(),
              checksums[static_cast<std::size_t>(i - 30)])
        << "step " << i;
  }
  EXPECT_EQ(listener.events, events);
}

// Allocate at init only: Step (default broadphase included) and CopyFrom must
// never allocate (DS review D3.1)
TEST(Rollback, StepAndCopyFromDoNotAllocate) {
  CountingAllocator allocator;
  auto world_config = kConfig;
  world_config.allocator = &allocator;
  CountingListener listener;
  neko::PhysicsWorld world{world_config};
  neko::PhysicsWorld snapshot{world_config};
  world.SetContactListener(&listener);
  BuildScene(world);
  world.Step();
  snapshot.CopyFrom(world);

  const int allocations_after_setup = allocator.allocation_count;
  for (int i = 0; i < 120; ++i) {
    world.Step();
    if (i % 10 == 0) {
      snapshot.CopyFrom(world);
    }
    if (i % 30 == 0) {
      world.CopyFrom(snapshot);
    }
  }
  EXPECT_GT(listener.events, 0);
  EXPECT_EQ(allocator.allocation_count, allocations_after_setup);
}

// Phases 4, 5 and 9 of the Splash Online plan (open point O3): one world
// with a one-way platform crossed and dropped through, a kinematic lift
// driven by SetKinematicTarget, a bullet teleported to fire through a
// trigger zone into a wall, and a seesaw on a limited revolute joint with a
// rope
namespace {
constexpr neko::PhysicsWorldConfig kFeatureConfig{.max_bodies = 64,
                                                  .max_colliders = 64,
                                                  .max_touching_pairs = 128,
                                                  .max_broadphase_pairs = 256,
                                                  .max_joints = 8,
                                                  .max_passing_pairs = 16};

constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

neko::ColliderIndex AddBox(neko::PhysicsWorld& world, neko::BodyIndex body,
                           Vec2f half_size, bool is_trigger = false) {
  const neko::ColliderIndex index = world.AddAabbCollider(body);
  world.collider(index).is_trigger = is_trigger;
  world.collider(index).restitution = Scalar{0};
  world.aabb(world.collider(index).shape_index).half_size = half_size;
  return index;
}

neko::BodyIndex AddStatic(neko::PhysicsWorld& world, Vec2f position) {
  const neko::BodyIndex body = world.AddBody();
  world.body(body).type = neko::BodyType::kStatic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = position;
  return body;
}

neko::BodyIndex AddCrate(neko::PhysicsWorld& world, Vec2f position) {
  const neko::BodyIndex body = world.AddBody();
  world.body(body).position = position;
  world.body(body).inertia_mode = neko::InertiaMode::kFixedRotation;
  AddBox(world, body, {Ratio(1, 4), Ratio(1, 4)});
  return body;
}

struct Rig {
  neko::BodyIndex jumper;
  neko::BodyIndex lift;
  neko::BodyIndex shot;
};

Rig BuildEveryFeature(neko::PhysicsWorld& world) {
  // One-way platform 4 m wide, 0.35 m thick, its top at y = 0
  const neko::BodyIndex platform = AddStatic(world, {Scalar{0}, -Ratio(7, 40)});
  world.collider(AddBox(world, platform, {Scalar{2}, Ratio(7, 40)}))
      .is_one_way = true;
  const neko::BodyIndex ground = AddStatic(world, {Scalar{0}, -Scalar{3}});
  AddBox(world, ground, {Scalar{8}, Ratio(1, 2)});
  // Jumper under the one-way platform
  const neko::BodyIndex jumper = AddCrate(world, {Scalar{-1}, -Ratio(9, 4)});
  // Kinematic lift with a crate, driven by SetKinematicTarget
  const neko::BodyIndex lift = world.AddBody();
  world.body(lift).type = neko::BodyType::kKinematic;
  world.body(lift).inverse_mass = Scalar{0};
  world.body(lift).position = {Scalar{4}, -Scalar{2}};
  AddBox(world, lift, {Ratio(1, 2), Ratio(1, 10)});
  AddCrate(world, {Scalar{4}, -Ratio(33, 20)});
  // Bullet shot crossing a trigger zone into a wall
  const neko::BodyIndex zone = AddStatic(world, {Scalar{-4}, Scalar{1}});
  AddBox(world, zone, {Ratio(1, 20), Ratio(1, 2)}, true);
  const neko::BodyIndex wall = AddStatic(world, {Scalar{-6}, Scalar{1}});
  AddBox(world, wall, {Ratio(1, 10), Scalar{2}});
  const neko::BodyIndex shot = world.AddBody();
  world.body(shot).gravity_scale = Scalar{0};
  world.body(shot).is_bullet = true;
  world.body(shot).position = {Scalar{3}, Scalar{1}};
  const neko::ColliderIndex shot_collider = world.AddCircleCollider(shot);
  world.collider(shot_collider).is_trigger = false;
  world.circle(world.collider(shot_collider).shape_index).radius = Ratio(1, 10);
  // Seesaw: plank on a revolute pivot with limits, rope to a weight
  const neko::BodyIndex pivot = AddStatic(world, {Scalar{0}, Scalar{3}});
  const neko::BodyIndex plank = world.AddBody();
  world.body(plank).position = {Scalar{0}, Scalar{3}};
  AddBox(world, plank, {Scalar{1}, Ratio(1, 10)});
  const auto hinge =
      world.AddRevoluteJoint(pivot, plank, {Scalar{0}, Scalar{3}});
  world.revolute_joint(hinge).enable_limit = true;
  world.revolute_joint(hinge).lower_angle = neko::Radian{-Ratio(1, 2)};
  world.revolute_joint(hinge).upper_angle = neko::Radian{Ratio(1, 2)};
  const neko::BodyIndex weight = AddCrate(world, {Scalar{1}, Scalar{2}});
  world.AddDistanceJoint(plank, weight);
  return {jumper, lift, shot};
}

void Drive(neko::PhysicsWorld& world, const Rig& rig, int step) {
  const int phase = step % 120;
  if (phase == 0) {
    world.body(rig.jumper).velocity.set_y(Scalar{7});
  }
  if (phase == 60) {
    world.DropThrough(rig.jumper);
  }
  if (phase == 10) {
    world.Teleport(rig.shot, {Scalar{3}, Scalar{1}}, neko::Radian{});
    world.body(rig.shot).velocity = {-Scalar{60}, Scalar{0}};
  }
  const Scalar y = phase < 60 ? Ratio(phase, 60) : Ratio(120 - phase, 60);
  world.SetKinematicTarget(rig.lift, {Scalar{4}, y - Scalar{2}},
                           neko::Radian{});
}

}  // namespace

TEST(Rollback, EveryFeatureReplaysWithoutAllocating) {
  CountingAllocator allocator;
  auto config = kFeatureConfig;
  config.allocator = &allocator;
  CountingListener listener;
  neko::PhysicsWorld world{config};
  neko::PhysicsWorld snapshot{config};
  world.SetContactListener(&listener);
  const Rig rig = BuildEveryFeature(world);
  Drive(world, rig, 0);
  world.Step();
  snapshot.CopyFrom(world);
  const int after_setup = allocator.allocation_count;

  std::vector<std::uint32_t> checksums;
  bool saw_passing = false;
  for (int step = 1; step < 480; ++step) {
    Drive(world, rig, step);
    world.Step();
    saw_passing = saw_passing || !world.passing_pairs().empty();
    checksums.push_back(world.GenerateChecksum());
    if (step == 200) {
      snapshot.CopyFrom(world);
    }
  }
  EXPECT_TRUE(saw_passing);
  EXPECT_GT(listener.events, 4);
  EXPECT_EQ(allocator.allocation_count, after_setup);

  // Roll back to step 200 (target set before it, mid-flight) and replay
  world.CopyFrom(snapshot);
  for (int step = 201; step < 480; ++step) {
    Drive(world, rig, step);
    world.Step();
    ASSERT_EQ(world.GenerateChecksum(),
              checksums[static_cast<std::size_t>(step - 1)])
        << "step " << step;
  }
  EXPECT_EQ(allocator.allocation_count, after_setup);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
