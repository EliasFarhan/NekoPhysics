// One-way colliders, the drop-through call, the enable flag and the integer
// user id of a collider (slice S2 of the night run of 2026-10-18).
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include <gtest/gtest.h>

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/scalar_traits.h"
#include "neko/physics/physics.h"

namespace {
using neko::BodyIndex;
using neko::ColliderIndex;
using neko::ColliderPair;
using neko::Radian;
using neko::Scalar;
using neko::Vec2f;

template <typename T>
float ToFloat(T value) {
  return neko::ScalarTraits<T>::ToFloat(value);
}

class CountingListener final : public neko::ContactListener {
 public:
  void OnTriggerEnter(const ColliderPair&) override { ++trigger_enter; }
  void OnTriggerExit(const ColliderPair&) override { ++trigger_exit; }
  void OnCollisionEnter(const ColliderPair&) override { ++collision_enter; }
  void OnCollisionExit(const ColliderPair&) override { ++collision_exit; }
  int trigger_enter = 0;
  int trigger_exit = 0;
  int collision_enter = 0;
  int collision_exit = 0;
};

constexpr neko::PhysicsWorldConfig kConfig{.max_bodies = 16,
                                           .max_colliders = 16,
                                           .max_touching_pairs = 32,
                                           .max_broadphase_pairs = 64,
                                           .max_joints = 0,
                                           .max_passing_pairs = 8};

// Half thickness of the platforms (0.35 m thick) and half size of the boxes
constexpr float kPlatformHalfHeight = 0.175f;
constexpr float kBoxHalf = 0.25f;
// Height of a box center resting on a platform at y = 0
constexpr float kRestHeight = kPlatformHalfHeight + kBoxHalf;

// A static one-way platform 4 m wide centered on `center`
ColliderIndex AddPlatform(neko::PhysicsWorld& world, Vec2f center,
                          bool is_one_way = true) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = neko::BodyType::kStatic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = center;
  const ColliderIndex index = world.AddAabbCollider(body);
  neko::Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.is_one_way = is_one_way;
  collider.restitution = Scalar{0};
  world.aabb(collider.shape_index).half_size = {Scalar{2},
                                                Scalar{kPlatformHalfHeight}};
  return index;
}

struct Box {
  BodyIndex body;
  ColliderIndex collider;
};

// A 0.5 m dynamic box of 1 kg that doesn't turn
Box AddBox(neko::PhysicsWorld& world, Vec2f position, Vec2f velocity = {}) {
  const BodyIndex body = world.AddBody();
  world.body(body).position = position;
  world.body(body).velocity = velocity;
  world.body(body).inertia_mode = neko::InertiaMode::kFixedRotation;
  const ColliderIndex index = world.AddAabbCollider(body);
  neko::Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  world.aabb(collider.shape_index).half_size = {Scalar{kBoxHalf},
                                                Scalar{kBoxHalf}};
  return {body, index};
}

void Steps(neko::PhysicsWorld& world, int count) {
  for (int i = 0; i < count; ++i) {
    world.Step();
  }
}

float Height(const neko::PhysicsWorld& world, BodyIndex body) {
  return ToFloat(world.body(body).position.y());
}
}  // namespace

// Falling from above, the box lands on the one-way platform like on a solid
// one: one enter event, never passing
TEST(OneWay, LandsFromAbove) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  const ColliderIndex platform = AddPlatform(world, {});
  const Box box = AddBox(world, {Scalar{0}, Scalar{2}});
  for (int i = 0; i < 120; ++i) {
    world.Step();
    EXPECT_FALSE(world.IsPassing({platform, box.collider})) << i;
  }
  EXPECT_NEAR(Height(world, box.body), kRestHeight, 0.01f);
  EXPECT_EQ(listener.collision_enter, 1);
  EXPECT_EQ(listener.collision_exit, 0);
  EXPECT_TRUE(world.passing_pairs().empty());
}

// Jumping from below, the box goes through the platform without a contact
// or an event, and lands on top of it on the way down
TEST(OneWay, JumpsThroughFromBelowAndLandsOnTop) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  const ColliderIndex platform = AddPlatform(world, {});
  const Box box =
      AddBox(world, {Scalar{0}, Scalar{-1.5f}}, {Scalar{0}, Scalar{8}});
  int passing_steps = 0;
  int events_while_passing = 0;
  for (int i = 0; i < 180; ++i) {
    world.Step();
    if (world.IsPassing({platform, box.collider})) {
      ++passing_steps;
      events_while_passing += listener.collision_enter;
      EXPECT_EQ(world.passing_pairs().size(), 1u);
    }
  }
  // About 0.85 m of overlap crossed at 6 to 7 m/s: 7 to 8 steps
  EXPECT_GE(passing_steps, 5);
  EXPECT_LE(passing_steps, 10);
  EXPECT_EQ(events_while_passing, 0);
  EXPECT_NEAR(Height(world, box.body), kRestHeight, 0.01f);
  EXPECT_EQ(listener.collision_enter, 1);
  EXPECT_EQ(listener.collision_exit, 0);
  EXPECT_TRUE(world.passing_pairs().empty());
}

// The same box against an ordinary platform bumps its head and falls back
TEST(OneWay, SolidPlatformBlocksFromBelow) {
  neko::PhysicsWorld world{kConfig};
  AddPlatform(world, {}, /*is_one_way=*/false);
  const Box box =
      AddBox(world, {Scalar{0}, Scalar{-1.5f}}, {Scalar{0}, Scalar{8}});
  float highest = -10.0f;
  for (int i = 0; i < 60; ++i) {
    world.Step();
    highest = std::max(highest, Height(world, box.body));
  }
  // Its center never got above the platform's center, and it falls back
  EXPECT_LT(highest, 0.0f);
  EXPECT_LT(Height(world, box.body), -1.0f);
}

// The one-way normal turns with its body: a platform turned upside down
// blocks from below and lets a falling box through
TEST(OneWay, NormalTurnsWithTheBody) {
  neko::PhysicsWorld world{kConfig};
  const ColliderIndex platform = AddPlatform(world, {});
  world.body(world.collider(platform).body_index)
      .rotation.set_angle(Radian{neko::Pi<Scalar>()});
  const Box falling = AddBox(world, {Scalar{-1}, Scalar{2}});
  // Falling up, toward the platform's bottom
  const Box rising = AddBox(world, {Scalar{1}, Scalar{-0.6f}});
  world.body(rising.body).gravity_scale = Scalar{-1};
  Steps(world, 120);
  EXPECT_LT(Height(world, falling.body), -2.0f);
  // Stopped under the platform, its top against the platform's bottom
  EXPECT_NEAR(Height(world, rising.body), -kRestHeight, 0.01f);
}

// The arc: a contact normal 80 degrees from the one-way normal is inside a
// 180 degree arc (90 degrees each side) and outside a 150 degree one (75)
TEST(OneWay, ArcDecidesTheTiltedNormals) {
  const auto run = [](float normal_degrees, float arc_degrees) {
    neko::PhysicsWorld world{kConfig};
    const ColliderIndex platform = AddPlatform(world, {});
    neko::Collider& collider = world.collider(platform);
    collider.one_way_normal = Vec2f::Up().Rotate(
        Radian{neko::Degree{Scalar{normal_degrees}}}.value());
    collider.one_way_arc = Radian{neko::Degree{Scalar{arc_degrees}}};
    const Box box = AddBox(world, {Scalar{0}, Scalar{1}});
    Steps(world, 90);
    return Height(world, box.body);
  };
  EXPECT_NEAR(run(80.0f, 180.0f), kRestHeight, 0.01f);
  EXPECT_LT(run(80.0f, 150.0f), -1.0f);
  EXPECT_NEAR(run(-80.0f, 180.0f), kRestHeight, 0.01f);
  EXPECT_LT(run(-80.0f, 150.0f), -1.0f);
  EXPECT_NEAR(run(70.0f, 150.0f), kRestHeight, 0.01f);
  // Normal pointing down: the top is the open side
  EXPECT_LT(run(180.0f, 180.0f), -1.0f);
}

// A side hit on a half plane is exactly at half the arc: it passes, with
// both scalars
TEST(OneWay, SideHitOnHalfPlanePasses) {
  neko::PhysicsWorld zero_gravity{
      {.gravity = Vec2f::Zero(), .max_bodies = 4, .max_colliders = 4}};
  const ColliderIndex platform = AddPlatform(zero_gravity, {});
  const Box box =
      AddBox(zero_gravity, {Scalar{-2.3f}, Scalar{0}}, {Scalar{3}, Scalar{0}});
  Steps(zero_gravity, 120);
  EXPECT_GT(ToFloat(zero_gravity.body(box.body).position.x()), 0.0f);
  EXPECT_TRUE(zero_gravity.passing_pairs().empty());
  // It was passing while it overlapped
  zero_gravity.body(box.body).position = {Scalar{-2.3f}, Scalar{0}};
  bool was_passing = false;
  for (int i = 0; i < 10; ++i) {
    zero_gravity.Step();
    was_passing =
        was_passing || zero_gravity.IsPassing({platform, box.collider});
  }
  EXPECT_TRUE(was_passing);
}

// The velocity rule: a shape that starts touching on the solid side but
// moves along the normal (it was crossing from below, found late) passes
TEST(OneWay, LeavingAlongTheNormalPasses) {
  const auto run = [](float vertical_speed) {
    neko::PhysicsWorld world{
        {.gravity = Vec2f::Zero(), .max_bodies = 4, .max_colliders = 4}};
    const ColliderIndex platform = AddPlatform(world, {});
    // Overlapping the top by 5 cm, center above the platform
    const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight - 0.05f}},
                           {Scalar{0}, Scalar{vertical_speed}});
    world.Step();
    return world.IsPassing({platform, box.collider});
  };
  EXPECT_TRUE(run(1.0f));
  EXPECT_FALSE(run(-1.0f));
  EXPECT_FALSE(run(0.0f));
}

// A moving one-way platform: the velocity rule is relative, so a platform
// rising into a box at rest catches it
TEST(OneWay, RisingPlatformCatchesARestingBox) {
  neko::PhysicsWorld world{
      {.gravity = Vec2f::Zero(), .max_bodies = 4, .max_colliders = 4}};
  const ColliderIndex platform = AddPlatform(world, {});
  neko::Body& lift = world.body(world.collider(platform).body_index);
  lift.type = neko::BodyType::kKinematic;
  lift.velocity = {Scalar{0}, Scalar{1}};
  const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight + 0.1f}});
  Steps(world, 30);
  EXPECT_FALSE(world.IsPassing({platform, box.collider}));
  EXPECT_GT(Height(world, box.body), kRestHeight + 0.3f);
}

// Two one-way colliders: both rules must say solid
TEST(OneWay, BothRulesApplyToTwoOneWayColliders) {
  const auto run = [](float box_normal_y) {
    neko::PhysicsWorld world{kConfig};
    AddPlatform(world, {});
    const Box box = AddBox(world, {Scalar{0}, Scalar{1}});
    neko::Collider& collider = world.collider(box.collider);
    collider.is_one_way = true;
    collider.one_way_normal = {Scalar{0}, Scalar{box_normal_y}};
    Steps(world, 90);
    return Height(world, box.body);
  };
  // The box's solid side faces down, toward the platform: solid
  EXPECT_NEAR(run(-1.0f), kRestHeight, 0.01f);
  // Its solid side faces up: the platform comes from its open side
  EXPECT_LT(run(1.0f), -1.0f);
}

// Triggers ignore the one-way flag
TEST(OneWay, TriggersAreUnaffected) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  AddPlatform(world, {});
  const Box box =
      AddBox(world, {Scalar{0}, Scalar{-1.5f}}, {Scalar{0}, Scalar{8}});
  world.collider(box.collider).is_trigger = true;
  Steps(world, 30);
  EXPECT_EQ(listener.trigger_enter, 1);
  EXPECT_EQ(listener.trigger_exit, 1);
  EXPECT_EQ(listener.collision_enter, 0);
  EXPECT_TRUE(world.passing_pairs().empty());
}

// Drop-through: the resting box falls through, with an exit event, and the
// pair leaves the table once the shapes are apart
TEST(OneWay, DropThroughFallsThroughThePlatform) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  const ColliderIndex platform = AddPlatform(world, {});
  const ColliderIndex lower = AddPlatform(world, {Scalar{0}, Scalar{-2}});
  const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight}});
  Steps(world, 30);
  ASSERT_NEAR(Height(world, box.body), kRestHeight, 0.01f);
  ASSERT_EQ(listener.collision_enter, 1);

  EXPECT_EQ(world.DropThrough(box.body), 1u);
  EXPECT_TRUE(world.IsPassing({platform, box.collider}));
  // A second call finds the pair already passing
  EXPECT_EQ(world.DropThrough(box.body), 0u);
  world.Step();
  EXPECT_EQ(listener.collision_exit, 1);
  EXPECT_TRUE(world.IsPassing({box.collider, platform}));
  Steps(world, 90);
  EXPECT_TRUE(world.passing_pairs().empty());
  // Landed on the lower one-way platform, not dropped through it
  EXPECT_NEAR(Height(world, box.body), -2.0f + kRestHeight, 0.01f);
  EXPECT_FALSE(world.IsPassing({lower, box.collider}));
  EXPECT_EQ(listener.collision_enter, 2);
}

// Drop-through only concerns one-way pairs of that body
TEST(OneWay, DropThroughIgnoresSolidGroundAndOtherBodies) {
  neko::PhysicsWorld world{kConfig};
  AddPlatform(world, {}, /*is_one_way=*/false);
  AddPlatform(world, {Scalar{5}, Scalar{0}});
  const Box on_solid = AddBox(world, {Scalar{0}, Scalar{kRestHeight}});
  const Box on_one_way = AddBox(world, {Scalar{5}, Scalar{kRestHeight}});
  Steps(world, 10);
  EXPECT_EQ(world.DropThrough(on_solid.body), 0u);
  EXPECT_TRUE(world.passing_pairs().empty());
  Steps(world, 30);
  EXPECT_NEAR(Height(world, on_solid.body), kRestHeight, 0.01f);
  EXPECT_NEAR(Height(world, on_one_way.body), kRestHeight, 0.01f);
}

// A passing pair keeps no warm-start impulse: the contact cache forgets it
TEST(OneWay, PassingPairHasNoContact) {
  neko::PhysicsWorld world{kConfig};
  const ColliderIndex platform = AddPlatform(world, {});
  const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight}});
  Steps(world, 10);
  ASSERT_EQ(world.touching_pairs().size(), 1u);
  world.DropThrough(box.body);
  world.Step();
  EXPECT_TRUE(world.touching_pairs().empty());
  EXPECT_TRUE(world.IsPassing({platform, box.collider}));
  // Falling freely: the step's velocity change is only gravity
  const float before = ToFloat(world.body(box.body).velocity.y());
  world.Step();
  EXPECT_NEAR(ToFloat(world.body(box.body).velocity.y()) - before,
              -9.81f / 60.0f, 1e-3f);
}

// Removing a collider forgets its passing pairs
TEST(OneWay, RemoveColliderForgetsPassingPairs) {
  neko::PhysicsWorld world{kConfig};
  const ColliderIndex platform = AddPlatform(world, {});
  const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight}});
  Steps(world, 10);
  world.DropThrough(box.body);
  ASSERT_EQ(world.passing_pairs().size(), 1u);
  world.RemoveCollider(platform);
  EXPECT_TRUE(world.passing_pairs().empty());
}

// Rollback: the passing table is copied and hashed, so a restored world
// replays a crossing bit for bit
TEST(OneWay, RollbackReplaysACrossing) {
  neko::PhysicsWorld world{kConfig};
  neko::PhysicsWorld snapshot{kConfig};
  const ColliderIndex platform = AddPlatform(world, {});
  const Box box =
      AddBox(world, {Scalar{0}, Scalar{-1.5f}}, {Scalar{0}, Scalar{8}});
  // Into the crossing
  int step = 0;
  while (!world.IsPassing({platform, box.collider}) && step < 60) {
    world.Step();
    ++step;
  }
  ASSERT_TRUE(world.IsPassing({platform, box.collider}));
  world.Step();
  snapshot.CopyFrom(world);
  EXPECT_EQ(snapshot.GenerateChecksum(), world.GenerateChecksum());
  EXPECT_TRUE(snapshot.IsPassing({platform, box.collider}));
  Steps(world, 120);
  const std::uint32_t reference = world.GenerateChecksum();
  const float reference_height = Height(world, box.body);

  // Diverge: forget the passing pair, the box is now stopped by the platform
  world.CopyFrom(snapshot);
  world.RemoveCollider(platform);
  Steps(world, 120);
  EXPECT_NE(world.GenerateChecksum(), reference);

  // Restore and replay
  world.CopyFrom(snapshot);
  Steps(world, 120);
  EXPECT_EQ(world.GenerateChecksum(), reference);
  EXPECT_EQ(Height(world, box.body), reference_height);
  EXPECT_NEAR(reference_height, kRestHeight, 0.01f);
}

// The passing table is part of the checksum: the same world with and without
// a passing pair differs
TEST(OneWay, PassingTableIsHashed) {
  neko::PhysicsWorld world{kConfig};
  neko::PhysicsWorld copy{kConfig};
  AddPlatform(world, {});
  const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight}});
  Steps(world, 10);
  copy.CopyFrom(world);
  ASSERT_EQ(copy.GenerateChecksum(), world.GenerateChecksum());
  world.DropThrough(box.body);
  EXPECT_NE(copy.GenerateChecksum(), world.GenerateChecksum());
}

// The one-way and enable fields are hashed only when used, user_id never
TEST(OneWay, ColliderChecksumOnlyWhenUsed) {
  const neko::Collider plain{};
  neko::Collider with_id{};
  with_id.user_id = 42;
  EXPECT_EQ(neko::GenerateChecksum(with_id), neko::GenerateChecksum(plain));
  neko::Collider tuned{};
  tuned.one_way_arc = Radian{Scalar{2}};
  EXPECT_EQ(neko::GenerateChecksum(tuned), neko::GenerateChecksum(plain));
  neko::Collider one_way{};
  one_way.is_one_way = true;
  EXPECT_NE(neko::GenerateChecksum(one_way), neko::GenerateChecksum(plain));
  neko::Collider narrow = one_way;
  narrow.one_way_arc = Radian{Scalar{2}};
  EXPECT_NE(neko::GenerateChecksum(narrow), neko::GenerateChecksum(one_way));
  neko::Collider disabled{};
  disabled.is_enabled = false;
  EXPECT_NE(neko::GenerateChecksum(disabled), neko::GenerateChecksum(plain));
}

// The defaults: not one-way, normal up, half plane, enabled, id 0
TEST(OneWay, Defaults) {
  const neko::Collider collider{};
  EXPECT_FALSE(collider.is_one_way);
  EXPECT_EQ(collider.one_way_normal, Vec2f::Up());
  EXPECT_EQ(collider.one_way_arc.value(), neko::Pi<Scalar>());
  EXPECT_TRUE(collider.is_enabled);
  EXPECT_EQ(collider.user_id, 0u);
}

// A disabled collider leaves the broadphase: the box falls through the
// ground, and the touching pair ends with an exit event
TEST(Enable, DisabledColliderIsSkipped) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  const ColliderIndex ground = AddPlatform(world, {}, /*is_one_way=*/false);
  const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight}});
  Steps(world, 10);
  ASSERT_EQ(listener.collision_enter, 1);
  ASSERT_EQ(world.BroadphasePairCount(), 1u);

  world.collider(ground).is_enabled = false;
  world.Step();
  EXPECT_EQ(listener.collision_exit, 1);
  EXPECT_EQ(world.BroadphasePairCount(), 0u);
  EXPECT_TRUE(world.touching_pairs().empty());
  Steps(world, 30);
  EXPECT_LT(Height(world, box.body), -0.5f);
}

// Enabled again, it is a new contact with an enter event
TEST(Enable, ReEnabledColliderComesBack) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  const ColliderIndex ground = AddPlatform(world, {}, /*is_one_way=*/false);
  world.collider(ground).is_enabled = false;
  const Box box = AddBox(world, {Scalar{0}, Scalar{1}});
  Steps(world, 6);
  world.collider(ground).is_enabled = true;
  Steps(world, 60);
  EXPECT_NEAR(Height(world, box.body), kRestHeight, 0.01f);
  EXPECT_EQ(listener.collision_enter, 1);
}

// Queries don't see a disabled collider
TEST(Enable, QueriesSkipDisabledColliders) {
  neko::PhysicsWorld world{kConfig};
  const ColliderIndex ground = AddPlatform(world, {}, /*is_one_way=*/false);
  std::array<ColliderIndex, 4> hits{};
  const neko::Aabbf box{{Scalar{-1}, Scalar{-1}}, {Scalar{1}, Scalar{1}}};
  EXPECT_EQ(world.OverlapAabb(box, hits), 1u);
  EXPECT_EQ(world.OverlapCircle(Vec2f::Zero(), Scalar{1}, hits), 1u);
  EXPECT_TRUE(
      world.RaycastClosest({Scalar{0}, Scalar{2}}, {Scalar{0}, Scalar{-2}})
          .has_value());
  world.collider(ground).is_enabled = false;
  EXPECT_EQ(world.OverlapAabb(box, hits), 0u);
  EXPECT_EQ(world.OverlapCircle(Vec2f::Zero(), Scalar{1}, hits), 0u);
  EXPECT_FALSE(
      world.RaycastClosest({Scalar{0}, Scalar{2}}, {Scalar{0}, Scalar{-2}})
          .has_value());
}

// A disabled collider has no mass in the inertia sum: a dumbbell with one
// weight disabled has the inertia of the other one alone
TEST(Enable, DisabledColliderLeavesTheInertia) {
  neko::PhysicsWorld world{kConfig};
  const BodyIndex body = world.AddBody();
  std::array<ColliderIndex, 2> weights{};
  for (std::size_t i = 0; i < 2; ++i) {
    weights[i] = world.AddCircleCollider(body);
    neko::Collider& collider = world.collider(weights[i]);
    collider.is_trigger = false;
    collider.offset = {Scalar{i == 0 ? 0.0f : 1.0f}, Scalar{0}};
    world.circle(collider.shape_index).radius = Scalar{0.25f};
  }
  const float both = ToFloat(world.ComputeInverseInertia(body));
  world.collider(weights[1]).is_enabled = false;
  const float one = ToFloat(world.ComputeInverseInertia(body));
  // m (r^2 / 2 + d^2) with m = 1 and r = 0.25: half the mass at d = 0 and
  // half at d = 1 for both, all of it at d = 0 for the centered one alone
  EXPECT_NEAR(both, 1.0f / (0.03125f + 0.5f), 1e-2f);
  EXPECT_NEAR(one, 1.0f / 0.03125f, 0.1f);
  // The world's own update agrees: same sums
  world.Step();
  EXPECT_EQ(world.body(body).inverse_inertia,
            world.ComputeInverseInertia(body));
  // A body whose colliders are all disabled doesn't spin
  world.collider(weights[0]).is_enabled = false;
  EXPECT_EQ(world.ComputeInverseInertia(body), Scalar{0});
}

// The enable flag is simulation state: copied and hashed
TEST(Enable, RollbackRestoresTheFlag) {
  neko::PhysicsWorld world{kConfig};
  neko::PhysicsWorld snapshot{kConfig};
  const ColliderIndex ground = AddPlatform(world, {}, /*is_one_way=*/false);
  const Box box = AddBox(world, {Scalar{0}, Scalar{kRestHeight}});
  Steps(world, 10);
  snapshot.CopyFrom(world);
  Steps(world, 30);
  const std::uint32_t reference = world.GenerateChecksum();
  world.CopyFrom(snapshot);
  world.collider(ground).is_enabled = false;
  EXPECT_NE(world.GenerateChecksum(), snapshot.GenerateChecksum());
  Steps(world, 30);
  EXPECT_LT(Height(world, box.body), 0.0f);
  world.CopyFrom(snapshot);
  EXPECT_TRUE(world.collider(ground).is_enabled);
  Steps(world, 30);
  EXPECT_EQ(world.GenerateChecksum(), reference);
}

// user_id is copied with the collider and left out of the checksum
TEST(UserId, CopiedButNotHashed) {
  neko::PhysicsWorld world{kConfig};
  neko::PhysicsWorld copy{kConfig};
  const ColliderIndex ground = AddPlatform(world, {});
  const Box box = AddBox(world, {Scalar{0}, Scalar{1}});
  world.collider(box.collider).user_id = 7;
  world.collider(ground).user_id = 0xFFFFFFFFu;
  copy.CopyFrom(world);
  const std::uint32_t checksum = world.GenerateChecksum();
  EXPECT_EQ(copy.collider(box.collider).user_id, 7u);
  EXPECT_EQ(copy.collider(ground).user_id, 0xFFFFFFFFu);
  world.collider(box.collider).user_id = 8;
  EXPECT_EQ(world.GenerateChecksum(), checksum);
  // A removed collider's slot comes back with id 0
  world.RemoveCollider(box.collider);
  const ColliderIndex reused = world.AddCircleCollider(box.body);
  EXPECT_EQ(reused, box.collider);
  EXPECT_EQ(world.collider(reused).user_id, 0u);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
