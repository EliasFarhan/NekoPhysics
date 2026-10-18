// Kinematic targets and teleports (slice S3 of the night run of 2026-10-18):
// PhysicsWorld::SetKinematicTarget and PhysicsWorld::Teleport.
#include <array>
#include <cmath>
#include <cstdint>

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
  void OnTriggerEnter(const ColliderPair&) override {}
  void OnTriggerExit(const ColliderPair&) override {}
  void OnCollisionEnter(const ColliderPair& pair) override {
    ++enter;
    last_enter = pair;
  }
  void OnCollisionExit(const ColliderPair& pair) override {
    ++exit;
    last_exit = pair;
  }
  int enter = 0;
  int exit = 0;
  ColliderPair last_enter{};
  ColliderPair last_exit{};
};

constexpr neko::PhysicsWorldConfig kConfig{.max_bodies = 16,
                                           .max_colliders = 16,
                                           .max_touching_pairs = 32,
                                           .max_broadphase_pairs = 64,
                                           .max_joints = 4,
                                           .max_passing_pairs = 8};

constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

struct Box {
  BodyIndex body;
  ColliderIndex collider;
};

// A solid box (AabbCollider) of the given type, no bounce
Box AddBox(neko::PhysicsWorld& world, neko::BodyType type, Vec2f position,
           Vec2f half_size) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = type;
  world.body(body).position = position;
  if (type == neko::BodyType::kDynamic) {
    world.body(body).inertia_mode = neko::InertiaMode::kFixedRotation;
  } else {
    world.body(body).inverse_mass = Scalar{0};
  }
  const ColliderIndex index = world.AddAabbCollider(body);
  neko::Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  world.aabb(collider.shape_index).half_size = half_size;
  return {body, index};
}

void Steps(neko::PhysicsWorld& world, int count) {
  for (int i = 0; i < count; ++i) {
    world.Step();
  }
}

// The path of the platform tests: a circle of radius 1.5 m around (2, 3),
// one turn every 4 s, the platform turning with it
Vec2f PathPoint(int step) {
  const Radian phase{static_cast<Scalar>(step) * neko::Pi<Scalar>() /
                     Scalar{120}};
  return Vec2f{Scalar{2}, Scalar{3}} +
         Vec2f{neko::Cos(phase), neko::Sin(phase)} * Ratio(3, 2);
}
Radian PathAngle(int step) {
  return Radian{static_cast<Scalar>(step) * neko::Pi<Scalar>() / Scalar{120}};
}
}  // namespace

// One step reaches the target exactly, with the velocities that cover the
// distance in one step
TEST(KinematicTarget, ReachesTheTargetInOneStep) {
  neko::PhysicsWorld world{kConfig};
  const Box platform =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{1}, Ratio(1, 5)});
  const Vec2f target{Ratio(1234, 1000), Ratio(1, 2)};
  const Radian angle{Ratio(3, 10)};
  world.SetKinematicTarget(platform.body, target, angle);
  const neko::Body& body = world.body(platform.body);
  EXPECT_EQ(body.velocity, target / world.dt());
  EXPECT_EQ(body.angular_velocity, angle.value() / world.dt());
  EXPECT_NEAR(ToFloat(body.velocity.x()), 1.234f * 60.0f, 0.1f);
  world.Step();
  EXPECT_EQ(body.position, target);
  EXPECT_EQ(body.rotation.angle().value(), angle.value());
  EXPECT_EQ(body.rotation.sin(), neko::Sin(angle));
  // The velocity stays: without a new target the platform keeps moving
  const Vec2f velocity = body.velocity;
  world.Step();
  EXPECT_EQ(body.velocity, velocity);
  EXPECT_GT(ToFloat(body.position.x()), 2.0f);
}

// Driven every step along a path, the platform is on the path after every
// step: no drift from rounding velocity * dt
TEST(KinematicTarget, FollowsAPathWithoutDrift) {
  neko::PhysicsWorld world{kConfig};
  const Box platform = AddBox(world, neko::BodyType::kKinematic, PathPoint(0),
                              {Scalar{1}, Ratio(1, 5)});
  for (int step = 1; step <= 600; ++step) {
    world.SetKinematicTarget(platform.body, PathPoint(step), PathAngle(step));
    world.Step();
    ASSERT_EQ(world.body(platform.body).position, PathPoint(step)) << step;
    ASSERT_EQ(world.body(platform.body).rotation.angle().value(),
              neko::WrapAngle(PathAngle(step)).value())
        << step;
  }
}

// The angle goes the short way around: from 3 rad to -3 rad is 0.28 rad
// counter-clockwise, not 6 rad clockwise
TEST(KinematicTarget, AngleTakesTheShortWay) {
  neko::PhysicsWorld world{kConfig};
  const Box platform =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{1}, Ratio(1, 5)});
  world.body(platform.body).rotation.set_angle(Radian{Scalar{3}});
  world.SetKinematicTarget(platform.body, {}, Radian{Scalar{-3}});
  const float turn =
      ToFloat(world.body(platform.body).angular_velocity * world.dt());
  EXPECT_NEAR(turn, 2.0f * 3.14159265f - 6.0f, 1e-3f);
  world.Step();
  EXPECT_EQ(world.body(platform.body).rotation.angle().value(), Scalar{-3});
}

// Only kinematic bodies: a dynamic or static body is left as it is
TEST(KinematicTarget, IgnoresOtherBodyTypes) {
  neko::PhysicsWorld world{kConfig};
  const Box crate =
      AddBox(world, neko::BodyType::kDynamic, {}, {Ratio(1, 5), Ratio(1, 5)});
  const Box wall = AddBox(world, neko::BodyType::kStatic,
                          {Scalar{3}, Scalar{0}}, {Ratio(1, 5), Scalar{1}});
  const std::uint32_t checksum = world.GenerateChecksum();
  world.SetKinematicTarget(crate.body, {Scalar{1}, Scalar{1}}, Radian{});
  world.SetKinematicTarget(wall.body, {Scalar{1}, Scalar{1}}, Radian{});
  EXPECT_EQ(world.GenerateChecksum(), checksum);
  world.Step();
  // One step of gravity, nothing more
  EXPECT_EQ(world.body(crate.body).velocity.x(), Scalar{0});
  EXPECT_LT(ToFloat(world.body(crate.body).velocity.y()), 0.0f);
  EXPECT_EQ(world.body(wall.body).position, (Vec2f{Scalar{3}, Scalar{0}}));
}

// The second call before a step replaces the first target
TEST(KinematicTarget, LastCallWins) {
  neko::PhysicsWorld world{kConfig};
  const Box platform =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{1}, Ratio(1, 5)});
  world.SetKinematicTarget(platform.body, {Scalar{1}, Scalar{0}}, Radian{});
  world.SetKinematicTarget(platform.body, {Scalar{0}, Scalar{2}}, Radian{});
  world.Step();
  EXPECT_EQ(world.body(platform.body).position, (Vec2f{Scalar{0}, Scalar{2}}));
}

// A crate resting on a lift that moves sideways at 1.5 m/s is carried by
// friction: the solver sees the lift's velocity
TEST(KinematicTarget, CarriesACrateByFriction) {
  neko::PhysicsWorld world{kConfig};
  const Box lift =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{3}, Ratio(1, 5)});
  const Box crate =
      AddBox(world, neko::BodyType::kDynamic, {Scalar{0}, Ratio(2, 5)},
             {Ratio(1, 5), Ratio(1, 5)});
  Steps(world, 30);  // settle
  const float start = ToFloat(world.body(crate.body).position.x());
  const Scalar per_step = Ratio(3, 2) * world.dt();
  for (int step = 1; step <= 60; ++step) {
    world.SetKinematicTarget(
        lift.body, {per_step * static_cast<Scalar>(step), Scalar{0}}, Radian{});
    world.Step();
  }
  const float lift_moved = ToFloat(world.body(lift.body).position.x());
  const float crate_moved =
      ToFloat(world.body(crate.body).position.x()) - start;
  EXPECT_NEAR(lift_moved, 1.5f, 0.01f);
  // Friction 0.5 accelerates the crate at up to 4.9 m/s^2: it catches up
  // with the lift in about 0.3 s and rides it from then on
  EXPECT_GT(crate_moved, 1.2f);
  EXPECT_LT(crate_moved, lift_moved);
  EXPECT_NEAR(ToFloat(world.body(crate.body).velocity.x()), 1.5f, 0.05f);
}

// A pending target is simulation state: copied and hashed, so a rollback
// restored between SetKinematicTarget and Step replays the same step
TEST(KinematicTarget, CopiedAndHashed) {
  neko::PhysicsWorld world{kConfig};
  const Box platform =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{1}, Ratio(1, 5)});
  AddBox(world, neko::BodyType::kDynamic, {Scalar{0}, Ratio(2, 5)},
         {Ratio(1, 5), Ratio(1, 5)});
  Steps(world, 10);
  neko::PhysicsWorld snapshot{kConfig};
  // The same velocities set by hand: only the pending target differs
  neko::PhysicsWorld by_hand{kConfig};
  by_hand.CopyFrom(world);
  world.SetKinematicTarget(platform.body, {Ratio(1, 50), Ratio(1, 100)},
                           Radian{Ratio(1, 100)});
  const std::uint32_t with_target = world.GenerateChecksum();
  snapshot.CopyFrom(world);
  EXPECT_EQ(snapshot.GenerateChecksum(), with_target);
  by_hand.body(platform.body).velocity = world.body(platform.body).velocity;
  by_hand.body(platform.body).angular_velocity =
      world.body(platform.body).angular_velocity;
  EXPECT_NE(by_hand.GenerateChecksum(), with_target);
  world.Step();
  const std::uint32_t after = world.GenerateChecksum();
  // Diverge, then restore and replay
  world.SetKinematicTarget(platform.body, {Scalar{-1}, Scalar{0}}, Radian{});
  world.Step();
  EXPECT_NE(world.GenerateChecksum(), after);
  world.CopyFrom(snapshot);
  world.Step();
  EXPECT_EQ(world.GenerateChecksum(), after);
  EXPECT_EQ(world.body(platform.body).position,
            (Vec2f{Ratio(1, 50), Ratio(1, 100)}));
  // Emptied by the step: after it, the world without a target has the same
  // checksum as the one moved by hand and snapped to the same place
  by_hand.Step();
  by_hand.body(platform.body).position = world.body(platform.body).position;
  by_hand.body(platform.body).rotation = world.body(platform.body).rotation;
  EXPECT_EQ(by_hand.GenerateChecksum(), world.GenerateChecksum());
}

// Removing the body or clearing the world forgets its target
TEST(KinematicTarget, RemoveBodyAndClearForgetTheTarget) {
  neko::PhysicsWorld world{kConfig};
  const Box platform =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{1}, Ratio(1, 5)});
  const std::uint32_t empty = [] {
    neko::PhysicsWorld fresh{kConfig};
    return fresh.GenerateChecksum();
  }();
  world.SetKinematicTarget(platform.body, {Scalar{1}, Scalar{0}}, Radian{});
  world.RemoveBody(platform.body);
  // A new body in the same slot doesn't inherit it
  const Box other =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{1}, Ratio(1, 5)});
  ASSERT_EQ(other.body, platform.body);
  world.Step();
  EXPECT_EQ(world.body(other.body).position, Vec2f{});
  world.SetKinematicTarget(other.body, {Scalar{1}, Scalar{0}}, Radian{});
  world.Clear();
  EXPECT_EQ(world.GenerateChecksum(), empty);
}

namespace {
// Two grounds side by side with a wall between them, and a crate resting on
// the left ground
struct WallScene {
  Box left_ground;
  Box right_ground;
  Box wall;
  Box crate;
};

WallScene AddWallScene(neko::PhysicsWorld& world) {
  WallScene scene{};
  scene.left_ground = AddBox(world, neko::BodyType::kStatic,
                             {Scalar{-3}, Scalar{0}}, {Scalar{2}, Ratio(1, 2)});
  scene.right_ground = AddBox(world, neko::BodyType::kStatic,
                              {Scalar{3}, Scalar{0}}, {Scalar{2}, Ratio(1, 2)});
  scene.wall = AddBox(world, neko::BodyType::kStatic, {Scalar{0}, Scalar{2}},
                      {Ratio(1, 4), Scalar{3}});
  scene.crate = AddBox(world, neko::BodyType::kDynamic,
                       {Scalar{-3}, Ratio(3, 4)}, {Ratio(1, 4), Ratio(1, 4)});
  return scene;
}

// A stack of `count` crates on the scene's crate, at x = -3
void AddStack(neko::PhysicsWorld& world, int count) {
  for (int i = 1; i <= count; ++i) {
    AddBox(world, neko::BodyType::kDynamic,
           {Scalar{-3}, Ratio(3, 4) + Ratio(1, 2) * static_cast<Scalar>(i)},
           {Ratio(1, 4), Ratio(1, 4)});
  }
}

const Vec2f kAcrossTheWall{Scalar{3}, Ratio(3, 4)};
}  // namespace

// Queries made right after the teleport see the body at its new place
TEST(Teleport, QueriesSeeTheNewPlace) {
  neko::PhysicsWorld world{kConfig};
  const WallScene scene = AddWallScene(world);
  Steps(world, 30);
  world.Teleport(scene.crate.body, kAcrossTheWall, Radian{});
  std::array<ColliderIndex, 4> hits{};
  EXPECT_EQ(world.OverlapCircle(kAcrossTheWall, Ratio(1, 10), hits), 1u);
  EXPECT_EQ(hits[0], scene.crate.collider);
  EXPECT_EQ(world.OverlapCircle({Scalar{-3}, Ratio(3, 4)}, Ratio(1, 10), hits),
            0u);
}

// Across a wall: the next step sends the exit event of the old ground and
// the enter event of the new one, and the crate rests there
TEST(Teleport, EventsAcrossAWall) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  const WallScene scene = AddWallScene(world);
  Steps(world, 30);
  ASSERT_EQ(listener.enter, 1);
  world.Teleport(scene.crate.body, kAcrossTheWall, Radian{});
  // Nothing until the next step
  EXPECT_EQ(listener.exit, 0);
  world.Step();
  EXPECT_EQ(listener.exit, 1);
  EXPECT_EQ(listener.last_exit,
            (ColliderPair{scene.left_ground.collider, scene.crate.collider}));
  EXPECT_EQ(listener.enter, 2);
  EXPECT_EQ(listener.last_enter,
            (ColliderPair{scene.right_ground.collider, scene.crate.collider}));
  Steps(world, 30);
  EXPECT_NEAR(ToFloat(world.body(scene.crate.body).position.x()), 3.0f, 1e-4f);
  EXPECT_NEAR(ToFloat(world.body(scene.crate.body).position.y()), 0.75f, 0.01f);
  EXPECT_EQ(listener.enter, 2);
  EXPECT_EQ(listener.exit, 1);
}

// The bottom box of a stack of three turning boxes (OrientedBoxCollider),
// moved elsewhere on the same ground: its cached ground impulses hold the
// stack's weight, split over two points. Teleported, it starts without them
// and barely turns; moved by hand (position set directly) the stale
// impulses match the new contact (same pair, same feature ids). The solver
// clamps most of them away, but not evenly over the two points: the box
// gets a spin
TEST(Teleport, ClearsTheWarmStart) {
  const auto run = [](bool use_teleport) {
    neko::PhysicsWorldConfig config = kConfig;
    config.velocity_iterations = 2;
    neko::PhysicsWorld world{config};
    AddBox(world, neko::BodyType::kStatic, {}, {Scalar{6}, Ratio(1, 2)});
    BodyIndex bottom{};
    for (int i = 0; i <= 3; ++i) {
      const BodyIndex body = world.AddBody();
      world.body(body).position = {
          Scalar{-3} + Ratio(i * 3, 100),
          Ratio(3, 4) + Ratio(1, 2) * static_cast<Scalar>(i)};
      const ColliderIndex collider = world.AddOrientedBoxCollider(body);
      world.collider(collider).is_trigger = false;
      world.collider(collider).restitution = Scalar{0};
      world.oriented_box(world.collider(collider).shape_index).half_size = {
          Ratio(1, 4), Ratio(1, 4)};
      if (i == 0) {
        bottom = body;
      }
    }
    Steps(world, 180);
    const Vec2f there{Scalar{3}, world.body(bottom).position.y()};
    if (use_teleport) {
      world.Teleport(bottom, there, world.body(bottom).rotation.angle());
    } else {
      world.body(bottom).position = there;
    }
    float spin = 0.0f;
    for (int i = 0; i < 30; ++i) {
      world.Step();
      spin = std::fmax(spin,
                       std::fabs(ToFloat(world.body(bottom).angular_velocity)));
    }
    return spin;
  };
  const float teleported = run(true);
  const float by_hand = run(false);
  EXPECT_LT(teleported, 0.05f);
  EXPECT_GT(by_hand, 3.0f * teleported);
}

// The steps after a teleport are the same as for a body created there with
// the same velocity: nothing of the old place is left
TEST(Teleport, SameAsCreatedThere) {
  neko::PhysicsWorld world{kConfig};
  const WallScene scene = AddWallScene(world);
  Steps(world, 30);
  neko::PhysicsWorld fresh{kConfig};
  const WallScene fresh_scene = AddWallScene(fresh);
  world.body(scene.crate.body).velocity = {Ratio(1, 2), Scalar{0}};
  world.Teleport(scene.crate.body, kAcrossTheWall, Radian{});
  fresh.body(fresh_scene.crate.body).position = kAcrossTheWall;
  fresh.body(fresh_scene.crate.body).velocity = {Ratio(1, 2), Scalar{0}};
  for (int i = 0; i < 10; ++i) {
    world.Step();
    fresh.Step();
    ASSERT_EQ(world.body(scene.crate.body).position,
              fresh.body(fresh_scene.crate.body).position)
        << i;
    ASSERT_EQ(world.body(scene.crate.body).velocity,
              fresh.body(fresh_scene.crate.body).velocity)
        << i;
  }
}

// The velocity is kept: a falling crate teleported up keeps falling at the
// same speed
TEST(Teleport, KeepsTheVelocity) {
  neko::PhysicsWorld world{kConfig};
  const Box crate = AddBox(world, neko::BodyType::kDynamic,
                           {Scalar{0}, Scalar{5}}, {Ratio(1, 4), Ratio(1, 4)});
  world.body(crate.body).angular_velocity = Scalar{2};
  Steps(world, 20);
  const Vec2f velocity = world.body(crate.body).velocity;
  world.Teleport(crate.body, {Scalar{1}, Scalar{8}}, Radian{Scalar{1}});
  EXPECT_EQ(world.body(crate.body).velocity, velocity);
  EXPECT_EQ(world.body(crate.body).angular_velocity, Scalar{2});
  EXPECT_EQ(world.body(crate.body).position, (Vec2f{Scalar{1}, Scalar{8}}));
  EXPECT_EQ(world.body(crate.body).rotation.angle().value(), Scalar{1});
}

// A pair passing through a one-way platform is forgotten: at its new place
// the one-way rule decides again
TEST(Teleport, ClearsPassingPairs) {
  neko::PhysicsWorld world{kConfig};
  const Box platform =
      AddBox(world, neko::BodyType::kStatic, {}, {Scalar{2}, Ratio(7, 40)});
  world.collider(platform.collider).is_one_way = true;
  const Box crate =
      AddBox(world, neko::BodyType::kDynamic, {Scalar{0}, Ratio(-3, 5)},
             {Ratio(1, 4), Ratio(1, 4)});
  world.body(crate.body).velocity = {Scalar{0}, Scalar{5}};
  const ColliderPair pair{platform.collider, crate.collider};
  for (int step = 0; step < 30 && !world.IsPassing(pair); ++step) {
    world.Step();
  }
  ASSERT_TRUE(world.IsPassing(pair));
  // Into the top of the platform, falling: a new pair from the solid side,
  // it lands
  world.Teleport(crate.body, {Scalar{0}, Ratio(2, 5)}, Radian{});
  EXPECT_FALSE(world.IsPassing(pair));
  world.body(crate.body).velocity = {Scalar{0}, Scalar{-1}};
  Steps(world, 60);
  EXPECT_FALSE(world.IsPassing(pair));
  EXPECT_NEAR(ToFloat(world.body(crate.body).position.y()), 0.425f, 0.01f);
}

// A teleport drops the body's pending target, and its joints restart from
// no impulse
TEST(Teleport, DropsTheTargetAndJointImpulses) {
  neko::PhysicsWorld world{kConfig};
  const Box platform =
      AddBox(world, neko::BodyType::kKinematic, {}, {Scalar{1}, Ratio(1, 5)});
  world.SetKinematicTarget(platform.body, {Scalar{1}, Scalar{0}}, Radian{});
  world.Teleport(platform.body, {Scalar{5}, Scalar{0}}, Radian{});
  world.body(platform.body).velocity = Vec2f{};
  world.Step();
  EXPECT_EQ(world.body(platform.body).position, (Vec2f{Scalar{5}, Scalar{0}}));

  const Box pivot =
      AddBox(world, neko::BodyType::kStatic, {Scalar{0}, Scalar{6}},
             {Ratio(1, 10), Ratio(1, 10)});
  const Box bob = AddBox(world, neko::BodyType::kDynamic,
                         {Scalar{0}, Scalar{4}}, {Ratio(1, 10), Ratio(1, 10)});
  const neko::JointIndex rod = world.AddDistanceJoint(pivot.body, bob.body);
  Steps(world, 10);
  ASSERT_NE(world.joint(rod).impulse, Scalar{0});
  world.Teleport(bob.body, {Scalar{2}, Scalar{6}}, Radian{});
  EXPECT_EQ(world.joint(rod).impulse, Scalar{0});
}

// Rollback across a teleport: restoring a snapshot taken before it and
// replaying gives the same checksum and the same events
TEST(Teleport, RollbackReplaysTheTeleport) {
  neko::PhysicsWorld world{kConfig};
  CountingListener listener;
  world.SetContactListener(&listener);
  const WallScene scene = AddWallScene(world);
  AddStack(world, 2);
  Steps(world, 40);
  neko::PhysicsWorld snapshot{kConfig};
  snapshot.CopyFrom(world);
  const std::uint32_t before = world.GenerateChecksum();
  const auto replay = [&world, &scene, &listener] {
    listener = CountingListener{};
    world.Teleport(scene.crate.body, kAcrossTheWall, Radian{});
    Steps(world, 20);
    return std::array<std::uint32_t, 3>{
        world.GenerateChecksum(), static_cast<std::uint32_t>(listener.enter),
        static_cast<std::uint32_t>(listener.exit)};
  };
  const auto first = replay();
  EXPECT_GT(first[1] + first[2], 0u);
  // Diverge: no teleport
  world.CopyFrom(snapshot);
  Steps(world, 20);
  EXPECT_NE(world.GenerateChecksum(), first[0]);
  world.CopyFrom(snapshot);
  EXPECT_EQ(world.GenerateChecksum(), before);
  world.Teleport(scene.crate.body, kAcrossTheWall, Radian{});
  EXPECT_NE(world.GenerateChecksum(), before);
  world.CopyFrom(snapshot);
  EXPECT_EQ(replay(), first);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
