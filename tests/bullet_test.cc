// Selective continuous collision (slice S4 of the night run of 2026-10-18):
// Body::is_bullet, the sweep of its solid colliders against static and
// kinematic ones and of its trigger pairs against every collider.
#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

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
using neko::ShapeType;
using neko::Vec2f;

template <typename T>
float ToFloat(T value) {
  return neko::ScalarTraits<T>::ToFloat(value);
}

constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

// Records every event in order
class RecordingListener final : public neko::ContactListener {
 public:
  enum class Kind : std::uint8_t {
    kTriggerEnter,
    kTriggerExit,
    kCollisionEnter,
    kCollisionExit
  };
  void OnTriggerEnter(const ColliderPair& pair) override {
    events.emplace_back(Kind::kTriggerEnter, pair);
  }
  void OnTriggerExit(const ColliderPair& pair) override {
    events.emplace_back(Kind::kTriggerExit, pair);
  }
  void OnCollisionEnter(const ColliderPair& pair) override {
    events.emplace_back(Kind::kCollisionEnter, pair);
  }
  void OnCollisionExit(const ColliderPair& pair) override {
    events.emplace_back(Kind::kCollisionExit, pair);
  }
  [[nodiscard]] int Count(Kind kind) const {
    int count = 0;
    for (const auto& event : events) {
      count += event.first == kind ? 1 : 0;
    }
    return count;
  }
  std::vector<std::pair<Kind, ColliderPair>> events;
};

constexpr neko::PhysicsWorldConfig kConfig{.max_bodies = 16,
                                           .max_colliders = 16,
                                           .max_touching_pairs = 32,
                                           .max_broadphase_pairs = 64,
                                           .max_joints = 0,
                                           .max_passing_pairs = 8};

struct Shape {
  BodyIndex body;
  ColliderIndex collider;
};

// A solid collider of `type` and half size (a circle uses half_size.x() as
// its radius) on a new body; no bounce
Shape AddShape(neko::PhysicsWorld& world, neko::BodyType type, ShapeType shape,
               Vec2f position, Vec2f half_size) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = type;
  world.body(body).position = position;
  if (type == neko::BodyType::kDynamic) {
    world.body(body).inertia_mode = neko::InertiaMode::kFixedRotation;
  } else {
    world.body(body).inverse_mass = Scalar{0};
  }
  ColliderIndex index = neko::kInvalidColliderIndex;
  switch (shape) {
    case ShapeType::kCircle:
      index = world.AddCircleCollider(body);
      world.circle(world.collider(index).shape_index).radius = half_size.x();
      break;
    case ShapeType::kAabb:
      index = world.AddAabbCollider(body);
      world.aabb(world.collider(index).shape_index).half_size = half_size;
      break;
    default:
      index = world.AddOrientedBoxCollider(body);
      world.oriented_box(world.collider(index).shape_index).half_size =
          half_size;
      break;
  }
  world.collider(index).is_trigger = false;
  world.collider(index).restitution = Scalar{0};
  return {body, index};
}

// The platform of the tests: 0.35 m thick, 4 m wide, its top at y = 0
Shape AddPlatform(neko::PhysicsWorld& world, ShapeType shape = ShapeType::kAabb,
                  neko::BodyType type = neko::BodyType::kStatic) {
  return AddShape(world, type, shape, {Scalar{0}, -Ratio(7, 40)},
                  {Scalar{2}, Ratio(7, 40)});
}

// A small shot: a circle of radius 0.1 m, or a box or oriented box of the
// same half size
Shape AddShot(neko::PhysicsWorld& world, ShapeType shape, Vec2f position,
              Vec2f velocity, bool is_bullet) {
  const Shape shot = AddShape(world, neko::BodyType::kDynamic, shape, position,
                              {Ratio(1, 10), Ratio(1, 10)});
  world.body(shot.body).velocity = velocity;
  world.body(shot.body).is_bullet = is_bullet;
  return shot;
}

void Steps(neko::PhysicsWorld& world, int count) {
  for (int i = 0; i < count; ++i) {
    world.Step();
  }
}

class BulletSpeedTest
    : public ::testing::TestWithParam<std::tuple<int, ShapeType>> {};

// A shot falling at 30, 50 and 80 m/s (0.5, 0.83 and 1.33 m per step) onto
// the 0.35 m platform lands on it as a bullet, whatever its shape; at 50
// and 80 m/s the same shot without the flag passes through
TEST_P(BulletSpeedTest, NoTunnelingThroughThePlatform) {
  const auto [speed, shape] = GetParam();
  for (const bool is_bullet : {true, false}) {
    neko::PhysicsWorld world{kConfig};
    AddPlatform(world);
    // 3.1 m above the top: the steps don't land exactly on it
    const Shape shot =
        AddShot(world, shape, {Scalar{0}, Ratio(31, 10)},
                {Scalar{0}, -static_cast<Scalar>(speed)}, is_bullet);
    Steps(world, 60);
    const float y = ToFloat(world.body(shot.body).position.y());
    if (is_bullet) {
      EXPECT_NEAR(y, 0.1f, 0.006f) << speed;
      EXPECT_NEAR(ToFloat(world.body(shot.body).velocity.y()), 0.0f, 0.01f)
          << speed;
    } else if (speed > 30) {
      EXPECT_LT(y, -1.0f) << speed;
    }
  }
}

// The same against a 0.38 m wide kinematic player box, shot sideways
TEST_P(BulletSpeedTest, NoTunnelingThroughThePlayer) {
  const auto [speed, shape] = GetParam();
  for (const bool is_bullet : {true, false}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                              .max_bodies = 4,
                              .max_colliders = 4,
                              .max_touching_pairs = 8,
                              .max_broadphase_pairs = 8,
                              .max_joints = 0}};
    AddShape(world, neko::BodyType::kKinematic, ShapeType::kAabb,
             {Scalar{0}, Ratio(3, 10)}, {Ratio(19, 100), Ratio(3, 10)});
    const Shape shot =
        AddShot(world, shape, {-Ratio(31, 10), Ratio(3, 10)},
                {static_cast<Scalar>(speed), Scalar{0}}, is_bullet);
    Steps(world, 30);
    const float x = ToFloat(world.body(shot.body).position.x());
    if (is_bullet) {
      // Stopped against the left side, at -0.19 - 0.1
      EXPECT_NEAR(x, -0.29f, 0.006f) << speed;
    } else if (speed > 30) {
      EXPECT_GT(x, 1.0f) << speed;
    }
  }
}

INSTANTIATE_TEST_SUITE_P(
    Speeds, BulletSpeedTest,
    ::testing::Combine(::testing::Values(30, 50, 80),
                       ::testing::Values(ShapeType::kCircle, ShapeType::kAabb,
                                         ShapeType::kOrientedBox)));

// The step that finds the impact puts the shot where it touches, half the
// slop into the platform, and keeps its velocity; the next step's contact
// stops it (no bounce) and sends the collision enter
TEST(Bullet, StopsAtTheTimeOfImpact) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                            .max_bodies = 4,
                            .max_colliders = 4,
                            .max_touching_pairs = 8,
                            .max_broadphase_pairs = 8,
                            .max_joints = 0}};
  RecordingListener listener;
  world.SetContactListener(&listener);
  AddPlatform(world);
  // 0.5 m above at 60 m/s: the impact is at 0.4 m of the 1 m of the step
  const Shape shot =
      AddShot(world, ShapeType::kCircle, {Scalar{0}, Ratio(1, 2)},
              {Scalar{0}, -Scalar{60}}, true);
  world.Step();
  const neko::Body& body = world.body(shot.body);
  EXPECT_NEAR(ToFloat(body.position.y()), 0.1f - 0.0025f, 0.0001f);
  EXPECT_NEAR(ToFloat(body.velocity.y()), -60.0f, 0.01f);
  EXPECT_TRUE(listener.events.empty());
  world.Step();
  EXPECT_EQ(listener.Count(RecordingListener::Kind::kCollisionEnter), 1);
  EXPECT_NEAR(ToFloat(body.velocity.y()), 0.0f, 0.001f);
  EXPECT_NEAR(ToFloat(body.position.y()), 0.0975f, 0.001f);
}

// The other targets: an oriented box (the platform turned by 30 degrees) and
// a circle, hit by every shape of shot
TEST(Bullet, HitsOrientedBoxesAndCircles) {
  for (const ShapeType shape :
       {ShapeType::kCircle, ShapeType::kAabb, ShapeType::kOrientedBox}) {
    {
      neko::PhysicsWorld world{kConfig};
      const Shape ramp = AddPlatform(world, ShapeType::kOrientedBox);
      world.body(ramp.body).rotation.set_angle(
          Radian{neko::Pi<Scalar>() / Scalar{6}});
      const Shape shot = AddShot(world, shape, {Scalar{0}, Scalar{3}},
                                 {Scalar{0}, -Scalar{80}}, true);
      world.Step();
      world.Step();
      // Above the ramp's top face, which passes 0.2 m above its center at
      // x = 0
      EXPECT_GT(ToFloat(world.body(shot.body).position.y()), 0.0f);
    }
    {
      neko::PhysicsWorld world{kConfig};
      AddShape(world, neko::BodyType::kStatic, ShapeType::kCircle, {},
               {Ratio(1, 5), Ratio(1, 5)});
      const Shape shot = AddShot(world, shape, {Ratio(1, 20), Scalar{3}},
                                 {Scalar{0}, -Scalar{80}}, true);
      world.Step();
      world.Step();
      EXPECT_GT(ToFloat(world.body(shot.body).position.y()), 0.2f);
    }
  }
}

// Of two thin walls crossed in the same step, the earliest stops the shot,
// whatever the order of their indices
TEST(Bullet, EarliestOfTwoWalls) {
  for (const bool near_first : {true, false}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                              .max_bodies = 4,
                              .max_colliders = 4,
                              .max_touching_pairs = 8,
                              .max_broadphase_pairs = 8,
                              .max_joints = 0}};
    const Vec2f half{Ratio(1, 20), Scalar{1}};
    const Vec2f near{Ratio(1, 2), Scalar{0}};
    const Vec2f far{Ratio(11, 10), Scalar{0}};
    AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
             near_first ? near : far, half);
    AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
             near_first ? far : near, half);
    const Shape shot =
        AddShot(world, ShapeType::kCircle, {}, {Scalar{80}, Scalar{0}}, true);
    world.Step();
    EXPECT_NEAR(ToFloat(world.body(shot.body).position.x()),
                0.45f - 0.1f + 0.0025f, 0.0001f)
        << near_first;
  }
}

// A one-way platform lets a shot through from below at 50 m/s and stops it
// from above
TEST(Bullet, OneWayBlocksOnlyFromAbove) {
  for (const bool from_below : {true, false}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                              .max_bodies = 4,
                              .max_colliders = 4,
                              .max_touching_pairs = 8,
                              .max_broadphase_pairs = 8,
                              .max_joints = 0,
                              .max_passing_pairs = 4}};
    const Shape platform = AddPlatform(world);
    world.collider(platform.collider).is_one_way = true;
    const Scalar side = from_below ? -Scalar{1} : Scalar{1};
    const Shape shot =
        AddShot(world, ShapeType::kAabb, {Scalar{0}, side * Ratio(31, 10)},
                {Scalar{0}, -side * Scalar{50}}, true);
    Steps(world, 30);
    const float y = ToFloat(world.body(shot.body).position.y());
    if (from_below) {
      EXPECT_GT(y, 10.0f);
    } else {
      EXPECT_NEAR(y, 0.1f, 0.006f);
    }
  }
}

// A trigger shot at 80 m/s through a player box that walks across its
// path: the enter event comes in the step it crosses, the exit at the next.
// Without the flag the shot jumps over the player and nothing happens.
TEST(Bullet, TriggerShotHitsAMovingPlayer) {
  for (const bool is_bullet : {true, false}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                              .max_bodies = 4,
                              .max_colliders = 4,
                              .max_touching_pairs = 8,
                              .max_broadphase_pairs = 8,
                              .max_joints = 0}};
    RecordingListener listener;
    world.SetContactListener(&listener);
    // The player walks up at 3 m/s into the shot's line, y = 0
    const Shape player =
        AddShape(world, neko::BodyType::kDynamic, ShapeType::kAabb,
                 {Ratio(13, 5), -Ratio(1, 2)}, {Ratio(19, 100), Ratio(3, 10)});
    world.body(player.body).velocity = {Scalar{0}, Scalar{3}};
    const Shape shot =
        AddShot(world, ShapeType::kCircle, {-Scalar{2}, Scalar{0}},
                {Scalar{80}, Scalar{0}}, is_bullet);
    world.collider(shot.collider).is_trigger = true;
    int enter_step = -1;
    int exit_step = -1;
    for (int step = 0; step < 30; ++step) {
      const std::size_t before = listener.events.size();
      world.Step();
      for (std::size_t i = before; i < listener.events.size(); ++i) {
        if (listener.events[i].first ==
            RecordingListener::Kind::kTriggerEnter) {
          enter_step = step;
        } else if (listener.events[i].first ==
                   RecordingListener::Kind::kTriggerExit) {
          exit_step = step;
        }
      }
    }
    if (is_bullet) {
      // The shot is at x = 2 after three steps and 3.33 after four, on
      // each side of the player (2.41 to 2.79), whose top crosses y = 0
      // during the fourth step
      EXPECT_EQ(enter_step, 3);
      EXPECT_EQ(exit_step, 4);
      ASSERT_EQ(listener.events.size(), 2u);
      EXPECT_EQ(listener.events[0].second,
                (ColliderPair{shot.collider, player.collider}));
    } else {
      EXPECT_TRUE(listener.events.empty());
    }
    // A trigger doesn't stop the shot
    EXPECT_GT(ToFloat(world.body(shot.body).position.x()), 30.0f);
  }
}

// Crossings are reported in the order of the path, equal times by collider
// index, and stop at a solid impact
TEST(Bullet, TriggerCrossingsInPathOrder) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                            .max_bodies = 8,
                            .max_colliders = 8,
                            .max_touching_pairs = 16,
                            .max_broadphase_pairs = 32,
                            .max_joints = 0}};
  RecordingListener listener;
  world.SetContactListener(&listener);
  const Vec2f half{Ratio(1, 50), Scalar{1}};
  const auto add_zone = [&world, half](Scalar x) {
    const Shape zone = AddShape(world, neko::BodyType::kStatic,
                                ShapeType::kAabb, {x, {}}, half);
    world.collider(zone.collider).is_trigger = true;
    return zone.collider;
  };
  // Indices 0 to 3 at x = 0.8, 0.5, 0.7, 0.5; a wall at 1.1 and a zone after
  // it at 1.25
  const ColliderIndex at_08 = add_zone(Ratio(4, 5));
  const ColliderIndex at_05 = add_zone(Ratio(1, 2));
  const ColliderIndex at_07 = add_zone(Ratio(7, 10));
  const ColliderIndex at_05_too = add_zone(Ratio(1, 2));
  AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
           {Ratio(11, 10), {}}, half);
  add_zone(Ratio(5, 4));
  const Shape shot =
      AddShot(world, ShapeType::kCircle, {}, {Scalar{80}, Scalar{0}}, true);
  world.Step();
  ASSERT_EQ(listener.events.size(), 4u);
  const std::array<ColliderIndex, 4> expected{at_05, at_05_too, at_07, at_08};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(listener.events[i].first, RecordingListener::Kind::kTriggerEnter);
    EXPECT_EQ(listener.events[i].second,
              (ColliderPair{shot.collider, expected[i]}));
  }
  // The crossed zones are touching pairs until the next step
  EXPECT_EQ(world.touching_pairs().size(), 4u);
  listener.events.clear();
  world.Step();
  EXPECT_EQ(listener.Count(RecordingListener::Kind::kTriggerExit), 4);
  EXPECT_EQ(listener.Count(RecordingListener::Kind::kCollisionEnter), 1);
}

// A shot that ends a step inside a trigger gets its enter event in that step
// and no exit at the next
TEST(Bullet, TriggerEndingInsideStaysTouching) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                            .max_bodies = 4,
                            .max_colliders = 4,
                            .max_touching_pairs = 8,
                            .max_broadphase_pairs = 8,
                            .max_joints = 0}};
  RecordingListener listener;
  world.SetContactListener(&listener);
  const Shape zone = AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
                              {Scalar{4}, {}}, {Scalar{3}, Scalar{1}});
  world.collider(zone.collider).is_trigger = true;
  AddShot(world, ShapeType::kCircle, {}, {Scalar{60}, Scalar{0}}, true);
  world.Step();
  EXPECT_EQ(listener.Count(RecordingListener::Kind::kTriggerEnter), 1);
  Steps(world, 3);
  EXPECT_EQ(listener.events.size(), 1u);
}

// The sweep uses the velocity the contacts left: a shot that bounces off the
// floor and flies up through a thin ceiling in the same step is stopped by
// it (the broadphase, run before the bounce, only saw it going down)
TEST(Bullet, SweepsTheVelocityAfterTheContacts) {
  for (const bool is_bullet : {true, false}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                              .max_bodies = 4,
                              .max_colliders = 4,
                              .max_touching_pairs = 8,
                              .max_broadphase_pairs = 8,
                              .max_joints = 0}};
    const Shape floor = AddPlatform(world);
    world.collider(floor.collider).restitution = Scalar{1};
    // A ceiling 4 cm thick, its bottom 0.5 m above the floor
    AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
             {Scalar{0}, Ratio(13, 25)}, {Scalar{2}, Ratio(1, 50)});
    // Already in the floor by 1 cm, going down at 60 m/s
    const Shape shot =
        AddShot(world, ShapeType::kCircle, {Scalar{0}, Ratio(9, 100)},
                {Scalar{0}, -Scalar{60}}, is_bullet);
    world.Step();
    const neko::Body& body = world.body(shot.body);
    EXPECT_GT(ToFloat(body.velocity.y()), 50.0f);
    if (is_bullet) {
      EXPECT_NEAR(ToFloat(body.position.y()), 0.4f + 0.0025f, 0.0001f);
    } else {
      EXPECT_GT(ToFloat(body.position.y()), 0.9f);
    }
  }
}

// A kinematic platform rising at 40 m/s into a shot falling at 40 m/s: the
// sweep follows both, 80 m/s apart, and the shot is stopped where they meet
TEST(Bullet, KinematicTargetMovesToo) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                            .max_bodies = 4,
                            .max_colliders = 4,
                            .max_touching_pairs = 8,
                            .max_broadphase_pairs = 8,
                            .max_joints = 0}};
  const Shape lift =
      AddPlatform(world, ShapeType::kAabb, neko::BodyType::kKinematic);
  world.body(lift.body).velocity = {Scalar{0}, Scalar{40}};
  // 1 m above the lift's top: they meet after 0.75 of the step
  const Shape shot =
      AddShot(world, ShapeType::kCircle, {Scalar{0}, Ratio(11, 10)},
              {Scalar{0}, -Scalar{40}}, true);
  world.Step();
  // The shot at 1.1 - 0.75 * 0.667 = 0.6, the lift's top at 0.667 when the
  // step ends: the next steps push the shot up with the lift
  EXPECT_NEAR(ToFloat(world.body(shot.body).position.y()), 0.6f - 0.0025f,
              0.001f);
  Steps(world, 5);
  const float top = ToFloat(world.body(lift.body).position.y()) + 0.175f;
  EXPECT_GT(ToFloat(world.body(shot.body).position.y()), top);
}

// A slow bullet is not swept: it moves exactly like the same body without
// the flag (a ball rolling and resting on the floor)
TEST(Bullet, SlowBulletIsNotSwept) {
  const auto run = [](bool is_bullet) {
    neko::PhysicsWorld world{kConfig};
    AddPlatform(world);
    // Falls 0.2 m (2 m/s at most, 3.3 cm per step, under half the radius)
    const Shape shot =
        AddShot(world, ShapeType::kCircle, {Scalar{0}, Ratio(3, 10)},
                {Scalar{1}, Scalar{0}}, is_bullet);
    Steps(world, 120);
    return world.body(shot.body).position;
  };
  EXPECT_EQ(run(true), run(false));
}

// A bullet sliding at 30 m/s on a floor of 1 m tiles grazes the next tile's
// corner at every seam (it rests a little into the floor): it must neither
// stop there nor be thrown up (review 1 of S4, B1)
TEST(Bullet, SlidesAcrossTiledFloor) {
  for (const ShapeType shape : {ShapeType::kCircle, ShapeType::kAabb}) {
    neko::PhysicsWorld world{{.max_bodies = 32,
                              .max_colliders = 32,
                              .max_touching_pairs = 32,
                              .max_broadphase_pairs = 64,
                              .max_joints = 0}};
    for (int tile = 0; tile < 30; ++tile) {
      AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
               {static_cast<Scalar>(tile) + Ratio(1, 2), -Ratio(1, 2)},
               {Ratio(1, 2), Ratio(1, 2)});
    }
    const Shape shot = AddShot(world, shape, {Ratio(1, 2), Ratio(1, 10)},
                               {Scalar{0}, Scalar{0}}, false);
    world.collider(shot.collider).friction = Scalar{0};
    Steps(world, 60);
    const Vec2f rest = world.body(shot.body).position;
    world.body(shot.body).velocity = {Scalar{30}, Scalar{0}};
    world.body(shot.body).is_bullet = true;
    for (int step = 0; step < 40; ++step) {
      world.Step();
      const neko::Body& body = world.body(shot.body);
      ASSERT_NEAR(ToFloat(body.position.y()), ToFloat(rest.y()), 0.003f)
          << step;
      ASSERT_NEAR(ToFloat(body.velocity.x()), 30.0f, 0.01f) << step;
      ASSERT_NEAR(ToFloat(body.velocity.y()), 0.0f, 0.2f) << step;
    }
    EXPECT_NEAR(ToFloat(world.body(shot.body).position.x()),
                ToFloat(rest.x()) + 20.0f, 0.02f);
  }
}

// Solid bullets ignore dynamic bodies (out of scope) and triggers' masks
// apply: a shot whose mask leaves out the wall flies through it
TEST(Bullet, FiltersApply) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                            .max_bodies = 4,
                            .max_colliders = 4,
                            .max_touching_pairs = 8,
                            .max_broadphase_pairs = 8,
                            .max_joints = 0}};
  const Vec2f half{Ratio(1, 20), Scalar{1}};
  const Shape wall = AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
                              {Ratio(1, 2), {}}, half);
  world.collider(wall.collider).category = 2;
  const Shape crate = AddShape(world, neko::BodyType::kDynamic,
                               ShapeType::kAabb, {Ratio(7, 5), {}}, half);
  const Shape shot =
      AddShot(world, ShapeType::kCircle, {}, {Scalar{120}, Scalar{0}}, true);
  world.collider(shot.collider).mask = 1;
  world.Step();
  EXPECT_NEAR(ToFloat(world.body(shot.body).position.x()), 2.0f, 0.001f);
  EXPECT_NEAR(ToFloat(world.body(crate.body).position.x()), 1.4f, 0.001f);
}

// Past max_bullet_trigger_hits the crossings are counted, not reported
TEST(Bullet, TriggerHitsOverflowIsCounted) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero(),
                            .max_bodies = 4,
                            .max_colliders = 4,
                            .max_touching_pairs = 8,
                            .max_broadphase_pairs = 8,
                            .max_joints = 0,
                            .max_bullet_trigger_hits = 1}};
  RecordingListener listener;
  world.SetContactListener(&listener);
  for (const Scalar x : {Ratio(1, 2), Ratio(4, 5)}) {
    const Shape zone =
        AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb, {x, {}},
                 {Ratio(1, 50), Scalar{1}});
    world.collider(zone.collider).is_trigger = true;
  }
  AddShot(world, ShapeType::kCircle, {}, {Scalar{80}, Scalar{0}}, true);
  world.Step();
  EXPECT_EQ(listener.Count(RecordingListener::Kind::kTriggerEnter), 1);
  EXPECT_EQ(world.step_overflow().bullet_trigger_hits, 1u);
  EXPECT_TRUE(world.step_overflow().IsAnyCounted());
  world.Step();
  EXPECT_EQ(world.step_overflow().bullet_trigger_hits, 0u);
}

// The flag is hashed (only when set) and copied
TEST(Bullet, FlagIsHashedAndCopied) {
  neko::PhysicsWorld world{kConfig};
  const Shape shot = AddShot(world, ShapeType::kCircle, {}, {}, false);
  const std::uint32_t before = world.GenerateChecksum();
  world.body(shot.body).is_bullet = true;
  EXPECT_NE(world.GenerateChecksum(), before);
  neko::PhysicsWorld copy{kConfig};
  copy.CopyFrom(world);
  EXPECT_TRUE(copy.body(shot.body).is_bullet);
  EXPECT_EQ(copy.GenerateChecksum(), world.GenerateChecksum());
}

// Rollback across the feature: a snapshot taken after a trigger crossing
// (the pair is listed until the next step) and before an impact, restored
// after a divergence, replays the same checksums and events
TEST(Bullet, RollbackReplaysSweeps) {
  neko::PhysicsWorld world{kConfig};
  RecordingListener listener;
  world.SetContactListener(&listener);
  AddPlatform(world);
  const Shape zone =
      AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
               {Scalar{0}, Scalar{2}}, {Scalar{1}, Ratio(1, 50)});
  world.collider(zone.collider).is_trigger = true;
  const Shape shot =
      AddShot(world, ShapeType::kCircle, {Scalar{0}, Ratio(31, 10)},
              {Scalar{0}, -Scalar{70}}, true);
  // The first step crosses the zone (a trigger pair of the bullet)
  world.Step();
  ASSERT_EQ(listener.Count(RecordingListener::Kind::kTriggerEnter), 1);
  neko::PhysicsWorld snapshot{kConfig};
  snapshot.CopyFrom(world);
  const std::uint32_t saved = world.GenerateChecksum();
  const auto run = [&world, &listener] {
    listener.events.clear();
    std::vector<std::uint32_t> checksums;
    for (int step = 0; step < 20; ++step) {
      world.Step();
      checksums.push_back(world.GenerateChecksum());
    }
    return std::pair{checksums, listener.events};
  };
  const auto first = run();
  // Diverge: the shot turned into a slow ball
  world.body(shot.body).is_bullet = false;
  world.body(shot.body).velocity = {Scalar{1}, Scalar{0}};
  Steps(world, 5);
  world.CopyFrom(snapshot);
  EXPECT_EQ(world.GenerateChecksum(), saved);
  const auto replay = run();
  EXPECT_EQ(first.first, replay.first);
  EXPECT_EQ(first.second, replay.second);
  // The replay saw the zone's exit and the landing
  EXPECT_EQ(listener.Count(RecordingListener::Kind::kTriggerExit), 1);
  EXPECT_EQ(listener.Count(RecordingListener::Kind::kCollisionEnter), 1);
}

// A body of half size `half` (circle radius or square box) landed on 50
// static 1 m tiles from 0.5 m above at 8 m/s, left `settle` steps, then sent
// sideways at 30 m/s for 40 steps, frictionless: its positions
std::vector<Vec2f> SlideAcrossSeams(ShapeType shape, Scalar half, int settle,
                                    bool is_bullet) {
  neko::PhysicsWorld world{{.max_bodies = 64,
                            .max_colliders = 64,
                            .max_touching_pairs = 64,
                            .max_broadphase_pairs = 128,
                            .max_joints = 0}};
  for (int tile = 0; tile < 50; ++tile) {
    AddShape(world, neko::BodyType::kStatic, ShapeType::kAabb,
             {static_cast<Scalar>(tile) + Ratio(1, 2), -Ratio(1, 2)},
             {Ratio(1, 2), Ratio(1, 2)});
  }
  const Shape body = AddShape(world, neko::BodyType::kDynamic, shape,
                              {Ratio(1, 2), half + Ratio(1, 2)}, {half, half});
  world.collider(body.collider).friction = Scalar{0};
  world.body(body.body).velocity = {Scalar{0}, -Scalar{8}};
  // Until it reaches the floor, then the settle steps: the solver leaves it
  // 2.8 cm deep 3 steps after the landing, at the slop (5 mm) after 60
  for (int step = 0; step < 100 && world.body(body.body).position.y() > half;
       ++step) {
    world.Step();
  }
  Steps(world, settle);
  world.body(body.body).velocity = {Scalar{30}, Scalar{0}};
  world.body(body.body).is_bullet = is_bullet;
  std::vector<Vec2f> positions;
  for (int step = 0; step < 40; ++step) {
    world.Step();
    positions.push_back(world.body(body.body).position);
  }
  return positions;
}

// A bullet sliding on a tiled floor crosses every seam exactly like the
// same body without the flag (bit-identical positions), whatever its size
// and however deep the solver left it, as long as its middle is above the
// floor's top: small shots (1 and 2 cm) at the slop and bodies that just
// landed. Boxes slide all the way on both scalars, circles in FLOAT (in
// FIXED16 a plain circle can be thrown up by a tile corner too, without the
// flag as with it)
TEST(Bullet, CrossesSeamsLikeAPlainBody) {
  struct Case {
    int half_mm;
    int settle;
  };
  const std::array<Case, 10> cases = {{{10, 20},
                                       {10, 60},
                                       {20, 20},
                                       {20, 60},
                                       {50, 3},
                                       {50, 20},
                                       {50, 60},
                                       {100, 3},
                                       {100, 20},
                                       {100, 60}}};
  for (const ShapeType shape : {ShapeType::kCircle, ShapeType::kAabb}) {
    for (const Case& c : cases) {
      const Scalar half = Ratio(c.half_mm, 1000);
      const auto bullet = SlideAcrossSeams(shape, half, c.settle, true);
      const auto plain = SlideAcrossSeams(shape, half, c.settle, false);
      for (std::size_t step = 0; step < bullet.size(); ++step) {
        ASSERT_EQ(bullet[step], plain[step])
            << c.half_mm << " mm, settle " << c.settle << ", step " << step;
      }
      if (shape == ShapeType::kAabb ||
          !neko::ScalarTraits<Scalar>::kIsFixedPoint) {
        EXPECT_GT(ToFloat(bullet.back().x()), 20.4f)
            << c.half_mm << " mm, settle " << c.settle;
      }
    }
  }
}
}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
