// Revolute joints (slice S5 of the night run of 2026-10-18, N10 and N11): the
// point constraint, the angle limits, the LuchaSteps-like rig and the
// retargeting of joints at runtime.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "neko/math/angle.h"
#include "neko/math/func.h"
#include "neko/math/scalar_traits.h"
#include "neko/physics/contact_listener.h"
#include "neko/physics/physics.h"

namespace {

using neko::BodyIndex;
using neko::ColliderPair;
using neko::JointIndex;
using neko::Radian;
using neko::RevoluteJointIndex;
using neko::Scalar;
using neko::Vec2f;

constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

template <typename T>
float ToFloat(T value) {
  return neko::ScalarTraits<T>::ToFloat(value);
}

float ToFloat(Radian angle) { return ToFloat(angle.value()); }

constexpr float kPi = 3.14159265f;
constexpr float kDegree = kPi / 180.0f;

Radian Degrees(int degrees) {
  return Radian{neko::Degree{static_cast<Scalar>(degrees)}};
}

BodyIndex AddStatic(neko::PhysicsWorld& world, Vec2f position) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = neko::BodyType::kStatic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = position;
  return body;
}

// A point mass: no collider, so no inertia and no spin
BodyIndex AddPoint(neko::PhysicsWorld& world, Vec2f position,
                   Scalar inverse_mass) {
  const BodyIndex body = world.AddBody();
  world.body(body).position = position;
  world.body(body).inverse_mass = inverse_mass;
  return body;
}

// A solid oriented box (a plank, a platform), whose inertia comes from its
// collider
BodyIndex AddBox(neko::PhysicsWorld& world, Vec2f position, Vec2f half_size,
                 Scalar inverse_mass, Scalar friction = Ratio(1, 2)) {
  const BodyIndex body = AddPoint(world, position, inverse_mass);
  const auto collider = world.AddOrientedBoxCollider(body);
  world.collider(collider).friction = friction;
  world.collider(collider).is_trigger = false;
  world.collider(collider).restitution = Scalar{0};
  world.oriented_box(world.collider(collider).shape_index).half_size =
      half_size;
  return body;
}

BodyIndex AddSolidCircle(neko::PhysicsWorld& world, Vec2f position,
                         Scalar radius) {
  const BodyIndex body = AddPoint(world, position, Scalar{1});
  const auto collider = world.AddCircleCollider(body);
  world.collider(collider).is_trigger = false;
  world.collider(collider).restitution = Scalar{0};
  world.circle(world.collider(collider).shape_index).radius = radius;
  return body;
}

float AnchorGap(const neko::PhysicsWorld& world, RevoluteJointIndex joint) {
  const auto anchors = world.RevoluteJointAnchors(joint);
  return ToFloat((anchors[1] - anchors[0]).Length());
}

// Period from the x zero crossings (interpolated), and the largest distance
// of the bob from the circle of radius length
struct Swing {
  float period = 0.0f;
  float max_radius_error = 0.0f;
};

Swing MeasureSwing(neko::PhysicsWorld& world, BodyIndex bob, float length,
                   int steps) {
  const float dt = ToFloat(world.dt());
  std::vector<float> crossings;
  Swing swing;
  float previous = ToFloat(world.body(bob).position.x());
  for (int step = 1; step <= steps; ++step) {
    world.Step();
    const Vec2f position = world.body(bob).position;
    const float x = ToFloat(position.x());
    if ((previous < 0.0f) != (x < 0.0f)) {
      crossings.push_back(
          dt * (static_cast<float>(step) - 1.0f + previous / (previous - x)));
    }
    previous = x;
    swing.max_radius_error = std::max(
        swing.max_radius_error, std::abs(ToFloat(position.Length()) - length));
  }
  if (crossings.size() >= 3) {
    swing.period = 2.0f * (crossings.back() - crossings.front()) /
                   static_cast<float>(crossings.size() - 1);
  }
  return swing;
}

// A 1 kg disk of radius 0.2 m, 2 m below a static pivot at the origin,
// released at 30 degrees, held by a pin (the disk's anchor 2 m from its
// center: the disk turns with the swing) or by a rigid distance joint to its
// center (the disk doesn't turn)
constexpr float kBobRadius = 0.2f;

neko::PhysicsWorld MakePendulum(bool is_pin, BodyIndex* bob) {
  neko::PhysicsWorld world;
  const BodyIndex pivot = AddStatic(world, Vec2f::Zero());
  const float angle = 30.0f * kDegree;
  *bob = AddSolidCircle(world,
                        {Scalar{2} * Scalar{std::sin(angle)},
                         -Scalar{2} * Scalar{std::cos(angle)}},
                        Scalar{kBobRadius});
  if (is_pin) {
    world.AddRevoluteJoint(pivot, *bob, Vec2f::Zero());
  } else {
    world.AddDistanceJoint(pivot, *bob);
  }
  return world;
}

class EventCounter final : public neko::ContactListener {
 public:
  void OnTriggerEnter(const ColliderPair&) override {}
  void OnTriggerExit(const ColliderPair&) override {}
  void OnCollisionEnter(const ColliderPair& pair) override {
    enters.push_back(pair);
  }
  void OnCollisionExit(const ColliderPair& pair) override {
    exits.push_back(pair);
  }
  std::vector<ColliderPair> enters;
  std::vector<ColliderPair> exits;
};

TEST(RevoluteJoint, AddPinsTheCurrentPoint) {
  neko::PhysicsWorld world;
  const BodyIndex a = AddBox(world, {Scalar{1}, Scalar{0}},
                             {Scalar{1}, Ratio(1, 10)}, Scalar{1});
  const BodyIndex b = AddBox(world, {Scalar{3}, Scalar{1}},
                             {Scalar{1}, Ratio(1, 10)}, Scalar{1});
  world.body(a).rotation.set_angle(Degrees(30));
  world.body(b).rotation.set_angle(Degrees(-60));
  const RevoluteJointIndex joint =
      world.AddRevoluteJoint(a, b, {Scalar{2}, Scalar{1}});
  ASSERT_NE(joint, neko::kInvalidRevoluteJointIndex);
  const neko::RevoluteJoint& data = world.revolute_joint(joint);
  EXPECT_TRUE(data.is_used);
  EXPECT_FALSE(data.enable_limit);
  EXPECT_FALSE(data.collide_connected);
  EXPECT_NEAR(ToFloat(data.reference_angle), -90.0f * kDegree, 1e-3f);
  const auto anchors = world.RevoluteJointAnchors(joint);
  EXPECT_NEAR(ToFloat(anchors[0].x()), 2.0f, 1e-3f);
  EXPECT_NEAR(ToFloat(anchors[0].y()), 1.0f, 1e-3f);
  EXPECT_LT(AnchorGap(world, joint), 2e-3f);
  EXPECT_NEAR(ToFloat(world.RevoluteJointAngle(joint)), 0.0f, 1e-3f);
}

// The distance pendulum is a point mass on a rod: 2 pi sqrt(L / g) (1 +
// theta^2 / 16) = 2.886 s at 30 degrees. The pinned disk turns with the
// swing, a physical pendulum of inertia m L^2 + m r^2 / 2 about the pivot:
// sqrt(1 + r^2 / (2 L^2)) = 1.0025 times longer. Both stay on the circle
// within a few mm.
TEST(RevoluteJoint, PinPendulumMatchesTheDistancePendulum) {
  BodyIndex pin_bob;
  BodyIndex rod_bob;
  neko::PhysicsWorld pin = MakePendulum(true, &pin_bob);
  neko::PhysicsWorld rod = MakePendulum(false, &rod_bob);
  const Swing pin_swing = MeasureSwing(pin, pin_bob, 2.0f, 900);
  const Swing rod_swing = MeasureSwing(rod, rod_bob, 2.0f, 900);
  const float theta = 30.0f * kDegree;
  const float predicted =
      2.0f * kPi * std::sqrt(2.0f / 9.81f) * (1.0f + theta * theta / 16.0f);
  const float physical =
      predicted * std::sqrt(1.0f + kBobRadius * kBobRadius / (2.0f * 4.0f));
  EXPECT_NEAR(rod_swing.period, predicted, 0.005f * predicted);
  EXPECT_NEAR(pin_swing.period, physical, 0.005f * predicted);
  EXPECT_GT(pin_swing.period, rod_swing.period);
  // The pin gives like the rod: tension sag and swing drift, a few mm
  EXPECT_LT(pin_swing.max_radius_error, 0.005f);
  EXPECT_LT(pin_swing.max_radius_error, 2.0f * rod_swing.max_radius_error);
  EXPECT_LT(AnchorGap(pin, RevoluteJointIndex{0}), 0.005f);
}

// A pin whose anchors are on top of each other holds the point in every
// direction; a zero-length distance joint has no direction there and lets
// the first step's sideways motion through
TEST(RevoluteJoint, ZeroLengthPinIsAPoint) {
  for (const bool is_pin : {true, false}) {
    neko::PhysicsWorld world;
    const BodyIndex pivot = AddStatic(world, {Scalar{1}, Scalar{1}});
    const BodyIndex bob = AddPoint(world, {Scalar{1}, Scalar{1}}, Scalar{1});
    world.body(bob).velocity = {Scalar{2}, Scalar{0}};
    if (is_pin) {
      world.AddRevoluteJoint(pivot, bob, {Scalar{1}, Scalar{1}});
    } else {
      world.AddDistanceJoint(pivot, bob);
    }
    float max_gap = 0.0f;
    for (int step = 0; step < 120; ++step) {
      world.Step();
      max_gap = std::max(max_gap, ToFloat((world.body(bob).position -
                                           world.body(pivot).position)
                                              .Length()));
    }
    if (is_pin) {
      // The soft step removes 90 % of the speed in the first step (its
      // mass_scale at 15 Hz): 0.2 m/s for a step, 3.4 mm
      EXPECT_LT(max_gap, 0.005f);
    } else {
      // 2 m/s for a whole step: 33 mm away before the joint has an axis
      EXPECT_GT(max_gap, 0.03f);
    }
  }
}

// A 10 kg plank 4 m long hinged at its left end to a static pivot falls
// until the limit stops it at -45 degrees, and rests there; flipped gravity
// sends it to +45
TEST(RevoluteJoint, LimitsHoldAtFortyFiveDegreesUnderLoad) {
  for (const int sign : {-1, 1}) {
    neko::PhysicsWorld world{
        {.gravity =
             Vec2f{Scalar{0}, static_cast<Scalar>(sign) * Ratio(981, 100)}}};
    const BodyIndex pivot = AddStatic(world, Vec2f::Zero());
    const BodyIndex plank = AddBox(world, {Scalar{2}, Scalar{0}},
                                   {Scalar{2}, Ratio(1, 10)}, Ratio(1, 10));
    // A 20 kg load at the tip, pinned too
    const BodyIndex load =
        AddPoint(world, {Scalar{4}, Scalar{0}}, Ratio(1, 20));
    const RevoluteJointIndex hinge =
        world.AddRevoluteJoint(pivot, plank, Vec2f::Zero());
    world.AddRevoluteJoint(plank, load, {Scalar{4}, Scalar{0}});
    world.revolute_joint(hinge).enable_limit = true;
    world.revolute_joint(hinge).lower_angle = Degrees(-45);
    world.revolute_joint(hinge).upper_angle = Degrees(45);
    float worst = 0.0f;
    for (int step = 0; step < 600; ++step) {
      world.Step();
      const float angle = ToFloat(world.RevoluteJointAngle(hinge));
      worst = std::max(worst, std::abs(angle) - 45.0f * kDegree);
    }
    const float angle = ToFloat(world.RevoluteJointAngle(hinge));
    EXPECT_NEAR(angle, static_cast<float>(sign) * 45.0f * kDegree,
                0.5f * kDegree)
        << sign;
    // Overshoot past the limit while it catches the swing of the load
    EXPECT_LT(worst, 4.0f * kDegree) << sign;
    EXPECT_LT(std::abs(ToFloat(world.body(plank).angular_velocity)), 0.01f);
    EXPECT_LT(AnchorGap(world, hinge), 0.02f);
    // The limit holding the plank pushes, never pulls
    const neko::RevoluteJoint& joint = world.revolute_joint(hinge);
    EXPECT_GE(ToFloat(joint.lower_impulse), 0.0f);
    EXPECT_GE(ToFloat(joint.upper_impulse), 0.0f);
    EXPECT_GT(ToFloat(sign < 0 ? joint.lower_impulse : joint.upper_impulse),
              0.0f);
  }
}

// Inside its limits a hinge turns freely: no limit impulse
TEST(RevoluteJoint, LimitsDontActInside) {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
  const BodyIndex pivot = AddStatic(world, Vec2f::Zero());
  const BodyIndex plank = AddBox(world, {Scalar{1}, Scalar{0}},
                                 {Scalar{1}, Ratio(1, 10)}, Scalar{1});
  const RevoluteJointIndex hinge =
      world.AddRevoluteJoint(pivot, plank, Vec2f::Zero());
  world.revolute_joint(hinge).enable_limit = true;
  world.revolute_joint(hinge).lower_angle = Degrees(-45);
  world.revolute_joint(hinge).upper_angle = Degrees(45);
  world.body(plank).angular_velocity = Ratio(1, 2);
  world.body(plank).velocity = {Scalar{0}, Ratio(1, 2)};
  for (int step = 0; step < 30; ++step) {
    world.Step();
    EXPECT_EQ(world.revolute_joint(hinge).lower_impulse, Scalar{0});
    EXPECT_EQ(world.revolute_joint(hinge).upper_impulse, Scalar{0});
  }
  // 0.5 rad/s for half a second
  EXPECT_NEAR(ToFloat(world.RevoluteJointAngle(hinge)), 0.25f, 0.01f);
}

// The LuchaSteps rig of Splash (Splash Online NekoPhysics plan, 3.3): an
// anchor beam on a static axle, two balances hanging from its ends, two 4 kg
// platforms hanging from the balances, five hinges limited to +-45 degrees,
// and two rigid distance joints of length 8 from the axle to the platforms,
// 7.5 m away at the start, which the hinges and limits keep from reaching 8
// (over-constrained, as in the level)
struct LuchaRig {
  explicit LuchaRig(Scalar gravity_y = -Ratio(981, 100))
      : world({.gravity = Vec2f{Scalar{0}, gravity_y}}) {
    axle = AddStatic(world, Vec2f::Zero());
    anchor =
        AddBox(world, Vec2f::Zero(), {Scalar{4}, Ratio(1, 8)}, Ratio(1, 8));
    balances[0] = AddBox(world, {-Scalar{4}, -Scalar{2}},
                         {Scalar{2}, Ratio(1, 10)}, Ratio(1, 4));
    balances[1] = AddBox(world, {Scalar{4}, -Scalar{2}},
                         {Scalar{2}, Ratio(1, 10)}, Ratio(1, 4));
    platforms[0] = AddBox(world, {-Scalar{7}, -Ratio(27, 10)},
                          {Scalar{1}, Ratio(3, 20)}, Ratio(1, 4), Ratio(2, 5));
    platforms[1] = AddBox(world, {Scalar{7}, -Ratio(27, 10)},
                          {Scalar{1}, Ratio(3, 20)}, Ratio(1, 4), Ratio(2, 5));
    hinges[0] = world.AddRevoluteJoint(axle, anchor, Vec2f::Zero());
    hinges[1] =
        world.AddRevoluteJoint(anchor, balances[0], {-Scalar{4}, Scalar{0}});
    hinges[2] =
        world.AddRevoluteJoint(anchor, balances[1], {Scalar{4}, Scalar{0}});
    hinges[3] = world.AddRevoluteJoint(balances[0], platforms[0],
                                       {-Scalar{6}, -Scalar{2}});
    hinges[4] = world.AddRevoluteJoint(balances[1], platforms[1],
                                       {Scalar{6}, -Scalar{2}});
    for (const RevoluteJointIndex hinge : hinges) {
      world.revolute_joint(hinge).enable_limit = true;
      world.revolute_joint(hinge).lower_angle = Degrees(-45);
      world.revolute_joint(hinge).upper_angle = Degrees(45);
    }
    // Each platform to the axle, 7.5 m now, 8 m long
    rods[0] = world.AddDistanceJoint(axle, platforms[0]);
    world.joint(rods[0]).length = Scalar{8};
    rods[1] = world.AddDistanceJoint(axle, platforms[1]);
    world.joint(rods[1]).length = Scalar{8};
  }
  neko::PhysicsWorld world;
  BodyIndex axle;
  BodyIndex anchor;
  std::array<BodyIndex, 2> balances{};
  std::array<BodyIndex, 2> platforms{};
  std::array<RevoluteJointIndex, 5> hinges{};
  std::array<JointIndex, 2> rods{};
};

// Largest body speed, hinge anchor gap, angle past a limit and rod length
// error of the rig over a window of steps
struct RigWindow {
  float max_speed = 0.0f;
  float max_gap = 0.0f;
  float max_past_limit = -1.0f;
  float max_rod_error = 0.0f;
};

RigWindow RunRig(LuchaRig& rig, int steps) {
  RigWindow window;
  for (int step = 0; step < steps; ++step) {
    rig.world.Step();
    for (const BodyIndex body : {rig.anchor, rig.balances[0], rig.balances[1],
                                 rig.platforms[0], rig.platforms[1]}) {
      window.max_speed = std::max(
          window.max_speed, ToFloat(rig.world.body(body).velocity.Length()));
    }
    for (const RevoluteJointIndex hinge : rig.hinges) {
      window.max_gap = std::max(window.max_gap, AnchorGap(rig.world, hinge));
      window.max_past_limit =
          std::max(window.max_past_limit,
                   std::abs(ToFloat(rig.world.RevoluteJointAngle(hinge))) -
                       45.0f * kDegree);
    }
    for (const JointIndex rod : rig.rods) {
      window.max_rod_error =
          std::max(window.max_rod_error,
                   std::abs(ToFloat(rig.world.JointDistance(rod)) - 8.0f));
    }
  }
  return window;
}

// 60 s of the rig. The rods snap from 7.5 to 8 m in the first steps (up to
// 10 m/s, hinges 12 cm apart for a moment); after that the rig swings
// without growing (nothing in it dissipates but the soft steps), the anchor
// stays on its axle and the hinges stay within their limits, each a few cm
// apart where the rods pull them
TEST(RevoluteJoint, LuchaStepsRigStaysStableForAMinute) {
  LuchaRig rig;
  EXPECT_NEAR(ToFloat(rig.world.JointDistance(rig.rods[0])), 7.5f, 0.01f);
  const RigWindow first = RunRig(rig, 600);
  EXPECT_LT(first.max_gap, 0.2f);
  EXPECT_LT(first.max_past_limit, 2.0f * kDegree);
  RigWindow previous = RunRig(rig, 600);
  for (int window = 2; window < 6; ++window) {
    const RigWindow current = RunRig(rig, 600);
    EXPECT_LE(current.max_speed, previous.max_speed) << window;
    EXPECT_LT(current.max_gap, 0.05f) << window;
    EXPECT_LT(current.max_past_limit, 0.0f) << window;
    EXPECT_LT(current.max_rod_error, 0.02f) << window;
    previous = current;
  }
  EXPECT_LT(previous.max_speed, 0.5f);
  EXPECT_LT(ToFloat(rig.world.body(rig.anchor).position.Length()), 0.01f);
  EXPECT_FALSE(rig.world.step_overflow().IsAnyCounted());
}

// Boxes dropped on one platform tip the rig; it carries them, hinges held
TEST(RevoluteJoint, LuchaStepsRigCarriesALoad) {
  LuchaRig rig;
  RunRig(rig, 600);
  for (int i = 0; i < 3; ++i) {
    AddBox(rig.world, {-Scalar{7}, static_cast<Scalar>(i) - Ratio(3, 2)},
           {Ratio(1, 4), Ratio(1, 4)}, Scalar{1}, Ratio(2, 5));
  }
  // The landing pulls the hinges apart by up to 7 cm, then the rig settles
  // into a swing no faster than before
  const RigWindow landing = RunRig(rig, 600);
  EXPECT_LT(landing.max_gap, 0.1f);
  EXPECT_LT(landing.max_past_limit, 2.0f * kDegree);
  const RigWindow loaded = RunRig(rig, 1200);
  EXPECT_LT(loaded.max_gap, 0.1f);
  EXPECT_LT(loaded.max_past_limit, 2.0f * kDegree);
  EXPECT_LT(loaded.max_speed, landing.max_speed);
  EXPECT_LT(ToFloat(rig.world.body(rig.anchor).position.Length()), 0.01f);
}

TEST(RevoluteJoint, CapacityRemovalAndReuse) {
  neko::PhysicsWorld world{{.max_revolute_joints = 1}};
  const BodyIndex a = AddStatic(world, Vec2f::Zero());
  const BodyIndex b = AddPoint(world, {Scalar{1}, Scalar{0}}, Scalar{1});
  const BodyIndex c = AddPoint(world, {Scalar{2}, Scalar{0}}, Scalar{1});
  const RevoluteJointIndex first = world.AddRevoluteJoint(a, b, Vec2f::Zero());
  EXPECT_EQ(world.AddRevoluteJoint(b, c, Vec2f::Zero()),
            neko::kInvalidRevoluteJointIndex);
  world.Step();
  EXPECT_EQ(world.step_overflow().revolute_joints, 1u);
  EXPECT_TRUE(world.step_overflow().IsAnyCounted());
  world.Step();
  EXPECT_EQ(world.step_overflow().revolute_joints, 0u);
  world.RemoveRevoluteJoint(first);
  EXPECT_FALSE(world.revolute_joint(first).is_used);
  EXPECT_EQ(world.AddRevoluteJoint(b, c, Vec2f::Zero()), first);
  // Removing a body removes its revolute joints
  world.RemoveBody(c);
  EXPECT_FALSE(world.revolute_joint(first).is_used);
}

TEST(RevoluteJoint, LinkedBodiesDontCollideUnlessAsked) {
  for (const bool collide : {false, true}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
    const BodyIndex a = AddSolidCircle(world, Vec2f::Zero(), Scalar{1});
    const BodyIndex b =
        AddSolidCircle(world, {Scalar{1}, Scalar{0}}, Scalar{1});
    const RevoluteJointIndex joint =
        world.AddRevoluteJoint(a, b, {Ratio(1, 2), Scalar{0}});
    world.revolute_joint(joint).collide_connected = collide;
    world.Step();
    EXPECT_EQ(world.touching_pairs().size(), collide ? 1u : 0u);
  }
}

// Retargeting a joint moves its collision filter: the old pair collides
// again (enter at the next step), the new one stops (exit), and the
// accumulated impulses start from zero
TEST(RevoluteJoint, RetargetMovesTheFilterAndResetsImpulses) {
  for (const bool is_revolute : {true, false}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
    EventCounter events;
    world.SetContactListener(&events);
    // Three overlapping circles in a row: a-b and b-c touch
    const BodyIndex a = AddSolidCircle(world, Vec2f::Zero(), Scalar{1});
    const BodyIndex b =
        AddSolidCircle(world, {Ratio(3, 2), Scalar{0}}, Scalar{1});
    const BodyIndex c =
        AddSolidCircle(world, {Scalar{3}, Scalar{0}}, Scalar{1});
    world.body(a).type = neko::BodyType::kStatic;
    world.body(a).inverse_mass = Scalar{0};
    RevoluteJointIndex hinge{};
    JointIndex rod{};
    if (is_revolute) {
      hinge = world.AddRevoluteJoint(a, b, {Ratio(3, 4), Scalar{0}});
    } else {
      rod = world.AddDistanceJoint(a, b);
    }
    world.Step();
    // Only b-c touches: a-b is filtered
    ASSERT_EQ(world.touching_pairs().size(), 1u) << is_revolute;
    EXPECT_EQ(events.enters.size(), 1u);
    const ColliderPair b_c{neko::ColliderIndex{1}, neko::ColliderIndex{2}};
    const ColliderPair a_b{neko::ColliderIndex{0}, neko::ColliderIndex{1}};
    EXPECT_EQ(events.enters[0], b_c);
    if (is_revolute) {
      EXPECT_NE(world.revolute_joint(hinge).impulse, Vec2f::Zero());
      world.SetRevoluteJointBodies(hinge, b, c);
      EXPECT_EQ(world.revolute_joint(hinge).impulse, Vec2f::Zero());
      EXPECT_EQ(world.revolute_joint(hinge).body_a, b);
      EXPECT_EQ(world.revolute_joint(hinge).body_b, c);
    } else {
      world.SetJointBodies(rod, b, c);
      EXPECT_EQ(world.joint(rod).impulse, Scalar{0});
      EXPECT_EQ(world.joint(rod).body_a, b);
    }
    events.enters.clear();
    world.Step();
    ASSERT_EQ(events.enters.size(), 1u) << is_revolute;
    EXPECT_EQ(events.enters[0], a_b);
    ASSERT_EQ(events.exits.size(), 1u) << is_revolute;
    EXPECT_EQ(events.exits[0], b_c);
    world.SetContactListener(nullptr);
  }
}

// The joint and its impulses are copied and hashed: a restored rig replays
// bit for bit, and a teleport clears the impulses of the body's joints
TEST(RevoluteJoint, RollbackReplaysTheRig) {
  LuchaRig rig;
  LuchaRig snapshot_rig;
  neko::PhysicsWorld& world = rig.world;
  neko::PhysicsWorld& snapshot = snapshot_rig.world;
  snapshot.Clear();
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  snapshot.CopyFrom(world);
  const std::uint32_t saved = world.GenerateChecksum();
  EXPECT_EQ(snapshot.GenerateChecksum(), saved);
  // Diverge: a push on a platform
  world.ApplyImpulse(rig.platforms[0], {Scalar{0}, -Scalar{5}});
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  EXPECT_NE(world.GenerateChecksum(), saved);
  world.CopyFrom(snapshot);
  EXPECT_EQ(world.GenerateChecksum(), saved);
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  const std::uint32_t expected = world.GenerateChecksum();
  world.CopyFrom(snapshot);
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  EXPECT_EQ(world.GenerateChecksum(), expected);
  // Every accumulated impulse is state
  for (const int field : {0, 1, 2}) {
    world.CopyFrom(snapshot);
    neko::RevoluteJoint& joint = world.revolute_joint(rig.hinges[1]);
    if (field == 0) {
      joint.impulse.set_x(joint.impulse.x() + Ratio(1, 100));
    } else if (field == 1) {
      joint.lower_impulse += Ratio(1, 100);
    } else {
      joint.upper_impulse += Ratio(1, 100);
    }
    EXPECT_NE(world.GenerateChecksum(), saved) << field;
  }
  // Teleport drops the warm start of the body's joints
  world.CopyFrom(snapshot);
  world.Teleport(rig.balances[0], world.body(rig.balances[0]).position,
                 world.body(rig.balances[0]).rotation.angle());
  EXPECT_EQ(world.revolute_joint(rig.hinges[1]).impulse, Vec2f::Zero());
  EXPECT_EQ(world.revolute_joint(rig.hinges[3]).impulse, Vec2f::Zero());
  EXPECT_NE(world.revolute_joint(rig.hinges[0]).impulse, Vec2f::Zero());
}

// Without warm starting the pin still holds, a little softer
TEST(RevoluteJoint, HoldsWithoutWarmStarting) {
  for (const bool warm : {true, false}) {
    neko::PhysicsWorld world{{.warm_starting = warm}};
    const BodyIndex pivot = AddStatic(world, Vec2f::Zero());
    const BodyIndex bob = AddPoint(world, {Scalar{1}, Scalar{0}}, Scalar{1});
    const RevoluteJointIndex pin =
        world.AddRevoluteJoint(pivot, bob, Vec2f::Zero());
    for (int step = 0; step < 300; ++step) {
      world.Step();
    }
    EXPECT_LT(AnchorGap(world, pin), 0.01f) << warm;
    // The step's impulse holds the weight either way: (0, 9.81 dt)
    EXPECT_NEAR(ToFloat(world.revolute_joint(pin).impulse.y()), 9.81f / 60.0f,
                0.01f)
        << warm;
  }
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
