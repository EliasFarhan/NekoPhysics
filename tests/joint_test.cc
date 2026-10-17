// Distance joints (M6): rigid rods, ropes and springs, solved with the
// contacts; measured against the pendulum and mass-spring predictions.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "neko/math/func.h"
#include "neko/physics/physics.h"

namespace {

using neko::BodyIndex;
using neko::DistanceJointType;
using neko::JointIndex;
using neko::Scalar;
using neko::Vec2f;

constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

template <typename T>
float ToFloat(T value) {
  if constexpr (std::is_same_v<T, float>) {
    return value;
  } else {
    return static_cast<float>(value);
  }
}

constexpr float kGravity = 9.81f;
constexpr float kPi = 3.14159265f;
// Static stretch of a soft-step joint per newton of load and per kilogram of
// effective mass: 1 / (2 pi f)^2 at the default 15 Hz
constexpr float kSagPerNewton =
    1.0f / (2.0f * kPi * 15.0f * 2.0f * kPi * 15.0f);

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

BodyIndex AddSolidCircle(neko::PhysicsWorld& world, Vec2f position,
                         Scalar radius) {
  const BodyIndex body = AddPoint(world, position, Scalar{1});
  const auto collider = world.AddCircleCollider(body);
  world.collider(collider).is_trigger = false;
  world.collider(collider).restitution = Scalar{0};
  world.circle(world.collider(collider).shape_index).radius = radius;
  return body;
}

// A pendulum: a 1 kg point mass on a rod of length L from a pivot at the
// origin, released at rest at the given angle from the vertical
struct Pendulum {
  Pendulum(Scalar length, float angle_degrees,
           DistanceJointType type = DistanceJointType::kRigid,
           Vec2f gravity = {Scalar{0}, -Ratio(981, 100)})
      : world({.gravity = gravity}) {
    const float angle = angle_degrees * kPi / 180.0f;
    pivot = AddStatic(world, Vec2f::Zero());
    bob = AddPoint(
        world,
        {length * Scalar{std::sin(angle)}, -length * Scalar{std::cos(angle)}},
        Scalar{1});
    rod = world.AddDistanceJoint(pivot, bob);
    world.joint(rod).type = type;
    world.joint(rod).length = length;
  }
  neko::PhysicsWorld world;
  BodyIndex pivot;
  BodyIndex bob;
  JointIndex rod;
};

// Period from the x zero crossings (interpolated), over the given steps
float MeasurePeriod(neko::PhysicsWorld& world, BodyIndex body, int steps,
                    float* max_length_error = nullptr, JointIndex joint = {},
                    float rest_length = 0.0f) {
  const float dt = ToFloat(world.dt());
  std::vector<float> crossings;
  float previous = ToFloat(world.body(body).position.x());
  for (int step = 1; step <= steps; ++step) {
    world.Step();
    const float x = ToFloat(world.body(body).position.x());
    if ((previous < 0.0f) != (x < 0.0f)) {
      crossings.push_back(
          dt * (static_cast<float>(step) - 1.0f + previous / (previous - x)));
    }
    previous = x;
    if (max_length_error != nullptr) {
      const float error =
          std::abs(ToFloat(world.JointDistance(joint)) - rest_length);
      *max_length_error = std::max(*max_length_error, error);
    }
  }
  if (crossings.size() < 3) {
    return 0.0f;
  }
  // Two crossings per period
  return 2.0f * (crossings.back() - crossings.front()) /
         static_cast<float>(crossings.size() - 1);
}

TEST(DistanceJoint, AddHoldsTheCurrentDistance) {
  neko::PhysicsWorld world;
  const BodyIndex a = AddStatic(world, {Scalar{1}, Scalar{2}});
  const BodyIndex b = AddPoint(world, {Scalar{4}, Scalar{6}}, Scalar{1});
  const JointIndex joint = world.AddDistanceJoint(a, b);
  ASSERT_NE(joint, neko::kInvalidJointIndex);
  EXPECT_EQ(world.joint(joint).type, DistanceJointType::kRigid);
  EXPECT_NEAR(ToFloat(world.joint(joint).length), 5.0f, 1e-4f);
  EXPECT_NEAR(ToFloat(world.JointDistance(joint)), 5.0f, 1e-4f);
}

// T = 2 pi sqrt(L / g) (1 + theta0^2 / 16) for a small amplitude theta0
TEST(DistanceJoint, PendulumPeriodIsTwoPiSqrtLOverG) {
  for (const int length : {1, 2, 4}) {
    Pendulum pendulum{static_cast<Scalar>(length), 10.0f};
    float max_error = 0.0f;
    const float period =
        MeasurePeriod(pendulum.world, pendulum.bob, 900, &max_error,
                      pendulum.rod, static_cast<float>(length));
    const float theta0 = 10.0f * kPi / 180.0f;
    const float predicted = 2.0f * kPi *
                            std::sqrt(static_cast<float>(length) / kGravity) *
                            (1.0f + theta0 * theta0 / 16.0f);
    EXPECT_NEAR(period, predicted, 0.005f * predicted) << "L = " << length;
    // The soft rod stretches by its static sag, 1.1 mm under 1 kg, plus a
    // little more where the swing adds tension
    EXPECT_LT(max_error, 1.5f * kGravity * kSagPerNewton) << "L = " << length;
  }
}

// A wide swing: the rod holds at 90 degrees too, and the bob passes the
// bottom at the speed sqrt(2 g L) of a free fall from the pivot height. The
// length error grows with the speed: (v dt)^2 / (2 L) off the circle per
// step, and a tension of 3 m g at the bottom
TEST(DistanceJoint, WideSwingKeepsLengthAndEnergy) {
  Pendulum pendulum{Scalar{2}, 90.0f};
  float max_error = 0.0f;
  float max_speed = 0.0f;
  for (int step = 0; step < 300; ++step) {
    pendulum.world.Step();
    max_error = std::max(
        max_error,
        std::abs(ToFloat(pendulum.world.JointDistance(pendulum.rod)) - 2.0f));
    max_speed =
        std::max(max_speed,
                 ToFloat(pendulum.world.body(pendulum.bob).velocity.Length()));
  }
  EXPECT_LT(max_error, 0.02f);
  const float predicted = std::sqrt(2.0f * kGravity * 2.0f);
  EXPECT_NEAR(max_speed, predicted, 0.02f * predicted);
}

// Hanging at rest, the rod pulls with the weight (impulse -m g dt) and the
// soft step stretches it by m g / (2 pi f)^2
TEST(DistanceJoint, HangingImpulseIsTheWeight) {
  Pendulum pendulum{Scalar{2}, 0.0f};
  for (int step = 0; step < 60; ++step) {
    pendulum.world.Step();
  }
  const float dt = ToFloat(pendulum.world.dt());
  EXPECT_NEAR(ToFloat(pendulum.world.joint(pendulum.rod).impulse),
              -kGravity * dt, 1e-3f);
  EXPECT_NEAR(ToFloat(pendulum.world.JointDistance(pendulum.rod)),
              2.0f + kGravity * kSagPerNewton, 1e-4f);
}

// Without gravity, a bob thrown at the pivot: a rod pushes back and keeps it
// at its length, a rope lets it in (it only pulls)
TEST(DistanceJoint, RodPushesRopeOnlyPulls) {
  for (const auto type :
       {DistanceJointType::kRigid, DistanceJointType::kRope}) {
    Pendulum pendulum{Scalar{2}, 0.0f, type, Vec2f::Zero()};
    pendulum.world.body(pendulum.bob).velocity = {Scalar{0}, Scalar{1}};
    float max_impulse = 0.0f;
    for (int step = 0; step < 30; ++step) {
      pendulum.world.Step();
      max_impulse = std::max(
          max_impulse, ToFloat(pendulum.world.joint(pendulum.rod).impulse));
    }
    const float distance = ToFloat(pendulum.world.JointDistance(pendulum.rod));
    if (type == DistanceJointType::kRigid) {
      EXPECT_NEAR(distance, 2.0f, 1e-3f);
      EXPECT_NEAR(ToFloat(pendulum.world.body(pendulum.bob).velocity.y()), 0.0f,
                  1e-3f);
      // It pushed: a positive impulse, most of the bob momentum of 1 N s in
      // the first step (the soft step stops it over a few)
      EXPECT_GT(max_impulse, 0.85f);
      EXPECT_LT(max_impulse, 1.0f);
    } else {
      // 0.5 s at 1 m/s toward the pivot, untouched
      EXPECT_NEAR(distance, 1.5f, 0.01f);
      EXPECT_EQ(pendulum.world.joint(pendulum.rod).impulse, Scalar{0});
    }
  }
}

// A slack rope: the bob falls freely until the rope is taut, then hangs at
// the rope length (caught in the step it would overstretch)
TEST(DistanceJoint, SlackRopeFallsFreelyThenCatches) {
  neko::PhysicsWorld world;
  const BodyIndex pivot = AddStatic(world, Vec2f::Zero());
  const BodyIndex bob = AddPoint(world, {Scalar{0}, Scalar{-1}}, Scalar{1});
  const JointIndex rope = world.AddDistanceJoint(pivot, bob);
  world.joint(rope).type = DistanceJointType::kRope;
  world.joint(rope).length = Scalar{3};
  // Free fall of 2 m takes sqrt(2 * 2 / g) = 0.64 s, 38 steps
  for (int step = 0; step < 30; ++step) {
    world.Step();
    EXPECT_EQ(world.joint(rope).impulse, Scalar{0}) << "step " << step;
  }
  const float t = 30.0f * ToFloat(world.dt());
  EXPECT_NEAR(ToFloat(world.body(bob).position.y()),
              -1.0f - 0.5f * kGravity * t * t, 0.05f);
  float max_distance = 0.0f;
  for (int step = 0; step < 120; ++step) {
    world.Step();
    max_distance = std::max(max_distance, ToFloat(world.JointDistance(rope)));
  }
  // Inelastic catch: a few millimeters of overstretch from the soft step,
  // then hanging at full length plus the static sag
  EXPECT_LT(max_distance, 3.01f);
  EXPECT_NEAR(ToFloat(world.JointDistance(rope)),
              3.0f + kGravity * kSagPerNewton, 2e-4f);
  EXPECT_NEAR(ToFloat(world.body(bob).velocity.y()), 0.0f, 1e-2f);
}

// Semi-implicit Euler of a mass on a spring (v -= w^2 x dt, then x += v dt)
// turns by an angle a per step with cos a = 1 - (w dt)^2 / 2: a period of
// 2 pi dt / a, slightly shorter than 2 pi / w
float DiscreteSpringPeriod(float omega, float dt) {
  return 2.0f * kPi * dt / std::acos(1.0f - 0.5f * omega * omega * dt * dt);
}

struct SpringRun {
  neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
  BodyIndex anchor;
  BodyIndex mass;
  JointIndex spring;
};

// A mass (inverse given) on a horizontal spring of rest length 2 from a
// static anchor at x = -2, stretched to x = stretch
void BuildSpring(SpringRun& run, Scalar inverse_mass, Scalar stiffness,
                 Scalar damping, Scalar stretch) {
  run.anchor = AddStatic(run.world, {Scalar{-2}, Scalar{0}});
  run.mass = AddPoint(run.world, {stretch, Scalar{0}}, inverse_mass);
  run.spring = run.world.AddDistanceJoint(run.anchor, run.mass);
  neko::DistanceJoint& joint = run.world.joint(run.spring);
  joint.type = DistanceJointType::kSpring;
  joint.length = Scalar{2};
  joint.stiffness = stiffness;
  joint.damping = damping;
}

// T = 2 pi sqrt(m / k)
TEST(SpringJoint, PeriodIsTwoPiSqrtMOverK) {
  for (const int k : {10, 40, 160}) {
    for (const int mass : {1, 2}) {
      SpringRun run;
      BuildSpring(run, Ratio(1, mass), static_cast<Scalar>(k), Scalar{0},
                  Ratio(1, 2));
      const float period = MeasurePeriod(run.world, run.mass, 600);
      const float omega =
          std::sqrt(static_cast<float>(k) / static_cast<float>(mass));
      const float exact = 2.0f * kPi / omega;
      const float discrete =
          DiscreteSpringPeriod(omega, ToFloat(run.world.dt()));
      EXPECT_NEAR(period, discrete, 0.003f * exact)
          << "k = " << k << ", m = " << mass;
      EXPECT_NEAR(period, exact, 0.02f * exact)
          << "k = " << k << ", m = " << mass;
      // A spring is a force, not a constraint: no impulse
      EXPECT_EQ(run.world.joint(run.spring).impulse, Scalar{0});
    }
  }
}

// Damping c: the amplitude decays as exp(-c / (2 m) t)
TEST(SpringJoint, DampingDecaysAsExpMinusCOverTwoM) {
  SpringRun run;
  BuildSpring(run, Scalar{1}, Scalar{40}, Ratio(1, 2), Scalar{1});
  const float dt = ToFloat(run.world.dt());
  // Peaks of |x| (where the speed changes sign) over the first 4 s
  std::vector<std::pair<float, float>> peaks;
  float previous_speed = 0.0f;
  for (int step = 1; step <= 240; ++step) {
    run.world.Step();
    const float speed = ToFloat(run.world.body(run.mass).velocity.x());
    if (step > 1 && (previous_speed < 0.0f) != (speed < 0.0f)) {
      peaks.emplace_back(
          dt * static_cast<float>(step),
          std::abs(ToFloat(run.world.body(run.mass).position.x())));
    }
    previous_speed = speed;
  }
  ASSERT_GE(peaks.size(), 4u);
  const float elapsed = peaks.back().first - peaks.front().first;
  const float ratio = peaks.back().second / peaks.front().second;
  EXPECT_NEAR(ratio, std::exp(-0.5f / 2.0f * elapsed), 0.03f);
}

// Hanging under gravity, a damped spring settles at length + m g / k
TEST(SpringJoint, HangingSpringStretchesByWeightOverStiffness) {
  neko::PhysicsWorld world;
  const BodyIndex anchor = AddStatic(world, Vec2f::Zero());
  const BodyIndex mass = AddPoint(world, {Scalar{0}, Scalar{-1}}, Ratio(1, 2));
  const JointIndex spring = world.AddDistanceJoint(anchor, mass);
  world.joint(spring).type = DistanceJointType::kSpring;
  world.joint(spring).stiffness = Scalar{50};
  world.joint(spring).damping = Scalar{5};
  for (int step = 0; step < 600; ++step) {
    world.Step();
  }
  EXPECT_NEAR(ToFloat(world.JointDistance(spring)),
              1.0f + 2.0f * kGravity / 50.0f, 2e-3f);
}

// Past w dt = 2 the explicit spring gains energy every step: documented, not
// a bug
TEST(SpringJoint, TooStiffForTheTimeStepExplodes) {
  SpringRun run;
  // w = sqrt(20000) = 141 rad/s, w dt = 2.36
  BuildSpring(run, Scalar{1}, Scalar{20000}, Scalar{0}, Ratio(1, 100));
  for (int step = 0; step < 10; ++step) {
    run.world.Step();
  }
  EXPECT_GT(std::abs(ToFloat(run.world.body(run.mass).position.x())), 0.1f);
}

// Two solid circles linked by a rod overlap at rest: no contact by default,
// a contact with collide_connected
TEST(DistanceJoint, LinkedBodiesDontCollideUnlessAsked) {
  for (const bool collide : {false, true}) {
    neko::PhysicsWorld world{{.gravity = Vec2f::Zero()}};
    const BodyIndex a = AddSolidCircle(world, Vec2f::Zero(), Scalar{1});
    const BodyIndex b =
        AddSolidCircle(world, {Scalar{1}, Scalar{0}}, Scalar{1});
    const JointIndex joint = world.AddDistanceJoint(a, b);
    world.joint(joint).collide_connected = collide;
    world.Step();
    EXPECT_EQ(world.touching_pairs().size(), collide ? 1u : 0u);
  }
}

// A box hung from its corner turns until its center is under the pivot
TEST(DistanceJoint, OffCenterAnchorTurnsTheBody) {
  neko::PhysicsWorld world;
  const BodyIndex pivot = AddStatic(world, Vec2f::Zero());
  const BodyIndex box = world.AddBody();
  world.body(box).position = {Ratio(1, 2), Ratio(-1, 2)};
  const auto collider = world.AddOrientedBoxCollider(box);
  // Solid: triggers have no mass, so no inertia
  world.collider(collider).is_trigger = false;
  world.oriented_box(world.collider(collider).shape_index).half_size = {
      Ratio(1, 2), Ratio(1, 2)};
  const JointIndex joint = world.AddDistanceJoint(pivot, box);
  // Anchor at the top-left corner of the box, on the pivot: a zero-length
  // rod
  world.joint(joint).local_anchor_b = {Ratio(-1, 2), Ratio(1, 2)};
  world.joint(joint).length = Scalar{0};
  float max_spin = 0.0f;
  for (int step = 0; step < 600; ++step) {
    world.Step();
    max_spin =
        std::max(max_spin, std::abs(ToFloat(world.body(box).angular_velocity)));
  }
  EXPECT_GT(max_spin, 1.0f);
  // Still swinging (no damping), but the anchor stays on the pivot
  EXPECT_LT(ToFloat(world.JointDistance(joint)), 0.02f);
}

// A zero-length rod between centers holds a body on its pivot
TEST(DistanceJoint, ZeroLengthRodHoldsThePoint) {
  neko::PhysicsWorld world;
  const BodyIndex pivot = AddStatic(world, {Scalar{1}, Scalar{1}});
  const BodyIndex bob = AddPoint(world, {Scalar{1}, Scalar{1}}, Scalar{1});
  const JointIndex joint = world.AddDistanceJoint(pivot, bob);
  EXPECT_EQ(world.joint(joint).length, Scalar{0});
  for (int step = 0; step < 120; ++step) {
    world.Step();
  }
  EXPECT_LT(ToFloat(world.JointDistance(joint)), 0.01f);
}

TEST(DistanceJoint, CapacityRemovalAndReuse) {
  neko::PhysicsWorld world{{.max_joints = 2}};
  const BodyIndex a = AddStatic(world, Vec2f::Zero());
  const BodyIndex b = AddPoint(world, {Scalar{1}, Scalar{0}}, Scalar{1});
  const BodyIndex c = AddPoint(world, {Scalar{2}, Scalar{0}}, Scalar{1});
  const JointIndex first = world.AddDistanceJoint(a, b);
  const JointIndex second = world.AddDistanceJoint(b, c);
  EXPECT_EQ(world.AddDistanceJoint(a, c), neko::kInvalidJointIndex);
  world.RemoveJoint(first);
  EXPECT_EQ(world.AddDistanceJoint(a, c), first);
  // Removing a body removes its joints
  world.RemoveBody(c);
  EXPECT_EQ(world.joint(first).type, DistanceJointType::kNone);
  EXPECT_EQ(world.joint(second).type, DistanceJointType::kNone);
}

// A chain with a heavy end mass: the joints and their impulses are copied and
// hashed, so a restored world replays bit for bit
TEST(DistanceJoint, RollbackReplaysTheChain) {
  neko::PhysicsWorld world;
  neko::PhysicsWorld snapshot;
  BodyIndex previous = AddStatic(world, Vec2f::Zero());
  for (int i = 1; i <= 5; ++i) {
    const BodyIndex link = AddPoint(world, {static_cast<Scalar>(i), Scalar{0}},
                                    i == 5 ? Ratio(1, 10) : Scalar{1});
    world.AddDistanceJoint(previous, link);
    previous = link;
  }
  for (int step = 0; step < 30; ++step) {
    world.Step();
  }
  snapshot.CopyFrom(world);
  const std::uint32_t saved = world.GenerateChecksum();
  EXPECT_EQ(snapshot.GenerateChecksum(), saved);
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  const std::uint32_t expected = world.GenerateChecksum();
  world.CopyFrom(snapshot);
  EXPECT_EQ(world.GenerateChecksum(), saved);
  for (int step = 0; step < 60; ++step) {
    world.Step();
  }
  EXPECT_EQ(world.GenerateChecksum(), expected);
  // The warm-start impulse is state: changing it changes the checksum
  world.joint(JointIndex{0}).impulse += Ratio(1, 100);
  EXPECT_NE(world.GenerateChecksum(), expected);
}

// Eight 1 kg links hanging from a static pivot, 0.5 m each. At rest each
// joint stretches by its load over (2 pi f)^2 and its effective mass: 8 g for
// the top one (1 kg, against the pivot), (9 - i) g for link i below (0.5
// kg between two links), 64 g / (2 pi f)^2 = 70.7 mm in all. Warm starting
// reaches it with 2 passes; cold, the passes don't converge and the chain
// sags more.
float HangingChainStretch(int passes, bool warm) {
  neko::PhysicsWorld world{
      {.velocity_iterations = passes, .warm_starting = warm}};
  BodyIndex previous = AddStatic(world, Vec2f::Zero());
  std::vector<JointIndex> joints;
  for (int i = 1; i <= 8; ++i) {
    const BodyIndex link =
        AddPoint(world, {Scalar{0}, -Ratio(i, 2)}, Scalar{1});
    joints.push_back(world.AddDistanceJoint(previous, link));
    previous = link;
  }
  for (int step = 0; step < 300; ++step) {
    world.Step();
  }
  float stretch = 0.0f;
  for (const JointIndex joint : joints) {
    stretch += ToFloat(world.JointDistance(joint)) - 0.5f;
  }
  return stretch;
}

TEST(DistanceJoint, HangingChainStretchesByItsLoadOverTheSoftness) {
  const float predicted = 64.0f * kGravity * kSagPerNewton;
  for (const int passes : {2, 4, 8}) {
    EXPECT_NEAR(HangingChainStretch(passes, true), predicted, 0.002f)
        << passes << " passes";
  }
  // One pass is not enough to converge (about twice the stretch), but it
  // stays bounded: Baumgarte with warm starting blew up here
  const float one_pass = HangingChainStretch(1, true);
  EXPECT_GT(one_pass, predicted);
  EXPECT_LT(one_pass, 3.0f * predicted);
  // Cold: more passes needed for the same result
  EXPECT_GT(HangingChainStretch(4, false), predicted + 0.02f);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
