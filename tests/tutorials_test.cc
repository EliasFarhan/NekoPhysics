#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "tutorials/00_bounce/bounce_sample.h"
#include "tutorials/01_rotation/rotation_sample.h"
#include "tutorials/02_rotated_box/rotated_box_sample.h"
#include "tutorials/03_contact_manifold/contact_manifold_sample.h"
#include "tutorials/04_off_center_impact/off_center_impact_sample.h"
#include "tutorials/05_friction/friction_sample.h"
#include "tutorials/06_stacking/stacking_sample.h"

namespace {

using neko::tutorial::BounceSample;
using neko::tutorial::ContactManifoldSample;
using neko::tutorial::FrictionSample;
using neko::tutorial::OffCenterImpactSample;
using neko::tutorial::RotatedBoxSample;
using neko::tutorial::RotationSample;
using neko::tutorial::StackingSample;

// static_cast<float> would be a useless cast when Scalar is float. A template
// so that if constexpr discards the branch that doesn't compile for the type.
template <typename T>
float ToFloat(T value) {
  if constexpr (std::is_same_v<T, float>) {
    return value;
  } else {
    return static_cast<float>(value);
  }
}

struct BounceRun {
  float predicted = 0.0f;
  std::optional<float> measured;
};

// Runs the bounce until the first apex (10 s at most) with the given
// parameters, in centimeters and percents
BounceRun RunBounce(int drop_height_cm, int restitution_percent) {
  BounceSample sample;
  sample.parameters()[BounceSample::kDropHeight].value = drop_height_cm;
  sample.parameters()[BounceSample::kRestitution].value = restitution_percent;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < 600 && !sample.measured_apex().has_value();
       ++step) {
    sample.FixedUpdate();
  }
  BounceRun run;
  run.predicted = ToFloat(sample.predicted_apex());
  if (sample.measured_apex().has_value()) {
    run.measured = ToFloat(*sample.measured_apex());
  }
  sample.End();
  return run;
}

class BounceTest : public ::testing::TestWithParam<std::pair<int, int>> {};

}  // namespace

// The experiment of chapter 00: the apex after one bounce is e^2 h, up to the
// time step. Two errors of about one step: the contact is found a step late,
// when the ball already sank by up to v dt, and it bounces at e (v + g dt)
// (that step's gravity included), which adds up to e^2 (v + g dt) dt to the
// apex; and the apex falls between two samples of the arc, which removes up
// to e (v + g dt) dt. Measured with FIXED16, the error goes from -0.22 v dt (e
// = 0.25) to +1.03 v dt (e = 1).
TEST_P(BounceTest, ApexIsRestitutionSquaredTimesHeight) {
  const auto [drop_height_cm, restitution_percent] = GetParam();
  const auto run = RunBounce(drop_height_cm, restitution_percent);
  ASSERT_TRUE(run.measured.has_value());
  const float drop_height = static_cast<float>(drop_height_cm) / 100.0f;
  const float restitution = static_cast<float>(restitution_percent) / 100.0f;
  EXPECT_FLOAT_EQ(run.predicted, restitution * restitution * drop_height)
      << "up to the rounding of the parameters";
  const float impact_speed = std::sqrt(2.0f * 9.81f * drop_height);
  const float dt = 1.0f / 60.0f;
  const float rebound_speed = restitution * (impact_speed + 9.81f * dt);
  // 5 mm for rounding (the penetration slop), on both sides
  EXPECT_GT(*run.measured, run.predicted - rebound_speed * dt - 0.005f);
  EXPECT_LT(*run.measured,
            run.predicted + restitution * rebound_speed * dt + 0.005f);
}

INSTANTIATE_TEST_SUITE_P(
    HeightsAndRestitutions, BounceTest,
    ::testing::Values(std::pair{100, 25}, std::pair{100, 100},
                      std::pair{400, 50}, std::pair{400, 75},
                      std::pair{600, 25}, std::pair{600, 100}));

// The controlled change has a measurable result: more restitution, higher
// apex
TEST(BounceTutorial, MoreRestitutionBouncesHigher) {
  float previous = -1.0f;
  for (int restitution_percent = 10; restitution_percent <= 100;
       restitution_percent += 10) {
    const auto run = RunBounce(400, restitution_percent);
    ASSERT_TRUE(run.measured.has_value()) << restitution_percent;
    EXPECT_GT(*run.measured, previous) << restitution_percent;
    previous = *run.measured;
  }
}

TEST(BounceTutorial, NoRestitutionNoBounce) {
  const auto run = RunBounce(400, 0);
  EXPECT_FALSE(run.measured.has_value());
}

TEST(BounceTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"00 Bounce", std::make_unique<BounceSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 120; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  const std::uint32_t first = run();
  registry.SetParameter(BounceSample::kRestitution, 80);
  const std::uint32_t changed = run();
  registry.SetParameter(BounceSample::kRestitution, 50);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

namespace {

// Parameters of the rotation experiment, in the units of its sliders:
// tenths of newtons, centimeters, tenths of kilograms, hundredths of seconds
struct RotationParameters {
  int force = 20;
  int lever_arm = 50;
  int mass = 10;
  int width = 200;
  int height = 100;
  int push_time = 100;
};

struct RotationRun {
  float predicted_inertia = 0.0f;
  float measured_inertia = 0.0f;
  float torque = 0.0f;
  float push_time = 0.0f;
  float predicted_spin = 0.0f;
  float spin = 0.0f;
  float spin_later = 0.0f;
  float predicted_turn = 0.0f;
  std::optional<float> measured_turn;
  bool center_moved = false;
};

// Runs the push, then one more second of coasting
RotationRun RunRotation(const RotationParameters& parameters) {
  RotationSample sample;
  auto values = sample.parameters();
  values[RotationSample::kForce].value = parameters.force;
  values[RotationSample::kLeverArm].value = parameters.lever_arm;
  values[RotationSample::kMass].value = parameters.mass;
  values[RotationSample::kWidth].value = parameters.width;
  values[RotationSample::kHeight].value = parameters.height;
  values[RotationSample::kPushTime].value = parameters.push_time;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  const neko::Vec2f start = sample.box().position;
  RotationRun run;
  for (int step = 0; step < sample.push_steps(); ++step) {
    sample.FixedUpdate();
  }
  run.spin = ToFloat(sample.spin());
  for (int step = 0; step < 60; ++step) {
    sample.FixedUpdate();
  }
  run.spin_later = ToFloat(sample.spin());
  run.predicted_inertia = ToFloat(sample.predicted_inertia());
  run.measured_inertia = ToFloat(sample.measured_inertia());
  run.torque = ToFloat(sample.torque());
  run.push_time = ToFloat(sample.push_time());
  run.predicted_spin = ToFloat(sample.predicted_spin());
  run.predicted_turn = ToFloat(sample.predicted_turn());
  if (sample.measured_turn().has_value()) {
    run.measured_turn = ToFloat(*sample.measured_turn());
  }
  run.center_moved = sample.box().position != start;
  sample.End();
  return run;
}

class RotationTest : public ::testing::TestWithParam<RotationParameters> {};

}  // namespace

// The experiment of chapter 01. A couple of torque 2 r F spins the box with
// alpha = tau / I: after T seconds omega = alpha T (exact with semi-implicit
// Euler, up to rounding), and the angle is alpha T^2 / 2 plus the time-step
// error alpha T dt / 2 (the angular velocity is updated before the angle).
TEST_P(RotationTest, SpinAndAngleFollowTorqueOverInertia) {
  const RotationParameters& parameters = GetParam();
  const auto run = RunRotation(parameters);
  const float mass = static_cast<float>(parameters.mass) / 10.0f;
  const float width = static_cast<float>(parameters.width) / 100.0f;
  const float height = static_cast<float>(parameters.height) / 100.0f;
  const float inertia = mass * (width * width + height * height) / 12.0f;
  const float torque = 2.0f * static_cast<float>(parameters.lever_arm) /
                       100.0f * static_cast<float>(parameters.force) / 10.0f;

  // Inertia from the box collider: m (w^2 + h^2) / 12
  EXPECT_NEAR(run.predicted_inertia, inertia, inertia * 0.002f);
  EXPECT_NEAR(run.measured_inertia, inertia, inertia * 0.005f);
  EXPECT_NEAR(run.torque, torque, torque * 0.001f);

  // Spin after the push, then coasting at that spin
  const float alpha = torque / inertia;
  const float spin = alpha * run.push_time;
  EXPECT_NEAR(run.predicted_spin, spin, spin * 0.005f);
  EXPECT_NEAR(run.spin, spin, spin * 0.01f);
  EXPECT_EQ(run.spin_later, run.spin) << "nothing slows it down";

  // Angle: ahead of alpha T^2 / 2 by about alpha T dt / 2
  ASSERT_TRUE(run.measured_turn.has_value());
  const float dt =
      run.push_time / static_cast<float>(parameters.push_time * 60 / 100);
  const float turn = alpha * run.push_time * run.push_time / 2.0f;
  const float time_step_error = alpha * run.push_time * dt / 2.0f;
  EXPECT_NEAR(run.predicted_turn, turn, turn * 0.005f);
  EXPECT_GT(*run.measured_turn, run.predicted_turn);
  EXPECT_NEAR(*run.measured_turn - turn, time_step_error,
              time_step_error * 0.25f + turn * 0.01f);

  EXPECT_FALSE(run.center_moved) << "a couple doesn't push the center";
}

INSTANTIATE_TEST_SUITE_P(
    Parameters, RotationTest,
    ::testing::Values(RotationParameters{},
                      RotationParameters{.force = 50, .lever_arm = 150},
                      RotationParameters{.mass = 50, .push_time = 200},
                      RotationParameters{.width = 50, .height = 300},
                      RotationParameters{
                          .force = 5, .lever_arm = 10, .push_time = 25}));

// The controlled changes have measurable results: twice the lever arm, twice
// the spin; twice the mass, half the spin
TEST(RotationTutorial, LeverArmAndMassScaleTheSpin) {
  const auto base = RunRotation({});
  const auto long_lever = RunRotation({.lever_arm = 100});
  const auto heavy = RunRotation({.mass = 20});
  EXPECT_NEAR(long_lever.spin / base.spin, 2.0f, 0.01f);
  EXPECT_NEAR(heavy.spin / base.spin, 0.5f, 0.005f);
}

TEST(RotationTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"01 Rotation", std::make_unique<RotationSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 120; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  const std::uint32_t first = run();
  registry.SetParameter(RotationSample::kForce, 40);
  const std::uint32_t changed = run();
  registry.SetParameter(RotationSample::kForce, 20);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

namespace {

struct RotatedBoxRun {
  float normal_x = 0.0f;
  float normal_y = 0.0f;
  std::optional<float> measured_x;
  std::optional<float> measured_y;
  std::optional<float> predicted_bounce_x;
  std::optional<float> predicted_bounce_y;
  std::optional<float> bounce_x;
  std::optional<float> bounce_y;
  std::optional<float> rebound_ratio;
};

// Drops the ball on the ramp and runs until the first bounce (5 s at most).
// Angle in degrees, restitution in percents, drop height in centimeters.
RotatedBoxRun RunRotatedBox(int angle, int restitution_percent,
                            int drop_height_cm = 250) {
  RotatedBoxSample sample;
  auto values = sample.parameters();
  values[RotatedBoxSample::kRampAngle].value = angle;
  values[RotatedBoxSample::kRestitution].value = restitution_percent;
  values[RotatedBoxSample::kDropHeight].value = drop_height_cm;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < 300 && !sample.measured_bounce().has_value();
       ++step) {
    sample.FixedUpdate();
  }
  RotatedBoxRun run;
  run.normal_x = ToFloat(sample.predicted_normal().x());
  run.normal_y = ToFloat(sample.predicted_normal().y());
  if (const auto normal = sample.measured_normal(); normal.has_value()) {
    run.measured_x = ToFloat(normal->x());
    run.measured_y = ToFloat(normal->y());
  }
  if (const auto bounce = sample.predicted_bounce(); bounce.has_value()) {
    run.predicted_bounce_x = ToFloat(bounce->x());
    run.predicted_bounce_y = ToFloat(bounce->y());
  }
  if (const auto bounce = sample.measured_bounce(); bounce.has_value()) {
    run.bounce_x = ToFloat(bounce->x());
    run.bounce_y = ToFloat(bounce->y());
  }
  if (const auto ratio = sample.rebound_ratio(); ratio.has_value()) {
    run.rebound_ratio = ToFloat(*ratio);
  }
  sample.End();
  return run;
}

class RotatedBoxTest : public ::testing::TestWithParam<std::pair<int, int>> {};

}  // namespace

// The experiment of chapter 02. The ramp's face normal is (-sin a, cos a);
// SAT picks it as the contact normal, the impulse is along it, and the ball
// leaves with w - (1 + e) (w . n) n: its normal speed scaled by e, its
// speed along the face unchanged.
TEST_P(RotatedBoxTest, BounceFollowsTheTurnedFaceNormal) {
  const auto [angle, restitution_percent] = GetParam();
  const auto run = RunRotatedBox(angle, restitution_percent);
  const float radians = static_cast<float>(angle) * 3.14159265f / 180.0f;
  EXPECT_NEAR(run.normal_x, -std::sin(radians), 0.001f);
  EXPECT_NEAR(run.normal_y, std::cos(radians), 0.001f);

  ASSERT_TRUE(run.measured_x.has_value());
  EXPECT_NEAR(*run.measured_x, run.normal_x, 0.002f);
  EXPECT_NEAR(*run.measured_y, run.normal_y, 0.002f);
  ASSERT_TRUE(run.bounce_x.has_value());
  ASSERT_TRUE(run.rebound_ratio.has_value());
  if (restitution_percent == 0) {
    // No bounce: the ball leaves at the Baumgarte push-out speed of its
    // overlap, at most baumgarte (0.2) times its approach speed
    EXPECT_GE(*run.rebound_ratio, 0.0f);
    EXPECT_LE(*run.rebound_ratio, 0.2f);
    return;
  }
  EXPECT_NEAR(*run.bounce_x, *run.predicted_bounce_x, 0.005f);
  EXPECT_NEAR(*run.bounce_y, *run.predicted_bounce_y, 0.005f);
  EXPECT_NEAR(*run.rebound_ratio,
              static_cast<float>(restitution_percent) / 100.0f, 0.003f);
}

INSTANTIATE_TEST_SUITE_P(AnglesAndRestitutions, RotatedBoxTest,
                         ::testing::Values(std::pair{0, 75}, std::pair{20, 75},
                                           std::pair{-30, 50},
                                           std::pair{45, 100},
                                           std::pair{-45, 25},
                                           std::pair{10, 0}));

// On a flat ramp the ball bounces straight up; tilted to the right
// (positive angle) its face points up-left, so the ball leaves to the left
TEST(RotatedBoxTutorial, TiltDecidesTheBounceSide) {
  const auto flat = RunRotatedBox(0, 75);
  ASSERT_TRUE(flat.bounce_x.has_value());
  EXPECT_EQ(*flat.bounce_x, 0.0f);
  EXPECT_GT(*flat.bounce_y, 0.0f);
  const auto left = RunRotatedBox(30, 75);
  const auto right = RunRotatedBox(-30, 75);
  ASSERT_TRUE(left.bounce_x.has_value() && right.bounce_x.has_value());
  EXPECT_LT(*left.bounce_x, 0.0f);
  EXPECT_GT(*right.bounce_x, 0.0f);
  EXPECT_NEAR(*left.bounce_x, -*right.bounce_x, 0.005f) << "mirror images";
}

TEST(RotatedBoxTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"02 Rotated box", std::make_unique<RotatedBoxSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 120; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  const std::uint32_t first = run();
  registry.SetParameter(RotatedBoxSample::kRampAngle, -20);
  const std::uint32_t changed = run();
  registry.SetParameter(RotatedBoxSample::kRampAngle, 20);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

namespace {

struct ManifoldParameters {
  int angle = 5;
  int sink_depth = 20;
  int width = 200;
  int spin = 0;
};

// Checks the manifold against the predicted corners of the incident face
void ExpectManifoldMatchesCorners(const ContactManifoldSample& sample) {
  EXPECT_EQ(sample.measured_point_count(), sample.predicted_point_count());
  for (std::size_t corner = 0; corner < 2; ++corner) {
    const auto predicted = sample.predicted_depth(corner);
    const auto measured = sample.measured_depth(corner);
    ASSERT_EQ(measured.has_value(), predicted.has_value()) << corner;
    if (predicted.has_value()) {
      EXPECT_NEAR(ToFloat(*measured), ToFloat(*predicted), 0.002f) << corner;
    }
  }
}

void BeginManifold(ContactManifoldSample& sample,
                   const ManifoldParameters& parameters) {
  auto values = sample.parameters();
  values[ContactManifoldSample::kAngle].value = parameters.angle;
  values[ContactManifoldSample::kSinkDepth].value = parameters.sink_depth;
  values[ContactManifoldSample::kWidth].value = parameters.width;
  values[ContactManifoldSample::kSpin].value = parameters.spin;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
}

class ContactManifoldTest
    : public ::testing::TestWithParam<ManifoldParameters> {};

}  // namespace

// The experiment of chapter 03: the manifold is the incident face's corners
// that are below the reference face, each with its depth below it
TEST_P(ContactManifoldTest, PointsAreTheSunkCornersOfTheIncidentFace) {
  ContactManifoldSample sample;
  BeginManifold(sample, GetParam());
  ExpectManifoldMatchesCorners(sample);
  sample.End();
}

INSTANTIATE_TEST_SUITE_P(
    AnglesAndDepths, ContactManifoldTest,
    ::testing::Values(ManifoldParameters{.angle = 0}, ManifoldParameters{},
                      ManifoldParameters{.angle = -10, .sink_depth = 30},
                      ManifoldParameters{.angle = 20, .sink_depth = 10},
                      ManifoldParameters{.angle = 40, .width = 400},
                      ManifoldParameters{.angle = 0, .sink_depth = 0}));

// The controlled change: flat, two points at the same depth; slightly tilted,
// two points at different depths; tilted further, one corner
TEST(ContactManifoldTutorial, TiltTurnsTwoPointsIntoOne) {
  const auto count = [](int angle) {
    ContactManifoldSample sample;
    BeginManifold(sample, {.angle = angle, .sink_depth = 20});
    const int points = sample.measured_point_count();
    sample.End();
    return points;
  };
  EXPECT_EQ(count(0), 2);
  EXPECT_EQ(count(5), 2);
  EXPECT_EQ(count(20), 1);
  EXPECT_EQ(count(-20), 1);
}

// Spinning through every face: the manifold keeps following the corners
TEST(ContactManifoldTutorial, SpinningBoxKeepsMatchingTheCorners) {
  ContactManifoldSample sample;
  BeginManifold(sample, {.angle = 0, .sink_depth = 30, .spin = 60});
  for (int step = 0; step < 6 * 60; ++step) {
    sample.FixedUpdate();
    ExpectManifoldMatchesCorners(sample);
    if (::testing::Test::HasFailure()) {
      FAIL() << "step " << step;
    }
  }
  sample.End();
}

TEST(ContactManifoldTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back(
      {"03 Contact manifold", std::make_unique<ContactManifoldSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 120; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  registry.SetParameter(ContactManifoldSample::kSpin, 30);
  const std::uint32_t first = run();
  registry.SetParameter(ContactManifoldSample::kSpin, 45);
  const std::uint32_t changed = run();
  registry.SetParameter(ContactManifoldSample::kSpin, 30);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

namespace {

struct ImpactParameters {
  int tilt = 20;
  int restitution = 50;
  int speed = 200;
  int width = 200;
};

struct ImpactRun {
  std::optional<float> lever_arm;
  float predicted_velocity = 0.0f;
  float measured_velocity = 0.0f;
  float predicted_spin = 0.0f;
  float measured_spin = 0.0f;
  float corner_rebound = 0.0f;
};

// Runs until the box hits the ground (10 s at most)
ImpactRun RunImpact(const ImpactParameters& parameters) {
  OffCenterImpactSample sample;
  auto values = sample.parameters();
  values[OffCenterImpactSample::kTilt].value = parameters.tilt;
  values[OffCenterImpactSample::kRestitution].value = parameters.restitution;
  values[OffCenterImpactSample::kSpeed].value = parameters.speed;
  values[OffCenterImpactSample::kWidth].value = parameters.width;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < 600 && !sample.lever_arm().has_value(); ++step) {
    sample.FixedUpdate();
  }
  ImpactRun run;
  if (sample.lever_arm().has_value()) {
    run.lever_arm = ToFloat(*sample.lever_arm());
    run.predicted_velocity = ToFloat(*sample.predicted_velocity());
    run.measured_velocity = ToFloat(*sample.measured_velocity());
    run.predicted_spin = ToFloat(*sample.predicted_spin());
    run.measured_spin = ToFloat(*sample.measured_spin());
    run.corner_rebound = ToFloat(*sample.corner_rebound());
  }
  sample.End();
  return run;
}

class OffCenterImpactTest : public ::testing::TestWithParam<ImpactParameters> {
};

}  // namespace

// The experiment of chapter 04: the impulse at the corner, from
// k = 1/m + (r x n)^2 / I, gives the velocity and the spin after the hit,
// and the corner leaves at -e times its approach speed
TEST_P(OffCenterImpactTest, ImpactMatchesTheImpulseAtTheCorner) {
  const ImpactParameters& parameters = GetParam();
  const auto run = RunImpact(parameters);
  ASSERT_TRUE(run.lever_arm.has_value());
  EXPECT_LT(*run.lever_arm, 0.0f) << "the left corner hits";
  if (parameters.restitution == 0) {
    // No bounce: the corner leaves at the Baumgarte push-out speed of its
    // overlap, at most baumgarte (0.2) times its approach speed
    EXPECT_GE(run.corner_rebound, 0.0f);
    EXPECT_LE(run.corner_rebound, 0.2f);
  } else {
    EXPECT_NEAR(run.measured_velocity, run.predicted_velocity, 0.005f);
    EXPECT_NEAR(run.measured_spin, run.predicted_spin, 0.01f);
    EXPECT_NEAR(run.corner_rebound,
                static_cast<float>(parameters.restitution) / 100.0f, 0.005f);
  }
  // Analytic check of the prediction itself, in float
  const float width = static_cast<float>(parameters.width) / 100.0f;
  const float inertia = (width * width + 1.0f) / 12.0f;
  const float speed = static_cast<float>(parameters.speed) / 100.0f;
  const float e = static_cast<float>(parameters.restitution) / 100.0f;
  const float impulse =
      (1.0f + e) * speed / (1.0f + *run.lever_arm * *run.lever_arm / inertia);
  EXPECT_NEAR(run.predicted_velocity, -speed + impulse, 0.005f);
  EXPECT_NEAR(run.predicted_spin, *run.lever_arm * impulse / inertia, 0.01f);
}

INSTANTIATE_TEST_SUITE_P(
    TiltsAndRestitutions, OffCenterImpactTest,
    ::testing::Values(ImpactParameters{}, ImpactParameters{.tilt = 5},
                      ImpactParameters{.tilt = 40, .restitution = 100},
                      ImpactParameters{.restitution = 0, .speed = 50},
                      ImpactParameters{.tilt = 10, .width = 300},
                      ImpactParameters{.tilt = 30, .width = 100}));

// The controlled change: the flatter the box lands, the longer the lever arm
// of its corner, the larger k = 1/m + l^2 / I and the less it bounces. The
// spin l j / I = (1 + e) v l / (1/m + l^2 / I) peaks when the lever arm is
// the radius of gyration sqrt(I / m) (0.645 m for the 2 x 1 box): at 30
// degrees, whose lever arm (0.616 m) is the closest
TEST(OffCenterImpactTutorial, LeverArmSharesTheImpulseBetweenBounceAndSpin) {
  float previous_lever = -10.0f;
  float previous_velocity = -10.0f;
  int fastest_spin_tilt = 0;
  float fastest_spin = 0.0f;
  for (int tilt = 5; tilt <= 40; tilt += 5) {
    const auto run = RunImpact({.tilt = tilt, .restitution = 100});
    ASSERT_TRUE(run.lever_arm.has_value()) << tilt;
    // Tilted more: shorter lever arm (closer to 0), more bounce
    EXPECT_GT(*run.lever_arm, previous_lever) << tilt;
    EXPECT_GT(run.measured_velocity, previous_velocity) << tilt;
    previous_lever = *run.lever_arm;
    previous_velocity = run.measured_velocity;
    if (-run.measured_spin > fastest_spin) {
      fastest_spin = -run.measured_spin;
      fastest_spin_tilt = tilt;
    }
  }
  EXPECT_EQ(fastest_spin_tilt, 30);
}

TEST(OffCenterImpactTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back(
      {"04 Off-center impact", std::make_unique<OffCenterImpactSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 120; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  const std::uint32_t first = run();
  registry.SetParameter(OffCenterImpactSample::kTilt, 30);
  const std::uint32_t changed = run();
  registry.SetParameter(OffCenterImpactSample::kTilt, 20);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

namespace {

struct FrictionRun {
  bool predicted_holds = false;
  float predicted_acceleration = 0.0f;
  float measured_acceleration = 0.0f;
  float distance = 0.0f;
};

// Runs the ramp for one second (the measurement); angle in degrees,
// friction in percents
FrictionRun RunFriction(int angle, int friction_percent) {
  FrictionSample sample;
  auto values = sample.parameters();
  values[FrictionSample::kRampAngle].value = angle;
  values[FrictionSample::kFriction].value = friction_percent;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < FrictionSample::kMeasureSteps; ++step) {
    sample.FixedUpdate();
  }
  FrictionRun run;
  run.predicted_holds = sample.predicted_holds();
  run.predicted_acceleration = ToFloat(sample.predicted_acceleration());
  run.measured_acceleration = ToFloat(*sample.measured_acceleration());
  run.distance = ToFloat(sample.distance());
  sample.End();
  return run;
}

class FrictionTest : public ::testing::TestWithParam<std::pair<int, int>> {};

}  // namespace

// The experiment of chapter 05: the crate holds when tan a <= mu, and
// otherwise slides at g (sin a - mu cos a). Measured with FIXED16 within
// 0.001 m/s^2 away from the boundary tan a = mu
TEST_P(FrictionTest, CrateHoldsOrSlidesAtGTimesSinMinusMuCos) {
  const auto [angle, friction_percent] = GetParam();
  const auto run = RunFriction(angle, friction_percent);
  const float radians = static_cast<float>(angle) * 3.14159265f / 180.0f;
  const float mu = static_cast<float>(friction_percent) / 100.0f;
  EXPECT_EQ(run.predicted_holds, std::tan(radians) <= mu);
  if (run.predicted_holds) {
    EXPECT_LT(std::abs(run.distance), 0.005f) << "held";
    EXPECT_LT(std::abs(run.measured_acceleration), 0.005f);
  } else {
    EXPECT_NEAR(run.predicted_acceleration,
                9.81f * (std::sin(radians) - mu * std::cos(radians)), 0.005f);
    // Within 0.001 with FIXED16; 3 % allows FLOAT's rounding of the start
    EXPECT_NEAR(run.measured_acceleration, run.predicted_acceleration,
                std::max(0.005f, 0.03f * run.predicted_acceleration));
  }
}

INSTANTIATE_TEST_SUITE_P(AnglesAndFrictions, FrictionTest,
                         ::testing::Values(std::pair{30, 40}, std::pair{20, 50},
                                           std::pair{45, 20}, std::pair{35, 50},
                                           std::pair{10, 0},
                                           std::pair{30, 60}));

TEST(FrictionTutorial, MoreFrictionSlowerSlide) {
  float previous = 100.0f;
  for (int friction = 0; friction <= 50; friction += 10) {
    const auto run = RunFriction(35, friction);
    EXPECT_LT(run.measured_acceleration, previous) << friction;
    previous = run.measured_acceleration;
  }
}

namespace {

struct StackRun {
  float sink = 0.0f;
  float drift = 0.0f;
};

// Runs the stack for 5 s
StackRun RunStacking(int boxes, int passes, bool warm_start, bool oriented) {
  StackingSample sample;
  auto values = sample.parameters();
  values[StackingSample::kBoxes].value = boxes;
  values[StackingSample::kPasses].value = passes;
  values[StackingSample::kWarmStart].value = warm_start ? 1 : 0;
  values[StackingSample::kOriented].value = oriented ? 1 : 0;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < 300; ++step) {
    sample.FixedUpdate();
  }
  StackRun run{ToFloat(sample.top_sink()), ToFloat(sample.drift())};
  sample.End();
  return run;
}

}  // namespace

// The experiment of chapter 06. With warm starting, the stack keeps within
// its slops (boxes * 5 mm); without, the same passes don't build up the
// bottom impulses: it sinks into itself, and turning boxes fall
TEST(StackingTutorial, WarmStartingHoldsTheStackWithFewPasses) {
  const auto warm = RunStacking(6, 4, true, true);
  const auto cold = RunStacking(6, 4, false, true);
  EXPECT_LT(warm.sink, 6 * 0.005f);
  EXPECT_LT(warm.drift, 0.05f);
  EXPECT_GT(cold.sink, 1.0f) << "the stack fell";
}

TEST(StackingTutorial, ColdStacksNeedMorePasses) {
  // AABBs can't fall, but they sink: 2 passes sink more than 8
  const auto two = RunStacking(6, 2, false, false);
  const auto eight = RunStacking(6, 8, false, false);
  EXPECT_GT(two.sink, eight.sink);
  EXPECT_GT(eight.sink, 6 * 0.005f) << "still more than the slops";
  EXPECT_LT(RunStacking(6, 8, true, false).sink, 6 * 0.005f);
}

// Ten AABBs with warm starting stay within their slops. Ten turning boxes
// with 8 passes are at the edge: they hold with FIXED16 but lean and drift
// with FLOAT, so only the AABB stack is pinned
TEST(StackingTutorial, TallStackWithWarmStarting) {
  const auto run = RunStacking(10, 8, true, false);
  EXPECT_LT(run.sink, 10 * 0.005f);
  EXPECT_EQ(run.drift, 0.0f) << "AABBs don't turn";
}

TEST(StackingTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"06 Stacking", std::make_unique<StackingSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 120; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  const std::uint32_t first = run();
  registry.SetParameter(StackingSample::kPasses, 3);
  const std::uint32_t changed = run();
  registry.SetParameter(StackingSample::kPasses, 8);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
