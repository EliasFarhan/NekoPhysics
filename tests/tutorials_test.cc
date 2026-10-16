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

namespace {

using neko::tutorial::BounceSample;
using neko::tutorial::ContactManifoldSample;
using neko::tutorial::RotatedBoxSample;
using neko::tutorial::RotationSample;

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
// time step. Semi-implicit Euler samples the arc every dt, so the measured
// apex is below the prediction, by less than the impact speed times dt.
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
  EXPECT_LT(*run.measured, run.predicted);
  EXPECT_GT(*run.measured, run.predicted - impact_speed * dt);
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
  EXPECT_NEAR(*run.bounce_x, *run.predicted_bounce_x, 0.005f);
  EXPECT_NEAR(*run.bounce_y, *run.predicted_bounce_y, 0.005f);
  ASSERT_TRUE(run.rebound_ratio.has_value());
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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
