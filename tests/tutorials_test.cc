#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "tutorials/00_bounce/bounce_sample.h"
#include "tutorials/01_rotation/rotation_sample.h"
#include "tutorials/01_rotation/solutions/disk_sample.h"
#include "tutorials/01_rotation/solutions/dumbbell_sample.h"
#include "tutorials/01_rotation/solutions/half_step_sample.h"
#include "tutorials/01_rotation/solutions/single_force_sample.h"
#include "tutorials/02_rotated_box/rotated_box_sample.h"
#include "tutorials/02_rotated_box/solutions/box_on_ramp_sample.h"
#include "tutorials/02_rotated_box/solutions/corner_sample.h"
#include "tutorials/02_rotated_box/solutions/tight_bounds_sample.h"
#include "tutorials/02_rotated_box/solutions/two_ramps_sample.h"
#include "tutorials/03_contact_manifold/contact_manifold_sample.h"
#include "tutorials/03_contact_manifold/solutions/box_on_tilted_box_sample.h"
#include "tutorials/03_contact_manifold/solutions/narrow_pillar_sample.h"
#include "tutorials/03_contact_manifold/solutions/points_over_a_turn_sample.h"
#include "tutorials/03_contact_manifold/solutions/two_aabbs_sample.h"
#include "tutorials/04_off_center_impact/off_center_impact_sample.h"
#include "tutorials/04_off_center_impact/solutions/ball_hits_box_sample.h"
#include "tutorials/04_off_center_impact/solutions/energy_budget_sample.h"
#include "tutorials/04_off_center_impact/solutions/fastest_spin_sample.h"
#include "tutorials/04_off_center_impact/solutions/passes_sample.h"
#include "tutorials/05_friction/friction_sample.h"
#include "tutorials/05_friction/solutions/mixed_surfaces_sample.h"
#include "tutorials/05_friction/solutions/push_to_start_sample.h"
#include "tutorials/05_friction/solutions/rolling_disk_sample.h"
#include "tutorials/05_friction/solutions/stopping_distance_sample.h"
#include "tutorials/06_stacking/solutions/heavy_on_light_sample.h"
#include "tutorials/06_stacking/solutions/impulse_per_box_sample.h"
#include "tutorials/06_stacking/solutions/pyramid_sample.h"
#include "tutorials/06_stacking/solutions/two_boxes_by_hand_sample.h"
#include "tutorials/06_stacking/stacking_sample.h"
#include "tutorials/07_pendulum/pendulum_sample.h"
#include "tutorials/08_chain/chain_sample.h"
#include "tutorials/09_spring/spring_sample.h"
#include "tutorials/10_explosion/explosion_sample.h"
#include "tutorials/11_jetpack/jetpack_sample.h"

namespace {

using neko::Scalar;
using neko::tutorial::BallHitsBoxSample;
using neko::tutorial::BounceSample;
using neko::tutorial::BoxOnRampSample;
using neko::tutorial::BoxOnTiltedBoxSample;
using neko::tutorial::ChainSample;
using neko::tutorial::ContactManifoldSample;
using neko::tutorial::CornerSample;
using neko::tutorial::DiskSample;
using neko::tutorial::DumbbellSample;
using neko::tutorial::EnergyBudgetSample;
using neko::tutorial::ExplosionSample;
using neko::tutorial::FastestSpinSample;
using neko::tutorial::FrictionSample;
using neko::tutorial::HalfStepSample;
using neko::tutorial::HeavyOnLightSample;
using neko::tutorial::ImpulsePerBoxSample;
using neko::tutorial::JetpackSample;
using neko::tutorial::MixedSurfacesSample;
using neko::tutorial::NarrowPillarSample;
using neko::tutorial::OffCenterImpactSample;
using neko::tutorial::PassesSample;
using neko::tutorial::PendulumSample;
using neko::tutorial::PointsOverATurnSample;
using neko::tutorial::PushToStartSample;
using neko::tutorial::PyramidSample;
using neko::tutorial::RollingDiskSample;
using neko::tutorial::RotatedBoxSample;
using neko::tutorial::RotationSample;
using neko::tutorial::SingleForceSample;
using neko::tutorial::SpringSample;
using neko::tutorial::StackingSample;
using neko::tutorial::StoppingDistanceSample;
using neko::tutorial::TightBoundsSample;
using neko::tutorial::TwoAabbsSample;
using neko::tutorial::TwoBoxesByHandSample;
using neko::tutorial::TwoRampsSample;

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

// Starts a solution sample with the given parameter values (index, value)
template <typename SampleType>
void BeginWith(SampleType& sample,
               std::initializer_list<std::pair<std::size_t, int>> values) {
  for (const auto& [index, value] : values) {
    sample.parameters()[index].value = value;
  }
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
}

template <typename SampleType>
void RunSteps(SampleType& sample, int steps) {
  for (int step = 0; step < steps; ++step) {
    sample.FixedUpdate();
  }
}

}  // namespace

// Chapter 01, exercise "single force": one force F at the lever arm r moves
// the center at F T / m and spins the box at r F T / I, half the couple's
// spin with the same F and r
TEST(RotationSolutions, SingleForceMovesTheCenterAndSpinsAtHalfTheCouple) {
  SingleForceSample sample;
  BeginWith(sample, {});
  RunSteps(sample, sample.push_steps());
  const float speed = ToFloat(sample.box().velocity.y());
  const float spin = ToFloat(sample.box().angular_velocity);
  const float predicted_speed = ToFloat(sample.predicted_speed());
  const float predicted_spin = ToFloat(sample.predicted_spin());
  EXPECT_EQ(ToFloat(sample.box().velocity.x()), 0.0f);
  EXPECT_NEAR(predicted_speed, 2.0f, 0.002f);
  EXPECT_NEAR(speed, predicted_speed, predicted_speed * 0.002f);
  EXPECT_NEAR(predicted_spin, 2.4f, 0.005f);
  EXPECT_NEAR(spin, predicted_spin, predicted_spin * 0.005f);
  const auto couple = RunRotation({});
  EXPECT_NEAR(spin / couple.spin, 0.5f, 0.003f);
  RunSteps(sample, 60);
  EXPECT_EQ(ToFloat(sample.box().velocity.y()), speed) << "coasting";
  EXPECT_EQ(ToFloat(sample.box().angular_velocity), spin) << "coasting";
  sample.End();

  // No lever arm: the force only moves the center
  BeginWith(sample, {{SingleForceSample::kLeverArm, 0}});
  RunSteps(sample, sample.push_steps());
  EXPECT_EQ(ToFloat(sample.box().angular_velocity), 0.0f);
  EXPECT_NEAR(ToFloat(sample.box().velocity.y()), predicted_speed,
              predicted_speed * 0.002f);
  sample.End();
}

// Chapter 01, exercise "disk": I = m R^2 / 2, and a disk of radius
// sqrt((w^2 + h^2) / 6) spins like the 2 x 1 m box
TEST(RotationSolutions, DiskInertiaIsHalfMassTimesRadiusSquared) {
  for (const int radius_cm : {25, 50, 91, 150}) {
    DiskSample sample;
    BeginWith(sample, {{DiskSample::kRadius, radius_cm}});
    const float radius = static_cast<float>(radius_cm) / 100.0f;
    const float inertia = radius * radius / 2.0f;
    EXPECT_NEAR(ToFloat(sample.predicted_inertia()), inertia, inertia * 0.002f)
        << radius_cm;
    EXPECT_NEAR(ToFloat(sample.measured_inertia()), inertia, inertia * 0.005f)
        << radius_cm;
    RunSteps(sample, sample.push_steps());
    const float spin = ToFloat(sample.disk().angular_velocity);
    const float predicted = ToFloat(sample.predicted_spin());
    EXPECT_NEAR(spin, predicted, predicted * 0.01f) << radius_cm;
    if (radius_cm == 91) {
      EXPECT_NEAR(spin, ToFloat(sample.box_spin()),
                  ToFloat(sample.box_spin()) * 0.01f);
    }
    sample.End();
  }
  EXPECT_NEAR(ToFloat(DiskSample::matching_radius()), 0.9129f, 0.001f);
}

// Chapter 01, exercise "half step": the spin is alpha T for any dt, and the
// angle's excess alpha T dt / 2 shrinks with dt
TEST(RotationSolutions, HalfStepShrinksTheAngleErrorWithTheTimeStep) {
  std::vector<float> excesses;
  for (const int substeps : {1, 2, 3, 4}) {
    HalfStepSample sample;
    BeginWith(sample, {{HalfStepSample::kSubsteps, substeps}});
    EXPECT_EQ(ToFloat(sample.dt() * static_cast<Scalar>(substeps)),
              ToFloat(neko::Sample::kFixedDeltaTime))
        << "exact substeps";
    RunSteps(sample, sample.push_frames());
    ASSERT_TRUE(sample.measured_turn().has_value());
    const float alpha = ToFloat(sample.angular_acceleration());
    const float time = ToFloat(sample.push_time());
    EXPECT_NEAR(ToFloat(sample.spin()), alpha * time, alpha * time * 0.01f)
        << substeps;
    const float excess =
        ToFloat(*sample.measured_turn()) - ToFloat(sample.predicted_turn());
    const float predicted = ToFloat(sample.predicted_excess());
    EXPECT_NEAR(excess, predicted, predicted * 0.1f + 0.002f) << substeps;
    excesses.push_back(excess);
    sample.End();
  }
  EXPECT_NEAR(excesses[0] / excesses[1], 2.0f, 0.2f);
  EXPECT_NEAR(excesses[0] / excesses[3], 4.0f, 0.6f);
}

// Chapter 01, exercise "dumbbell": each box adds (m / 2) d^2 to the inertia
// (parallel axis theorem), I = m (s^2 / 6 + d^2)
TEST(RotationSolutions, DumbbellFollowsTheParallelAxisTheorem) {
  std::vector<float> inertias;
  for (const int offset_cm : {50, 100, 200}) {
    DumbbellSample sample;
    BeginWith(sample, {{DumbbellSample::kOffset, offset_cm}});
    const float offset = static_cast<float>(offset_cm) / 100.0f;
    const float inertia = 0.25f / 6.0f + offset * offset;
    EXPECT_NEAR(ToFloat(sample.predicted_inertia()), inertia, inertia * 0.002f)
        << offset_cm;
    EXPECT_NEAR(ToFloat(sample.offset_inertia()), offset * offset, 0.001f)
        << offset_cm;
    EXPECT_NEAR(ToFloat(sample.measured_inertia()), inertia, inertia * 0.005f)
        << offset_cm;
    RunSteps(sample, sample.push_steps());
    const float predicted = ToFloat(sample.predicted_spin());
    EXPECT_NEAR(ToFloat(sample.dumbbell().angular_velocity), predicted,
                predicted * 0.01f)
        << offset_cm;
    inertias.push_back(ToFloat(sample.measured_inertia()));
    sample.End();
  }
  EXPECT_GT(inertias[1], inertias[0]);
  EXPECT_GT(inertias[2], inertias[1]);
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

// Chapter 02, exercise "corner": when the clamp lands on the ramp's corner,
// the normal points from the corner to the ball's center (at the step where
// the world found the contact), not along the face normal
TEST(RotatedBoxSolutions, CornerNormalPointsFromTheCornerToTheCenter) {
  for (const int angle : {0, 10, 20, 30}) {
    for (const int offset_cm : {0, 5, 10, 15}) {
      CornerSample sample;
      BeginWith(sample, {{CornerSample::kRampAngle, angle},
                         {CornerSample::kOffset, offset_cm}});
      for (int step = 0; step < 300 && !sample.measured_normal().has_value();
           ++step) {
        sample.FixedUpdate();
      }
      ASSERT_TRUE(sample.measured_normal().has_value()) << angle << offset_cm;
      ASSERT_TRUE(sample.predicted_normal().has_value());
      EXPECT_TRUE(sample.hit_corner()) << angle << " " << offset_cm;
      const neko::Vec2f measured = *sample.measured_normal();
      const neko::Vec2f predicted = *sample.predicted_normal();
      EXPECT_NEAR(ToFloat(measured.x()), ToFloat(predicted.x()), 0.002f)
          << angle << " " << offset_cm;
      EXPECT_NEAR(ToFloat(measured.y()), ToFloat(predicted.y()), 0.002f)
          << angle << " " << offset_cm;
      if (angle >= 10) {
        EXPECT_LT(ToFloat(neko::Vec2f::Dot(measured, sample.face_normal())),
                  0.99f)
            << "not the face normal";
      }
      ASSERT_TRUE(sample.sink().has_value());
      EXPECT_GE(ToFloat(*sample.sink()), 0.0f);
      sample.End();
    }
  }
}

// Chapter 02, exercise "tight bounds": the broadphase box of a turning box
// is |cos| hx + |sin| hy by |sin| hx + |cos| hy, as wide as the farthest
// corner; a probe out of the box's reach is paired with it whenever the
// bounds' corner covers it, and never touched
TEST(RotatedBoxSolutions, TightBoundsPairTheProbeWithoutTouchingIt) {
  for (const auto& [width, height, spin] :
       {std::tuple{200, 100, 100}, std::tuple{300, 100, 300},
        std::tuple{100, 250, 170}}) {
    TightBoundsSample sample;
    BeginWith(sample, {{TightBoundsSample::kWidth, width},
                       {TightBoundsSample::kHeight, height},
                       {TightBoundsSample::kSpin, spin}});
    // Two turns
    const int steps = 2 * 377 * 100 / spin + 1;
    for (int step = 0; step < steps; ++step) {
      sample.FixedUpdate();
      const neko::Vec2f bounds = sample.predicted_extents();
      const neko::Vec2f corners = sample.corner_extents();
      ASSERT_NEAR(ToFloat(bounds.x()), ToFloat(corners.x()), 0.001f) << step;
      ASSERT_NEAR(ToFloat(bounds.y()), ToFloat(corners.y()), 0.001f) << step;
    }
    EXPECT_EQ(sample.mismatches(), 0) << width << " " << height;
    EXPECT_EQ(sample.touching_steps(), 0) << width << " " << height;
    EXPECT_GT(sample.paired_steps(), 0) << width << " " << height;
    EXPECT_LT(sample.paired_steps(), sample.steps()) << width << " " << height;
    const float diagonal = ToFloat(sample.half_diagonal());
    EXPECT_NEAR(ToFloat(sample.widest()), diagonal, diagonal * 0.002f)
        << width << " " << height;
    sample.End();
  }
}

// Chapter 02, exercise "two ramps": the first bounce reflects about A's face,
// the flight to B takes the first root of the contact quadratic (within the
// time-step error), and the second bounce reflects about B's face
TEST(RotatedBoxSolutions, TwoRampsBounceTwiceAfterAPredictedFlight) {
  for (const int angle : {15, 20, 30}) {
    for (const int restitution : {50, 75, 100}) {
      TwoRampsSample sample;
      BeginWith(sample, {{TwoRampsSample::kRampAngle, angle},
                         {TwoRampsSample::kRestitution, restitution}});
      for (int step = 0; step < 600 && !sample.second_normal().has_value();
           ++step) {
        sample.FixedUpdate();
      }
      ASSERT_TRUE(sample.measured_bounce().has_value()) << angle;
      EXPECT_NEAR(ToFloat(sample.measured_bounce()->x()),
                  ToFloat(sample.predicted_bounce()->x()), 0.005f)
          << angle << " " << restitution;
      EXPECT_NEAR(ToFloat(sample.measured_bounce()->y()),
                  ToFloat(sample.predicted_bounce()->y()), 0.005f)
          << angle << " " << restitution;
      ASSERT_TRUE(sample.predicted_flight().has_value());
      ASSERT_TRUE(sample.second_normal().has_value())
          << angle << " " << restitution;
      const float dt = ToFloat(neko::Sample::kFixedDeltaTime);
      EXPECT_NEAR(ToFloat(*sample.measured_flight()),
                  ToFloat(*sample.predicted_flight()), 1.5f * dt)
          << angle << " " << restitution;
      EXPECT_NEAR(ToFloat(sample.second_normal()->x()),
                  ToFloat(sample.normal_b().x()), 0.002f)
          << angle << " " << restitution;
      EXPECT_NEAR(ToFloat(sample.second_normal()->y()),
                  ToFloat(sample.normal_b().y()), 0.002f)
          << angle << " " << restitution;
      EXPECT_NEAR(ToFloat(*sample.second_ratio()),
                  static_cast<float>(restitution) / 100.0f, 0.005f)
          << angle << " " << restitution;
      sample.End();
    }
  }
}

// Chapter 02, exercise "box on ramp": of the four face axes, the ramp's y
// axis has the smallest overlap; the contact normal is the ramp's face
// normal, whatever the angle of the falling box
TEST(RotatedBoxSolutions, BoxOnRampTakesTheAxisOfSmallestOverlap) {
  for (const auto& [ramp_angle, box_angle] :
       {std::pair{20, 0}, std::pair{20, 30}, std::pair{20, -45},
        std::pair{20, 90}, std::pair{20, 20}, std::pair{-15, 60}}) {
    BoxOnRampSample sample;
    BeginWith(sample, {{BoxOnRampSample::kRampAngle, ramp_angle},
                       {BoxOnRampSample::kBoxAngle, box_angle}});
    for (int step = 0; step < 300 && !sample.measured_normal().has_value();
         ++step) {
      sample.FixedUpdate();
    }
    ASSERT_TRUE(sample.measured_normal().has_value()) << box_angle;
    ASSERT_TRUE(sample.overlaps().has_value());
    EXPECT_EQ(sample.best_axis(), 1) << ramp_angle << " " << box_angle;
    for (const Scalar overlap : *sample.overlaps()) {
      EXPECT_GE(ToFloat(overlap), 0.0f) << "no separating axis";
    }
    const neko::Vec2f measured = *sample.measured_normal();
    EXPECT_NEAR(ToFloat(measured.x()), ToFloat(sample.predicted_normal()->x()),
                0.002f)
        << ramp_angle << " " << box_angle;
    EXPECT_NEAR(ToFloat(measured.y()), ToFloat(sample.predicted_normal()->y()),
                0.002f)
        << ramp_angle << " " << box_angle;
    EXPECT_NEAR(ToFloat(measured.x()), ToFloat(sample.ramp_normal().x()),
                0.002f);
    EXPECT_NEAR(ToFloat(measured.y()), ToFloat(sample.ramp_normal().y()),
                0.002f);
    sample.End();
  }
}

// The solution programs are deterministic: a restart replays the same run
TEST(TutorialSolutions, SameRunAfterRestart) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"single force", std::make_unique<SingleForceSample>()});
  samples.push_back({"disk", std::make_unique<DiskSample>()});
  samples.push_back({"half step", std::make_unique<HalfStepSample>()});
  samples.push_back({"dumbbell", std::make_unique<DumbbellSample>()});
  samples.push_back({"corner", std::make_unique<CornerSample>()});
  samples.push_back({"tight bounds", std::make_unique<TightBoundsSample>()});
  samples.push_back({"two ramps", std::make_unique<TwoRampsSample>()});
  samples.push_back({"box on ramp", std::make_unique<BoxOnRampSample>()});
  samples.push_back({"narrow pillar", std::make_unique<NarrowPillarSample>()});
  samples.push_back(
      {"points over a turn", std::make_unique<PointsOverATurnSample>()});
  samples.push_back({"two AABBs", std::make_unique<TwoAabbsSample>()});
  samples.push_back(
      {"box on a tilted box", std::make_unique<BoxOnTiltedBoxSample>()});
  samples.push_back({"fastest spin", std::make_unique<FastestSpinSample>()});
  samples.push_back({"energy budget", std::make_unique<EnergyBudgetSample>()});
  samples.push_back({"ball hits box", std::make_unique<BallHitsBoxSample>()});
  samples.push_back({"passes", std::make_unique<PassesSample>()});
  samples.push_back(
      {"stopping distance", std::make_unique<StoppingDistanceSample>()});
  samples.push_back({"rolling disk", std::make_unique<RollingDiskSample>()});
  samples.push_back(
      {"mixed surfaces", std::make_unique<MixedSurfacesSample>()});
  samples.push_back({"push to start", std::make_unique<PushToStartSample>()});
  samples.push_back(
      {"impulse per box", std::make_unique<ImpulsePerBoxSample>()});
  samples.push_back(
      {"two boxes by hand", std::make_unique<TwoBoxesByHandSample>()});
  samples.push_back({"heavy on light", std::make_unique<HeavyOnLightSample>()});
  samples.push_back({"pyramid", std::make_unique<PyramidSample>()});
  const std::size_t count = samples.size();
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 120; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  for (std::size_t i = 0; i < count; ++i) {
    registry.Select(i, context);
    const std::uint32_t first = run();
    registry.Select(i, context);
    EXPECT_EQ(run(), first) << i;
  }
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

// Chapter 03, exercise "narrow pillar": with the pillar's top as the
// reference face, the box's bottom edge is clipped to the pillar's side
// planes; an end past a plane lands on the pillar's edge, its depth
// interpolated along the box's edge
TEST(ContactManifoldSolutions, NarrowPillarClipsTheEdgeToThePillar) {
  for (const auto& [angle, offset_cm, sink_cm] :
       {std::tuple{5, 120, 10}, std::tuple{0, 120, 10}, std::tuple{10, 120, 20},
        std::tuple{0, 0, 10}}) {
    NarrowPillarSample sample;
    BeginWith(sample, {{NarrowPillarSample::kAngle, angle},
                       {NarrowPillarSample::kOffset, offset_cm},
                       {NarrowPillarSample::kSinkDepth, sink_cm}});
    ASSERT_TRUE(sample.pillar_is_reference()) << angle << " " << offset_cm;
    const auto predicted = sample.prediction();
    const neko::Contact& contact = sample.contact();
    ASSERT_EQ(static_cast<int>(contact.point_count), predicted.count);
    for (std::size_t i = 0; i < contact.point_count; ++i) {
      EXPECT_NEAR(ToFloat(contact.points[i].position.x()),
                  ToFloat(predicted.points[i].position.x()), 0.002f)
          << angle << " " << offset_cm << " " << i;
      EXPECT_NEAR(ToFloat(contact.points[i].penetration),
                  ToFloat(predicted.points[i].depth), 0.002f)
          << angle << " " << offset_cm << " " << i;
    }
    // The box overhangs the pillar's right edge: the second point is on it
    const float right_edge = ToFloat(sample.pillar_center().x()) + 0.5f;
    const std::size_t last = std::size_t{contact.point_count} - 1;
    EXPECT_NEAR(ToFloat(contact.points[last].position.x()), right_edge, 0.002f);
    sample.End();
  }
}

// Chapter 03, exercise "points over a turn": a face tilted by up to phi*
// keeps both corners down, with c + L sin phi* = N cos phi*; the turn has
// (4 phi*_bottom + 4 phi*_side) / 360 of its steps with two points, within
// half a step at each of the eight window edges
TEST(ContactManifoldSolutions, TwoPointsWhileTheFaceIsWithinPhiStar) {
  for (const auto& [sink_cm, spin] :
       {std::pair{30, 60}, std::pair{20, 60}, std::pair{45, 60},
        std::pair{10, 60}, std::pair{30, 30}}) {
    PointsOverATurnSample sample;
    BeginWith(sample, {{PointsOverATurnSample::kSinkDepth, sink_cm},
                       {PointsOverATurnSample::kSpin, spin}});
    while (!sample.turn_done()) {
      sample.FixedUpdate();
    }
    // phi* in closed form: sin(phi - beta) = -c / R, R = sqrt(L^2 + N^2),
    // tan beta = N / L, for the bottom face (L = 1, N = 0.5)
    const float c = 0.5f - static_cast<float>(sink_cm) / 100.0f;
    const float degrees = 180.0f / 3.14159265f;
    const float bottom =
        (std::atan2(0.5f, 1.0f) + std::asin(-c / std::sqrt(1.25f))) * degrees;
    EXPECT_NEAR(ToFloat(sample.bottom_threshold()), bottom, 0.01f) << sink_cm;
    EXPECT_NEAR(static_cast<float>(sample.two_point_steps()),
                ToFloat(sample.predicted_two_point_steps()), 4.0f)
        << sink_cm << " " << spin;
    EXPECT_EQ(sample.no_point_steps(), 0) << sink_cm;
    EXPECT_EQ(sample.two_point_steps() + sample.one_point_steps(),
              sample.steps_per_turn());
    sample.End();
  }
}

// Chapter 03, exercise "two AABBs": the smaller overlap gives the normal,
// and the points are the ends of the overlap on the block's face, both at
// the penetration. Ties are left out: an exact tie in Fixed16 can round
// either way in float.
TEST(ContactManifoldSolutions, TwoAabbsTouchAtTheEndsOfTheOverlap) {
  for (const auto& [on_ledge_cm, sink_cm] :
       {std::pair{60, 20}, std::pair{10, 20}, std::pair{100, 50},
        std::pair{5, 50}, std::pair{30, 10}}) {
    TwoAabbsSample sample;
    BeginWith(sample, {{TwoAabbsSample::kOnLedge, on_ledge_cm},
                       {TwoAabbsSample::kSinkDepth, sink_cm}});
    const neko::Contact& contact = sample.contact();
    ASSERT_EQ(contact.point_count, 2) << on_ledge_cm << " " << sink_cm;
    EXPECT_EQ(contact.contact_normal, sample.predicted_normal());
    EXPECT_NEAR(ToFloat(contact.penetration),
                ToFloat(sample.predicted_penetration()), 0.001f);
    const auto points = sample.predicted_points();
    for (std::size_t i = 0; i < 2; ++i) {
      EXPECT_NEAR(ToFloat(contact.points[i].position.x()),
                  ToFloat(points[i].x()), 0.001f)
          << on_ledge_cm << " " << sink_cm << " " << i;
      EXPECT_NEAR(ToFloat(contact.points[i].position.y()),
                  ToFloat(points[i].y()), 0.001f)
          << on_ledge_cm << " " << sink_cm << " " << i;
      EXPECT_EQ(contact.points[i].penetration, contact.penetration);
    }
    sample.End();
  }
}

// Chapter 03, exercise "box on a tilted box": chapter 03's prediction, in the
// ramp's frame, matches the manifold of two oriented boxes
TEST(ContactManifoldSolutions, TiltedBoxManifoldIsChapter03InTheRampFrame) {
  for (const auto& [ramp, tilt, slide_cm, sink_cm] :
       {std::tuple{20, 3, 0, 20}, std::tuple{20, 0, 0, 20},
        std::tuple{20, -5, 0, 20}, std::tuple{20, 15, 0, 20},
        std::tuple{-30, 5, 200, 30}, std::tuple{10, -20, -300, 40}}) {
    BoxOnTiltedBoxSample sample;
    BeginWith(sample, {{BoxOnTiltedBoxSample::kRampAngle, ramp},
                       {BoxOnTiltedBoxSample::kRelativeTilt, tilt},
                       {BoxOnTiltedBoxSample::kSlide, slide_cm},
                       {BoxOnTiltedBoxSample::kSinkDepth, sink_cm}});
    EXPECT_EQ(static_cast<int>(sample.contact().point_count),
              sample.predicted_point_count())
        << ramp << " " << tilt;
    for (std::size_t corner = 0; corner < 2; ++corner) {
      const auto predicted = sample.predicted_depth(corner);
      const auto measured = sample.measured_depth(corner);
      ASSERT_EQ(measured.has_value(), predicted.has_value())
          << ramp << " " << tilt << " " << corner;
      if (predicted.has_value()) {
        EXPECT_NEAR(ToFloat(*measured), ToFloat(*predicted), 0.002f)
            << ramp << " " << tilt << " " << corner;
      }
    }
    EXPECT_NEAR(ToFloat(sample.contact().contact_normal.x()),
                ToFloat(sample.predicted_normal().x()), 0.002f);
    EXPECT_NEAR(ToFloat(sample.contact().contact_normal.y()),
                ToFloat(sample.predicted_normal().y()), 0.002f);
    sample.End();
  }
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

// Runs a chapter 04 solution until its impact step (10 s at most)
template <typename SampleType, typename Done>
void RunUntilImpact(SampleType& sample, Done done) {
  for (int step = 0; step < 600 && !done(); ++step) {
    sample.FixedUpdate();
  }
}

}  // namespace

// Chapter 04, exercise "fastest spin": the measured spin over the integer
// tilts peaks where the predicted one does, where the lever arm is closest
// to the radius of gyration, at (1 + e) v / (2 k)
TEST(OffCenterImpactSolutions, FastestSpinIsAtTheRadiusOfGyration) {
  for (const int width_cm : {300, 200, 100}) {
    int fastest_tilt = 0;
    int predicted_tilt = 0;
    float fastest_spin = 0.0f;
    float bound = 0.0f;
    for (int tilt = 5; tilt <= 40; ++tilt) {
      FastestSpinSample sample;
      BeginWith(sample, {{FastestSpinSample::kTilt, tilt},
                         {FastestSpinSample::kWidth, width_cm}});
      RunUntilImpact(sample,
                     [&sample] { return sample.measured_spin().has_value(); });
      ASSERT_TRUE(sample.measured_spin().has_value()) << tilt;
      const float spin = -ToFloat(*sample.measured_spin());
      EXPECT_NEAR(spin, -ToFloat(sample.predicted_spin(tilt)), 0.01f) << tilt;
      if (spin > fastest_spin) {
        fastest_spin = spin;
        fastest_tilt = tilt;
      }
      predicted_tilt = sample.fastest_tilt();
      bound = ToFloat(sample.spin_bound());
      sample.End();
    }
    EXPECT_EQ(fastest_tilt, predicted_tilt) << width_cm;
    EXPECT_NEAR(fastest_spin, bound, 0.002f * bound) << width_cm;
    // hx cos a - hy sin a = k, solved in float
    const float hx = static_cast<float>(width_cm) / 200.0f;
    const float k = std::sqrt((4.0f * hx * hx + 1.0f) / 12.0f);
    const float radius = std::sqrt(hx * hx + 0.25f);
    const float best =
        (std::acos(k / radius) - std::atan2(0.5f, hx)) * 180.0f / 3.14159265f;
    EXPECT_NEAR(static_cast<float>(fastest_tilt), best, 1.0f) << width_cm;
  }
}

// Chapter 04, exercise "energy budget": the hit loses (1 - e^2) vn^2 / (2 k),
// less than a flat landing would, and nothing when it is elastic
TEST(OffCenterImpactSolutions, CornerHitLosesOneMinusESquaredOverK) {
  for (const auto& [tilt, restitution, width_cm] :
       {std::tuple{20, 50, 200}, std::tuple{20, 100, 200},
        std::tuple{40, 50, 200}, std::tuple{5, 20, 200},
        std::tuple{10, 70, 300}, std::tuple{30, 50, 100}}) {
    EnergyBudgetSample sample;
    BeginWith(sample, {{EnergyBudgetSample::kTilt, tilt},
                       {EnergyBudgetSample::kRestitution, restitution},
                       {EnergyBudgetSample::kWidth, width_cm}});
    RunUntilImpact(sample,
                   [&sample] { return sample.measured_loss().has_value(); });
    ASSERT_TRUE(sample.measured_loss().has_value()) << tilt;
    const float loss = ToFloat(*sample.measured_loss());
    EXPECT_NEAR(loss, ToFloat(*sample.predicted_loss()), 0.002f)
        << tilt << " " << restitution;
    if (restitution == 100) {
      EXPECT_NEAR(loss, 0.0f, 0.002f) << tilt;
    } else {
      EXPECT_LT(loss, ToFloat(sample.flat_loss())) << tilt;
    }
    sample.End();
  }
}

// Chapter 04, exercise "ball hits box": two moving bodies share the impulse
// j = (1 + e) v / (1/M + 1/m + x^2 / I); only the box spins
TEST(OffCenterImpactSolutions, BallAndBoxShareTheImpulse) {
  for (const auto& [offset_cm, ball_mass, restitution, speed] :
       {std::tuple{60, 50, 50, 300}, std::tuple{0, 50, 50, 300},
        std::tuple{-90, 50, 100, 300}, std::tuple{60, 200, 50, 300},
        std::tuple{30, 25, 20, 200}}) {
    BallHitsBoxSample sample;
    BeginWith(sample, {{BallHitsBoxSample::kOffset, offset_cm},
                       {BallHitsBoxSample::kBallMass, ball_mass},
                       {BallHitsBoxSample::kRestitution, restitution},
                       {BallHitsBoxSample::kSpeed, speed}});
    RunUntilImpact(
        sample, [&sample] { return sample.measured_box_spin().has_value(); });
    ASSERT_TRUE(sample.measured_box_spin().has_value()) << offset_cm;
    EXPECT_NEAR(ToFloat(*sample.measured_ball_velocity()),
                ToFloat(sample.predicted_ball_velocity()), 0.005f)
        << offset_cm << " " << ball_mass;
    EXPECT_NEAR(ToFloat(*sample.measured_box_velocity()),
                ToFloat(sample.predicted_box_velocity()), 0.005f)
        << offset_cm << " " << ball_mass;
    EXPECT_NEAR(ToFloat(*sample.measured_box_spin()),
                ToFloat(sample.predicted_box_spin()), 0.01f)
        << offset_cm << " " << ball_mass;
    EXPECT_NEAR(ToFloat(*sample.point_rebound()),
                static_cast<float>(restitution) / 100.0f, 0.005f)
        << offset_cm << " " << ball_mass;
    EXPECT_NEAR(ToFloat(*sample.measured_box_drift()), 0.0f, 0.001f);
    sample.End();
  }
}

// Chapter 04, exercise "one pass vs eight": the solver's loop by hand gives
// the world's velocity and spin after each number of passes; one pass leaves
// the box spinning, eight reach the exact answer
TEST(OffCenterImpactSolutions, PassesConvergeToTheSymmetricImpulse) {
  for (const int restitution : {50, 100}) {
    for (const int passes : {1, 2, 3, 4, 8}) {
      PassesSample sample;
      BeginWith(sample, {{PassesSample::kPasses, passes},
                         {PassesSample::kRestitution, restitution}});
      RunUntilImpact(sample,
                     [&sample] { return sample.measured().has_value(); });
      ASSERT_TRUE(sample.measured().has_value()) << passes;
      const auto predicted = sample.predicted(passes);
      EXPECT_NEAR(ToFloat(sample.measured()->velocity),
                  ToFloat(predicted.velocity), 0.005f)
          << restitution << " " << passes;
      EXPECT_NEAR(ToFloat(sample.measured()->spin), ToFloat(predicted.spin),
                  0.005f)
          << restitution << " " << passes;
      if (passes == 1) {
        EXPECT_GT(ToFloat(sample.measured()->spin), 0.5f) << restitution;
      }
      if (passes == 8) {
        EXPECT_NEAR(ToFloat(sample.measured()->velocity),
                    ToFloat(sample.exact_velocity()), 0.002f);
        EXPECT_NEAR(ToFloat(sample.measured()->spin), 0.0f, 0.002f);
      }
      sample.End();
    }
  }
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

// Chapter 05, exercise "stopping distance": kinetic friction brakes at mu g,
// so the box stops after v / (mu g) and v^2 / (2 mu g). The steps end on a
// whole step, and semi-implicit Euler moves with the speed at the end of each
// step: the distance comes out short by half a step's travel, v dt / 2
TEST(FrictionSolutions, BoxStopsAfterVOverMuGAndVSquaredOverTwoMuG) {
  constexpr float dt = 1.0f / 60.0f;
  for (const auto& [speed_cm, friction] :
       {std::pair{300, 50}, std::pair{100, 20}, std::pair{500, 20},
        std::pair{200, 75}, std::pair{500, 80}}) {
    StoppingDistanceSample sample;
    BeginWith(sample, {{StoppingDistanceSample::kSpeed, speed_cm},
                       {StoppingDistanceSample::kFriction, friction}});
    RunUntilImpact(sample,
                   [&sample] { return sample.measured_time().has_value(); });
    ASSERT_TRUE(sample.measured_time().has_value()) << speed_cm;
    const float speed = static_cast<float>(speed_cm) / 100.0f;
    const float deceleration = static_cast<float>(friction) / 100.0f * 9.81f;
    const float time = speed / deceleration;
    const float distance = speed * speed / (2.0f * deceleration);
    EXPECT_NEAR(ToFloat(sample.predicted_time()), time, 0.001f);
    EXPECT_NEAR(ToFloat(sample.predicted_distance()), distance, 0.002f);
    const float measured_time = ToFloat(*sample.measured_time());
    EXPECT_GE(measured_time, time - 0.001f) << speed_cm << " " << friction;
    EXPECT_LE(measured_time, time + dt + 0.001f) << speed_cm << " " << friction;
    EXPECT_NEAR(ToFloat(*sample.measured_distance()),
                distance - speed * dt / 2.0f, 0.01f)
        << speed_cm << " " << friction;
    sample.End();
  }
}

// Chapter 05, exercise "rolling disk": friction at the bottom point slows the
// center and spins the disk up until it rolls, at 2/3 of its speed whatever
// the friction, with w = -v / r; then nothing slows it
TEST(FrictionSolutions, SlidingDiskRollsAtTwoThirdsOfItsSpeed) {
  constexpr float dt = 1.0f / 60.0f;
  for (const auto& [speed_cm, friction, radius_cm] :
       {std::tuple{300, 30, 50}, std::tuple{300, 50, 50},
        std::tuple{600, 10, 25}, std::tuple{200, 100, 100},
        std::tuple{300, 30, 100}}) {
    RollingDiskSample sample;
    BeginWith(sample, {{RollingDiskSample::kSpeed, speed_cm},
                       {RollingDiskSample::kFriction, friction},
                       {RollingDiskSample::kRadius, radius_cm}});
    RunUntilImpact(sample, [&sample] { return sample.measured().has_value(); });
    ASSERT_TRUE(sample.measured().has_value()) << speed_cm;
    const auto rolling = *sample.measured();
    const float speed = static_cast<float>(speed_cm) / 100.0f;
    const float radius = static_cast<float>(radius_cm) / 100.0f;
    const float mu_g = static_cast<float>(friction) / 100.0f * 9.81f;
    EXPECT_NEAR(ToFloat(sample.predicted_time()), speed / (3.0f * mu_g),
                0.001f);
    EXPECT_NEAR(ToFloat(sample.predicted_distance()),
                5.0f * speed * speed / (18.0f * mu_g), 0.002f);
    EXPECT_NEAR(ToFloat(rolling.speed), 2.0f * speed / 3.0f, 0.002f)
        << speed_cm << " " << friction;
    EXPECT_NEAR(ToFloat(rolling.spin) * radius, -ToFloat(rolling.speed), 0.01f);
    const float time = ToFloat(rolling.time);
    EXPECT_GE(time, ToFloat(sample.predicted_time()) - 0.001f);
    EXPECT_LE(time, ToFloat(sample.predicted_time()) + dt + 0.001f);
    EXPECT_NEAR(ToFloat(rolling.distance), ToFloat(sample.predicted_distance()),
                speed * dt);
    RunSteps(sample, 120);
    EXPECT_NEAR(ToFloat(sample.disk().velocity.x()), ToFloat(rolling.speed),
                0.002f)
        << "rolls on";
    sample.End();
  }
}

// Chapter 05, exercise "mixed surfaces": a contact uses sqrt(mu_c mu_r), so
// ice on rubber holds only up to atan(0.224) = 12.6 degrees
TEST(FrictionSolutions, ContactUsesTheGeometricMeanOfTheTwoSurfaces) {
  for (const auto& [angle, crate, ramp] :
       {std::tuple{15, 5, 100}, std::tuple{12, 5, 100}, std::tuple{30, 20, 80},
        std::tuple{30, 10, 100}, std::tuple{20, 50, 50},
        std::tuple{40, 5, 5}}) {
    MixedSurfacesSample sample;
    BeginWith(sample, {{MixedSurfacesSample::kRampAngle, angle},
                       {MixedSurfacesSample::kCrateFriction, crate},
                       {MixedSurfacesSample::kRampFriction, ramp}});
    RunSteps(sample, MixedSurfacesSample::kMeasureSteps);
    const float mu = std::sqrt(static_cast<float>(crate * ramp)) / 100.0f;
    EXPECT_NEAR(ToFloat(sample.contact_friction()), mu, 0.0005f);
    const float steepest = std::atan(mu) * 180.0f / 3.14159265f;
    EXPECT_EQ(sample.steepest_holding_angle(), static_cast<int>(steepest))
        << crate << " " << ramp;
    const float radians = static_cast<float>(angle) * 3.14159265f / 180.0f;
    EXPECT_EQ(sample.predicted_holds(), std::tan(radians) <= mu);
    const float measured = ToFloat(*sample.measured_acceleration());
    if (sample.predicted_holds()) {
      EXPECT_LT(std::abs(measured), 0.005f) << angle;
    } else {
      const float predicted =
          9.81f * (std::sin(radians) - mu * std::cos(radians));
      EXPECT_NEAR(ToFloat(sample.predicted_acceleration()), predicted, 0.005f);
      // Within 0.001 with FIXED16; 3 % allows FLOAT's start (chapter 05)
      EXPECT_NEAR(measured, predicted, std::max(0.005f, 0.03f * predicted))
          << angle << " " << crate << " " << ramp;
    }
    sample.End();
  }
}

// Chapter 05, exercise "push to start": static friction pushes back exactly
// as hard as the push, up to mu m g; past it the box slides at F / m - mu g
TEST(FrictionSolutions, StaticFrictionMatchesThePushUpToMuMG) {
  for (const auto& [push_cn, friction] :
       {std::pair{400, 50}, std::pair{0, 50}, std::pair{250, 30},
        std::pair{600, 50}, std::pair{900, 50}, std::pair{300, 20},
        std::pair{900, 80}}) {
    PushToStartSample sample;
    BeginWith(sample, {{PushToStartSample::kPush, push_cn},
                       {PushToStartSample::kFriction, friction}});
    RunSteps(sample, PushToStartSample::kMeasureSteps);
    const float push = static_cast<float>(push_cn) / 100.0f;
    const float limit = static_cast<float>(friction) / 100.0f * 9.81f;
    EXPECT_NEAR(ToFloat(sample.static_limit()), limit, 0.002f);
    EXPECT_EQ(sample.predicted_holds(), push <= limit);
    const float measured = ToFloat(*sample.measured_acceleration());
    const float friction_force = ToFloat(*sample.measured_friction());
    if (push <= limit) {
      EXPECT_LT(std::abs(measured), 0.005f) << push_cn << " " << friction;
      EXPECT_NEAR(friction_force, push, 0.005f) << push_cn << " " << friction;
    } else {
      // 0.01 allows FLOAT's first steps (0.0051 at 9 N with 0.8)
      EXPECT_NEAR(measured, push - limit, 0.01f) << push_cn << " " << friction;
      EXPECT_NEAR(friction_force, limit, 0.01f) << push_cn << " " << friction;
    }
    sample.End();
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

// Chapter 06, exercise "impulse per box": at rest every contact carries the
// weight above it, (n - k) m g dt, warm or cold (only the depth differs).
// The first step, solved from zero, falls short, more so at the bottom of a
// tall stack
TEST(StackingSolutions, RestingContactsCarryTheWeightAbove) {
  for (const int boxes : {4, 6, 10}) {
    std::vector<float> first_cold;
    for (const int warm : {0, 1}) {
      ImpulsePerBoxSample sample;
      BeginWith(sample, {{ImpulsePerBoxSample::kBoxes, boxes},
                         {ImpulsePerBoxSample::kPasses, 8},
                         {ImpulsePerBoxSample::kWarmStart, warm}});
      RunSteps(sample, 300);
      for (int k = 0; k < boxes; ++k) {
        const float predicted = ToFloat(sample.predicted_impulse(k));
        EXPECT_NEAR(predicted, static_cast<float>(boxes - k) * 9.81f / 60.0f,
                    0.001f);
        EXPECT_NEAR(ToFloat(sample.measured_impulse(k)), predicted, 0.001f)
            << boxes << " " << warm << " " << k;
        const float first = ToFloat(sample.first_step_impulse(k));
        EXPECT_LT(first, predicted) << boxes << " " << k;
        if (warm == 0) {
          first_cold.push_back(first);
        } else {
          // Nothing to warm start from at the first step
          EXPECT_EQ(first, first_cold[static_cast<std::size_t>(k)]);
        }
      }
      if (boxes == 10) {
        EXPECT_LT(ToFloat(sample.first_step_impulse(0)),
                  0.4f * ToFloat(sample.predicted_impulse(0)));
      }
      sample.End();
    }
  }
}

// Chapter 06, exercise "two boxes by hand": the solver's loop by hand gives
// the world's first step; each pass halves both velocities
TEST(StackingSolutions, EachPassHalvesTheFirstStepError) {
  for (const int passes : {1, 2, 3, 4, 8}) {
    TwoBoxesByHandSample sample;
    BeginWith(sample, {{TwoBoxesByHandSample::kPasses, passes}});
    sample.FixedUpdate();
    ASSERT_TRUE(sample.measured().has_value());
    const auto hand = sample.predicted(passes);
    const auto world = *sample.measured();
    EXPECT_NEAR(ToFloat(world.bottom_velocity), ToFloat(hand.bottom_velocity),
                0.0002f)
        << passes;
    EXPECT_NEAR(ToFloat(world.top_velocity), ToFloat(hand.top_velocity),
                0.0002f)
        << passes;
    EXPECT_NEAR(ToFloat(world.bottom_impulse), ToFloat(hand.bottom_impulse),
                0.0002f)
        << passes;
    EXPECT_NEAR(ToFloat(world.top_impulse), ToFloat(hand.top_impulse), 0.0002f)
        << passes;
    const float error =
        -9.81f / 60.0f / static_cast<float>(1 << static_cast<unsigned>(passes));
    EXPECT_NEAR(ToFloat(world.bottom_velocity), error, 0.0002f) << passes;
    EXPECT_NEAR(ToFloat(world.top_velocity), error, 0.0002f) << passes;
    sample.End();
  }
}

// Chapter 06, exercise "heavy on light": without warm starting the stack
// sinks deeper as the top box gets heavier; with it, a ratio of 10 holds
// within the slops but a ratio of 20 bounces
TEST(StackingSolutions, HeavyTopSinksTheStackDeeper) {
  float previous = 0.0f;
  for (const int mass : {1, 5, 10, 20}) {
    HeavyOnLightSample sample;
    BeginWith(sample, {{HeavyOnLightSample::kTopMass, mass},
                       {HeavyOnLightSample::kPasses, 8},
                       {HeavyOnLightSample::kWarmStart, 0}});
    RunSteps(sample, 300);
    const float sink = ToFloat(sample.top_sink());
    if (mass == 1) {
      EXPECT_EQ(sink, ToFloat(sample.reference_top_sink()));
    } else {
      EXPECT_GT(sink, previous) << mass;
    }
    previous = sink;
    sample.End();
  }
  EXPECT_GT(previous, 4.0f * 0.02f) << "M = 20: four times the slops";

  HeavyOnLightSample warm;
  BeginWith(warm, {{HeavyOnLightSample::kTopMass, 10},
                   {HeavyOnLightSample::kPasses, 8},
                   {HeavyOnLightSample::kWarmStart, 1}});
  RunSteps(warm, 300);
  EXPECT_LT(ToFloat(warm.top_sink()), 4 * 0.005f);
  warm.End();

  HeavyOnLightSample bouncing;
  BeginWith(bouncing, {{HeavyOnLightSample::kTopMass, 20},
                       {HeavyOnLightSample::kPasses, 8},
                       {HeavyOnLightSample::kWarmStart, 1}});
  float largest = 0.0f;
  for (int step = 0; step < 600; ++step) {
    bouncing.FixedUpdate();
    largest = std::max(largest, std::abs(ToFloat(bouncing.top_sink())));
  }
  EXPECT_GT(largest, 0.1f) << "the heavy box bounces";
  bouncing.End();
}

// Chapter 06, exercise "pyramid": with warm starting four passes hold a
// pyramid of turning boxes; without, the boxes creep apart
TEST(StackingSolutions, WarmStartingHoldsThePyramid) {
  PyramidSample warm;
  BeginWith(warm, {{PyramidSample::kRows, 4},
                   {PyramidSample::kPasses, 4},
                   {PyramidSample::kWarmStart, 1}});
  RunSteps(warm, 600);
  EXPECT_LT(ToFloat(warm.top_sink()), 0.02f);
  EXPECT_LT(ToFloat(warm.drift()), 0.01f);
  EXPECT_EQ(warm.fallen(), 0);
  warm.End();

  PyramidSample cold;
  BeginWith(cold, {{PyramidSample::kRows, 4},
                   {PyramidSample::kPasses, 4},
                   {PyramidSample::kWarmStart, 0}});
  RunSteps(cold, 300);
  const float drift_5s = ToFloat(cold.drift());
  RunSteps(cold, 300);
  const float drift_10s = ToFloat(cold.drift());
  EXPECT_GT(drift_5s, 0.2f);
  EXPECT_GT(drift_10s, drift_5s) << "still creeping";
  cold.End();
}

namespace {

struct PendulumRun {
  float predicted_period = 0.0f;
  float measured_period = 0.0f;
  float max_length_error = 0.0f;
  float predicted_max_stretch = 0.0f;
  int slack_steps = 0;
};

// Runs the pendulum for 10 s; length in tenths of a meter, amplitude in
// degrees
PendulumRun RunPendulum(int length_tenths, int amplitude, bool rope) {
  PendulumSample sample;
  auto values = sample.parameters();
  values[PendulumSample::kLength].value = length_tenths;
  values[PendulumSample::kAmplitude].value = amplitude;
  values[PendulumSample::kRope].value = rope ? 1 : 0;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < 600; ++step) {
    sample.FixedUpdate();
  }
  PendulumRun run;
  run.predicted_period = ToFloat(sample.predicted_period());
  run.measured_period = ToFloat(sample.measured_period().value_or(Scalar{0}));
  run.max_length_error = ToFloat(sample.max_length_error());
  run.predicted_max_stretch = ToFloat(sample.predicted_max_stretch());
  run.slack_steps = sample.slack_steps();
  sample.End();
  return run;
}

class PendulumTest : public ::testing::TestWithParam<std::pair<int, int>> {};

}  // namespace

// The experiment of chapter 07: the period is 2 pi sqrt(L / g) with the
// amplitude series, and the rod stretches by its tension sag plus the swing
// drift
TEST_P(PendulumTest, PeriodIsTwoPiSqrtLOverG) {
  const auto [length_tenths, amplitude] = GetParam();
  const auto run = RunPendulum(length_tenths, amplitude, false);
  EXPECT_NEAR(run.measured_period, run.predicted_period,
              0.005f * run.predicted_period);
  EXPECT_GT(run.max_length_error, 0.7f * run.predicted_max_stretch);
  EXPECT_LT(run.max_length_error, 1.3f * run.predicted_max_stretch);
  EXPECT_EQ(run.slack_steps, 0);
}

INSTANTIATE_TEST_SUITE_P(LengthsAndAmplitudes, PendulumTest,
                         ::testing::Values(std::pair{10, 10}, std::pair{20, 10},
                                           std::pair{40, 10}, std::pair{20, 30},
                                           std::pair{20, 60}));

// Past 90 degrees a rope goes slack near the top and the bob falls inside
// the circle; a rod holds it on the circle
TEST(PendulumTutorial, RopeGoesSlackPastNinetyDegrees) {
  EXPECT_GT(RunPendulum(20, 150, true).slack_steps, 0);
  EXPECT_EQ(RunPendulum(20, 150, false).slack_steps, 0);
  // Below 90 degrees the rope stays taut: it swings like the rod
  const auto rope = RunPendulum(20, 30, true);
  const auto rod = RunPendulum(20, 30, false);
  EXPECT_EQ(rope.slack_steps, 0);
  EXPECT_NEAR(rope.measured_period, rod.measured_period, 0.01f);
}

TEST(PendulumTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"07 Pendulum", std::make_unique<PendulumSample>()});
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
  registry.SetParameter(PendulumSample::kAmplitude, 40);
  const std::uint32_t changed = run();
  registry.SetParameter(PendulumSample::kAmplitude, 10);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

namespace {

struct ChainRun {
  float predicted = 0.0f;
  float stretch = 0.0f;
  float max_stretch = 0.0f;
};

// Runs the chain for 5 s
ChainRun RunChain(int links, int end_mass, int passes, bool horizontal) {
  ChainSample sample;
  auto values = sample.parameters();
  values[ChainSample::kLinks].value = links;
  values[ChainSample::kEndMass].value = end_mass;
  values[ChainSample::kPasses].value = passes;
  values[ChainSample::kStart].value = horizontal ? 1 : 0;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < 300; ++step) {
    sample.FixedUpdate();
  }
  ChainRun run{ToFloat(sample.predicted_rest_stretch()),
               ToFloat(sample.stretch()), ToFloat(sample.max_stretch())};
  sample.End();
  return run;
}

}  // namespace

// The experiment of chapter 08: a hanging chain settles at the soft-step
// prediction sum g (N - i + M) k_i / w^2, also with a heavy end, from 4
// passes on
TEST(ChainTutorial, HangingChainSettlesAtThePredictedStretch) {
  for (const int end_mass : {1, 10}) {
    for (const int passes : {4, 8}) {
      const auto run = RunChain(8, end_mass, passes, false);
      EXPECT_NEAR(run.stretch, run.predicted, 0.03f * run.predicted)
          << "end mass " << end_mass << ", " << passes << " passes";
    }
  }
  // 8 links of 1 kg: 64 g / (2 pi 15)^2
  EXPECT_NEAR(RunChain(8, 1, 8, false).predicted,
              64.0f * 9.81f / (900.0f * 3.14159265f * 3.14159265f), 1e-4f);
}

// A swinging chain stretches more with a heavy end, and less with more
// passes
TEST(ChainTutorial, HeavyEndStretchesASwingingChain) {
  const auto light = RunChain(8, 1, 4, true);
  const auto heavy = RunChain(8, 20, 4, true);
  EXPECT_GT(heavy.max_stretch, 2.0f * light.max_stretch);
  const auto heavy_more_passes = RunChain(8, 20, 16, true);
  EXPECT_LT(heavy_more_passes.max_stretch, heavy.max_stretch);
}

TEST(ChainTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"08 Chain", std::make_unique<ChainSample>()});
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
  registry.SetParameter(ChainSample::kEndMass, 10);
  const std::uint32_t changed = run();
  registry.SetParameter(ChainSample::kEndMass, 1);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

namespace {

struct SpringRun {
  float predicted_period = 0.0f;
  float measured_period = 0.0f;
  float omega_dt = 0.0f;
  float displacement = 0.0f;
  bool exploded = false;
};

// Runs the spring for the given steps; mass in tenths of a kilogram, damping
// in tenths of N s/m
SpringRun RunSpring(int stiffness, int mass_tenths, int damping_tenths,
                    int steps = 600) {
  SpringSample sample;
  auto values = sample.parameters();
  values[SpringSample::kStiffness].value = stiffness;
  values[SpringSample::kMass].value = mass_tenths;
  values[SpringSample::kDamping].value = damping_tenths;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < steps; ++step) {
    sample.FixedUpdate();
  }
  SpringRun run;
  run.predicted_period = ToFloat(sample.predicted_period());
  run.measured_period = ToFloat(sample.measured_period().value_or(Scalar{0}));
  run.omega_dt = ToFloat(sample.omega_dt());
  run.displacement = ToFloat(sample.displacement());
  run.exploded = sample.exploded();
  sample.End();
  return run;
}

class SpringTest : public ::testing::TestWithParam<std::pair<int, int>> {};

}  // namespace

// The experiment of chapter 09: T = 2 pi sqrt(m / k), and more exactly the
// discrete period 2 pi dt / acos(1 - (w dt)^2 / 2) of semi-implicit Euler,
// shorter by about (w dt)^2 / 24
TEST_P(SpringTest, PeriodIsTwoPiSqrtMOverK) {
  const auto [stiffness, mass_tenths] = GetParam();
  const auto run = RunSpring(stiffness, mass_tenths, 0);
  const float dt = ToFloat(neko::Sample::kFixedDeltaTime);
  const float discrete = 2.0f * 3.14159265f * dt /
                         std::acos(1.0f - 0.5f * run.omega_dt * run.omega_dt);
  EXPECT_FALSE(run.exploded);
  EXPECT_NEAR(run.measured_period, discrete, 0.003f * run.predicted_period);
  const float shortening = run.omega_dt * run.omega_dt / 24.0f;
  EXPECT_NEAR(run.measured_period, run.predicted_period * (1.0f - shortening),
              0.01f * run.predicted_period);
  EXPECT_LT(run.measured_period, run.predicted_period);
}

INSTANTIATE_TEST_SUITE_P(
    StiffnessesAndMasses, SpringTest,
    ::testing::Values(std::pair{40, 10}, std::pair{160, 10}, std::pair{40, 20},
                      std::pair{1000, 10}, std::pair{1000, 2}));

// Damped, it settles at the equilibrium stretch m g / k
TEST(SpringTutorial, DampedSpringSettlesAtTheEquilibrium) {
  // Damping ratio c / (2 sqrt(k m)) = 4 / (2 sqrt(40)) = 0.32
  const auto run = RunSpring(40, 10, 40, 900);
  EXPECT_NEAR(run.displacement, 0.0f, 1e-3f);
}

// The explicit spring is stable only while w dt < 2
TEST(SpringTutorial, ExplodesPastOmegaDtOfTwo) {
  const auto stable = RunSpring(1000, 1, 0);    // w dt = 1.67
  const auto unstable = RunSpring(2000, 1, 0);  // w dt = 2.36
  EXPECT_LT(stable.omega_dt, 2.0f);
  EXPECT_FALSE(stable.exploded);
  EXPECT_GT(unstable.omega_dt, 2.0f);
  EXPECT_TRUE(unstable.exploded);
}

TEST(SpringTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"09 Spring", std::make_unique<SpringSample>()});
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
  registry.SetParameter(SpringSample::kStiffness, 100);
  const std::uint32_t changed = run();
  registry.SetParameter(SpringSample::kStiffness, 40);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

// The experiment of chapter 10: the blast finds the targets in its radius
// whose category it affects, and pushes those the line of sight reaches
struct ExplosionRun {
  int in_radius = 0;
  int visible = 0;
  std::vector<bool> in_radius_targets;
  std::vector<bool> visible_targets;
};

ExplosionRun RunExplosion(int radius_tenths, bool wall, bool hit_shielded) {
  ExplosionSample sample;
  auto values = sample.parameters();
  values[ExplosionSample::kRadius].value = radius_tenths;
  values[ExplosionSample::kWallOn].value = wall ? 1 : 0;
  values[ExplosionSample::kHitShielded].value = hit_shielded ? 1 : 0;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < ExplosionSample::kBlastStep; ++step) {
    sample.FixedUpdate();
  }
  ExplosionRun run;
  EXPECT_TRUE(sample.exploded());
  run.in_radius = sample.in_radius_count();
  run.visible = sample.visible_count();
  for (const auto& target : sample.targets()) {
    run.in_radius_targets.push_back(target.in_radius);
    run.visible_targets.push_back(target.visible);
  }
  sample.End();
  return run;
}

// Speeds of the targets just after the blast
std::vector<float> BlastSpeeds(int radius_tenths, bool wall,
                               bool hit_shielded) {
  ExplosionSample sample;
  auto values = sample.parameters();
  values[ExplosionSample::kRadius].value = radius_tenths;
  values[ExplosionSample::kWallOn].value = wall ? 1 : 0;
  values[ExplosionSample::kHitShielded].value = hit_shielded ? 1 : 0;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < ExplosionSample::kBlastStep; ++step) {
    sample.FixedUpdate();
  }
  // Horizontal speed: the vertical one also holds the bounce off the ground
  std::vector<float> speeds;
  for (std::size_t i = 0; i < ExplosionSample::kTargetCount; ++i) {
    speeds.push_back(std::abs(ToFloat(sample.target_velocity(i).x())));
  }
  return speeds;
}

TEST(ExplosionTutorial, DefaultBlast) {
  const ExplosionRun run = RunExplosion(35, true, false);
  // The crate, the barrel and the crate behind the wall are in the radius;
  // the shielded crate is filtered out and the far crate is out of reach
  EXPECT_EQ(run.in_radius, 3);
  EXPECT_EQ(run.visible, 2);
  EXPECT_EQ(run.in_radius_targets,
            (std::vector<bool>{true, true, false, true, false}));
  EXPECT_EQ(run.visible_targets,
            (std::vector<bool>{true, true, false, false, false}));
  const std::vector<float> speeds = BlastSpeeds(35, true, false);
  EXPECT_GT(speeds[0], 0.5f);
  EXPECT_GT(speeds[1], 0.5f);
  EXPECT_LT(speeds[2], 1e-3f);  // not affected
  EXPECT_LT(speeds[3], 1e-3f);  // behind the wall
  EXPECT_LT(speeds[4], 1e-3f);  // out of reach
}

TEST(ExplosionTutorial, WallShieldedAndRadius) {
  // Without the wall the crate behind it is seen
  ExplosionRun run = RunExplosion(35, false, false);
  EXPECT_EQ(run.in_radius, 3);
  EXPECT_EQ(run.visible, 3);
  // Asking for the shielded category adds the shielded crate
  run = RunExplosion(35, true, true);
  EXPECT_EQ(run.in_radius, 4);
  EXPECT_EQ(run.visible, 3);
  EXPECT_TRUE(run.visible_targets[2]);
  EXPECT_GT(BlastSpeeds(35, true, true)[2], 0.5f);
  // A wider blast reaches the far crate
  run = RunExplosion(50, true, false);
  EXPECT_EQ(run.in_radius, 4);
  EXPECT_TRUE(run.visible_targets[4]);
}

constexpr float kPi = 3.14159265f;

// The laser before the blast: t and normal against the post, the plank's
// top face and the ground, computed by hand
TEST(ExplosionTutorial, LaserHits) {
  struct Case {
    int degrees;
    float normal_x;
    float normal_y;
  };
  const float plank = 15.0f * kPi / 180.0f;
  for (const Case& c :
       {Case{0, -1.0f, 0.0f}, Case{20, -std::sin(plank), std::cos(plank)},
        Case{60, 0.0f, 1.0f}}) {
    ExplosionSample sample;
    sample.parameters()[ExplosionSample::kLaserAngle].value = c.degrees;
    sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
    sample.FixedUpdate();
    const auto hit = sample.laser_hit();
    ASSERT_TRUE(hit.has_value()) << c.degrees;
    EXPECT_NEAR(ToFloat(hit->normal.x()), c.normal_x, 2e-3f) << c.degrees;
    EXPECT_NEAR(ToFloat(hit->normal.y()), c.normal_y, 2e-3f) << c.degrees;
    // The point is on the segment at the fraction
    const auto start = sample.laser_start();
    const auto end = sample.laser_end();
    const float t = ToFloat(hit->fraction);
    EXPECT_NEAR(ToFloat(hit->point.x()),
                ToFloat(start.x()) + t * ToFloat(end.x() - start.x()), 2e-3f);
    EXPECT_NEAR(ToFloat(hit->point.y()),
                ToFloat(start.y()) + t * ToFloat(end.y() - start.y()), 2e-3f);
    if (c.degrees == 0) {
      // The post's left side, 8.7 m along 12 m
      EXPECT_NEAR(t, 8.7f / 12.0f, 1e-3f);
    } else if (c.degrees == 60) {
      // The ground top, 5.9 m below, along 12 sin 60 m of drop
      EXPECT_NEAR(t, 5.9f / (12.0f * std::sin(60.0f * kPi / 180.0f)), 1e-3f);
    }
  }
}

TEST(ExplosionTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"10 Explosion", std::make_unique<ExplosionSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 90; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  const std::uint32_t first = run();
  registry.SetParameter(ExplosionSample::kWallOn, 0);
  const std::uint32_t changed = run();
  registry.SetParameter(ExplosionSample::kWallOn, 1);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

// The experiment of chapter 11: thrust gives T / m - g s, the wind closes
// the gap to its speed by (1 - c dt / m) per step, the knockback jumps the
// velocity by J / m
struct JetpackRun {
  float predicted_acceleration = 0.0f;
  float measured_acceleration = 0.0f;
  float predicted_drift = 0.0f;
  float measured_drift = 0.0f;
  float predicted_kick = 0.0f;
  float measured_kick = 0.0f;
  float hover_thrust = 0.0f;
  int wind_steps = 0;
};

JetpackRun RunJetpack(int thrust, int mass_tenths, int gravity_scale_tenths,
                      int wind_tenths, int knockback_tenths,
                      int steps = JetpackSample::kKnockbackStep) {
  JetpackSample sample;
  auto values = sample.parameters();
  values[JetpackSample::kThrust].value = thrust;
  values[JetpackSample::kMass].value = mass_tenths;
  values[JetpackSample::kGravityScale].value = gravity_scale_tenths;
  values[JetpackSample::kWind].value = wind_tenths;
  values[JetpackSample::kKnockback].value = knockback_tenths;
  sample.Begin(neko::SampleContext{neko::sample_config::kWorldBounds});
  for (int step = 0; step < steps; ++step) {
    sample.FixedUpdate();
  }
  JetpackRun run;
  run.predicted_acceleration = ToFloat(sample.predicted_acceleration());
  run.measured_acceleration =
      ToFloat(sample.measured_acceleration().value_or(Scalar{0}));
  run.predicted_drift = ToFloat(sample.predicted_drift());
  run.measured_drift = ToFloat(sample.measured_drift().value_or(Scalar{0}));
  run.predicted_kick = ToFloat(sample.predicted_kick());
  run.measured_kick = ToFloat(sample.measured_kick().value_or(Scalar{0}));
  run.hover_thrust = ToFloat(sample.hover_thrust());
  run.wind_steps = sample.wind_steps();
  return run;
}

class JetpackTest
    : public ::testing::TestWithParam<std::tuple<int, int, int, int, int>> {};

TEST_P(JetpackTest, MatchesThePredictions) {
  const auto [thrust, mass, scale, wind, knockback] = GetParam();
  const JetpackRun run = RunJetpack(thrust, mass, scale, wind, knockback);
  EXPECT_NEAR(run.measured_acceleration, run.predicted_acceleration,
              0.01f * std::abs(run.predicted_acceleration) + 0.02f);
  ASSERT_GT(run.wind_steps, 0);
  EXPECT_NEAR(run.measured_drift, run.predicted_drift,
              0.002f * std::abs(run.predicted_drift) + 2e-3f);
  EXPECT_NEAR(run.measured_kick, run.predicted_kick,
              0.002f * std::abs(run.predicted_kick) + 1e-3f);
}

INSTANTIATE_TEST_SUITE_P(Chapter11, JetpackTest,
                         ::testing::Values(std::make_tuple(30, 10, 10, 30, 40),
                                           std::make_tuple(30, 20, 10, 30, 40),
                                           std::make_tuple(20, 10, 5, -20, 40),
                                           std::make_tuple(60, 40, 10, 50, 100),
                                           std::make_tuple(10, 5, 0, 30, 20)));

// Doubling the mass halves the kick and the thrust's share of the
// acceleration; the hover thrust is m g s
TEST(JetpackTutorial, MassAndGravityScale) {
  const JetpackRun light = RunJetpack(30, 10, 10, 30, 40);
  const JetpackRun heavy = RunJetpack(30, 20, 10, 30, 40);
  EXPECT_NEAR(heavy.measured_kick, 0.5f * light.measured_kick, 1e-3f);
  EXPECT_NEAR(light.hover_thrust, 9.81f, 0.01f);
  EXPECT_NEAR(heavy.hover_thrust, 2.0f * 9.81f, 0.02f);
  EXPECT_NEAR(RunJetpack(30, 10, 0, 30, 40).hover_thrust, 0.0f, 1e-3f);
  // Half gravity: the same thrust lifts faster
  const JetpackRun floaty = RunJetpack(30, 10, 5, 30, 40);
  EXPECT_NEAR(floaty.measured_acceleration - light.measured_acceleration,
              0.5f * 9.81f, 0.05f);
}

// Below the hover thrust the player stays on the ground: no wind, no drift
TEST(JetpackTutorial, TooWeakStaysDown) {
  const JetpackRun run = RunJetpack(9, 10, 10, 30, 40);
  EXPECT_EQ(run.wind_steps, 0);
  EXPECT_NEAR(run.measured_acceleration, 0.0f, 0.02f);
}

TEST(JetpackTutorial, SameParametersSameRun) {
  std::vector<neko::SampleRegistry::Entry> samples;
  samples.push_back({"11 Jetpack", std::make_unique<JetpackSample>()});
  neko::SampleRegistry registry{std::move(samples)};
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  const auto run = [&registry] {
    for (int step = 0; step < 240; ++step) {
      registry.FixedUpdate();
    }
    return registry.current()->GenerateChecksum();
  };
  registry.Select(0, context);
  const std::uint32_t first = run();
  registry.SetParameter(JetpackSample::kWind, -30);
  const std::uint32_t changed = run();
  registry.SetParameter(JetpackSample::kWind, 30);
  const std::uint32_t restored = run();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
