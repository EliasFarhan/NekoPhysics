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

namespace {

using neko::tutorial::BounceSample;

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

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
