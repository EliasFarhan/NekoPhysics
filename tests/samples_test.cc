#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "main/samples/sample_rng.h"

namespace {

constexpr int kSampleSteps = 300;

struct SampleRun {
  std::uint32_t initial_checksum = 0;
  std::uint32_t final_checksum = 0;
  std::size_t drawn_shapes = 0;
  neko::Aabbf camera{};
};

SampleRun RunSample(neko::SampleRegistry& registry, std::size_t index) {
  const neko::SampleContext context{neko::sample_config::kWorldBounds};
  SampleRun run;
  registry.Select(index, context);
  run.initial_checksum = registry.current()->GenerateChecksum();
  for (int step = 0; step < kSampleSteps; ++step) {
    registry.FixedUpdate();
  }
  run.final_checksum = registry.current()->GenerateChecksum();
  neko::DrawList draw_list;
  registry.Draw(draw_list);
  run.drawn_shapes = draw_list.circles.size() + draw_list.boxes.size() +
                     draw_list.lines.size();
  run.camera = draw_list.camera;
  registry.Select(neko::SampleRegistry::kNone, context);
  return run;
}

class SampleTest : public ::testing::TestWithParam<std::size_t> {};

struct GoldenSampleChecksums {
  std::uint32_t initial = 0;
  std::uint32_t final = 0;
};

// Checksums of each sample with Scalar = Fixed16 and the PC sample_config,
// before and after kSampleSteps steps. Pinned on 2026-10-16 with MSVC 14.51,
// after the samples were made y-up. A change means a sample's simulation
// changed: on purpose (re-pin, and say why in the commit) or by accident.
constexpr std::array<GoldenSampleChecksums, 6> kGoldenSampleChecksums = {{
    {0x86675e94u, 0x12f15718u},  // 01 Kinematic
    {0xaf6f2b7du, 0xc3937196u},  // 02 Planets
    {0xc9ce3c39u, 0x03aa8553u},  // 03 Triggers
    {0x48069a77u, 0xda83ca14u},  // 04 Ground
    {0xc959e990u, 0x446cd868u},  // 05 Collisions
    {0xcbbe58dfu, 0xc5137570u},  // 06 Move Into
}};

}  // namespace

// The samples build their scenes with a fixed-seed SampleRng and simulate only
// in FixedUpdate: two runs of the same sample give the same state, whether it's
// restarted in the same registry or built by another one
TEST_P(SampleTest, SameSeedGivesSameChecksum) {
  neko::SampleRegistry registry;
  const auto first = RunSample(registry, GetParam());
  const auto restarted = RunSample(registry, GetParam());
  neko::SampleRegistry other_registry;
  const auto other = RunSample(other_registry, GetParam());

  EXPECT_NE(first.initial_checksum, first.final_checksum)
      << registry.name(GetParam()) << " doesn't move";
  EXPECT_EQ(first.initial_checksum, restarted.initial_checksum);
  EXPECT_EQ(first.final_checksum, restarted.final_checksum);
  EXPECT_EQ(first.initial_checksum, other.initial_checksum);
  EXPECT_EQ(first.final_checksum, other.final_checksum);
}

TEST_P(SampleTest, GoldenChecksums) {
  if constexpr (!std::is_same_v<neko::Scalar, neko::Fixed16>) {
    GTEST_SKIP() << "golden checksums are for NEKO_SCALAR=FIXED16";
  } else {
    neko::SampleRegistry registry;
    const auto run = RunSample(registry, GetParam());
    const auto& golden = kGoldenSampleChecksums[GetParam()];
    EXPECT_EQ(run.initial_checksum, golden.initial)
        << registry.name(GetParam());
    EXPECT_EQ(run.final_checksum, golden.final) << registry.name(GetParam());
  }
}

TEST_P(SampleTest, DrawsInsideTheWorldBounds) {
  neko::SampleRegistry registry;
  const auto run = RunSample(registry, GetParam());
  EXPECT_GT(run.drawn_shapes, 0u);
  EXPECT_EQ(run.camera.min_bound(),
            neko::sample_config::kWorldBounds.min_bound());
  EXPECT_EQ(run.camera.max_bound(),
            neko::sample_config::kWorldBounds.max_bound());
}

INSTANTIATE_TEST_SUITE_P(AllSamples, SampleTest,
                         ::testing::Range(std::size_t{0}, std::size_t{6}));

TEST(SampleRegistry, HasSixSamples) {
  const neko::SampleRegistry registry;
  EXPECT_EQ(registry.size(), 6u);
  EXPECT_EQ(registry.current(), nullptr);
}

TEST(SampleRng, IntegerRangeIsInclusive) {
  neko::SampleRng rng{42};
  bool saw_min = false;
  bool saw_max = false;
  for (int i = 0; i < 1000; ++i) {
    const int value = rng.Range(-2, 2);
    ASSERT_GE(value, -2);
    ASSERT_LE(value, 2);
    saw_min = saw_min || value == -2;
    saw_max = saw_max || value == 2;
  }
  EXPECT_TRUE(saw_min);
  EXPECT_TRUE(saw_max);
}

TEST(SampleRng, ScalarRangeStaysInBounds) {
  neko::SampleRng rng{7};
  const auto min_value = neko::Ratio(-3, 2);
  const auto max_value = neko::Ratio(36, 5);
  for (int i = 0; i < 1000; ++i) {
    const auto value = rng.Range(min_value, max_value);
    ASSERT_GE(value, min_value);
    ASSERT_LE(value, max_value);
  }
}

TEST(SampleRng, SameSeedSameSequence) {
  neko::SampleRng a{1234};
  neko::SampleRng b{1234};
  for (int i = 0; i < 100; ++i) {
    ASSERT_EQ(a.Next(), b.Next());
  }
}

TEST(SampleRng, RandomColorsAreBright) {
  neko::SampleRng rng{99};
  for (int i = 0; i < 1000; ++i) {
    const auto color = rng.RandomColor();
    ASSERT_GE(color, neko::sample_color::kFirstBright);
    ASSERT_LE(color, neko::sample_color::kLastBright);
  }
}

namespace {

std::string Format(int value, int scale) {
  std::array<char, 16> buffer{};
  const auto length = neko::FormatDecimal(value, scale, buffer);
  return {buffer.data(), length};
}

// One body launched upward at a speed set by a parameter
class LaunchSample final : public neko::Sample {
 public:
  void Begin([[maybe_unused]] const neko::SampleContext& context) override {
    ++begin_count;
    const auto index = world_.AddBody();
    world_.body(index).velocity = {neko::Scalar{0}, parameters_[0].ToScalar()};
  }
  void FixedUpdate() override { world_.Step(); }
  void Draw([[maybe_unused]] neko::DrawList& draw_list) const override {}
  void End() override { world_.Clear(); }
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }
  [[nodiscard]] std::span<neko::SampleParameter> parameters() override {
    return parameters_;
  }

  int begin_count = 0;

 private:
  neko::PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<neko::SampleParameter, 1> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = -20,
       .max = 50,
       .step = 5,
       .scale = 10,
       .value = 10},
  }};
};

struct LaunchRegistry {
  LaunchRegistry() {
    auto owned = std::make_unique<LaunchSample>();
    sample = owned.get();
    std::vector<neko::SampleRegistry::Entry> entries;
    entries.push_back({"Launch", std::move(owned)});
    registry = std::make_unique<neko::SampleRegistry>(std::move(entries));
    registry->Select(0, neko::SampleContext{});
  }
  std::uint32_t RunChecksum() {
    for (int step = 0; step < 10; ++step) {
      registry->FixedUpdate();
    }
    return registry->current()->GenerateChecksum();
  }
  LaunchSample* sample = nullptr;
  std::unique_ptr<neko::SampleRegistry> registry;
};

}  // namespace

TEST(SampleParameter, FormatsDecimals) {
  EXPECT_EQ(Format(75, 100), "0.75");
  EXPECT_EQ(Format(5, 100), "0.05");
  EXPECT_EQ(Format(150, 100), "1.50");
  EXPECT_EQ(Format(-15, 10), "-1.5");
  EXPECT_EQ(Format(0, 1000), "0.000");
  EXPECT_EQ(Format(-7, 1), "-7");
  EXPECT_EQ(Format(0, 1), "0");
  EXPECT_EQ(Format(-2147483647 - 1, 1), "-2147483648");
}

TEST(SampleReadout, ToDecimalRoundsHalfAwayFromZero) {
  EXPECT_EQ(neko::ToDecimal(neko::Ratio(3, 4), 100), 75);
  EXPECT_EQ(neko::ToDecimal(neko::Ratio(-3, 4), 100), -75);
  EXPECT_EQ(neko::ToDecimal(neko::Ratio(1, 3), 100), 33);
  EXPECT_EQ(neko::ToDecimal(neko::Ratio(2, 3), 100), 67);
  EXPECT_EQ(neko::ToDecimal(neko::Ratio(-2, 3), 100), -67);
  EXPECT_EQ(neko::ToDecimal(neko::Scalar{5}, 1), 5);
}

TEST(SampleParameter, ToScalarIsTheRatio) {
  const neko::SampleParameter parameter{.scale = 100, .value = 75};
  EXPECT_EQ(parameter.ToScalar(), neko::Ratio(75, 100));
}

TEST(SampleParameter, SetParameterClampsSnapsAndRestarts) {
  LaunchRegistry launch;
  ASSERT_EQ(launch.registry->parameters().size(), 1u);
  EXPECT_EQ(launch.sample->begin_count, 1);

  launch.registry->SetParameter(0, 23);
  EXPECT_EQ(launch.registry->parameters()[0].value, 20);  // -20 + 8 * 5
  EXPECT_EQ(launch.sample->begin_count, 2);
  launch.registry->SetParameter(0, 1000);
  EXPECT_EQ(launch.registry->parameters()[0].value, 50);
  launch.registry->SetParameter(0, -1000);
  EXPECT_EQ(launch.registry->parameters()[0].value, -20);
  launch.registry->SetParameter(1, 0);  // out of range: no restart
  EXPECT_EQ(launch.sample->begin_count, 4);
}

// The knob of an experiment: changing it changes the run, and setting it back
// gives the original run again
TEST(SampleParameter, SameValueSameRun) {
  LaunchRegistry launch;
  const auto first = launch.RunChecksum();
  launch.registry->SetParameter(0, 30);
  const auto changed = launch.RunChecksum();
  launch.registry->SetParameter(0, 10);
  const auto restored = launch.RunChecksum();
  EXPECT_NE(first, changed);
  EXPECT_EQ(first, restored);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
