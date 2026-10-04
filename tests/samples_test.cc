#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "main/samples/sample_rng.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

namespace
{

constexpr int kSampleSteps = 300;

struct SampleRun
{
    std::uint32_t initial_checksum = 0;
    std::uint32_t final_checksum = 0;
    std::size_t drawn_shapes = 0;
    neko::Aabbf camera{};
};

SampleRun RunSample(neko::SampleRegistry& registry, std::size_t index)
{
    const neko::SampleContext context{ neko::sample_config::kWorldBounds };
    SampleRun run;
    registry.Select(index, context);
    run.initial_checksum = registry.current()->GenerateChecksum();
    for (int step = 0; step < kSampleSteps; ++step)
    {
        registry.FixedUpdate();
    }
    run.final_checksum = registry.current()->GenerateChecksum();
    neko::DrawList draw_list;
    registry.Draw(draw_list);
    run.drawn_shapes = draw_list.circles.size() + draw_list.boxes.size() + draw_list.lines.size();
    run.camera = draw_list.camera;
    registry.Select(neko::SampleRegistry::kNone, context);
    return run;
}

class SampleTest : public ::testing::TestWithParam<std::size_t>
{
};

} // namespace

// The samples build their scenes with a fixed-seed SampleRng and simulate only in FixedUpdate: two runs of the same
// sample give the same state, whether it's restarted in the same registry or built by another one
TEST_P(SampleTest, SameSeedGivesSameChecksum)
{
    neko::SampleRegistry registry;
    const auto first = RunSample(registry, GetParam());
    const auto restarted = RunSample(registry, GetParam());
    neko::SampleRegistry other_registry;
    const auto other = RunSample(other_registry, GetParam());

    EXPECT_NE(first.initial_checksum, first.final_checksum) << registry.name(GetParam()) << " doesn't move";
    EXPECT_EQ(first.initial_checksum, restarted.initial_checksum);
    EXPECT_EQ(first.final_checksum, restarted.final_checksum);
    EXPECT_EQ(first.initial_checksum, other.initial_checksum);
    EXPECT_EQ(first.final_checksum, other.final_checksum);
}

TEST_P(SampleTest, DrawsInsideTheWorldBounds)
{
    neko::SampleRegistry registry;
    const auto run = RunSample(registry, GetParam());
    EXPECT_GT(run.drawn_shapes, 0u);
    EXPECT_EQ(run.camera.min_bound(), neko::sample_config::kWorldBounds.min_bound());
    EXPECT_EQ(run.camera.max_bound(), neko::sample_config::kWorldBounds.max_bound());
}

INSTANTIATE_TEST_SUITE_P(AllSamples, SampleTest, ::testing::Range(std::size_t{ 0 }, std::size_t{ 6 }));

TEST(SampleRegistry, HasSixSamples)
{
    const neko::SampleRegistry registry;
    EXPECT_EQ(registry.size(), 6u);
    EXPECT_EQ(registry.current(), nullptr);
}

TEST(SampleRng, IntegerRangeIsInclusive)
{
    neko::SampleRng rng{ 42 };
    bool saw_min = false;
    bool saw_max = false;
    for (int i = 0; i < 1000; ++i)
    {
        const int value = rng.Range(-2, 2);
        ASSERT_GE(value, -2);
        ASSERT_LE(value, 2);
        saw_min = saw_min || value == -2;
        saw_max = saw_max || value == 2;
    }
    EXPECT_TRUE(saw_min);
    EXPECT_TRUE(saw_max);
}

TEST(SampleRng, ScalarRangeStaysInBounds)
{
    neko::SampleRng rng{ 7 };
    const auto min_value = neko::Ratio(-3, 2);
    const auto max_value = neko::Ratio(36, 5);
    for (int i = 0; i < 1000; ++i)
    {
        const auto value = rng.Range(min_value, max_value);
        ASSERT_GE(value, min_value);
        ASSERT_LE(value, max_value);
    }
}

TEST(SampleRng, SameSeedSameSequence)
{
    neko::SampleRng a{ 1234 };
    neko::SampleRng b{ 1234 };
    for (int i = 0; i < 100; ++i)
    {
        ASSERT_EQ(a.Next(), b.Next());
    }
}

TEST(SampleRng, RandomColorsAreBright)
{
    neko::SampleRng rng{ 99 };
    for (int i = 0; i < 1000; ++i)
    {
        const auto color = rng.RandomColor();
        ASSERT_GE(color, neko::sample_color::kFirstBright);
        ASSERT_LE(color, neko::sample_color::kLastBright);
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
