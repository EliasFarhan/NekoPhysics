#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "main/samples/sample_rng.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

namespace
{

constexpr int SAMPLE_STEPS = 300;

struct SampleRun
{
    std::uint32_t initialChecksum = 0;
    std::uint32_t finalChecksum = 0;
    std::size_t drawnShapes = 0;
    neko::Aabbf camera{};
};

SampleRun RunSample(neko::SampleRegistry& registry, std::size_t index)
{
    const neko::SampleContext context{ neko::sample_config::WORLD_BOUNDS };
    SampleRun run;
    registry.Select(index, context);
    run.initialChecksum = registry.current()->GenerateChecksum();
    for (int step = 0; step < SAMPLE_STEPS; ++step)
    {
        registry.FixedUpdate();
    }
    run.finalChecksum = registry.current()->GenerateChecksum();
    neko::DrawList drawList;
    registry.Draw(drawList);
    run.drawnShapes = drawList.circles.size() + drawList.boxes.size() + drawList.lines.size();
    run.camera = drawList.camera;
    registry.Select(neko::SampleRegistry::NONE, context);
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
    neko::SampleRegistry otherRegistry;
    const auto other = RunSample(otherRegistry, GetParam());

    EXPECT_NE(first.initialChecksum, first.finalChecksum) << registry.name(GetParam()) << " doesn't move";
    EXPECT_EQ(first.initialChecksum, restarted.initialChecksum);
    EXPECT_EQ(first.finalChecksum, restarted.finalChecksum);
    EXPECT_EQ(first.initialChecksum, other.initialChecksum);
    EXPECT_EQ(first.finalChecksum, other.finalChecksum);
}

TEST_P(SampleTest, DrawsInsideTheWorldBounds)
{
    neko::SampleRegistry registry;
    const auto run = RunSample(registry, GetParam());
    EXPECT_GT(run.drawnShapes, 0u);
    EXPECT_EQ(run.camera.minBound, neko::sample_config::WORLD_BOUNDS.minBound);
    EXPECT_EQ(run.camera.maxBound, neko::sample_config::WORLD_BOUNDS.maxBound);
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
    bool sawMin = false;
    bool sawMax = false;
    for (int i = 0; i < 1000; ++i)
    {
        const int value = rng.Range(-2, 2);
        ASSERT_GE(value, -2);
        ASSERT_LE(value, 2);
        sawMin = sawMin || value == -2;
        sawMax = sawMax || value == 2;
    }
    EXPECT_TRUE(sawMin);
    EXPECT_TRUE(sawMax);
}

TEST(SampleRng, ScalarRangeStaysInBounds)
{
    neko::SampleRng rng{ 7 };
    const auto minValue = neko::Ratio(-3, 2);
    const auto maxValue = neko::Ratio(36, 5);
    for (int i = 0; i < 1000; ++i)
    {
        const auto value = rng.Range(minValue, maxValue);
        ASSERT_GE(value, minValue);
        ASSERT_LE(value, maxValue);
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
        ASSERT_GE(color, neko::sample_color::FIRST_BRIGHT);
        ASSERT_LE(color, neko::sample_color::LAST_BRIGHT);
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
