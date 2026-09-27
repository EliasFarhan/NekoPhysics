#include "physics/physics.h"
#include "scenes/falling_pile.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <type_traits>
#include <utility>

// Golden checksums of the falling pile scene with Scalar = Fixed16. They were the same with MSVC 14.51 (/O2) and
// GCC 15.2 (-O0 and -O2, -fno-exceptions -fno-rtti) on 2026-09-27; the DS test ROM must print the same values.
// A change here means the simulation changed: either on purpose (update the values, and the ROM expectations), or
// determinism broke.
TEST(Scene, FallingPileGoldenChecksums)
{
    if constexpr (!std::is_same_v<neko::Scalar, neko::Fixed16>)
    {
        GTEST_SKIP() << "golden checksums are for NEKO_SCALAR=FIXED16";
    }
    else
    {
        constexpr std::pair<int, std::uint32_t> expected[] = {
            { 0, 0x61c2e799u },
            { 1, 0xfa00ce4cu },
            { 60, 0xbee5c5bau },
            { 120, 0x2cc11c14u },
            { 300, 0xe2ff67d3u },
            { 600, 0xa68114abu },
        };
        neko::PhysicsWorld world{};
        neko::scenes::BuildFallingPile(world);
        int step = 0;
        for (const auto& [expectedStep, checksum] : expected)
        {
            while (step < expectedStep)
            {
                world.Step();
                ++step;
            }
            EXPECT_EQ(world.GenerateChecksum(), checksum) << "step " << step;
        }
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
