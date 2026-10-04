#include <cstdint>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "neko/physics/physics.h"

#include "tests/scenes/falling_pile.h"

// A change here means the simulation changed: either on purpose (update
// neko::scenes::kFallingPileGoldenChecksums, and the ROM expectations), or
// determinism broke.
TEST(Scene, FallingPileGoldenChecksums) {
  if constexpr (!std::is_same_v<neko::Scalar, neko::Fixed16>) {
    GTEST_SKIP() << "golden checksums are for NEKO_SCALAR=FIXED16";
  } else {
    neko::PhysicsWorld world{};
    neko::scenes::BuildFallingPile(world);
    int step = 0;
    for (const auto& [expected_step, checksum] :
         neko::scenes::kFallingPileGoldenChecksums) {
      while (step < expected_step) {
        world.Step();
        ++step;
      }
      EXPECT_EQ(world.GenerateChecksum(), checksum) << "step " << step;
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
