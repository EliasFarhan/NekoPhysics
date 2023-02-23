#include <gtest/gtest.h>

#include "math/shape.h"

TEST(Aabb, Same)
{
    constexpr neko::Aabbf aabb1 = neko::Aabbf::FromCenter({}, neko::Vec2f::one());
    constexpr neko::Aabbf aabb2 = neko::Aabbf::FromCenter({}, neko::Vec2f::one());
    EXPECT_TRUE(neko::Intersect(aabb1, aabb2));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}