#include <gtest/gtest.h>

#include "math/shape.h"

TEST(Circle, Intersect)
{
    constexpr neko::Circlef c1{ neko::Vec2f{}, neko::Scalar{1} };
    constexpr neko::Circlef c2{ neko::Vec2f{neko::Scalar{1.5f}, neko::Scalar{0}}, neko::Scalar{1} };
    constexpr neko::Circlef c3{ neko::Vec2f{neko::Scalar{2.5f}, neko::Scalar{0}}, neko::Scalar{1} };
    EXPECT_TRUE(neko::Intersect(c1, c2));
    EXPECT_FALSE(neko::Intersect(c1, c3));
}

// d² and r² overflow 16.16 past ~181 (DS review D1.6): 256² wraps to 0, 200² wraps negative
TEST(Circle, FarApartDoNotIntersect)
{
    constexpr neko::Circlef c1{ neko::Vec2f{}, neko::Scalar{1} };
    constexpr neko::Circlef c2{ neko::Vec2f{neko::Scalar{256}, neko::Scalar{0}}, neko::Scalar{1} };
    EXPECT_FALSE(neko::Intersect(c1, c2));

    constexpr neko::Circlef big1{ neko::Vec2f{}, neko::Scalar{100} };
    constexpr neko::Circlef big2{ neko::Vec2f{neko::Scalar{150}, neko::Scalar{0}}, neko::Scalar{100} };
    EXPECT_TRUE(neko::Intersect(big1, big2));
}

TEST(Circle, Contains)
{
    constexpr neko::Circlef c{ neko::Vec2f{}, neko::Scalar{1} };
    EXPECT_TRUE(c.Contains(neko::Vec2f{ neko::Scalar{0.5f}, neko::Scalar{0.5f} }));
    EXPECT_FALSE(c.Contains(neko::Vec2f{ neko::Scalar{0.8f}, neko::Scalar{0.8f} }));
    EXPECT_FALSE(c.Contains(neko::Vec2f{ neko::Scalar{256}, neko::Scalar{0} }));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
