#include <gtest/gtest.h>

#include "math/shape.h"

TEST(Aabb, Same)
{
    constexpr neko::Aabbf aabb1 = neko::Aabbf::FromCenter({}, neko::Vec2f::one());
    constexpr neko::Aabbf aabb2 = neko::Aabbf::FromCenter({}, neko::Vec2f::one());
    EXPECT_TRUE(neko::Intersect(aabb1, aabb2));
}

TEST(Aabb, Inside)
{
    constexpr neko::Aabbf aabb1 = neko::Aabbf::FromCenter({}, neko::Vec2f(3, 1));
    constexpr neko::Aabbf aabb2 = neko::Aabbf::FromCenter({}, neko::Vec2f(1, 3));
    EXPECT_TRUE(neko::Intersect(aabb1, aabb2));
}

TEST(Aabb, WithCircle)
{
    constexpr neko::Aabbf aabb1 = neko::Aabbf::FromCenter({}, neko::Vec2f::one());
    constexpr neko::Circlef circle1{neko::Vec2f{}, neko::Scalar{1.0f}};
    EXPECT_TRUE(neko::Intersect(aabb1, circle1));


    constexpr neko::Circlef circle2{neko::Vec2f{neko::Scalar{0}, neko::Scalar{10}}, neko::Scalar{1.0f}};
    EXPECT_FALSE(neko::Intersect(aabb1, circle2));

    constexpr neko::Circlef circle3{neko::Vec2f{neko::Scalar{2}, neko::Scalar{0}}, neko::Scalar{1.1f}};
    EXPECT_TRUE(neko::Intersect(aabb1, circle3));

    constexpr neko::Circlef circle4{ neko::Vec2f{neko::Scalar{2.5f}, neko::Scalar{2.5f}}, neko::Scalar{1.5f} };
    EXPECT_FALSE(neko::Intersect(aabb1, circle4));


    constexpr neko::Circlef circle5{ neko::Vec2f{neko::Scalar{3.5f}, neko::Scalar{3.5f}}, neko::Scalar{3.0f} };
    EXPECT_FALSE(neko::Intersect(aabb1, circle5));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}