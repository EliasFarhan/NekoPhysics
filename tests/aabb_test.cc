#include <gtest/gtest.h>

#include "neko/math/shape.h"

TEST(Aabb, Same)
{
    neko::Aabbf aabb1 = neko::Aabbf::FromCenter({}, neko::Vec2f::One());
    neko::Aabbf aabb2 = neko::Aabbf::FromCenter({}, neko::Vec2f::One());
    EXPECT_TRUE(neko::Intersect(aabb1, aabb2));
}

TEST(Aabb, Inside)
{
    neko::Aabbf aabb1 = neko::Aabbf::FromCenter({}, neko::Vec2f(neko::Scalar {3}, neko::Scalar {1}));
    neko::Aabbf aabb2 = neko::Aabbf::FromCenter({}, neko::Vec2f(neko::Scalar {1}, neko::Scalar {3}));
    EXPECT_TRUE(neko::Intersect(aabb1, aabb2));
}

TEST(Aabb, WithCircle)
{
    neko::Aabbf aabb1 = neko::Aabbf::FromCenter({}, neko::Vec2f::One());
    neko::Circlef circle1{neko::Vec2f{}, neko::Scalar{1.0f}};
    EXPECT_TRUE(neko::Intersect(aabb1, circle1));


    neko::Circlef circle2{neko::Vec2f{neko::Scalar{0}, neko::Scalar{10}}, neko::Scalar{1.0f}};
    EXPECT_FALSE(neko::Intersect(aabb1, circle2));

    constexpr neko::Circlef circle3{neko::Vec2f{neko::Scalar{2}, neko::Scalar{0}}, neko::Scalar{1.1f}};
    EXPECT_TRUE(neko::Intersect(aabb1, circle3));

    constexpr neko::Circlef circle4{ neko::Vec2f{neko::Scalar{2.5f}, neko::Scalar{2.5f}}, neko::Scalar{1.5f} };
    EXPECT_FALSE(neko::Intersect(aabb1, circle4));


    constexpr neko::Circlef circle5{ neko::Vec2f{neko::Scalar{3.5f}, neko::Scalar{3.5f}}, neko::Scalar{3.0f} };
    EXPECT_FALSE(neko::Intersect(aabb1, circle5));
}

// The old test checked two edge midpoints instead of the four corners (review S2.2)
TEST(Aabb, WithCircleOverCorner)
{
    const neko::Aabbf aabb = neko::Aabbf::FromCenter({}, neko::Vec2f::One());
    constexpr neko::Scalar radius{ 0.8f };
    // 0.707 from each corner
    EXPECT_TRUE(neko::Intersect(aabb, neko::Circlef{ neko::Vec2f{neko::Scalar{-1.5f}, neko::Scalar{1.5f}}, radius }));
    EXPECT_TRUE(neko::Intersect(aabb, neko::Circlef{ neko::Vec2f{neko::Scalar{1.5f}, neko::Scalar{-1.5f}}, radius }));
    EXPECT_TRUE(neko::Intersect(aabb, neko::Circlef{ neko::Vec2f{neko::Scalar{1.5f}, neko::Scalar{1.5f}}, radius }));
    EXPECT_TRUE(neko::Intersect(aabb, neko::Circlef{ neko::Vec2f{neko::Scalar{-1.5f}, neko::Scalar{-1.5f}}, radius }));
    // 0.849 from the corner
    EXPECT_FALSE(neko::Intersect(aabb, neko::Circlef{ neko::Vec2f{neko::Scalar{-1.6f}, neko::Scalar{1.6f}}, radius }));
}

TEST(Aabb, WithCircleFarAway)
{
    const neko::Aabbf aabb = neko::Aabbf::FromCenter({}, neko::Vec2f::One());
    EXPECT_FALSE(neko::Intersect(aabb, neko::Circlef{ neko::Vec2f{neko::Scalar{257}, neko::Scalar{0}}, neko::Scalar{1} }));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}