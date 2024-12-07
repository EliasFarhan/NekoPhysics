#include "math/vec2.h"
#include <gtest/gtest.h>



TEST(Vec2f, Const)
{
    constexpr neko::Vec2f zero = neko::Vec2f::zero();
    EXPECT_FLOAT_EQ(float{ zero.x }, 0.0f);
    EXPECT_FLOAT_EQ(float{ zero.y }, 0.0f);

    constexpr neko::Vec2f up = neko::Vec2f::up();
    EXPECT_FLOAT_EQ(float{ up.x }, 0.0f);
    EXPECT_FLOAT_EQ(float{ up.y }, 1.0f);

    constexpr neko::Vec2f one = neko::Vec2f::one();
    EXPECT_FLOAT_EQ(float{ one.x }, 1.0f);
    EXPECT_FLOAT_EQ(float{ one.y }, 1.0f);
}

TEST(Vec2f, Get)
{
    constexpr neko::Vec2f zeroV{};
    EXPECT_FLOAT_EQ(float{ zeroV.x }, 0.0f);
    EXPECT_FLOAT_EQ(float{ zeroV.y }, 0.0f);

    constexpr auto x =  1.25f;
    constexpr auto y = -2.25f;
    constexpr neko::Vec2f v{ neko::Scalar{x}, neko::Scalar{y} };
    EXPECT_FLOAT_EQ(x, float{v.x});
    EXPECT_FLOAT_EQ(y, float{v.y});
}

TEST(Vec2f, Set)
{
    constexpr float x = 1.125f;
    constexpr float y = -2.125f;
    constexpr float newX = -1.125f;
    constexpr float newY = 2.125f;

    neko::Vec2f v{ neko::Scalar{x}, neko::Scalar{y} };
    v.x = neko::Scalar{newX};
    v.y = neko::Scalar{ newY };
    EXPECT_FLOAT_EQ(newX, float{v.x});
    EXPECT_FLOAT_EQ(newY, float{ v.y });
    EXPECT_NE(x, float{v.x});
    EXPECT_NE(y, float{v.y});
}

TEST(Vec2f, Add)
{
    constexpr neko::Vec2f v1{neko::Scalar{1.0f}, neko::Scalar{ 2.1f} };
    constexpr neko::Vec2f v2{neko::Scalar{1.3f}, neko::Scalar{-2.4f} };

    auto result = v1 + v2;
    EXPECT_FLOAT_EQ(float{ result.x }, float{ v1.x + v2.x });
    EXPECT_FLOAT_EQ(float{ result.y }, float{ v1.y + v2.y });

    auto tmp = v1;
    tmp += v2;
    EXPECT_FLOAT_EQ(float{tmp.x}, float{v1.x + v2.x});
    EXPECT_FLOAT_EQ(float{tmp.y}, float{v1.y + v2.y});
}


TEST(Vec2f, Sub)
{
    constexpr neko::Vec2f v1{ neko::Scalar{1.0f}, neko::Scalar{2.1f} };
    constexpr neko::Vec2f v2{ neko::Scalar{1.3f}, neko::Scalar{-2.4f }};

    auto result = v1 - v2;
    EXPECT_FLOAT_EQ(float{result.x}, float{v1.x - v2.x});
    EXPECT_FLOAT_EQ(float{result.y}, float{v1.y - v2.y});

    auto neg = -v1;
    EXPECT_FLOAT_EQ(float{neg.x}, float{-v1.x});
    EXPECT_FLOAT_EQ(float{neg.y}, float{-v1.y});

    auto tmp = v1;
    tmp -= v2;
    EXPECT_FLOAT_EQ(float{tmp.x}, float{v1.x - v2.x});
    EXPECT_FLOAT_EQ(float{tmp.y}, float{v1.y - v2.y});
}

TEST(Vec2f, Mul)
{
    constexpr neko::Vec2f v1{neko::Scalar{1.0f}, neko::Scalar{2.1f} };
    constexpr neko::Vec2f v2{neko::Scalar{1.3f}, neko::Scalar{-2.4f} };

    auto result1 = v1 * v2;
    EXPECT_FLOAT_EQ(float{result1.x}, float{v1.x * v2.x});
    EXPECT_FLOAT_EQ(float{result1.y}, float{v1.y * v2.y});

    constexpr neko::Scalar f{3.141f};

    auto result2 = v1 * f;
    EXPECT_FLOAT_EQ(float{result2.x}, float{v1.x * f});
    EXPECT_FLOAT_EQ(float{result2.y}, float{v1.y * f});

    auto tmp = v1;
    tmp *= f;
    EXPECT_FLOAT_EQ(float{tmp.x}, float{v1.x * f});
    EXPECT_FLOAT_EQ(float{tmp.y}, float{v1.y * f});
}

TEST(Vec2f, Div)
{
    constexpr neko::Vec2f v1{ neko::Scalar{1.0f}, neko::Scalar{2.125f} };

    constexpr neko::Scalar f{3.141f};

    auto result = v1 / f;
    EXPECT_FLOAT_EQ(float{ result.x }, float{ v1.x * (neko::Scalar{1} / f) });
    EXPECT_FLOAT_EQ(float{ result.y }, float{ v1.y * (neko::Scalar{1} / f) });

    auto tmp = v1;
    tmp /= f;
    EXPECT_FLOAT_EQ(float{ tmp.x }, float{ v1.x / f });
    EXPECT_FLOAT_EQ(float{ tmp.y }, float{ v1.y / f });
}

TEST(Vec2f, Dot)
{
    constexpr neko::Vec2f v1{ neko::Scalar{1.0f}, neko::Scalar{2.1f} };
    constexpr neko::Vec2f v2{ neko::Scalar{1.3f}, neko::Scalar{-2.4f} };
    

    auto result = neko::Vec2f::Dot(v1, v2);
    EXPECT_FLOAT_EQ(float{ result }, float{ v1.x * v2.x + v1.y * v2.y });
}


TEST(Vec2f, SquareLength)
{
    constexpr neko::Vec2f v1{ neko::Scalar{1.0f}, neko::Scalar{2.1f} };


    auto result = v1.SquareLength();
    EXPECT_FLOAT_EQ(float{ result }, float{ v1.x * v1.x + v1.y * v1.y });
}

TEST(Vec2f, Length)
{
    constexpr neko::Vec2f v1{ neko::Scalar{1.0f}, neko::Scalar{2.1f} };

    const auto result = v1.Length();
    EXPECT_FLOAT_EQ(float{ result }, float{ neko::Sqrt(v1.x * v1.x + v1.y * v1.y) });
}

TEST(Vec2f, Normalized)
{
    constexpr neko::Vec2f v1{ neko::Scalar{1.0f}, neko::Scalar{2.1f} };
    const auto result = v1.Normalized();
    
    EXPECT_NEAR(float{ result.Length() }, 1.0f, 0.0125f);

    if constexpr (std::is_floating_point_v<neko::Scalar>)
    {
        constexpr neko::Vec2f v2{};
        const auto tmp = v2.Normalized();
        EXPECT_TRUE(std::isnan(float{ tmp.x }));
        EXPECT_TRUE(std::isnan(float{ tmp.y }));
    }
}

TEST(Vec2f, Perpendicular)
{
    constexpr neko::Vec2f v1{ neko::Scalar{1.0f}, neko::Scalar{2.1f} };
    auto result = v1.Perpendicular();
    EXPECT_FLOAT_EQ(float{ neko::Vec2f::Dot(v1, result) }, 0.0f);

    auto result2 = v1.Perpendicular2();
    EXPECT_FLOAT_EQ(float{ neko::Vec2f::Dot(v1, result2) }, 0.0f);
}


TEST(Vec2f, Clamp)
{
    constexpr neko::Vec2f v1{ neko::Scalar{-2.0f}, neko::Scalar{-2.0f} };
    constexpr neko::Vec2f v2{ neko::Scalar{0.2f}, neko::Scalar{0.1f }};
    constexpr neko::Vec2f v3{ neko::Scalar{2.2f}, neko::Scalar{2.1f }};
    constexpr neko::Vec2f minV{ neko::Scalar{-1.0f}, neko::Scalar{-1.0f }};
    constexpr neko::Vec2f maxV{ neko::Scalar{1.1f}, neko::Scalar{0.9f }};

    auto result1 = neko::Vec2f::Clamp(v1, minV, maxV);
    auto result2 = neko::Vec2f::Clamp(v2, minV, maxV);
    auto result3 = neko::Vec2f::Clamp(v3, minV, maxV);

    EXPECT_FLOAT_EQ(float{ result1.x }, float{ minV.x });
    EXPECT_FLOAT_EQ(float{ result1.y }, float{ minV.y });
                                         
    EXPECT_FLOAT_EQ(float{ result2.x }, float{ v2.x });
    EXPECT_FLOAT_EQ(float{ result2.y }, float{ v2.y });
                                         
    EXPECT_FLOAT_EQ(float{ result3.x }, float{ maxV.x });
    EXPECT_FLOAT_EQ(float{ result3.y }, float{ maxV.y });
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}