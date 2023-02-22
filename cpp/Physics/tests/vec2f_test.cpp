#include <numbers>

#include "math/vec2.h"
#include <gtest/gtest.h>



TEST(Vec2f, Const)
{
    constexpr neko::Vec2f zero = neko::Vec2f::zero();
    EXPECT_FLOAT_EQ(zero.x, 0.0f);
    EXPECT_FLOAT_EQ(zero.y, 0.0f);

    constexpr neko::Vec2f up = neko::Vec2f::up();
    EXPECT_FLOAT_EQ(up.x, 0.0f);
    EXPECT_FLOAT_EQ(up.y, 1.0f);
}

TEST(Vec2f, Get)
{
    constexpr neko::Vec2f zeroV{};
    EXPECT_FLOAT_EQ(zeroV.x, 0.0f);
    EXPECT_FLOAT_EQ(zeroV.y, 0.0f);

    constexpr float x = 1.1f;
    constexpr float y = -2.1f;
    constexpr neko::Vec2f v{ x, y };
    EXPECT_FLOAT_EQ(x, v.x);
    EXPECT_FLOAT_EQ(y, v.y);
}

TEST(Vec2f, Set)
{
    constexpr float x = 1.1f;
    constexpr float newX = -1.1f;
    constexpr float y = -2.1f;
    constexpr float newY = 2.1f;

    neko::Vec2f v{ x, y };
    v.x = newX;
    v.y = newY;
    EXPECT_FLOAT_EQ(newX, v.x);
    EXPECT_FLOAT_EQ(newY, v.y);
    EXPECT_NE(x, v.x);
    EXPECT_NE(y, v.y);
}

TEST(Vec2f, Add)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };
    constexpr neko::Vec2f v2{ 1.3f, -2.4f };

    constexpr auto result = v1 + v2;
    EXPECT_FLOAT_EQ(result.x, v1.x+v2.x);
    EXPECT_FLOAT_EQ(result.y, v1.y+v2.y);

    auto tmp = v1;
    tmp += v2;
    EXPECT_FLOAT_EQ(tmp.x, v1.x + v2.x);
    EXPECT_FLOAT_EQ(tmp.y, v1.y + v2.y);
}


TEST(Vec2f, Sub)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };
    constexpr neko::Vec2f v2{ 1.3f, -2.4f };

    constexpr auto result = v1 - v2;
    EXPECT_FLOAT_EQ(result.x, v1.x - v2.x);
    EXPECT_FLOAT_EQ(result.y, v1.y - v2.y);

    constexpr auto neg = -v1;
    EXPECT_FLOAT_EQ(neg.x, -v1.x);
    EXPECT_FLOAT_EQ(neg.y, -v1.y);

    auto tmp = v1;
    tmp -= v2;
    EXPECT_FLOAT_EQ(tmp.x, v1.x - v2.x);
    EXPECT_FLOAT_EQ(tmp.y, v1.y - v2.y);
}

TEST(Vec2f, Mul)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };
    constexpr neko::Vec2f v2{ 1.3f, -2.4f };

    constexpr auto result1 = v1 * v2;
    EXPECT_FLOAT_EQ(result1.x, v1.x * v2.x);
    EXPECT_FLOAT_EQ(result1.y, v1.y * v2.y);

    constexpr float f = 3.141f;

    constexpr auto result2 = v1 * f;
    EXPECT_FLOAT_EQ(result2.x, v1.x * f);
    EXPECT_FLOAT_EQ(result2.y, v1.y * f);

    auto tmp = v1;
    tmp *= f;
    EXPECT_FLOAT_EQ(tmp.x, v1.x * f);
    EXPECT_FLOAT_EQ(tmp.y, v1.y * f);
}

TEST(Vec2f, Div)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };

    constexpr float f = 3.141f;

    constexpr auto result = v1 / f;
    EXPECT_FLOAT_EQ(result.x, v1.x / f);
    EXPECT_FLOAT_EQ(result.y, v1.y / f);
}

TEST(Vec2f, Dot)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };
    constexpr neko::Vec2f v2{ 1.3f, -2.4f };
    

    constexpr auto result = neko::Vec2f::Dot(v1, v2);
    EXPECT_FLOAT_EQ(result, v1.x*v2.x+v1.y*v2.y);
}


TEST(Vec2f, SquareLength)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };


    constexpr auto result = v1.SquareLength();
    EXPECT_FLOAT_EQ(result, v1.x * v1.x + v1.y * v1.y);
}

TEST(Vec2f, Length)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };

    const auto result = v1.Length();
    EXPECT_FLOAT_EQ(result, std::sqrt(v1.x * v1.x + v1.y * v1.y));
}

TEST(Vec2f, Normalized)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };
    const auto result = v1.Normalized();
    
    EXPECT_FLOAT_EQ(result.Length(), 1.0f);

    constexpr neko::Vec2f v2{};
    const auto tmp = v2.Normalized();
    EXPECT_TRUE(std::isnan(tmp.x));
    EXPECT_TRUE(std::isnan(tmp.y));
}

TEST(Vec2f, Perpendicular)
{
    constexpr neko::Vec2f v1{ 1.0f, 2.1f };
    constexpr auto result = v1.Perpendicular();
    EXPECT_FLOAT_EQ(neko::Vec2f::Dot(v1, result), 0.0f);

    constexpr auto result2 = v1.Perpendicular2();
    EXPECT_FLOAT_EQ(neko::Vec2f::Dot(v1, result2), 0.0f);
}


TEST(Vec2f, Clamp)
{
    constexpr neko::Vec2f v1{ -2.0f, -2.0f };
    constexpr neko::Vec2f v2{ 0.2f, 0.1f };
    constexpr neko::Vec2f v3{ 2.2f, 2.1f };
    constexpr neko::Vec2f minV{ -1.0f, -1.0f };
    constexpr neko::Vec2f maxV{ 1.1f, 0.9f };

    constexpr auto result1 = neko::Vec2f::Clamp(v1, minV, maxV);
    constexpr auto result2 = neko::Vec2f::Clamp(v2, minV, maxV);
    constexpr auto result3 = neko::Vec2f::Clamp(v3, minV, maxV);

    EXPECT_FLOAT_EQ(result1.x, minV.x);
    EXPECT_FLOAT_EQ(result1.y, minV.y);

    EXPECT_FLOAT_EQ(result2.x, v2.x);
    EXPECT_FLOAT_EQ(result2.y, v2.y);

    EXPECT_FLOAT_EQ(result3.x, maxV.x);
    EXPECT_FLOAT_EQ(result3.y, maxV.y);
}