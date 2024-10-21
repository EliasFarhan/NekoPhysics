
#include "math/fixed.h"
#include "math/fixed_lut.h"

#include <gtest/gtest.h>


TEST(Fixed, Construct)
{
    constexpr std::int32_t i = 32;
    constexpr auto f = neko::Fixed{ i };
    EXPECT_EQ(i, static_cast<std::int32_t>(f));
}


struct FixedIntOperationFixture : public ::testing::TestWithParam<std::pair<int, int>>
{
};

TEST_P(FixedIntOperationFixture, Add)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 + f2;
    EXPECT_EQ(i1 + i2, static_cast<std::int32_t>(result));

}

TEST_P(FixedIntOperationFixture, Sub)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 - f2;
    EXPECT_EQ(i1 - i2, static_cast<std::int32_t>(result));


	const auto neg1 = -f1;
	const auto neg2 = -f2;
	EXPECT_EQ(-i1, static_cast<std::int32_t>(neg1));
	EXPECT_EQ(-i2, static_cast<std::int32_t>(neg2));

}

TEST_P(FixedIntOperationFixture, Mul)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 * f2;
    EXPECT_EQ(i1 * i2, static_cast<std::int32_t>(result));

}

static constexpr float epsilon = 0.0001f;
TEST_P(FixedIntOperationFixture, Div)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 / f2;
    EXPECT_NEAR(static_cast<float>(i1) / static_cast<float>(i2),
        static_cast<float>(result), epsilon);

}

INSTANTIATE_TEST_SUITE_P(PositiveNumbers,
    FixedIntOperationFixture,
    testing::Values(
        std::pair{ 15, 3 },
        std::pair{ 1, 2 },
        std::pair{ 23,32 },
        std::pair{ 0,32 }
    )
);

INSTANTIATE_TEST_SUITE_P(NegativeNumbers,
    FixedIntOperationFixture,
    testing::Values(
        std::pair{-1, 1},
        std::pair{-1, -1},
        std::pair{ -25, 7 },
        std::pair{ 23,-32 }
    )
);


struct FixedFloatOperationFixture : public ::testing::TestWithParam<std::pair<float, float>>
{
};


TEST_P(FixedFloatOperationFixture, Add)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 + f2;
    EXPECT_NEAR(i1 + i2, static_cast<float>(result), epsilon);

}

TEST_P(FixedFloatOperationFixture, Sub)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 - f2;
    EXPECT_NEAR(i1 - i2, static_cast<float>(result), epsilon);


}

TEST_P(FixedFloatOperationFixture, Mul)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 * f2;
    EXPECT_NEAR(i1 * i2, static_cast<float>(result), epsilon);

}

TEST_P(FixedFloatOperationFixture, Div)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 / f2;
    EXPECT_NEAR(i1 / i2, static_cast<float>(result), epsilon);

}

INSTANTIATE_TEST_SUITE_P(PositiveNumbers,
    FixedFloatOperationFixture,
    testing::Values(
        std::pair{ 1.0f, 2.0f },
        std::pair{ 10.0f, 5.0f },
        std::pair{ 1.5f, 3.2f },
        std::pair{ 0.0f, 4.3f }
    )
);

INSTANTIATE_TEST_SUITE_P(NegativeNumbers,
    FixedFloatOperationFixture,
    testing::Values(
        std::pair{ -1.0f, 2.0f },
        std::pair{ -1.0f, -1.0f },
        std::pair{ -25.0f, 7.0f },
        std::pair{ 23.0f,-32.0f }
    )
);
constexpr float sqrtPrecision = 0.0125f;
TEST(LookupTable, Sqrt)
{
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sqrt(neko::Fixed(0))), sqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Sqrt(neko::Fixed(1))), sqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(3)), float(neko::Sqrt(neko::Fixed(9))), sqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(7)), float(neko::Sqrt(neko::Fixed(49))), sqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(9)), float(neko::Sqrt(neko::Fixed(81))), sqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(10)), float(neko::Sqrt(neko::Fixed(100))), sqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(11)), float(neko::Sqrt(neko::Fixed(121))), sqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(16)), float(neko::Sqrt(neko::Fixed(256))), sqrtPrecision);
}

constexpr float sinPrecision = 0.0075f;
TEST(LookupTable, Sin)
{
    EXPECT_NEAR(float(neko::Fixed16(-1, 2)), float(neko::Sin(-neko::pi / neko::Fixed(6))), sinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sin(neko::Fixed(0))), sinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1, 2)), float(neko::Sin(neko::pi / neko::Fixed(6))), sinPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(2)) / neko::Fixed16(2)), float(neko::Sin(neko::pi / neko::Fixed(4))), sinPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(3)) / neko::Fixed16(2)), float(neko::Sin(neko::pi * neko::Fixed(1,3))), sinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Sin(neko::pi / neko::Fixed(2))), sinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sin(neko::pi)), sinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(-1)), float(neko::Sin(neko::pi*neko::Fixed16(3, 2))), sinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sin(neko::pi*neko::Fixed16(2))), sinPrecision);
}

constexpr float cosPrecision = 0.0075f;
TEST(LookupTable, Cos)
{
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(3)) / neko::Fixed16(2)), float(neko::Cos(neko::Fixed(-neko::pi / neko::Fixed16{6}))), cosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Cos(neko::Fixed(0))), cosPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(3))/neko::Fixed16(2)), float(neko::Cos(neko::Fixed(neko::pi / neko::Fixed16{6}))), cosPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(2))/neko::Fixed16(2)), float(neko::Cos(neko::Fixed(neko::pi / neko::Fixed16{4}))), cosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1,2)), float(neko::Cos(neko::pi / neko::Fixed16{3})), cosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Cos(neko::pi / neko::Fixed16{2})), cosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(-1)), float(neko::Cos(neko::pi)), cosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Cos(neko::pi*neko::Fixed16{3, 2})), cosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Cos(neko::pi*neko::Fixed16{2})), cosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Cos(neko::pi*neko::Fixed16{5,2})), cosPrecision);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}