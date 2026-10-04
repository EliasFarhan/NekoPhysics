#include "neko/math/const.h"
#include "neko/math/fixed.h"
#include "neko/math/fixed_trig.h"
#include "neko/math/func.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
using neko::Fixed16;

constexpr Fixed16 Raw(std::int32_t raw) { return Fixed16::FromUnderlyingValue(raw); }

// numeric_limits (DS review D1.8)
static_assert(std::numeric_limits<Fixed16>::is_specialized);
static_assert(std::numeric_limits<Fixed16>::max() == Raw(std::numeric_limits<std::int32_t>::max()));
static_assert(std::numeric_limits<Fixed16>::lowest() == Raw(std::numeric_limits<std::int32_t>::min()));
static_assert(std::numeric_limits<Fixed16>::epsilon() == Raw(1));

// Multiplication rounds to nearest, ties away from zero, so it is symmetric (D1.5)
static_assert(Raw(3) * Fixed16{ 0.5f } == Raw(2));
static_assert(Raw(-3) * Fixed16{ 0.5f } == Raw(-2));
static_assert(Raw(1) * Fixed16{ 0.25f } == Raw(0));
static_assert([] { auto f = Raw(-3); f *= Fixed16{ 0.5f }; return f; }() == Raw(-2));

// Division truncates toward zero (like the DS divider) and saturates on zero (D1.3)
static_assert(Fixed16{ 1 } / Fixed16{ 3 } == Raw(21845));
static_assert(Fixed16{ -1 } / Fixed16{ 3 } == Raw(-21845));
static_assert([] { auto f = Fixed16{ 7 }; f /= Fixed16{ 2 }; return f; }() == Fixed16{ 3.5f });
static_assert(Fixed16{ 1 } / Fixed16{ 0 } == std::numeric_limits<Fixed16>::max());
static_assert(Fixed16{ -1 } / Fixed16{ 0 } == std::numeric_limits<Fixed16>::lowest());
static_assert(Fixed16{ 0 } / Fixed16{ 0 } == Fixed16{ 0 });

// What the DS hardware hooks must reproduce (the ROM in nds/ compares them on the DS)
static_assert(neko::internal::Div64(-7, 2) == -3);
static_assert(neko::internal::Div64(std::int64_t{ 1 } << 40, 3) == 1431655765); // low 32 bits of 366503875925
static_assert(neko::internal::Div64(std::int64_t{ -1 } << 47, 1) == 0);
static_assert(neko::internal::ISqrt64(~std::uint64_t{ 0 }) == 0xFFFFFFFFu);
static_assert(neko::internal::ISqrt64(99) == 9);

// Conversions: from float rounds to nearest, to integer truncates toward zero (S4.2)
static_assert(Fixed16{ 0.1f } == Raw(6554));
static_assert(Fixed16{ -0.1f } == Raw(-6554));
static_assert(static_cast<std::int32_t>(Fixed16{ -0.5f }) == 0);
static_assert(static_cast<std::int32_t>(Fixed16{ -1.5f }) == -1);
static_assert(static_cast<std::int32_t>(Fixed16{ 1.5f }) == 1);

// Sqrt is exact (floor), which is what the DS 64-bit sqrt unit returns (D1.7)
static_assert(neko::Sqrt(Fixed16{ 0 }) == Fixed16{ 0 });
static_assert(neko::Sqrt(Raw(1)) == Raw(256));
static_assert(neko::Sqrt(Fixed16{ 2 }) == Raw(92681));
static_assert(neko::Sqrt(Fixed16{ 10000 }) == Fixed16{ 100 });
static_assert(neko::Sqrt(Fixed16{ 16384 }) == Fixed16{ 128 });
static_assert(neko::Sqrt(std::numeric_limits<Fixed16>::max()) == Raw(11863283));

// Sin/Cos: compile-time table, radians turned into a phase by one multiplication (DS review D1.9)
static_assert(neko::Sin(Fixed16{ 0 }) == Fixed16{ 0 });
static_assert(neko::Cos(Fixed16{ 0 }) == Fixed16{ 1 });
static_assert(neko::internal::kSinTable[neko::internal::kSinTableSize / 4] == 65536);
static_assert(neko::internal::kSinTable[neko::internal::kSinTableSize] == 0);
}  // namespace

TEST(Fixed16, SinCosMatchStd)
{
    for (float x = -20.0f; x <= 20.0f; x += 0.0137f)
    {
        const Fixed16 angle{ x };
        const double exact = static_cast<double>(static_cast<float>(angle));
        EXPECT_NEAR(static_cast<float>(neko::Sin(angle)), std::sin(exact), 5e-5) << x;
        EXPECT_NEAR(static_cast<float>(neko::Cos(angle)), std::cos(exact), 5e-5) << x;
    }
}

TEST(Fixed16, TanMatchesStd)
{
    for (float x = -1.4f; x <= 1.4f; x += 0.01f)
    {
        const Fixed16 angle{ x };
        const double exact = std::tan(static_cast<double>(static_cast<float>(angle)));
        EXPECT_NEAR(static_cast<float>(neko::Tan(angle)), exact, 5e-5 * (1.0 + exact * exact)) << x;
    }
}

TEST(Fixed16, MulMatchesExactProductWithinHalfUlp)
{
    for (std::int32_t a = -2'000'000; a <= 2'000'000; a += 9'973)
    {
        for (std::int32_t b = -300'000; b <= 300'000; b += 7'919)
        {
            const auto result = Raw(a) * Raw(b);
            EXPECT_EQ(result, -(Raw(-a) * Raw(b))) << a << " * " << b;
            const double exact = static_cast<double>(a) * static_cast<double>(b) / 65536.0;
            EXPECT_LE(std::abs(static_cast<double>(result.underlying_value()) - exact), 0.5) << a << " * " << b;
        }
    }
}

TEST(Fixed16, SqrtIsFloorOfExactRoot)
{
    for (std::int64_t raw = 0; raw <= std::numeric_limits<std::int32_t>::max(); raw += 1'234'567)
    {
        const auto root = static_cast<std::uint64_t>(neko::Sqrt(Raw(static_cast<std::int32_t>(raw))).underlying_value());
        const auto square = static_cast<std::uint64_t>(raw) << 16;
        EXPECT_LE(root * root, square) << raw;
        EXPECT_GT((root + 1) * (root + 1), square) << raw;
    }
}


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

static constexpr float kEpsilon = 0.0001f;
TEST_P(FixedIntOperationFixture, Div)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 / f2;
    EXPECT_NEAR(static_cast<float>(i1) / static_cast<float>(i2),
        static_cast<float>(result), kEpsilon);

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
    EXPECT_NEAR(i1 + i2, static_cast<float>(result), kEpsilon);

}

TEST_P(FixedFloatOperationFixture, Sub)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 - f2;
    EXPECT_NEAR(i1 - i2, static_cast<float>(result), kEpsilon);


}

TEST_P(FixedFloatOperationFixture, Mul)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 * f2;
    EXPECT_NEAR(i1 * i2, static_cast<float>(result), kEpsilon);

}

TEST_P(FixedFloatOperationFixture, Div)
{
    auto [i1, i2] = GetParam();
    const auto f1 = neko::Fixed{ i1 };
    const auto f2 = neko::Fixed{ i2 };
    const auto result = f1 / f2;
    EXPECT_NEAR(i1 / i2, static_cast<float>(result), kEpsilon);

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
constexpr float kSqrtPrecision = 0.0125f;
TEST(LookupTable, Sqrt)
{
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sqrt(neko::Fixed(0))), kSqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Sqrt(neko::Fixed(1))), kSqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(3)), float(neko::Sqrt(neko::Fixed(9))), kSqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(7)), float(neko::Sqrt(neko::Fixed(49))), kSqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(9)), float(neko::Sqrt(neko::Fixed(81))), kSqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(10)), float(neko::Sqrt(neko::Fixed(100))), kSqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(11)), float(neko::Sqrt(neko::Fixed(121))), kSqrtPrecision);
    EXPECT_NEAR(float(neko::Fixed16(16)), float(neko::Sqrt(neko::Fixed(256))), kSqrtPrecision);
}

constexpr float kSinPrecision = 0.0075f;
TEST(LookupTable, Sin)
{
    EXPECT_NEAR(float(neko::Fixed16(-1, 2)), float(neko::Sin(-neko::Pi<neko::Fixed16>() / neko::Fixed(6))), kSinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sin(neko::Fixed(0))), kSinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1, 2)), float(neko::Sin(neko::Pi<neko::Fixed16>() / neko::Fixed(6))), kSinPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(2)) / neko::Fixed16(2)), float(neko::Sin(neko::Pi<neko::Fixed16>() / neko::Fixed(4))), kSinPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(3)) / neko::Fixed16(2)), float(neko::Sin(neko::Pi<neko::Fixed16>() * neko::Fixed(1,3))), kSinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Sin(neko::Pi<neko::Fixed16>() / neko::Fixed(2))), kSinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sin(neko::Pi<neko::Fixed16>())), kSinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(-1)), float(neko::Sin(neko::Pi<neko::Fixed16>()*neko::Fixed16(3, 2))), kSinPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Sin(neko::Pi<neko::Fixed16>()*neko::Fixed16(2))), kSinPrecision);
}

constexpr float kCosPrecision = 0.0075f;
TEST(LookupTable, Cos)
{
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(3)) / neko::Fixed16(2)), float(neko::Cos(-neko::Pi<neko::Fixed16>() / neko::Fixed16{6})), kCosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Cos(neko::Fixed(0))), kCosPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(3))/neko::Fixed16(2)), float(neko::Cos(neko::Pi<neko::Fixed16>() / neko::Fixed16{6})), kCosPrecision);
    EXPECT_NEAR(float(neko::Sqrt(neko::Fixed16(2))/neko::Fixed16(2)), float(neko::Cos(neko::Pi<neko::Fixed16>() / neko::Fixed16{4})), kCosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1,2)), float(neko::Cos(neko::Pi<neko::Fixed16>() / neko::Fixed16{3})), kCosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Cos(neko::Pi<neko::Fixed16>() / neko::Fixed16{2})), kCosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(-1)), float(neko::Cos(neko::Pi<neko::Fixed16>())), kCosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Cos(neko::Pi<neko::Fixed16>()*neko::Fixed16{3, 2})), kCosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(1)), float(neko::Cos(neko::Pi<neko::Fixed16>()*neko::Fixed16{2})), kCosPrecision);
    EXPECT_NEAR(float(neko::Fixed16(0)), float(neko::Cos(neko::Pi<neko::Fixed16>()*neko::Fixed16{5,2})), kCosPrecision);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}