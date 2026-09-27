#include "math/angle.h"

#include <gtest/gtest.h>

#include <numbers>

// angle.h did not compile (Pi() without template argument), nothing included it (review S3.1)
TEST(Angle, DegreeToRadian)
{
    const neko::Radian radian = neko::Degree{ neko::Scalar{90} };
    EXPECT_NEAR(static_cast<float>(radian.value()), std::numbers::pi_v<float> / 2.0f, 1e-4f);
    const neko::Degree degree = neko::Radian{ neko::Pi<neko::Scalar>() };
    EXPECT_NEAR(static_cast<float>(degree.value()), 180.0f, 1e-2f);
}

TEST(Angle, Trigonometry)
{
    EXPECT_NEAR(static_cast<float>(neko::Sin(neko::Radian{ neko::Degree{ neko::Scalar{30} } })), 0.5f, 1e-4f);
    EXPECT_NEAR(static_cast<float>(neko::Cos(neko::Radian{ neko::Degree{ neko::Scalar{60} } })), 0.5f, 1e-4f);
    EXPECT_NEAR(static_cast<float>(neko::Tan(neko::Radian{ neko::Degree{ neko::Scalar{45} } })), 1.0f, 1e-3f);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
