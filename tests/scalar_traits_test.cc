// neko::ScalarTraits and the deterministic inverse trigonometry (Asin, Atan2)
#include "neko/math/scalar_traits.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <numbers>
#include <type_traits>
#include <utility>

#include <gtest/gtest.h>

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/fixed.h"
#include "neko/math/vec2.h"

namespace {

using Fixed16Traits = neko::ScalarTraits<neko::Fixed16>;
using FloatTraits = neko::ScalarTraits<float>;

constexpr double kStep = 1.0 / 65536.0;

double ToDouble(neko::Fixed16 value) {
  return static_cast<double>(value.underlying_value()) * kStep;
}

neko::Fixed16 Raw(std::int32_t raw) {
  return neko::Fixed16::FromUnderlyingValue(raw);
}

// Deterministic sequence covering the whole 32-bit range
struct Lcg {
  std::uint32_t state = 12345u;
  std::int32_t Next() {
    state = state * 1664525u + 1013904223u;
    return static_cast<std::int32_t>(state);
  }
};

TEST(ScalarTraits, Flags) {
  static_assert(Fixed16Traits::kIsDeterministic);
  static_assert(Fixed16Traits::kIsFixedPoint);
  static_assert(Fixed16Traits::kIsCrossPlatformExact);
  static_assert(!FloatTraits::kIsDeterministic);
  static_assert(!FloatTraits::kIsFixedPoint);
  static_assert(!FloatTraits::kIsCrossPlatformExact);
  static_assert(neko::ScalarTraits<neko::Fixed8>::kIsFixedPoint);
  EXPECT_EQ(Fixed16Traits::kDisplayName, "Fixed16");
  EXPECT_EQ(neko::ScalarTraits<neko::Fixed8>::kDisplayName, "Fixed8");
  EXPECT_EQ(FloatTraits::kDisplayName, "float");
}

TEST(ScalarTraits, BitsRoundTrip) {
  static_assert(Fixed16Traits::ToBits(neko::Fixed16{1}) == 65536);
  static_assert(Fixed16Traits::FromBits(32768) == neko::Fixed16{1, 2});
  static_assert(FloatTraits::ToBits(1.0f) == 0x3F800000u);
  static_assert(FloatTraits::FromBits(0x3F800000u) == 1.0f);
  for (const std::int32_t raw : {0, 1, -1, 65536, -123456789,
                                 std::numeric_limits<std::int32_t>::min()}) {
    EXPECT_EQ(Fixed16Traits::ToBits(Fixed16Traits::FromBits(raw)), raw);
  }
}

TEST(ScalarTraits, Constants) {
  EXPECT_EQ(Fixed16Traits::Epsilon(), Raw(1));
  EXPECT_EQ(Fixed16Traits::Max(),
            Raw(std::numeric_limits<std::int32_t>::max()));
  EXPECT_EQ(Fixed16Traits::FromInt(-3), neko::Fixed16{-3});
  EXPECT_EQ(Fixed16Traits::FromRatio(3, 2), Raw(98304));
  // Truncated toward zero, like Fixed16{1, 3}
  EXPECT_EQ(Fixed16Traits::FromRatio(1, 3), Raw(21845));
  EXPECT_EQ(FloatTraits::FromRatio(3, 2), 1.5f);
  EXPECT_EQ(FloatTraits::FromInt(-3), -3.0f);
  EXPECT_EQ(FloatTraits::Epsilon(), std::numeric_limits<float>::epsilon());
  EXPECT_GT(Fixed16Traits::Tolerance(), Fixed16Traits::Epsilon());
  EXPECT_GT(FloatTraits::Tolerance(), FloatTraits::Epsilon());
}

TEST(ScalarTraits, AbsAndFloor) {
  EXPECT_EQ(Fixed16Traits::Abs(neko::Fixed16{-2}), neko::Fixed16{2});
  EXPECT_EQ(Fixed16Traits::Floor(neko::Fixed16{5, 2}), neko::Fixed16{2});
  EXPECT_EQ(Fixed16Traits::Floor(neko::Fixed16{-5, 2}), neko::Fixed16{-3});
  EXPECT_EQ(Fixed16Traits::Floor(neko::Fixed16{-2}), neko::Fixed16{-2});
  EXPECT_EQ(Fixed16Traits::Floor(Raw(-1)), neko::Fixed16{-1});
  EXPECT_EQ(Fixed16Traits::Floor(Raw(65535)), neko::Fixed16{0});
  EXPECT_EQ(FloatTraits::Floor(-2.5f), -3.0f);
  EXPECT_EQ(FloatTraits::Abs(-2.5f), 2.5f);
}

TEST(ScalarTraits, ForwardsTheExistingFunctions) {
  for (const std::int32_t raw : {0, 1, 65536, 100000, -100000, 3000000}) {
    EXPECT_EQ(Fixed16Traits::Sqrt(Raw(raw)), neko::Sqrt(Raw(raw)));
    EXPECT_EQ(Fixed16Traits::Sin(Raw(raw)), neko::Sin(Raw(raw)));
    EXPECT_EQ(Fixed16Traits::Cos(Raw(raw)), neko::Cos(Raw(raw)));
  }
}

// Every 16.16 input from -1 to 1 (131073 values)
TEST(Fixed16Asin, ErrorOverTheWholeDomain) {
  double max_error = 0.0;
  for (std::int32_t raw = -65536; raw <= 65536; ++raw) {
    const double exact = std::asin(static_cast<double>(raw) * kStep);
    const double error =
        std::abs(ToDouble(Fixed16Traits::Asin(Raw(raw))) - exact);
    max_error = std::max(max_error, error);
    ASSERT_LE(error, ToDouble(Fixed16Traits::AngleTolerance())) << raw;
  }
  std::cout << "Fixed16 Asin max error: " << max_error / kStep << " steps ("
            << max_error << " rad)\n";
}

TEST(Fixed16Asin, ExactValues) {
  EXPECT_EQ(Fixed16Traits::Asin(neko::Fixed16{0}), neko::Fixed16{0});
  // round(pi / 2 * 65536) = round(102943.7)
  EXPECT_EQ(Fixed16Traits::Asin(neko::Fixed16{1}), Raw(102944));
  EXPECT_EQ(Fixed16Traits::Asin(neko::Fixed16{-1}), Raw(-102944));
  // pi / 6 * 65536 = 34314.6
  EXPECT_EQ(Fixed16Traits::Asin(neko::Fixed16{1, 2}), Raw(34315));
  static_assert(Fixed16Traits::Asin(neko::Fixed16{1}) ==
                neko::Fixed16::FromUnderlyingValue(102944));
}

TEST(Fixed16Asin, ClampsOutsideTheDomain) {
  EXPECT_EQ(Fixed16Traits::Asin(Raw(65537)), Raw(102944));
  EXPECT_EQ(Fixed16Traits::Asin(neko::Fixed16{100}), Raw(102944));
  EXPECT_EQ(Fixed16Traits::Asin(Raw(std::numeric_limits<std::int32_t>::min())),
            Raw(-102944));
  EXPECT_EQ(Fixed16Traits::Asin(Raw(std::numeric_limits<std::int32_t>::max())),
            Raw(102944));
}

TEST(Fixed16Asin, OddAndNeverDecreasing) {
  neko::Fixed16 previous = Fixed16Traits::Asin(neko::Fixed16{-1});
  for (std::int32_t raw = -65536; raw <= 65536; ++raw) {
    const auto value = Fixed16Traits::Asin(Raw(raw));
    ASSERT_EQ(Fixed16Traits::Asin(Raw(-raw)), -value) << raw;
    ASSERT_GE(value, previous) << raw;
    previous = value;
  }
}

TEST(Fixed16Asin, SinOfAsinGivesTheValueBack) {
  for (std::int32_t raw = -65536; raw <= 65536; raw += 7) {
    const auto back = neko::Sin(Fixed16Traits::Asin(Raw(raw)));
    // The table sine's own error (5e-5) dominates
    EXPECT_NEAR(ToDouble(back), static_cast<double>(raw) * kStep, 6e-5) << raw;
  }
}

TEST(Fixed16Atan2, AxesAndDiagonals) {
  const neko::Fixed16 one{1};
  const neko::Fixed16 zero{0};
  EXPECT_EQ(Fixed16Traits::Atan2(zero, zero), zero);
  EXPECT_EQ(Fixed16Traits::Atan2(zero, one), zero);
  EXPECT_EQ(Fixed16Traits::Atan2(one, zero), Raw(102944));
  EXPECT_EQ(Fixed16Traits::Atan2(-one, zero), Raw(-102944));
  // y == 0 counts as positive: pi, like std::atan2(+0, -1). Equal to
  // neko::Pi<Fixed16>()
  EXPECT_EQ(Fixed16Traits::Atan2(zero, -one), neko::Pi<neko::Fixed16>());
  // pi / 4 * 65536 = 51471.85, 3 pi / 4 * 65536 = 154415.6
  EXPECT_EQ(Fixed16Traits::Atan2(one, one), Raw(51472));
  EXPECT_EQ(Fixed16Traits::Atan2(one, -one), Raw(154416));
  EXPECT_EQ(Fixed16Traits::Atan2(-one, -one), Raw(-154416));
  EXPECT_EQ(Fixed16Traits::Atan2(-one, one), Raw(-51472));
  // Only the direction matters
  EXPECT_EQ(Fixed16Traits::Atan2(neko::Fixed16{300}, neko::Fixed16{300}),
            Raw(51472));
  EXPECT_EQ(Fixed16Traits::Atan2(Raw(1), Raw(1)), Raw(51472));
  static_assert(Fixed16Traits::Atan2(neko::Fixed16{1}, neko::Fixed16{1}) ==
                neko::Fixed16::FromUnderlyingValue(51472));
}

TEST(Fixed16Atan2, QuadrantSigns) {
  const neko::Fixed16 a{3};
  const neko::Fixed16 b{1, 2};
  for (const auto& [y, x] : {std::pair{a, b}, std::pair{b, a}}) {
    const auto q1 = Fixed16Traits::Atan2(y, x);
    const auto q2 = Fixed16Traits::Atan2(y, -x);
    const auto q3 = Fixed16Traits::Atan2(-y, -x);
    const auto q4 = Fixed16Traits::Atan2(-y, x);
    const auto half_pi = Raw(102944);
    EXPECT_GT(q1, neko::Fixed16{0});
    EXPECT_LT(q1, half_pi);
    EXPECT_GT(q2, half_pi);
    EXPECT_LT(q3, -half_pi);
    EXPECT_LT(q4, neko::Fixed16{0});
    EXPECT_GT(q4, -half_pi);
    // Mirror images: exact
    EXPECT_EQ(q4, -q1);
    EXPECT_EQ(q3, -q2);
  }
}

double Atan2Error(std::int32_t y, std::int32_t x) {
  const double exact =
      std::atan2(static_cast<double>(y), static_cast<double>(x));
  return std::abs(ToDouble(Fixed16Traits::Atan2(Raw(y), Raw(x))) - exact);
}

TEST(Fixed16Atan2, ErrorOnADenseGrid) {
  double max_error = 0.0;
  // Every raw pair from -4 to 4 in steps of 1/64 (513 x 513 vectors), then
  // small vectors, where the ratio of two raw integers is coarse
  for (std::int32_t y = -262144; y <= 262144; y += 1024) {
    for (std::int32_t x = -262144; x <= 262144; x += 1024) {
      max_error = std::max(max_error, Atan2Error(y, x));
    }
  }
  for (std::int32_t y = -40; y <= 40; ++y) {
    for (std::int32_t x = -40; x <= 40; ++x) {
      max_error = std::max(max_error, Atan2Error(y, x));
    }
  }
  // The whole 32-bit range, including INT32_MIN
  Lcg lcg;
  for (int i = 0; i < 200000; ++i) {
    max_error = std::max(max_error, Atan2Error(lcg.Next(), lcg.Next()));
  }
  constexpr auto min_raw = std::numeric_limits<std::int32_t>::min();
  constexpr auto max_raw = std::numeric_limits<std::int32_t>::max();
  for (const auto& [y, x] :
       {std::pair{min_raw, min_raw}, std::pair{min_raw, max_raw},
        std::pair{max_raw, min_raw}, std::pair{0, min_raw},
        std::pair{min_raw, 0}, std::pair{1, min_raw}}) {
    max_error = std::max(max_error, Atan2Error(y, x));
  }
  std::cout << "Fixed16 Atan2 max error: " << max_error / kStep << " steps ("
            << max_error << " rad)\n";
  EXPECT_LE(max_error, ToDouble(Fixed16Traits::AngleTolerance()));
}

TEST(Fixed16Atan2, NeverDecreasingAlongAQuarterCircle) {
  // Turning from +x to +y: (r cos, r sin) with x falling and y rising
  neko::Fixed16 previous{0};
  constexpr std::int32_t radius = 1 << 20;
  for (std::int32_t y = 0; y <= radius; y += 37) {
    const auto x = static_cast<std::int32_t>(std::sqrt(
        static_cast<double>(radius) * radius - static_cast<double>(y) * y));
    const auto angle = Fixed16Traits::Atan2(Raw(y), Raw(x));
    ASSERT_GE(angle, previous) << y;
    previous = angle;
  }
}

TEST(FloatTraits, InverseTrig) {
  EXPECT_FLOAT_EQ(FloatTraits::Asin(1.0f), std::numbers::pi_v<float> / 2.0f);
  // Clamped: no NaN just past 1
  EXPECT_FLOAT_EQ(FloatTraits::Asin(1.0001f), std::numbers::pi_v<float> / 2.0f);
  EXPECT_FLOAT_EQ(FloatTraits::Asin(-2.0f), -std::numbers::pi_v<float> / 2.0f);
  EXPECT_EQ(FloatTraits::Atan2(0.0f, 0.0f), 0.0f);
  EXPECT_FLOAT_EQ(FloatTraits::Atan2(0.0f, -1.0f), std::numbers::pi_v<float>);
  EXPECT_FLOAT_EQ(FloatTraits::Atan2(1.0f, 1.0f),
                  std::numbers::pi_v<float> / 4.0f);
  // float keeps the sign of a negative zero y (documented), Fixed16 has none
  EXPECT_FLOAT_EQ(FloatTraits::Atan2(-0.0f, -1.0f), -std::numbers::pi_v<float>);
}

TEST(ScalarTraits, TypeAlias) {
  static_assert(std::is_same_v<FloatTraits::Type, float>);
  static_assert(std::is_same_v<Fixed16Traits::Type, neko::Fixed16>);
}

// The Radian API on the configured Scalar
TEST(Angle, AsinAndAtan2ReturnRadians) {
  using Traits = neko::ScalarTraits<neko::Scalar>;
  const neko::Radian half_pi = neko::Asin(neko::Scalar{1});
  EXPECT_NEAR(Traits::ToFloat(half_pi.value()),
              std::numbers::pi_v<float> / 2.0f, 1e-4f);
  const neko::Radian diagonal = neko::Atan2(neko::Scalar{-2}, neko::Scalar{-2});
  EXPECT_NEAR(Traits::ToFloat(diagonal.value()),
              -3.0f * std::numbers::pi_v<float> / 4.0f, 1e-4f);
  EXPECT_EQ(neko::Atan2(neko::Scalar{0}, neko::Scalar{0}).value(),
            neko::Scalar{0});
  // Sin(Asin(x)) and the direction of Atan2 come back
  const auto x = Traits::FromRatio(3, 5);
  EXPECT_NEAR(Traits::ToFloat(neko::Sin(neko::Asin(x))), 0.6f, 1e-4f);
}

// The float tolerance is checked against the exact angle of the float input
TEST(FloatTraits, AngleToleranceHolds) {
  const auto tolerance = static_cast<double>(FloatTraits::AngleTolerance());
  double worst = 0.0;
  for (int i = -20000; i <= 20000; ++i) {
    const float x = static_cast<float>(i) / 20000.0f;
    worst = std::max(worst, std::abs(static_cast<double>(FloatTraits::Asin(x)) -
                                     std::asin(static_cast<double>(x))));
  }
  for (int iy = -100; iy <= 100; ++iy) {
    for (int ix = -100; ix <= 100; ++ix) {
      const float y = static_cast<float>(iy) / 7.0f;
      const float x = static_cast<float>(ix) / 7.0f;
      worst = std::max(
          worst,
          std::abs(static_cast<double>(FloatTraits::Atan2(y, x)) -
                   std::atan2(static_cast<double>(y), static_cast<double>(x))));
    }
  }
  std::cout << "float inverse trig worst error " << worst << " rad\n";
  EXPECT_LE(worst, tolerance);
}

// Large numerators no longer go through the integer part
TEST(ScalarTraits, FromRatioWideNumerators) {
  EXPECT_EQ(Fixed16Traits::FromRatio(100000, 1000), neko::Fixed16{100});
  EXPECT_EQ(Fixed16Traits::FromRatio(-1000000, 100000), neko::Fixed16{-10});
  EXPECT_EQ(Fixed16Traits::FromRatio(1, 100000), Raw(0));
  // Same bits as Fixed16{num, den} wherever that one doesn't overflow
  for (int num = -300; num <= 300; num += 7) {
    for (int den = -250; den <= 250; den += 11) {
      if (den == 0) {
        continue;
      }
      ASSERT_EQ(Fixed16Traits::FromRatio(num, den), (neko::Fixed16{num, den}))
          << num << " / " << den;
    }
  }
  using Fixed8Traits = neko::ScalarTraits<neko::Fixed8>;
  EXPECT_EQ(Fixed8Traits::FromRatio(300, 200),
            neko::Fixed8::FromUnderlyingValue(96));
}

// Vec2 of a type without ScalarTraits keeps its float path
TEST(Vec2Traits, TypesWithoutTraitsKeepLength) {
  EXPECT_EQ((neko::Vec2<double>{3.0, 4.0}.Length()), 5.0);
  EXPECT_EQ((neko::Vec2<double>{3.0, 4.0}.Normalized().x()), 0.6);
  EXPECT_EQ(neko::CompareLength(neko::Vec2i{3, 4}, 5), 0);
  EXPECT_LT(neko::CompareLength(neko::Vec2i{3, 4}, 6), 0);
}

// Fixed8 lengths saturate at the largest Fixed8 instead of wrapping
TEST(Vec2Traits, Fixed8LengthSaturates) {
  const auto one_and_half = neko::Fixed8::FromUnderlyingValue(96);
  const neko::Vec2<neko::Fixed8> v{one_and_half, one_and_half};
  EXPECT_EQ(v.Length(), std::numeric_limits<neko::Fixed8>::max());
  const auto half = neko::Fixed8::FromUnderlyingValue(32);
  const neko::Vec2<neko::Fixed8> small{half, neko::Fixed8{0}};
  EXPECT_EQ(small.Length(), half);
}

}  // namespace

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
