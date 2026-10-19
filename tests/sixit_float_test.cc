// The sixit deterministic float (NEKO_SCALAR=SIXIT): its ScalarTraits, and
// constant evaluation giving the same bits as the run-time instructions
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <type_traits>

#include <gtest/gtest.h>

#include "neko/math/const.h"
#include "neko/math/scalar_traits.h"

#ifdef NEKO_SCALAR_SIXIT
#include "neko/math/sixit_float.h"

namespace {

using Sixit = sixit::dmath::ieee_float_inline_asm;
using Traits = neko::ScalarTraits<Sixit>;

// Recorded on Linux clang 22 and GCC 16 (x64); see
// InverseTrigBitsAreTheSameEverywhere
constexpr std::uint32_t kSixitMathHash = 0x0FD77370u;

static_assert(std::is_same_v<neko::Scalar, Sixit>);
static_assert(Traits::kIsDeterministic);
static_assert(!Traits::kIsFixedPoint);
static_assert(Traits::kIsCrossPlatformExact);

// Constants are constexpr: the reason for the constant-evaluation branch
constexpr Sixit kThree{3};
constexpr Sixit kQuarter = Sixit{1} / Sixit{4};
constexpr Sixit kHalfPi = neko::Pi<Sixit>() / Sixit{2};
static_assert(Traits::ToBits(kThree) == 0x40400000u);
static_assert(Traits::ToBits(kQuarter) == 0x3E800000u);
static_assert(Traits::ToBits(-kQuarter) == 0xBE800000u);
static_assert(kQuarter < kThree && kThree > kQuarter && kThree != kQuarter);
static_assert(Traits::FromRatio(1, 3) == Sixit{1} / Sixit{3});
static_assert(Traits::ToBits(Traits::FromBits(0x3F800001u)) == 0x3F800001u);

// Deterministic values spread over many magnitudes and both signs
constexpr std::size_t kCount = 256;
constexpr std::array<std::uint32_t, kCount> MakeBits() {
  std::array<std::uint32_t, kCount> bits{};
  std::uint32_t state = 12345u;
  for (auto& b : bits) {
    state = state * 1664525u + 1013904223u;
    // Exponent between 2^-20 and 2^20, random mantissa and sign
    const std::uint32_t exponent = 107u + (state >> 8) % 41u;
    b = (state & 0x80000000u) | (exponent << 23) | (state & 0x7FFFFFu);
  }
  return bits;
}
constexpr auto kBits = MakeBits();

struct Results {
  std::array<std::uint32_t, kCount> add{};
  std::array<std::uint32_t, kCount> sub{};
  std::array<std::uint32_t, kCount> mul{};
  std::array<std::uint32_t, kCount> div{};
  std::array<std::uint32_t, kCount> chain{};
  std::array<bool, kCount> less{};
};

// One operation per entry, then a chain a * b + c / a - b, which a
// contracting compiler would turn into an FMA
constexpr Results Compute() {
  Results r{};
  for (std::size_t i = 0; i < kCount; ++i) {
    const Sixit a = Traits::FromBits(kBits[i]);
    const Sixit b = Traits::FromBits(kBits[(i + 1) % kCount]);
    const Sixit c = Traits::FromBits(kBits[(i + 7) % kCount]);
    r.add[i] = Traits::ToBits(a + b);
    r.sub[i] = Traits::ToBits(a - b);
    r.mul[i] = Traits::ToBits(a * b);
    r.div[i] = Traits::ToBits(a / b);
    Sixit chain = a * b;
    chain += c / a;
    chain -= b;
    r.chain[i] = Traits::ToBits(chain);
    r.less[i] = a < b;
  }
  return r;
}

TEST(SixitFloat, ConstantEvaluationMatchesRunTime) {
  constexpr Results at_compile_time = Compute();
  // volatile keeps the run-time computation out of constant folding
  volatile std::size_t count = kCount;
  Results at_run_time{};
  for (std::size_t i = 0; i < count; ++i) {
    const Sixit a = Traits::FromBits(kBits[i]);
    const Sixit b = Traits::FromBits(kBits[(i + 1) % kCount]);
    const Sixit c = Traits::FromBits(kBits[(i + 7) % kCount]);
    at_run_time.add[i] = Traits::ToBits(a + b);
    at_run_time.sub[i] = Traits::ToBits(a - b);
    at_run_time.mul[i] = Traits::ToBits(a * b);
    at_run_time.div[i] = Traits::ToBits(a / b);
    Sixit chain = a * b;
    chain += c / a;
    chain -= b;
    at_run_time.chain[i] = Traits::ToBits(chain);
    at_run_time.less[i] = a < b;
  }
  for (std::size_t i = 0; i < kCount; ++i) {
    EXPECT_EQ(at_compile_time.add[i], at_run_time.add[i]) << i;
    EXPECT_EQ(at_compile_time.sub[i], at_run_time.sub[i]) << i;
    EXPECT_EQ(at_compile_time.mul[i], at_run_time.mul[i]) << i;
    EXPECT_EQ(at_compile_time.div[i], at_run_time.div[i]) << i;
    EXPECT_EQ(at_compile_time.chain[i], at_run_time.chain[i]) << i;
    EXPECT_EQ(at_compile_time.less[i], at_run_time.less[i]) << i;
  }
}

TEST(SixitFloat, IntegersConvertRoundedToNearest) {
  volatile int big = 16777217;  // 2^24 + 1, halfway: rounds to even 2^24
  EXPECT_EQ(Traits::ToBits(Sixit{big}), 0x4B800000u);
  volatile int minus_seven = -7;
  EXPECT_EQ(Traits::ToBits(Sixit{minus_seven}), 0xC0E00000u);
  EXPECT_EQ(Traits::FromInt(5), Sixit{5});
}

TEST(SixitFloat, Functions) {
  EXPECT_EQ(Traits::Sqrt(Sixit{9}), kThree);
  EXPECT_EQ(Traits::Abs(-kQuarter), kQuarter);
  EXPECT_EQ(Traits::Floor(Traits::FromRatio(-5, 2)), Sixit{-3});
  EXPECT_EQ(Traits::Sin(Sixit{0}), Sixit{0});
  EXPECT_EQ(Traits::Cos(Sixit{0}), Sixit{1});
  EXPECT_LT(Traits::Abs(Traits::Sin(kHalfPi) - Sixit{1}), Traits::Tolerance());
  EXPECT_LT(Traits::Abs(neko::Tan(neko::Pi<Sixit>() / Sixit{4}) - Sixit{1}),
            Traits::Tolerance());
  EXPECT_EQ(Traits::ToFloat(kQuarter), 0.25f);
  EXPECT_EQ(Traits::kDisplayName, "sixit");
}

// Error against the exact angle of the sixit input, computed in double
TEST(SixitFloat, InverseTrigWithinTheAngleTolerance) {
  const auto tolerance =
      static_cast<double>(Traits::ToFloat(Traits::AngleTolerance()));
  double worst_asin = 0.0;
  for (int i = -20000; i <= 20000; ++i) {
    const Sixit x = Traits::FromRatio(i, 20000);
    const double exact = std::asin(static_cast<double>(Traits::ToFloat(x)));
    worst_asin = std::max(
        worst_asin,
        std::abs(static_cast<double>(Traits::ToFloat(Traits::Asin(x))) -
                 exact));
  }
  double worst_atan2 = 0.0;
  for (int iy = -100; iy <= 100; ++iy) {
    for (int ix = -100; ix <= 100; ++ix) {
      const Sixit y = Traits::FromRatio(iy, 7);
      const Sixit x = Traits::FromRatio(ix, 7);
      const double exact = std::atan2(static_cast<double>(Traits::ToFloat(y)),
                                      static_cast<double>(Traits::ToFloat(x)));
      worst_atan2 = std::max(
          worst_atan2,
          std::abs(static_cast<double>(Traits::ToFloat(Traits::Atan2(y, x))) -
                   exact));
    }
  }
  std::cout << "sixit worst error: asin " << worst_asin << " rad, atan2 "
            << worst_atan2 << " rad\n";
  EXPECT_LE(worst_asin, tolerance);
  EXPECT_LE(worst_atan2, tolerance);
}

TEST(SixitFloat, InverseTrigEdgeCases) {
  const Sixit half_pi = Traits::Asin(Sixit{1});
  EXPECT_LT(Traits::Abs(half_pi - kHalfPi), Traits::AngleTolerance());
  // Clamped: no NaN just past 1
  EXPECT_EQ(Traits::Asin(Traits::FromRatio(10001, 10000)), half_pi);
  EXPECT_EQ(Traits::Asin(Sixit{-2}), -half_pi);
  EXPECT_EQ(Traits::Asin(Sixit{0}), Sixit{0});
  // Odd
  for (int i = 1; i <= 1000; ++i) {
    const Sixit x = Traits::FromRatio(i, 1000);
    EXPECT_EQ(Traits::Asin(-x), -Traits::Asin(x)) << i;
  }
  EXPECT_EQ(Traits::ToBits(Traits::Atan2(Sixit{0}, Sixit{0})), 0u);
  const Sixit pi = Traits::Atan2(Sixit{0}, Sixit{-1});
  EXPECT_LT(Traits::Abs(pi - neko::Pi<Sixit>()), Traits::AngleTolerance());
  // Like std::atan2, the sign of a zero y counts
  EXPECT_EQ(Traits::Atan2(-Sixit{0}, Sixit{-1}), -pi);
  EXPECT_LT(Traits::Abs(Traits::Atan2(Sixit{1}, Sixit{1}) -
                        neko::Pi<Sixit>() / Sixit{4}),
            Traits::AngleTolerance());
  EXPECT_LT(Traits::Abs(Traits::Atan2(Sixit{-2}, Sixit{-2}) +
                        Sixit{3} * neko::Pi<Sixit>() / Sixit{4}),
            Traits::AngleTolerance());
}

// FNV-1a of the bits of every result: the same number on every platform
// (Windows MSVC, Linux clang and GCC, NX, macOS arm64) or SIXIT isn't
// cross-platform exact
TEST(SixitFloat, InverseTrigBitsAreTheSameEverywhere) {
  std::uint32_t hash = 2166136261u;
  const auto mix = [&hash](Sixit value) {
    const std::uint32_t bits = Traits::ToBits(value);
    for (int shift = 0; shift < 32; shift += 8) {
      hash = (hash ^ ((bits >> shift) & 0xFFu)) * 16777619u;
    }
  };
  for (int i = -20000; i <= 20000; ++i) {
    const Sixit x = Traits::FromRatio(i, 20000);
    mix(Traits::Asin(x));
    mix(Traits::Sin(x * Sixit{7}));
    mix(Traits::Cos(x * Sixit{7}));
    mix(Traits::Sqrt(Traits::Abs(x) * Sixit{100}));
  }
  for (int iy = -100; iy <= 100; ++iy) {
    for (int ix = -100; ix <= 100; ++ix) {
      mix(Traits::Atan2(Traits::FromRatio(iy, 7), Traits::FromRatio(ix, 7)));
    }
  }
  std::cout << "sixit math hash " << std::hex << hash << std::dec << "\n";
  EXPECT_EQ(hash, kSixitMathHash);
}

}  // namespace

#else

TEST(SixitFloat, NeedsSixitScalar) {
  GTEST_SKIP() << "the sixit float tests are for NEKO_SCALAR=SIXIT";
}

#endif

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
