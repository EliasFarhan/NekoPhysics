#ifndef NEKO_MATH_FIXED_TRIG_H_
#define NEKO_MATH_FIXED_TRIG_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "neko/math/fixed.h"

namespace neko {
namespace internal {
inline constexpr std::size_t kSinTableBits = 10;
inline constexpr std::size_t kSinTableSize = std::size_t{1} << kSinTableBits;

// Compile-time sine (Taylor series on [-pi/2, pi/2]), only used to build the
// table
consteval double ConstevalSin(double x) {
  constexpr double pi = 3.14159265358979323846;
  while (x > pi) {
    x -= 2.0 * pi;
  }
  while (x < -pi) {
    x += 2.0 * pi;
  }
  if (x > pi / 2.0) {
    x = pi - x;
  } else if (x < -pi / 2.0) {
    x = -pi - x;
  }
  const double square = x * x;
  double term = x;
  double sum = x;
  for (int n = 1; n < 12; ++n) {
    term *= -square / static_cast<double>((2 * n) * (2 * n + 1));
    sum += term;
  }
  return sum;
}

// sin over one full turn in 16.16, plus the first value again so interpolation
// never wraps
consteval std::array<std::int32_t, kSinTableSize + 1> MakeSinTable() {
  constexpr double pi = 3.14159265358979323846;
  std::array<std::int32_t, kSinTableSize + 1> table{};
  for (std::size_t i = 0; i <= kSinTableSize; ++i) {
    const double value = ConstevalSin(2.0 * pi * static_cast<double>(i) /
                                      static_cast<double>(kSinTableSize)) *
                         65536.0;
    table[i] =
        static_cast<std::int32_t>(value >= 0.0 ? value + 0.5 : value - 0.5);
  }
  return table;
}

inline constexpr auto kSinTable = MakeSinTable();

// Angle in radians to a phase in 1/2^32 of a turn. One multiplication, no
// division, and the wrap-around to [0, 2 pi) is the uint32_t overflow.
[[nodiscard]] constexpr std::uint32_t RadiansToPhase(Fixed16 radians) noexcept {
  // round(2^32 / (2 pi))
  constexpr std::int64_t phase_per_radian = 683565276;
  return static_cast<std::uint32_t>(
      (static_cast<std::int64_t>(radians.underlying_value()) *
       phase_per_radian) >>
      16);
}

[[nodiscard]] constexpr Fixed16 SinFromPhase(std::uint32_t phase) noexcept {
  constexpr std::uint32_t fraction_bits = 32 - kSinTableBits;
  const auto index = phase >> fraction_bits;
  const auto fraction = static_cast<std::int64_t>(
      phase & ((std::uint32_t{1} << fraction_bits) - 1));
  const auto v1 = kSinTable[index];
  const auto v2 = kSinTable[index + 1];
  constexpr std::int64_t half = std::int64_t{1} << (fraction_bits - 1);
  const auto interpolation =
      (static_cast<std::int64_t>(v2 - v1) * fraction + half) >> fraction_bits;
  return Fixed16::FromUnderlyingValue(
      static_cast<std::int32_t>(v1 + interpolation));
}

// Inverse trigonometry in integers only. Angles are computed in Q30 (2^30 per
// radian, so a quarter turn fits in 31 bits) and rounded to 16.16 once, at the
// end.
inline constexpr std::size_t kAtanTableBits = 8;
inline constexpr std::size_t kAtanTableSize = std::size_t{1} << kAtanTableBits;
// round(pi / 2 * 2^30) and round(pi * 2^30)
inline constexpr std::int64_t kHalfPiQ30 = 1686629713;
inline constexpr std::int64_t kPiQ30 = 3373259426;

// Compile-time arctangent on [0, 1], only used to build the table. Above
// tan(pi / 8), atan(x) = pi / 4 + atan((x - 1) / (x + 1)) brings the argument
// back to |u| <= 0.4143, where the series u - u^3 / 3 + u^5 / 5... converges
// fast.
consteval double ConstevalAtan(double x) {
  constexpr double pi = 3.14159265358979323846;
  constexpr double tan_eighth_turn = 0.41421356237309504880;
  double offset = 0.0;
  if (x > tan_eighth_turn) {
    offset = pi / 4.0;
    x = (x - 1.0) / (x + 1.0);
  }
  const double square = x * x;
  double power = x;
  double sum = 0.0;
  for (int n = 0; n < 40; ++n) {
    const double term = power / static_cast<double>(2 * n + 1);
    sum += (n % 2 == 0) ? term : -term;
    power *= square;
  }
  return offset + sum;
}

// atan(i / 256) in Q30 for i in 0..256, plus the last value again so that
// interpolation at t = 1 never reads past the end
consteval std::array<std::int32_t, kAtanTableSize + 2> MakeAtanTable() {
  std::array<std::int32_t, kAtanTableSize + 2> table{};
  for (std::size_t i = 0; i <= kAtanTableSize; ++i) {
    const double value = ConstevalAtan(static_cast<double>(i) /
                                       static_cast<double>(kAtanTableSize)) *
                         1073741824.0;
    table[i] = static_cast<std::int32_t>(value + 0.5);
  }
  table[kAtanTableSize + 1] = table[kAtanTableSize];
  return table;
}

inline constexpr auto kAtanTable = MakeAtanTable();

// atan(t) in Q30 for t in Q24 in [0, 2^24] (0 to 1): table lookup and linear
// interpolation (error below 2e-6 rad, far under one 16.16 step)
[[nodiscard]] constexpr std::int64_t AtanQ30(std::int32_t t) noexcept {
  constexpr std::int32_t fraction_bits = 24 - static_cast<int>(kAtanTableBits);
  const auto index = static_cast<std::size_t>(t >> fraction_bits);
  const auto fraction =
      static_cast<std::int64_t>(t & ((std::int32_t{1} << fraction_bits) - 1));
  const auto v1 = static_cast<std::int64_t>(kAtanTable[index]);
  const auto v2 = static_cast<std::int64_t>(kAtanTable[index + 1]);
  constexpr std::int64_t half = std::int64_t{1} << (fraction_bits - 1);
  return v1 + (((v2 - v1) * fraction + half) >> fraction_bits);
}

// Angle of the vector (x, y), both not negative and in the same unit, in Q30:
// from 0 (along +x) to pi / 2 (along +y). 0 for the zero vector. The smaller
// component is divided by the larger one, so the ratio stays in [0, 1].
[[nodiscard]] constexpr std::int64_t FirstQuadrantAngleQ30(
    std::uint64_t x, std::uint64_t y) noexcept {
  if (x == 0 && y == 0) {
    return 0;
  }
  // The divider takes a 32-bit divisor: halve both (same angle) while the
  // larger doesn't fit. Only a raw INT32_MIN component gets here.
  constexpr auto max_divisor =
      static_cast<std::uint64_t>(std::numeric_limits<std::int32_t>::max());
  while (x > max_divisor || y > max_divisor) {
    x >>= 1;
    y >>= 1;
  }
  const bool is_steep = y > x;
  const auto numerator = static_cast<std::int64_t>(is_steep ? x : y);
  const auto denominator = static_cast<std::int32_t>(is_steep ? y : x);
  // ratio in Q24, truncated, in [0, 2^24]
  const std::int32_t ratio = Div64(numerator << 24, denominator);
  const std::int64_t angle = AtanQ30(ratio);
  return is_steep ? kHalfPiQ30 - angle : angle;
}

// Rounds a not negative Q30 angle to the nearest 16.16 step (ties up) and
// gives it the sign: symmetric, so f(-x) == -f(x) exactly
[[nodiscard]] constexpr Fixed16 RoundQ30Angle(std::int64_t angle,
                                              bool is_negative) noexcept {
  constexpr std::int64_t half = std::int64_t{1} << 13;
  const auto raw = static_cast<std::int32_t>((angle + half) >> 14);
  return Fixed16::FromUnderlyingValue(is_negative ? -raw : raw);
}

// atan2(y, x) in radians, in (-pi, pi]. Atan2(0, 0) is 0, and y == 0 counts as
// positive (Atan2(0, -1) is pi, like std::atan2(+0, -1)). Error at most about
// 0.6 of a step against the exact angle of the 16.16 inputs.
[[nodiscard]] constexpr Fixed16 Atan2Fixed16(Fixed16 y, Fixed16 x) noexcept {
  const auto raw_y = static_cast<std::int64_t>(y.underlying_value());
  const auto raw_x = static_cast<std::int64_t>(x.underlying_value());
  const auto abs_y = static_cast<std::uint64_t>(raw_y < 0 ? -raw_y : raw_y);
  const auto abs_x = static_cast<std::uint64_t>(raw_x < 0 ? -raw_x : raw_x);
  std::int64_t angle = FirstQuadrantAngleQ30(abs_x, abs_y);
  if (raw_x < 0) {
    angle = kPiQ30 - angle;
  }
  return RoundQ30Angle(angle, raw_y < 0);
}

// asin(x) in radians, in [-pi / 2, pi / 2]; |x| > 1 is clamped to +/- 1.
// asin(x) = atan2(x, sqrt(1 - x^2)), with 1 - x^2 computed exactly as
// (1 - |x|)(1 + |x|) in Q32 and its square root in Q24, so the steep end near
// |x| = 1 keeps its precision.
[[nodiscard]] constexpr Fixed16 AsinFixed16(Fixed16 x) noexcept {
  constexpr std::int64_t one = std::int64_t{1} << 16;
  std::int64_t abs_x = x.underlying_value();
  const bool is_negative = abs_x < 0;
  if (is_negative) {
    abs_x = -abs_x;
  }
  if (abs_x > one) {
    abs_x = one;
  }
  // (1 - x^2) in Q32, shifted to Q48 so the root is in Q24
  const auto one_minus_square =
      static_cast<std::uint64_t>((one - abs_x) * (one + abs_x)) << 16;
  const auto cosine = static_cast<std::uint64_t>(ISqrt64(one_minus_square));
  const auto sine = static_cast<std::uint64_t>(abs_x) << 8;
  return RoundQ30Angle(FirstQuadrantAngleQ30(cosine, sine), is_negative);
}
}  // namespace internal

/// Sine of an angle in radians, deterministic and without floating point.
///
/// Looks the angle up in a 1024-entry sine table (built at compile time)
/// and linearly interpolates between the two nearest entries. The absolute
/// error is below 5e-5, there is no division, and any angle works: the
/// angle is wrapped to one turn for free by integer overflow.
///
/// @param radians The angle in radians.
/// @return The sine, in [-1, 1].
/// @see @ref Sin(Radian)
[[nodiscard]] constexpr Fixed16 Sin(Fixed16 radians) noexcept {
  return internal::SinFromPhase(internal::RadiansToPhase(radians));
}

/// Cosine of an angle in radians, deterministic and without floating point.
///
/// Uses the same table as @ref Sin, shifted by a quarter turn
/// (`cos(x) = sin(x + pi / 2)`), so it has the same accuracy.
///
/// @param radians The angle in radians.
/// @return The cosine, in [-1, 1].
[[nodiscard]] constexpr Fixed16 Cos(Fixed16 radians) noexcept {
  // cos(x) = sin(x + quarter turn)
  return internal::SinFromPhase(internal::RadiansToPhase(radians) +
                                (std::uint32_t{1} << 30));
}

/// Tangent of an angle in radians, computed as @ref Sin / @ref Cos.
///
/// Near odd multiples of pi / 2 the cosine is (close to) zero: the result
/// grows very large, and saturates instead of crashing when the cosine is
/// exactly zero (see the division of @ref Fixed).
///
/// @param radians The angle in radians.
/// @return The tangent.
[[nodiscard]] constexpr Fixed16 Tan(Fixed16 radians) noexcept {
  return Sin(radians) / Cos(radians);
}

}  // namespace neko

#endif  // NEKO_MATH_FIXED_TRIG_H_
