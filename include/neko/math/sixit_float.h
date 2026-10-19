#ifndef NEKO_MATH_SIXIT_FLOAT_H_
#define NEKO_MATH_SIXIT_FLOAT_H_

#include <bit>
#include <cstdint>
#include <limits>
#include <string_view>

#include "neko/math/func.h"
#include "neko/math/scalar_traits.h"
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wold-style-cast"
#pragma clang diagnostic ignored "-Wself-assign-overloaded"
#pragma clang diagnostic ignored "-Wdeprecated-literal-operator"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wold-style-cast"
#endif
#include "sixit/dmath/gamefloat/ieee_float_inline_asm.h"
#include "sixit/dmath/mathf/abs.h"
#include "sixit/dmath/mathf/asin.h"
#include "sixit/dmath/mathf/atan2.h"
#include "sixit/dmath/mathf/cos.h"
#include "sixit/dmath/mathf/floor.h"
#include "sixit/dmath/mathf/sin.h"
#include "sixit/dmath/mathf/sqrt.h"
#include "sixit/dmath/mathf/tan.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

namespace neko {

static_assert(
    sixit::dmath::fp_traits<sixit::dmath::ieee_float_inline_asm>::is_supported,
    "NEKO_SCALAR=SIXIT needs x86, x64 or RISC-V, or arm64 with GCC "
    "or clang (sixit::dmath::ieee_float_inline_asm)");

/// Square root for the sixit deterministic float.
///
/// Used when `NEKO_SCALAR=SIXIT`. Calls `sixit::dmath::mathf::sqrt`
/// instead of the `std` version, whose result could differ between
/// platforms.
template <>
inline sixit::dmath::ieee_float_inline_asm Sqrt(
    sixit::dmath::ieee_float_inline_asm value) {
  return sixit::dmath::mathf::sqrt(value);
}

/// Sine of an angle in radians for the sixit deterministic float.
///
/// Used when `NEKO_SCALAR=SIXIT`. Calls `sixit::dmath::mathf::sin`
/// instead of the `std` version, whose result could differ between
/// platforms.
template <>
inline sixit::dmath::ieee_float_inline_asm Sin(
    sixit::dmath::ieee_float_inline_asm value) {
  return sixit::dmath::mathf::sin(value);
}

/// Cosine of an angle in radians for the sixit deterministic float.
///
/// Used when `NEKO_SCALAR=SIXIT`. Calls `sixit::dmath::mathf::cos`
/// instead of the `std` version, whose result could differ between
/// platforms.
template <>
inline sixit::dmath::ieee_float_inline_asm Cos(
    sixit::dmath::ieee_float_inline_asm value) {
  return sixit::dmath::mathf::cos(value);
}

/// Tangent of an angle in radians for the sixit deterministic float.
///
/// Used when `NEKO_SCALAR=SIXIT`. Calls `sixit::dmath::mathf::tan`.
template <>
inline sixit::dmath::ieee_float_inline_asm Tan(
    sixit::dmath::ieee_float_inline_asm value) {
  return sixit::dmath::mathf::tan(value);
}

/// @ref ScalarTraits of the sixit deterministic float
/// (`sixit::dmath::ieee_float_inline_asm`, `NEKO_SCALAR=SIXIT`).
///
/// An IEEE 754 single precision float whose every operation is one explicit
/// instruction (`addss`, `mulss`, `fadd`...): the compiler can't fuse a
/// multiply and an add into an FMA, reorder operations or keep extra
/// precision, and the math functions are sixit's own (`mathf`, ported from
/// musl) instead of the platform's libm. Replays are bit for bit identical
/// on every supported CPU and compiler. In constant evaluation (a `constexpr`
/// constant) each operation is the same single IEEE operation done by the
/// compiler, so constants have the same bits as at run time.
template <>
struct ScalarTraits<sixit::dmath::ieee_float_inline_asm> {
  /// The number type these traits describe.
  using Type = sixit::dmath::ieee_float_inline_asm;
  /// The raw bits of a value: the IEEE 754 single precision pattern.
  using Bits = std::uint32_t;

  /// True: every operation is one IEEE instruction, rounded to nearest.
  static constexpr bool kIsDeterministic = true;
  /// False: the value is floating point.
  static constexpr bool kIsFixedPoint = false;
  /// True: x64, x86, arm64 and RISC-V give the same bits.
  static constexpr bool kIsCrossPlatformExact = true;
  /// Name shown in tools and readouts: `"sixit"`.
  static constexpr std::string_view kDisplayName = "sixit";

  /// Square root (`sixit::dmath::mathf::sqrt`).
  ///
  /// @param value The value, not negative.
  /// @return The square root.
  [[nodiscard]] static Type Sqrt(Type value) { return neko::Sqrt(value); }
  /// Sine of an angle in radians (`sixit::dmath::mathf::sin`).
  ///
  /// @param radians The angle.
  /// @return The sine.
  [[nodiscard]] static Type Sin(Type radians) { return neko::Sin(radians); }
  /// Cosine of an angle in radians (`sixit::dmath::mathf::cos`).
  ///
  /// @param radians The angle.
  /// @return The cosine.
  [[nodiscard]] static Type Cos(Type radians) { return neko::Cos(radians); }
  /// Arcsine in radians, in the range from -pi / 2 to pi / 2
  /// (`sixit::dmath::mathf::asin`).
  ///
  /// @param value The sine, clamped to the range from -1 to 1 first, so
  /// rounding just past 1 gives pi / 2 instead of NaN.
  /// @return The angle.
  [[nodiscard]] static Type Asin(Type value) {
    return sixit::dmath::mathf::asin(Clamp(value, Type{-1}, Type{1}));
  }
  /// Angle of the vector (`x`, `y`) in radians, in the range from -pi to pi
  /// (`sixit::dmath::mathf::atan2`).
  ///
  /// Like `std::atan2`, the sign of a zero counts: a negative `x` with `y`
  /// equal to -0 gives -pi.
  ///
  /// @param y The y component.
  /// @param x The x component.
  /// @return The angle; 0 for (+0, +0).
  [[nodiscard]] static Type Atan2(Type y, Type x) {
    return sixit::dmath::mathf::atan2(y, x);
  }
  /// Absolute value (clears the sign bit).
  ///
  /// @param value The value.
  /// @return `|value|`.
  [[nodiscard]] static Type Abs(Type value) {
    return sixit::dmath::mathf::abs(value);
  }
  /// Largest whole number not greater than @p value
  /// (`sixit::dmath::mathf::floor`).
  ///
  /// @param value The value.
  /// @return The rounded-down value.
  [[nodiscard]] static Type Floor(Type value) {
    return sixit::dmath::mathf::floor(value);
  }

  /// Raw bits of @p value, for checksums and exact tests.
  ///
  /// @param value The value.
  /// @return Its IEEE 754 bit pattern.
  [[nodiscard]] static constexpr Bits ToBits(Type value) noexcept {
    return std::bit_cast<Bits>(value.to_float());
  }
  /// Value with the raw bits @p bits, the inverse of @ref ToBits.
  ///
  /// @param bits An IEEE 754 bit pattern.
  /// @return The value.
  [[nodiscard]] static constexpr Type FromBits(Bits bits) noexcept {
    return Type{std::bit_cast<float>(bits)};
  }

  /// Gap between 1 and the next value, about 1.2e-7.
  ///
  /// @return The machine epsilon of single precision.
  [[nodiscard]] static constexpr Type Epsilon() noexcept {
    return Type{std::numeric_limits<float>::epsilon()};
  }
  /// Largest finite value, about 3.4e38.
  ///
  /// @return The largest value.
  [[nodiscard]] static constexpr Type Max() noexcept {
    return Type{std::numeric_limits<float>::max()};
  }
  /// Default tolerance to compare two results of a few operations on values
  /// around 1: 1e-5, as for `float`.
  ///
  /// @return The tolerance.
  [[nodiscard]] static constexpr Type Tolerance() noexcept {
    return Type{1e-5f};
  }
  /// Largest error of @ref Asin and @ref Atan2 against the exact angle of
  /// their inputs: 1e-6 rad, as for `float`.
  ///
  /// @return The tolerance.
  [[nodiscard]] static constexpr Type AngleTolerance() noexcept {
    return Type{1e-6f};
  }

  /// Converts the integer `value`, rounded to nearest.
  ///
  /// @param value The integer.
  /// @return The value.
  [[nodiscard]] static constexpr Type FromInt(int value) noexcept {
    return Type{value};
  }
  /// The ratio `num / den`: one IEEE division, rounded to nearest.
  ///
  /// @param num The numerator.
  /// @param den The denominator, not 0.
  /// @return `num / den`.
  [[nodiscard]] static constexpr Type FromRatio(int num, int den) noexcept {
    return Type{num} / Type{den};
  }
  /// Converts @p value to `float`, for display only.
  ///
  /// @param value The value.
  /// @return The same value as a `float`.
  [[nodiscard]] static constexpr float ToFloat(Type value) noexcept {
    return value.to_float();
  }
};
}  // namespace neko

#endif  // NEKO_MATH_SIXIT_FLOAT_H_
