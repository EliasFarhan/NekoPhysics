#ifndef NEKO_MATH_SCALAR_TRAITS_H_
#define NEKO_MATH_SCALAR_TRAITS_H_

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string_view>

#include "neko/math/fixed.h"
#include "neko/math/fixed_trig.h"
#include "neko/math/func.h"

namespace neko {

/// What a number type can do and how, in one place: the switch between
/// @ref Fixed16 and `float` for generic code.
///
/// The engine is written once against @ref Scalar, but a few things depend on
/// the number type: whether results replay bit for bit on another machine,
/// how to read the raw bits of a value (checksums, tests), which square root or
/// arcsine to call, and what "close enough" means in a test. Instead of
/// `#ifdef NEKO_SCALAR_FIXED16` blocks, generic code asks the traits:
///
/// @code
/// using Traits = neko::ScalarTraits<neko::Scalar>;
/// if constexpr (Traits::kIsFixedPoint) {
///   // integer-only path: raw bits, shifts
/// }
/// const neko::Scalar angle = Traits::Asin(Traits::FromRatio(1, 2));  // pi / 6
/// @endcode
///
/// Specialized for `float` and every @ref Fixed (the math functions `Sqrt`,
/// `Sin`, `Cos`, `Asin` and `Atan2` exist for @ref Fixed16 only). Using it with
/// another type does not compile, on purpose. The traits are modeled on sixit's
/// `fp_traits`; the sixit float (`NEKO_SCALAR=SIXIT`) has no specialization yet
/// (see `const.h`).
///
/// @tparam T The number type.
/// @see @ref Scalar
template <typename T>
struct ScalarTraits;

/// @ref ScalarTraits of `float`: the standard library, fast on PC, not
/// deterministic across machines.
///
/// `std::sin`, `std::asin` and friends may return different last bits with
/// another compiler, standard library, CPU or optimization flag, so a `float`
/// simulation can't promise bit-identical replays between two machines.
template <>
struct ScalarTraits<float> {
  /// The raw bits of a value: the IEEE 754 single precision pattern.
  using Bits = std::uint32_t;

  /// False: results can change with the compiler, its flags or the CPU.
  static constexpr bool kIsDeterministic = false;
  /// False: `float` is floating point.
  static constexpr bool kIsFixedPoint = false;
  /// False: two platforms may disagree on the last bits.
  static constexpr bool kIsCrossPlatformExact = false;
  /// Name shown in tools and readouts: `"float"`.
  static constexpr std::string_view kDisplayName = "float";

  /// Square root (`std::sqrt`).
  ///
  /// @param value The value, not negative.
  /// @return The square root.
  [[nodiscard]] static float Sqrt(float value) { return std::sqrt(value); }
  /// Sine of an angle in radians (`std::sin`).
  ///
  /// @param radians The angle.
  /// @return The sine.
  [[nodiscard]] static float Sin(float radians) { return std::sin(radians); }
  /// Cosine of an angle in radians (`std::cos`).
  ///
  /// @param radians The angle.
  /// @return The cosine.
  [[nodiscard]] static float Cos(float radians) { return std::cos(radians); }
  /// Arcsine in radians, in the range from -pi / 2 to pi / 2 (`std::asin`).
  ///
  /// @param value The sine, clamped to the range from -1 to 1 first, so
  /// rounding just past 1 gives pi / 2 instead of NaN.
  /// @return The angle.
  [[nodiscard]] static float Asin(float value) {
    return std::asin(Clamp(value, -1.0f, 1.0f));
  }
  /// Angle of the vector (@p x, @p y) in radians, in the range from -pi
  /// (excluded) to pi (`std::atan2`).
  ///
  /// @param y The y component.
  /// @param x The x component.
  /// @return The angle; 0 for the zero vector.
  [[nodiscard]] static float Atan2(float y, float x) {
    return std::atan2(y, x);
  }
  /// Absolute value.
  ///
  /// @param value The value.
  /// @return `|value|`.
  [[nodiscard]] static float Abs(float value) { return std::abs(value); }
  /// Largest whole number not greater than @p value (`std::floor`).
  ///
  /// @param value The value.
  /// @return The rounded-down value.
  [[nodiscard]] static float Floor(float value) { return std::floor(value); }

  /// Raw bits of @p value (`std::bit_cast`), for checksums and exact tests.
  ///
  /// @param value The value.
  /// @return Its IEEE 754 bit pattern.
  [[nodiscard]] static constexpr Bits ToBits(float value) noexcept {
    return std::bit_cast<Bits>(value);
  }
  /// Value with the raw bits @p bits, the inverse of @ref ToBits.
  ///
  /// @param bits An IEEE 754 bit pattern.
  /// @return The value.
  [[nodiscard]] static constexpr float FromBits(Bits bits) noexcept {
    return std::bit_cast<float>(bits);
  }

  /// Gap between 1 and the next value, about 1.2e-7.
  [[nodiscard]] static constexpr float Epsilon() noexcept {
    return std::numeric_limits<float>::epsilon();
  }
  /// Largest finite value, about 3.4e38.
  [[nodiscard]] static constexpr float Max() noexcept {
    return std::numeric_limits<float>::max();
  }
  /// Default tolerance to compare two results of a few operations on values
  /// around 1: 1e-5.
  [[nodiscard]] static constexpr float Tolerance() noexcept { return 1e-5f; }
  /// Largest error of @ref Asin and @ref Atan2 against the exact angle of
  /// their inputs, as tested: 1e-6 rad.
  [[nodiscard]] static constexpr float AngleTolerance() noexcept {
    return 1e-6f;
  }

  /// Converts the integer @p value.
  ///
  /// @param value The integer.
  /// @return The same value as a `float`.
  [[nodiscard]] static constexpr float FromInt(int value) noexcept {
    return static_cast<float>(value);
  }
  /// The ratio @p num / @p den, the deterministic way to write a fraction.
  ///
  /// @param num The numerator.
  /// @param den The denominator, not 0.
  /// @return `num / den`, rounded to nearest.
  [[nodiscard]] static constexpr float FromRatio(int num, int den) noexcept {
    return static_cast<float>(num) / static_cast<float>(den);
  }
  /// Converts @p value to `float`, for display only.
  ///
  /// @param value The value.
  /// @return @p value itself.
  [[nodiscard]] static constexpr float ToFloat(float value) noexcept {
    return value;
  }
};

/// @ref ScalarTraits of a @ref Fixed: integers only, the same bits on every
/// machine.
///
/// Every operation of @ref Fixed16, including @ref Sqrt(Fixed16), the table
/// sine and the inverse trigonometry here, uses integer arithmetic with
/// defined rounding, so a PC and a Nintendo DS replaying the same inputs reach
/// exactly the same state.
///
/// @tparam T The integer type that stores the value.
/// @tparam Exp The number of fractional bits.
/// @tparam MulT The wider integer type for products.
template <typename T, T Exp, typename MulT>
struct ScalarTraits<Fixed<T, Exp, MulT>> {
  /// The number type these traits describe.
  using Type = Fixed<T, Exp, MulT>;
  /// The raw bits of a value: the scaled integer
  /// (@ref Fixed::underlying_value).
  using Bits = T;

  /// True: every operation is integer arithmetic with defined rounding.
  static constexpr bool kIsDeterministic = true;
  /// True: the value is a scaled integer.
  static constexpr bool kIsFixedPoint = true;
  /// True: the PC and the DS compute the same bits.
  static constexpr bool kIsCrossPlatformExact = true;
  /// Name shown in tools and readouts: `"Fixed16"`, `"Fixed8"` or `"Fixed"`.
  static constexpr std::string_view kDisplayName =
      (Exp == 16 && sizeof(T) == 4)  ? "Fixed16"
      : (Exp == 6 && sizeof(T) == 1) ? "Fixed8"
                                     : "Fixed";

  /// Square root, exact (floor of the true root), see @ref Sqrt(Fixed16).
  ///
  /// @param value The value; negative values give 0.
  /// @return The square root.
  [[nodiscard]] static constexpr Type Sqrt(Type value) noexcept {
    return neko::Sqrt(value);
  }
  /// Sine of an angle in radians, from the 1024-entry table.
  ///
  /// @param radians The angle.
  /// @return The sine, error below 5e-5.
  [[nodiscard]] static constexpr Type Sin(Type radians) noexcept {
    return neko::Sin(radians);
  }
  /// Cosine of an angle in radians, from the same table as @ref Sin.
  ///
  /// @param radians The angle.
  /// @return The cosine, error below 5e-5.
  [[nodiscard]] static constexpr Type Cos(Type radians) noexcept {
    return neko::Cos(radians);
  }
  /// Arcsine in radians, in the range from -pi / 2 to pi / 2, with integers
  /// only.
  ///
  /// Computed as the angle of the vector `(sqrt(1 - x^2), x)`, with
  /// `1 - x^2` exact in 64 bits, a 257-entry arctangent table and one
  /// division. At most one step (1 / 65536 rad) from the exact arcsine of the
  /// 16.16 input, odd (`Asin(-x) == -Asin(x)`) and never decreasing.
  ///
  /// @param value The sine; values beyond 1 or -1 are clamped.
  /// @return The angle; exactly 0 for 0 and 102944 / 65536 (pi / 2 rounded)
  /// for 1.
  [[nodiscard]] static constexpr Type Asin(Type value) noexcept {
    return internal::AsinFixed16(value);
  }
  /// Angle of the vector (@p x, @p y) in radians, in the range from -pi
  /// (excluded) to pi, with integers only.
  ///
  /// The smaller component is divided by the larger one, the arctangent of
  /// that ratio (between 0 and 1) is read from a 257-entry table with linear
  /// interpolation, and the octant puts it back in place. At most one step
  /// from the exact angle of the 16.16 inputs.
  ///
  /// @param y The y component.
  /// @param x The x component.
  /// @return The angle; 0 for the zero vector, pi for a negative @p x with a
  /// zero `y`.
  [[nodiscard]] static constexpr Type Atan2(Type y, Type x) noexcept {
    return internal::Atan2Fixed16(y, x);
  }
  /// Absolute value.
  ///
  /// @param value The value.
  /// @return `|value|`.
  [[nodiscard]] static constexpr Type Abs(Type value) noexcept {
    return neko::Abs(value);
  }
  /// Largest whole number not greater than @p value: clears the fractional
  /// bits (two's complement rounds negative values down).
  ///
  /// @param value The value.
  /// @return The rounded-down value.
  [[nodiscard]] static constexpr Type Floor(Type value) noexcept {
    constexpr auto fraction_mask = static_cast<T>((T{1} << Exp) - 1);
    return Type::FromUnderlyingValue(
        static_cast<T>(value.underlying_value() & ~fraction_mask));
  }

  /// Raw bits of @p value: the scaled integer.
  ///
  /// @param value The value.
  /// @return @ref Fixed::underlying_value.
  [[nodiscard]] static constexpr Bits ToBits(Type value) noexcept {
    return value.underlying_value();
  }
  /// Value with the raw bits @p bits (@ref Fixed::FromUnderlyingValue).
  ///
  /// @param bits The scaled integer.
  /// @return The value.
  [[nodiscard]] static constexpr Type FromBits(Bits bits) noexcept {
    return Type::FromUnderlyingValue(bits);
  }

  /// One step, the smallest positive value (1 / 65536 for @ref Fixed16).
  [[nodiscard]] static constexpr Type Epsilon() noexcept {
    return Type::FromUnderlyingValue(T{1});
  }
  /// Largest value (about 32768 for @ref Fixed16).
  [[nodiscard]] static constexpr Type Max() noexcept {
    return Type::FromUnderlyingValue(std::numeric_limits<T>::max());
  }
  /// Default tolerance to compare two results of a few operations on values
  /// around 1: 4 steps.
  [[nodiscard]] static constexpr Type Tolerance() noexcept {
    return Type::FromUnderlyingValue(T{4});
  }
  /// Largest error of @ref Asin and @ref Atan2 against the exact angle of
  /// their inputs, as tested: 1 step.
  [[nodiscard]] static constexpr Type AngleTolerance() noexcept {
    return Type::FromUnderlyingValue(T{1});
  }

  /// Converts the integer @p value exactly.
  ///
  /// @param value The integer; it must fit in the integer part.
  /// @return The value.
  [[nodiscard]] static constexpr Type FromInt(int value) noexcept {
    return Type{value};
  }
  /// The ratio @p num / @p den, without floating point (truncated toward
  /// zero).
  ///
  /// @param num The numerator.
  /// @param den The denominator, not 0.
  /// @return `num / den`.
  [[nodiscard]] static constexpr Type FromRatio(int num, int den) noexcept {
    return Type{static_cast<T>(num), static_cast<T>(den)};
  }
  /// Converts @p value to `float`, for display only.
  ///
  /// @warning On the DS this links software floating point: keep it out of
  /// the simulation and of the DS build.
  /// @param value The value.
  /// @return The nearest `float`.
  [[nodiscard]] static constexpr float ToFloat(Type value) noexcept {
    return static_cast<float>(value);
  }
};

}  // namespace neko

#endif  // NEKO_MATH_SCALAR_TRAITS_H_
