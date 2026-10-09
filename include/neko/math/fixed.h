#ifndef NEKO_MATH_FIXED_H_
#define NEKO_MATH_FIXED_H_

#include <compare>
#include <cstdint>
#include <limits>
#include <type_traits>

#if defined(NEKO_PLATFORM_NDS)
#include <nds/arm9/math.h>
#endif

namespace neko {

namespace internal {
// Floor of the square root of a 64-bit integer, computed bit by bit.
// Exact, so it matches the DS hardware square root unit (REG_SQRTCNT, 64-bit
// mode).
[[nodiscard]] constexpr std::uint32_t ISqrt64Portable(
    std::uint64_t value) noexcept {
  std::uint64_t result = 0;
  std::uint64_t bit = std::uint64_t{1} << 62;
  while (bit > value) {
    bit >>= 2;
  }
  while (bit != 0) {
    if (value >= result + bit) {
      value -= result + bit;
      result = (result >> 1) + bit;
    } else {
      result >>= 1;
    }
    bit >>= 2;
  }
  return static_cast<std::uint32_t>(result);
}

// 64/32 signed division, truncated toward zero, keeping the low 32 bits of the
// quotient. den must not be 0.
[[nodiscard]] constexpr std::int32_t Div64Portable(std::int64_t num,
                                                   std::int32_t den) noexcept {
  return static_cast<std::int32_t>(num / den);
}

// DS hardware math hooks: the divider and the square root unit are shared
// memory-mapped registers, so the physics must not run inside IRQ handlers (an
// interrupted operation would return the handler's result). Constant evaluation
// keeps the portable path, and both paths give bit-exact results.

// Floor of the square root of a 64-bit integer (REG_SQRTCNT 64-bit mode on the
// DS).
[[nodiscard]] constexpr std::uint32_t ISqrt64(std::uint64_t value) noexcept {
#if defined(NEKO_PLATFORM_NDS)
  // From 2^62 up (roots >= 2^31), melonDS 1.1 returns the root + 1; not checked
  // on hardware, so these inputs (only vector lengths >= 32768) stay on the
  // portable path
  if !consteval {
    if (value < (std::uint64_t{1} << 62)) {
      // libnds' sqrt64 takes a long long
      return sqrt64(
          static_cast<long long>(value));  // NOLINT(google-runtime-int)
    }
  }
#endif
  return ISqrt64Portable(value);
}

// Same as Div64Portable (REG_DIVCNT 64/32 mode on the DS). den must not be 0.
[[nodiscard]] constexpr std::int32_t Div64(std::int64_t num,
                                           std::int32_t den) noexcept {
#if defined(NEKO_PLATFORM_NDS)
  if !consteval {
    return div64(num, den);
  }
#endif
  return Div64Portable(num, den);
}
}  // namespace internal

/// A fixed-point number: a real number stored as a scaled integer.
///
/// A fixed-point value is an integer `underlying_value()` that implicitly
/// means `underlying_value() / 2^Exp`. With the default `Fixed16` (16.16),
/// the top 16 bits hold the integer part and the low 16 bits the fraction,
/// so `1.0` is stored as `65536` and the smallest step is `1 / 65536`.
///
/// Why not `float`? Two reasons that matter for this library:
/// - The Nintendo DS CPU (ARM946E-S) has no FPU. Every `float` operation
///   is emulated in software and is very slow; integer math is native.
/// - Determinism. Rollback netcode re-simulates past frames and expects
///   every machine to compute exactly the same bits. Floating point results
///   can change with the compiler, its flags or the CPU, while integer
///   arithmetic is the same everywhere. `Fixed` defines its rounding
///   precisely so the PC build and the DS build agree bit for bit.
///
/// Rounding rules, which the DS port must reproduce bit for bit:
/// - multiplication rounds to nearest, ties away from zero;
/// - division truncates toward zero (like the DS divider) and saturates
///   when dividing by zero;
/// - conversion from floating point rounds to nearest, conversion to
///   integer truncates toward zero.
///
/// Overflow is not checked (it behaves like the underlying integer), so keep
/// values well inside the range: for `Fixed16` about -32768 to 32767.99998.
///
/// @code
/// using neko::Fixed16;
/// constexpr Fixed16 half{0.5f};      // from float, at compile time
/// constexpr Fixed16 three{3};        // from int
/// constexpr Fixed16 third{1, 3};     // the ratio 1 / 3
/// Fixed16 x = three * half;          // 1.5
/// x += third;
/// float shown = static_cast<float>(x);  // for display only
/// @endcode
///
/// @tparam T Signed integer type that stores the value.
/// @tparam Exp Number of fractional bits.
/// @tparam MulT Wider integer type used for the intermediate results of
/// multiplication and division, so they do not overflow.
/// @note Prefer the alias `neko::Scalar` in game code: it is `Fixed16` by
/// default and `float` when built with `NEKO_SCALAR=FLOAT`.
/// @see neko::Fixed16
/// @see neko::Scalar
template <typename T = std::int32_t, T Exp = 16, typename MulT = std::int64_t>
class Fixed {
 public:
  /// Constructs zero.
  constexpr Fixed() = default;
  /// Copy constructor.
  constexpr Fixed(const Fixed&) = default;
  /// Move constructor.
  constexpr Fixed(Fixed&&) = default;
  /// Copy assignment, on lvalues only.
  ///
  /// Assigning to a temporary (`v.x() = ...`, `v.x() += ...`) does not
  /// compile, which catches code that would silently modify a copy. Use the
  /// setters of the owning type instead.
  constexpr Fixed& operator=(const Fixed&) & = default;
  /// Move assignment, on lvalues only.
  constexpr Fixed& operator=(Fixed&&) & = default;

  /// Converts a floating point value, rounding to the nearest step.
  ///
  /// Meant for constants (`Fixed16{0.5f}`), ideally evaluated at compile
  /// time. At run time on the DS it would pull in software floating point,
  /// which the DS build forbids.
  ///
  /// @param f The value to convert.
  template <class OtherT>
    requires std::is_floating_point_v<OtherT>
  constexpr explicit Fixed(OtherT f) {
    const OtherT scaled = f * static_cast<OtherT>(kOne);
    underlying_value_ = static_cast<T>(
        scaled >= OtherT{0} ? scaled + OtherT{0.5} : scaled - OtherT{0.5});
  }

  /// Converts an integer exactly (`Fixed16{3}` is 3.0).
  ///
  /// @param n The value to convert. It must fit in the integer part.
  template <class OtherT>
    requires std::is_integral_v<OtherT>
  constexpr explicit Fixed(OtherT n)
      : underlying_value_(static_cast<T>(static_cast<T>(n) * kOne)) {}

  /// Constructs the ratio @p up / @p down without any floating point.
  ///
  /// This is the deterministic way to write fractional constants:
  /// `Fixed16{3, 2}` is 1.5. The division truncates toward zero.
  ///
  /// @param up Numerator.
  /// @param down Denominator.
  constexpr explicit Fixed(T up, T down)
      : underlying_value_((Fixed{up} / Fixed{down}).underlying_value_) {}

  /// Converts from a fixed-point type with another format.
  ///
  /// Gaining fractional bits is exact; losing them truncates toward zero.
  ///
  /// @param other The value to convert.
  template <typename OtherT, OtherT OtherExp, typename OtherMulT>
  constexpr explicit Fixed(Fixed<OtherT, OtherExp, OtherMulT> other) {
    const auto other_value = static_cast<MulT>(other.underlying_value());
    if constexpr (Exp >= OtherExp) {
      underlying_value_ =
          static_cast<T>(other_value * (MulT{1} << (Exp - OtherExp)));
    } else {
      underlying_value_ =
          static_cast<T>(other_value / (MulT{1} << (OtherExp - Exp)));
    }
  }

  /// Converts to `float`, for display or debugging only.
  ///
  /// @warning Never feed the result back into the simulation: it breaks
  /// determinism, and on the DS it links software floating point.
  [[nodiscard]] explicit constexpr operator float() const noexcept {
    return static_cast<float>(underlying_value_) / static_cast<float>(kOne);
  }

  /// Converts to the integer type `T`, truncating toward zero.
  [[nodiscard]] explicit constexpr operator T() const noexcept {
    return static_cast<T>(underlying_value_ / kOne);
  }

  /// Adds two values (exact, overflow is not checked).
  [[nodiscard]] constexpr Fixed operator+(Fixed other) const noexcept {
    return FromUnderlyingValue(
        static_cast<T>(underlying_value_ + other.underlying_value_));
  }

  /// Adds @p other to this value.
  constexpr Fixed& operator+=(Fixed other) & noexcept {
    return *this = *this + other;
  }

  /// Subtracts two values (exact, overflow is not checked).
  [[nodiscard]] constexpr Fixed operator-(Fixed other) const noexcept {
    return FromUnderlyingValue(
        static_cast<T>(underlying_value_ - other.underlying_value_));
  }

  /// Subtracts @p other from this value.
  constexpr Fixed& operator-=(Fixed other) & noexcept {
    return *this = *this - other;
  }

  /// Returns the opposite value.
  [[nodiscard]] constexpr Fixed operator-() const noexcept {
    return FromUnderlyingValue(static_cast<T>(-underlying_value_));
  }

  /// Multiplies two values, rounding to nearest (ties away from zero).
  ///
  /// The raw product has `2 * Exp` fractional bits, so it is computed in the
  /// wider `MulT` and shifted back by `Exp` bits.
  ///
  /// @param other The right-hand operand.
  /// @return The rounded product.
  [[nodiscard]] constexpr Fixed operator*(Fixed other) const noexcept {
    const MulT product = static_cast<MulT>(underlying_value_) *
                         static_cast<MulT>(other.underlying_value_);
    // + half, minus one ulp of the product when negative, then floor: rounds
    // half away from zero
    constexpr MulT half = MulT{1} << (Exp - 1);
    const MulT negative = product < 0 ? MulT{1} : MulT{0};
    return FromUnderlyingValue(
        static_cast<T>((product + half - negative) >> Exp));
  }

  /// Multiplies this value by @p other.
  constexpr Fixed& operator*=(Fixed other) & noexcept {
    return *this = *this * other;
  }

  /// Divides two values, truncating toward zero.
  ///
  /// For `Fixed16` this uses the DS hardware divider on the DS and the same
  /// integer division elsewhere, with bit-identical results. Dividing by zero
  /// does not crash: it saturates to the largest (or lowest) value with the
  /// sign of the numerator, and `0 / 0` is 0.
  ///
  /// @warning On the DS the divider is a shared hardware register, so do not
  /// run physics code inside interrupt handlers.
  /// @param other The divisor.
  /// @return The truncated quotient.
  [[nodiscard]] constexpr Fixed operator/(Fixed other) const noexcept {
    if (other.underlying_value_ == 0) {
      if (underlying_value_ == 0) {
        return Fixed{};
      }
      return FromUnderlyingValue(underlying_value_ > 0
                                     ? std::numeric_limits<T>::max()
                                     : std::numeric_limits<T>::min());
    }
    const MulT numerator =
        static_cast<MulT>(underlying_value_) * static_cast<MulT>(kOne);
    if constexpr (std::is_same_v<T, std::int32_t> &&
                  std::is_same_v<MulT, std::int64_t>) {
      // Hardware divider on the DS
      return FromUnderlyingValue(
          internal::Div64(numerator, other.underlying_value_));
    } else {
      return FromUnderlyingValue(static_cast<T>(
          numerator / static_cast<MulT>(other.underlying_value_)));
    }
  }

  /// Divides this value by @p other.
  constexpr Fixed& operator/=(Fixed other) & noexcept {
    return *this = *this / other;
  }

  /// Exact equality (compares the underlying integers).
  [[nodiscard]] constexpr bool operator==(const Fixed& other) const noexcept =
      default;
  /// Three-way comparison, giving `<`, `<=`, `>` and `>=`.
  [[nodiscard]] constexpr auto operator<=>(const Fixed& other) const noexcept =
      default;

  /// Builds a value from its raw scaled integer.
  ///
  /// `Fixed16::FromUnderlyingValue(65536)` is 1.0 and
  /// `Fixed16::FromUnderlyingValue(1)` is the smallest positive step. Useful
  /// for exact constants and for serialization.
  ///
  /// @param value The raw integer, equal to the real value times `2^Exp`.
  /// @return The fixed-point value.
  [[nodiscard]] static constexpr Fixed FromUnderlyingValue(T value) noexcept {
    Fixed f;
    f.underlying_value_ = value;
    return f;
  }
  /// Returns the raw scaled integer (the real value times `2^Exp`).
  [[nodiscard]] constexpr T underlying_value() const noexcept {
    return underlying_value_;
  }

 private:
  static constexpr T kOne = T{1} << Exp;
  T underlying_value_ = 0;
};

/// 16.16 fixed point: 32-bit storage, 16 fractional bits.
///
/// The default `neko::Scalar` and the number type used on the Nintendo DS.
/// Range about -32768 to 32767.99998, precision `1 / 65536` (about 1.5e-5).
using Fixed16 = Fixed<std::int32_t, 16, std::int64_t>;
/// Tiny 8-bit fixed point (2.6 format): range about -2 to 1.98, step 1/64.
using Fixed8 = Fixed<std::int8_t, 6, std::int16_t>;

// Named like a standard type trait (std::is_floating_point_v)
// NOLINTBEGIN(readability-identifier-naming)
/// True when `T` is a `neko::Fixed` type, false otherwise.
///
/// Lets generic code pick a fixed-point specific path with
/// `if constexpr (neko::is_fixed_v<T>)`, like `std::is_floating_point_v`.
///
/// @tparam T The type to test.
template <typename T>
inline constexpr bool is_fixed_v = false;

/// Specialization of `is_fixed_v` that is true for every `neko::Fixed`.
template <typename T, T Exp, typename MulT>
inline constexpr bool is_fixed_v<Fixed<T, Exp, MulT>> = true;
// NOLINTEND(readability-identifier-naming)

/// Square root of a `Fixed16`, exact and deterministic.
///
/// Computes the floor of the true square root at 16.16 precision using only
/// integer math: the DS hardware square root unit on the DS, and an
/// equivalent bit-by-bit algorithm elsewhere (also usable at compile time).
/// Both give the same bits.
///
/// @param value The value. Negative values (and zero) return 0.
/// @return The square root, rounded down to the nearest step.
/// @warning On the DS the square root unit is a shared hardware register, so
/// do not run physics code inside interrupt handlers.
[[nodiscard]] constexpr Fixed16 Sqrt(Fixed16 value) noexcept {
  if (value.underlying_value() <= 0) {
    return Fixed16{};
  }
  // sqrt(raw / 2^16) * 2^16 == sqrt(raw * 2^16)
  const auto squared = static_cast<std::uint64_t>(value.underlying_value())
                       << 16;
  return Fixed16::FromUnderlyingValue(
      static_cast<std::int32_t>(internal::ISqrt64(squared)));
}

}  // namespace neko

// Standard library interface names
// NOLINTBEGIN(readability-identifier-naming)
template <typename T, T Exp, typename MulT>
class std::numeric_limits<neko::Fixed<T, Exp, MulT>> {
  using Fixed = neko::Fixed<T, Exp, MulT>;

 public:
  static constexpr bool is_specialized = true;
  static constexpr bool is_signed = std::numeric_limits<T>::is_signed;
  static constexpr bool is_integer = false;
  static constexpr bool is_exact = true;
  static constexpr bool has_infinity = false;
  static constexpr bool has_quiet_NaN = false;
  static constexpr bool has_signaling_NaN = false;
  static constexpr bool is_bounded = true;
  static constexpr bool is_modulo = false;
  static constexpr int radix = 2;
  static constexpr int digits = std::numeric_limits<T>::digits;
  static constexpr std::float_round_style round_style = std::round_to_nearest;

  [[nodiscard]] static constexpr Fixed min() noexcept {
    return Fixed::FromUnderlyingValue(T{1});
  }
  [[nodiscard]] static constexpr Fixed max() noexcept {
    return Fixed::FromUnderlyingValue(std::numeric_limits<T>::max());
  }
  [[nodiscard]] static constexpr Fixed lowest() noexcept {
    return Fixed::FromUnderlyingValue(std::numeric_limits<T>::min());
  }
  [[nodiscard]] static constexpr Fixed epsilon() noexcept {
    return Fixed::FromUnderlyingValue(T{1});
  }
  [[nodiscard]] static constexpr Fixed round_error() noexcept {
    return Fixed{0.5f};
  }
};
// NOLINTEND(readability-identifier-naming)

#endif  // NEKO_MATH_FIXED_H_
