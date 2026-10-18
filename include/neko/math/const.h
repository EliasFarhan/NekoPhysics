#ifndef NEKO_MATH_CONST_H_
#define NEKO_MATH_CONST_H_

#include "neko/math/fixed.h"
#ifdef NEKO_SCALAR_SIXIT
#include "neko/math/sixit_float.h"
#endif
#include <cmath>
#include <numbers>

namespace neko {

#if defined(NEKO_SCALAR_SIXIT)
/// The number type used by the whole physics engine (`NEKO_SCALAR=SIXIT`).
// TODO(S1.8): ScalarTraits<sixit::dmath::ieee_float_inline_asm> goes in
// neko/math/scalar_traits.h (deterministic, cross-platform exact on x86, sixit
// mathf functions), once the SIXIT build is repaired. Needed by Asin and Atan2
// (neko/math/angle.h), DivideBy (src/physics.cc), the tests and the samples
// that ask ScalarTraits<Scalar>; Vec2 falls back to the float path without it.
using Scalar = sixit::dmath::ieee_float_inline_asm;
#elif defined(NEKO_SCALAR_FLOAT)
/// The number type used by the whole physics engine (`NEKO_SCALAR=FLOAT`).
using Scalar = float;
#else
/// The number type used by the whole physics engine.
///
/// Chosen at configure time with the CMake cache variable `NEKO_SCALAR`:
/// - `FIXED16` (default): @ref Fixed16, 16.16 fixed point. Deterministic
///   on every platform and fast on the Nintendo DS, which has no FPU.
/// - `FLOAT`: plain `float`, convenient for debugging on PC, but not
///   guaranteed to give the same bits on different machines or compilers,
///   so not suitable for rollback netcode across platforms.
/// - `SIXIT`: `sixit::dmath::ieee_float_inline_asm`, a deterministic
///   software-controlled float (x86 only).
///
/// Write game code against `Scalar`, and build constants from integers
/// (`Scalar{3}`, `Scalar{1} / Scalar{4}`) rather than float literals, so it
/// compiles and stays deterministic with every choice.
using Scalar = Fixed16;
#endif

/// The constant pi (3.14159...) in the number type `T`.
///
/// @code
/// const neko::Scalar half_turn = neko::Pi<neko::Scalar>();
/// @endcode
///
/// @tparam T The number type.
/// @return pi, rounded to the precision of `T`.
template <typename T>
constexpr T Pi() {
#ifdef NEKO_SCALAR_SIXIT
  if constexpr (std::is_same_v<T, sixit::dmath::ieee_float_inline_asm>) {
    return sixit::dmath::ieee_float_inline_asm(std::numbers::pi_v<float>);
  } else
#endif
  {
    return std::numbers::pi_v<T>;
  }
}

/// Euler's number e (2.71828...) in the number type `T`.
///
/// @tparam T The number type.
/// @return e, rounded to the precision of `T`.
template <typename T>
constexpr T Exp() {
#ifdef NEKO_SCALAR_SIXIT
  if constexpr (std::is_same_v<T, sixit::dmath::ieee_float_inline_asm>) {
    return sixit::dmath::ieee_float_inline_asm(std::numbers::e_v<float>);
  } else
#endif
  {
    return std::numbers::e_v<T>;
  }
}

/// Pi in 16.16 fixed point (@ref Fixed16, the default @ref Fixed), as the
/// exact raw value `205887`.
///
/// Written as a raw integer instead of a conversion from a floating point
/// constant, so it is identical on every platform.
///
/// @return Pi, about 3.14159.
template <>
constexpr Fixed16 Pi() {
  return Fixed16::FromUnderlyingValue(205887);
}

/// e in 16.16 fixed point (@ref Fixed16, the default @ref Fixed), as the
/// exact raw value `178145`.
///
/// @return e, about 2.71828.
template <>
constexpr Fixed16 Exp() {
  return Fixed16::FromUnderlyingValue(178145);
}

}  // namespace neko

#endif  // NEKO_MATH_CONST_H_
