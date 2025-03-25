#pragma once

#include "math/func.h"
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wold-style-cast"
#pragma clang diagnostic ignored "-Wself-assign-overloaded"
#endif
#include "sixit/dmath/mathf/floor.h"
#include "sixit/dmath/gamefloat/ieee_float_inline_asm.h"
#include "sixit/dmath/mathf/sqrt.h"
#include "sixit/dmath/mathf/sin.h"
#include "sixit/dmath/mathf/cos.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#endif
#include <cmath>

namespace neko
{
template<>
inline sixit::dmath::ieee_float_inline_asm Sqrt(sixit::dmath::ieee_float_inline_asm value)
{
	return sixit::dmath::mathf::sqrt(value);
}

template<>
inline sixit::dmath::ieee_float_inline_asm Sin(sixit::dmath::ieee_float_inline_asm value)
{
	return sixit::dmath::mathf::sin(value);
}

template<>
inline sixit::dmath::ieee_float_inline_asm Cos(sixit::dmath::ieee_float_inline_asm value)
{
	return sixit::dmath::mathf::cos(value);
}
}