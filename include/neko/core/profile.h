#pragma once

/**
 * @brief Profiling zone covering the enclosing function: Tracy on PC when ENABLE_PROFILING is ON, nothing otherwise.
 * The DS test ROM times Step with the hardware timers instead.
 */
#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#define NEKO_PROFILE_FUNCTION() ZoneScoped
#else
#define NEKO_PROFILE_FUNCTION() static_cast<void>(0)
#endif
