#ifndef NEKO_CORE_PROFILE_H_
#define NEKO_CORE_PROFILE_H_

// Profiling zone covering the enclosing function: Tracy on PC when
// ENABLE_PROFILING is ON, nothing otherwise. The DS test ROM times Step with
// the hardware timers instead (see neko/core/step_profile.h).
//
// Put NEKO_PROFILE_FUNCTION(); as the first statement of a function; the
// Tracy profiler then shows how long each call took. MrDocs does not extract
// macros, so this stays a plain comment.
#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#define NEKO_PROFILE_FUNCTION() ZoneScoped
#else
#define NEKO_PROFILE_FUNCTION() static_cast<void>(0)
#endif

#endif  // NEKO_CORE_PROFILE_H_
