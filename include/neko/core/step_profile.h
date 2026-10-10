#ifndef NEKO_CORE_STEP_PROFILE_H_
#define NEKO_CORE_STEP_PROFILE_H_

#include <cstdint>

#if defined(NEKO_PLATFORM_NDS)
#include <nds/timers.h>
#endif

namespace neko {

/// Phases of @ref PhysicsWorld::Step tracked by the DS test ROM's per-phase
/// breakdown.
///
/// On the DS there is no Tracy: to find out where the time of a step goes,
/// @ref PhysicsWorld::Step measures each phase with the hardware timers and
/// adds the result to the matching slot of @ref StepPhaseTicks.
enum class StepPhase : std::uint8_t {
  kIntegration,    ///< Applying forces and moving the bodies.
  kBroadphase,     ///< Building the broadphase and finding possible pairs.
  kDetectContact,  ///< Exact shape tests of the narrowphase.
  kResolve,        ///< Separating and bouncing the touching bodies.
  kSortAndNotify,  ///< Sorting touching pairs and calling the listener.
  kCount           ///< Number of phases, not a phase.
};

/// Accumulated ticks per @ref StepPhase.
///
/// Bus-clock ticks on the DS (33.51 MHz, see libnds' `cpuStartTiming`),
/// always zero elsewhere.
struct StepPhaseTicks {
  /// Ticks spent in each phase, indexed by `static_cast<std::size_t>(phase)`.
  std::uint32_t ticks[static_cast<std::size_t>(StepPhase::kCount)] = {};
};

#if defined(NEKO_PLATFORM_NDS)
/// Where @ref PhysicsWorld::Step accumulates its phase timings (DS only).
///
/// Set by the DS test ROM before a profiling run; nullptr (the default) means
/// "don't collect", so Step() pays only a null check on every platform that
/// isn't actively measuring.
inline StepPhaseTicks* g_step_phase_ticks = nullptr;

/// Returns the current hardware timer value, or 0 when not collecting.
///
/// @pre `cpuStartTiming` was called when @ref g_step_phase_ticks is set.
[[nodiscard]] inline std::uint32_t StepProfileNow() noexcept {
  return g_step_phase_ticks != nullptr ? cpuGetTiming() : 0;
}

/// Adds the ticks elapsed since @p since to `phase`, when collecting.
///
/// @param phase Phase that just ended.
/// @param since Value returned by @ref StepProfileNow at its start.
inline void StepProfileMark(StepPhase phase, std::uint32_t since) noexcept {
  if (g_step_phase_ticks != nullptr) {
    g_step_phase_ticks->ticks[static_cast<std::size_t>(phase)] +=
        cpuGetTiming() - since;
  }
}
#else
/// Returns the start time of a phase; always 0 outside the DS.
///
/// Together with @ref StepProfileMark it brackets each phase of
/// @ref PhysicsWorld::Step. Off the DS both are empty `constexpr` functions,
/// so the compiler removes them.
///
/// @code
/// const auto start = neko::StepProfileNow();
/// ResolveBroadphase();
/// neko::StepProfileMark(neko::StepPhase::kBroadphase, start);
/// @endcode
[[nodiscard]] constexpr std::uint32_t StepProfileNow() noexcept { return 0; }
/// Records the end of a phase; does nothing outside the DS.
///
/// On the DS it adds the elapsed hardware ticks to the phase's slot of
/// @ref StepPhaseTicks.
constexpr void StepProfileMark(StepPhase, std::uint32_t) noexcept {}
#endif

}  // namespace neko

#endif  // NEKO_CORE_STEP_PROFILE_H_
