#pragma once

#include <cstdint>

#if defined(NEKO_PLATFORM_NDS)
#include <nds/timers.h>
#endif

namespace neko
{

/**
 * @brief Phases of PhysicsWorld::Step tracked by the DS test ROM's per-phase breakdown.
 */
enum class StepPhase : std::uint8_t
{
    Integration,
    Broadphase,
    DetectContact,
    Resolve,
    SortAndNotify,
    Count
};

/**
 * @brief Accumulated ticks per StepPhase. Bus-clock ticks on the DS (see cpuStartTiming), meaningless elsewhere.
 */
struct StepPhaseTicks
{
    std::uint32_t ticks[static_cast<std::size_t>(StepPhase::Count)] = {};
};

#if defined(NEKO_PLATFORM_NDS)
/**
 * @brief Set by the DS test ROM before a profiling run; nullptr (the default) means "don't collect", so Step()
 * pays only a null check on every platform that isn't actively measuring.
 */
inline StepPhaseTicks* g_stepPhaseTicks = nullptr;

[[nodiscard]] inline std::uint32_t StepProfileNow() noexcept
{
    return g_stepPhaseTicks != nullptr ? cpuGetTiming() : 0;
}

inline void StepProfileMark(StepPhase phase, std::uint32_t since) noexcept
{
    if (g_stepPhaseTicks != nullptr)
    {
        g_stepPhaseTicks->ticks[static_cast<std::size_t>(phase)] += cpuGetTiming() - since;
    }
}
#else
[[nodiscard]] constexpr std::uint32_t StepProfileNow() noexcept { return 0; }
constexpr void StepProfileMark(StepPhase, std::uint32_t) noexcept {}
#endif

} // namespace neko
