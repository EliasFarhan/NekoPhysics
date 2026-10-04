#ifndef MAIN_SAMPLES_SAMPLE_CONFIG_H_
#define MAIN_SAMPLES_SAMPLE_CONFIG_H_

#include "neko/math/const.h"
#include "neko/math/shape.h"
#include "neko/physics/physics_type.h"

#include <cstddef>

namespace neko
{

// num / den computed in Scalar, so scenes are built without any float arithmetic (no soft-float on the DS)
constexpr Scalar Ratio(int num, int den)
{
    return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

// Per-platform sample sizes. The DS keeps every sample under its 128 OAM sprites and its 4 MB of RAM.
namespace sample_config
{
#if defined(NEKO_PLATFORM_NDS)
// 12.8 x 9.6 m: exactly 20 px/m on the 256x192 top screen
inline constexpr Aabbf kWorldBounds{ { Scalar{ 0 }, Scalar{ 0 } }, { Ratio(64, 5), Ratio(48, 5) } };
inline constexpr Index kKinematicCount = 120;
inline constexpr Index kPlanetCount = 120;
inline constexpr Index kTriggerCount = 120;
inline constexpr Index kCollisionCount = 50;
#else
// 12.8 x 7.2 m: 100 px/m in a 1280x720 window
inline constexpr Aabbf kWorldBounds{ { Scalar{ 0 }, Scalar{ 0 } }, { Ratio(64, 5), Ratio(36, 5) } };
inline constexpr Index kKinematicCount = 1'000;
inline constexpr Index kPlanetCount = 1'000;
inline constexpr Index kTriggerCount = 1'000;
inline constexpr Index kCollisionCount = 50;
#endif

// DrawList capacities reserved by the hosts: the largest sample, and the 4 lines of every quadtree node
// (depth 3: 1 + 4 + 16 + 64 = 85 nodes)
inline constexpr std::size_t kMaxDrawCircles = std::size_t{ kTriggerCount } > std::size_t{ kKinematicCount }
    ? std::size_t{ kTriggerCount } : std::size_t{ kKinematicCount };
inline constexpr std::size_t kMaxDrawBoxes = 8;
inline constexpr std::size_t kMaxDrawLines = 85 * 4;
}  // namespace sample_config

}  // namespace neko

#endif  // MAIN_SAMPLES_SAMPLE_CONFIG_H_
