#ifndef MAIN_SAMPLES_SAMPLE_CONFIG_H_
#define MAIN_SAMPLES_SAMPLE_CONFIG_H_

#include <cstddef>

#include "neko/math/const.h"
#include "neko/math/shape.h"
#include "neko/physics/physics_type.h"

namespace neko {

/// Returns @p num / @p den computed in `Scalar`.
///
/// Write sample constants with it instead of float literals: `Ratio(981, 100)`
/// rather than `9.81f`. The division happens in `Scalar` (16.16 fixed point on
/// the DS), so scenes are built without any float arithmetic: the DS has no
/// FPU and the samples ROM must not link soft-float, and the result is the
/// same on every platform.
///
/// @param num Numerator.
/// @param den Denominator, not zero.
/// @return The ratio, rounded as `Scalar` division rounds.
constexpr Scalar Ratio(int num, int den) {
  return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

/// Per-platform sample sizes: world bounds, body counts and draw capacities.
///
/// The values depend on `NEKO_PLATFORM_NDS`: the DS keeps every sample under
/// its 128 OAM sprites and its 4 MB of RAM, the PC shows bigger scenes. The
/// documentation shows the PC values. A sample reads its sizes from here
/// rather than hard-coding them, so the same code runs on both platforms.
namespace sample_config {
#if defined(NEKO_PLATFORM_NDS)
/// World rectangle shown by the hosts, in meters.
///
/// 12.8 x 9.6 m on the DS: exactly 20 px/m on the 256x192 top screen.
inline constexpr Aabbf kWorldBounds{{Scalar{0}, Scalar{0}},
                                    {Ratio(64, 5), Ratio(48, 5)}};
/// Number of circles in @ref KinematicSample.
inline constexpr Index kKinematicCount = 120;
/// Number of planets in @ref PlanetSample.
inline constexpr Index kPlanetCount = 120;
/// Number of trigger circles in @ref TriggersSample.
inline constexpr Index kTriggerCount = 120;
/// Number of colliding circles in @ref CollisionsSample.
inline constexpr Index kCollisionCount = 50;
#else
/// World rectangle shown by the hosts, in meters.
///
/// 12.8 x 7.2 m on PC: 100 px/m in a 1280x720 window. 12.8 x 9.6 m on the DS:
/// exactly 20 px/m on the 256x192 top screen.
inline constexpr Aabbf kWorldBounds{{Scalar{0}, Scalar{0}},
                                    {Ratio(64, 5), Ratio(36, 5)}};
/// Number of circles in @ref KinematicSample (120 on the DS).
inline constexpr Index kKinematicCount = 1'000;
/// Number of planets in @ref PlanetSample (120 on the DS).
inline constexpr Index kPlanetCount = 1'000;
/// Number of trigger circles in @ref TriggersSample (120 on the DS).
inline constexpr Index kTriggerCount = 1'000;
/// Number of colliding circles in @ref CollisionsSample.
inline constexpr Index kCollisionCount = 50;
#endif

/// @ref DrawList circle capacity reserved by the hosts: the largest sample.
inline constexpr std::size_t kMaxDrawCircles =
    std::size_t{kTriggerCount} > std::size_t{kKinematicCount}
        ? std::size_t{kTriggerCount}
        : std::size_t{kKinematicCount};
/// @ref DrawList box capacity reserved by the hosts.
inline constexpr std::size_t kMaxDrawBoxes = 8;
/// @ref DrawList line capacity reserved by the hosts.
///
/// Room for the 4 lines of every quadtree node drawn by @ref TriggersSample
/// (depth 3: 1 + 4 + 16 + 64 = 85 nodes).
inline constexpr std::size_t kMaxDrawLines = 85 * 4;
}  // namespace sample_config

}  // namespace neko

#endif  // MAIN_SAMPLES_SAMPLE_CONFIG_H_
