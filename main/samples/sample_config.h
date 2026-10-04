#ifndef MAIN_SAMPLES_SAMPLE_CONFIG_H_
#define MAIN_SAMPLES_SAMPLE_CONFIG_H_

#include "neko/math/const.h"
#include "neko/math/shape.h"
#include "neko/physics/physics_type.h"

#include <cstddef>

namespace neko
{

/**
 * @brief num / den computed in Scalar, so scenes are built without any float arithmetic (no soft-float on the DS)
 */
constexpr Scalar Ratio(int num, int den)
{
    return static_cast<Scalar>(num) / static_cast<Scalar>(den);
}

/**
 * @brief Per-platform sample sizes. The DS keeps every sample under its 128 OAM sprites and its 4 MB of RAM.
 */
namespace sample_config
{
#if defined(NEKO_PLATFORM_NDS)
/**
 * @brief 12.8 x 9.6 m: exactly 20 px/m on the 256x192 top screen
 */
inline constexpr Aabbf WORLD_BOUNDS{ { Scalar{ 0 }, Scalar{ 0 } }, { Ratio(64, 5), Ratio(48, 5) } };
inline constexpr Index KINEMATIC_COUNT = 120;
inline constexpr Index PLANET_COUNT = 120;
inline constexpr Index TRIGGER_COUNT = 120;
inline constexpr Index COLLISION_COUNT = 50;
#else
/**
 * @brief 12.8 x 7.2 m: 100 px/m in a 1280x720 window
 */
inline constexpr Aabbf WORLD_BOUNDS{ { Scalar{ 0 }, Scalar{ 0 } }, { Ratio(64, 5), Ratio(36, 5) } };
inline constexpr Index KINEMATIC_COUNT = 1'000;
inline constexpr Index PLANET_COUNT = 1'000;
inline constexpr Index TRIGGER_COUNT = 1'000;
inline constexpr Index COLLISION_COUNT = 50;
#endif

/**
 * @brief DrawList capacities reserved by the hosts: the largest sample, and the 4 lines of every quadtree node
 * (depth 3: 1 + 4 + 16 + 64 = 85 nodes)
 */
inline constexpr std::size_t MAX_DRAW_CIRCLES = std::size_t{ TRIGGER_COUNT } > std::size_t{ KINEMATIC_COUNT }
    ? std::size_t{ TRIGGER_COUNT } : std::size_t{ KINEMATIC_COUNT };
inline constexpr std::size_t MAX_DRAW_BOXES = 8;
inline constexpr std::size_t MAX_DRAW_LINES = 85 * 4;
} // namespace sample_config

} // namespace neko

#endif  // MAIN_SAMPLES_SAMPLE_CONFIG_H_
