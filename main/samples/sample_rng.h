#ifndef MAIN_SAMPLES_SAMPLE_RNG_H_
#define MAIN_SAMPLES_SAMPLE_RNG_H_

#include "main/samples/draw_list.h"
#include "main/samples/sample_config.h"

#include <cstdint>

namespace neko
{

/**
 * @brief Seeded xorshift32 for the sample scenes. Scalar ranges are built from integer ratios, so the same seed gives
 * the same scene on PC FIXED16 and on the DS, with no float and no std::random_device.
 */
class SampleRng
{
public:
    explicit constexpr SampleRng(std::uint32_t seed) noexcept : state_(seed != 0 ? seed : DEFAULT_SEED)
    {
    }

    constexpr std::uint32_t Next() noexcept
    {
        state_ ^= state_ << 13;
        state_ ^= state_ >> 17;
        state_ ^= state_ << 5;
        return state_;
    }

    /**
     * @brief Integer in [minValue, maxValue]
     */
    constexpr int Range(int minValue, int maxValue) noexcept
    {
        const auto span = static_cast<std::uint32_t>(maxValue - minValue) + 1u;
        return minValue + static_cast<int>(Next() % span);
    }

    /**
     * @brief Scalar in [minValue, maxValue], in RESOLUTION steps
     */
    constexpr Scalar Range(Scalar minValue, Scalar maxValue) noexcept
    {
        return minValue + (maxValue - minValue) * Ratio(Range(0, RESOLUTION), RESOLUTION);
    }

    /**
     * @brief One of the non-grey palette entries
     */
    constexpr ColorId RandomColor() noexcept
    {
        return static_cast<ColorId>(Range(sample_color::FIRST_BRIGHT, sample_color::LAST_BRIGHT));
    }

private:
    static constexpr std::uint32_t DEFAULT_SEED = 0x9E3779B9u;
    static constexpr int RESOLUTION = 4096;
    std::uint32_t state_;
};

} // namespace neko

#endif  // MAIN_SAMPLES_SAMPLE_RNG_H_
