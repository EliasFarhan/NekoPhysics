#ifndef MAIN_SAMPLES_SAMPLE_RNG_H_
#define MAIN_SAMPLES_SAMPLE_RNG_H_

#include <cstdint>

#include "main/samples/draw_list.h"
#include "main/samples/sample_config.h"

namespace neko {

// Seeded xorshift32 for the sample scenes. Scalar ranges are built from integer
// ratios, so the same seed gives the same scene on PC FIXED16 and on the DS,
// with no float and no std::random_device.
class SampleRng {
 public:
  explicit constexpr SampleRng(std::uint32_t seed) noexcept
      : state_(seed != 0 ? seed : kDefaultSeed) {}

  constexpr std::uint32_t Next() noexcept {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;
    return state_;
  }

  // Integer in [min_value, max_value]
  constexpr int Range(int min_value, int max_value) noexcept {
    const auto span = static_cast<std::uint32_t>(max_value - min_value) + 1u;
    return min_value + static_cast<int>(Next() % span);
  }

  // Scalar in [min_value, max_value], in kResolution steps
  constexpr Scalar Range(Scalar min_value, Scalar max_value) noexcept {
    return min_value +
           (max_value - min_value) * Ratio(Range(0, kResolution), kResolution);
  }

  // One of the non-grey palette entries
  constexpr ColorId RandomColor() noexcept {
    return static_cast<ColorId>(
        Range(sample_color::kFirstBright, sample_color::kLastBright));
  }

 private:
  static constexpr std::uint32_t kDefaultSeed = 0x9E3779B9u;
  static constexpr int kResolution = 4096;
  std::uint32_t state_;
};

}  // namespace neko

#endif  // MAIN_SAMPLES_SAMPLE_RNG_H_
