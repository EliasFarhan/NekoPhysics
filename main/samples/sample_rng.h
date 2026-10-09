#ifndef MAIN_SAMPLES_SAMPLE_RNG_H_
#define MAIN_SAMPLES_SAMPLE_RNG_H_

#include <cstdint>

#include "main/samples/draw_list.h"
#include "main/samples/sample_config.h"

namespace neko {

/// Seeded xorshift32 random generator for the sample scenes.
///
/// Use it in @ref Sample::Begin with a constant seed, one per sample (the
/// samples use `0x01C0FFEEu`, `0x02C0FFEEu`...), to scatter bodies, speeds and
/// colors. Scalar ranges are built from integer ratios, so the same seed gives
/// the same scene on PC with `FIXED16` and on the DS, with no float and no
/// `std::random_device`. Never seed it from the time: a restart would then
/// give a different scene and break the determinism tests.
///
/// @code
/// SampleRng rng{kSeed};
/// body.position = {rng.Range(bounds.min_bound().x(), bounds.max_bound().x()),
///                  rng.Range(bounds.min_bound().y(), bounds.max_bound().y())};
/// const ColorId color = rng.RandomColor();
/// @endcode
///
/// @note It is not a good general-purpose generator (`Range` uses a modulo),
/// just a small, fast and reproducible one.
class SampleRng {
 public:
  /// Creates a generator from a seed.
  ///
  /// @param seed Initial state; 0 (which xorshift can't leave) is replaced by
  /// a fixed non-zero default.
  explicit constexpr SampleRng(std::uint32_t seed) noexcept
      : state_(seed != 0 ? seed : kDefaultSeed) {}

  /// Advances the state and returns it.
  ///
  /// @return A 32-bit pseudo-random value, never 0.
  constexpr std::uint32_t Next() noexcept {
    state_ ^= state_ << 13;
    state_ ^= state_ >> 17;
    state_ ^= state_ << 5;
    return state_;
  }

  /// Returns an integer in [@p min_value, @p max_value].
  ///
  /// @param min_value Lower bound, inclusive.
  /// @param max_value Upper bound, inclusive, not less than @p min_value.
  /// @return A pseudo-random integer in the closed range.
  constexpr int Range(int min_value, int max_value) noexcept {
    const auto span = static_cast<std::uint32_t>(max_value - min_value) + 1u;
    return min_value + static_cast<int>(Next() % span);
  }

  /// Returns a Scalar in [@p min_value, @p max_value].
  ///
  /// The range is split in 4096 steps and computed with @ref Ratio, without
  /// any float arithmetic.
  ///
  /// @param min_value Lower bound, inclusive.
  /// @param max_value Upper bound, inclusive.
  /// @return A pseudo-random value in the closed range.
  constexpr Scalar Range(Scalar min_value, Scalar max_value) noexcept {
    return min_value +
           (max_value - min_value) * Ratio(Range(0, kResolution), kResolution);
  }

  /// Returns one of the non-grey palette colors.
  ///
  /// @return A color between @ref sample_color::kFirstBright and
  /// @ref sample_color::kLastBright.
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
