#include "tutorials/17_same_numbers/solutions/libm_sin_sample.h"

#include <bit>
#include <cmath>
#include <numbers>

#include "neko/math/scalar_traits.h"

namespace neko::tutorial {
namespace {
// Distance in units in the last place between two floats of the same sign
int UlpDistance(float a, float b) {
  const auto bits_a = static_cast<std::int64_t>(std::bit_cast<std::int32_t>(a));
  const auto bits_b = static_cast<std::int64_t>(std::bit_cast<std::int32_t>(b));
  const std::int64_t distance =
      bits_a > bits_b ? bits_a - bits_b : bits_b - bits_a;
  return static_cast<int>(distance);
}

constexpr int kDrawPoints = 128;
}  // namespace

void LibmSinSample::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;
  wrong_angles_.clear();
  wrong_count_ = 0;
  max_ulp_error_ = 0;
  for (int i = 0; i < kAngleCount; ++i) {
    const auto angle =
        static_cast<float>(2.0 * std::numbers::pi * static_cast<double>(i) /
                           static_cast<double>(kAngleCount));
    const float library = std::sin(angle);
    const auto reference =
        static_cast<float>(std::sin(static_cast<double>(angle)));
    if (library != reference) {
      ++wrong_count_;
      wrong_angles_.push_back(i);
      // Near the zeros the two can have opposite signs: count that as far
      const int error = (library < 0.0f) == (reference < 0.0f)
                            ? UlpDistance(library, reference)
                            : 1 << 30;
      max_ulp_error_ = error > max_ulp_error_ ? error : max_ulp_error_;
    }
  }
  readouts_[0].value = kAngleCount;
  readouts_[0].is_valid = true;
  readouts_[1].value = wrong_count_;
  readouts_[1].is_valid = true;
  readouts_[2].value = max_ulp_error_;
  readouts_[2].is_valid = true;
  readouts_[3].value = ScalarTraits<Scalar>::kIsCrossPlatformExact ? 1 : 0;
  readouts_[3].is_valid = true;
}

void LibmSinSample::Draw(DrawList& draw_list) const {
  const Vec2f center = bounds_.GetCenter();
  const Scalar width = bounds_.GetHalfSize().x() * Ratio(9, 5);
  const Scalar height = bounds_.GetHalfSize().y() * Ratio(3, 5);
  const Scalar left = center.x() - width / Scalar{2};
  const auto point = [&](int i, int count) {
    // Through float: Fixed16 can't hold 65536
    const auto t =
        static_cast<float>(static_cast<double>(i) / static_cast<double>(count));
    const auto angle =
        static_cast<float>(2.0 * std::numbers::pi * static_cast<double>(i) /
                           static_cast<double>(count));
    return Vec2f{left + width * Scalar{t},
                 center.y() + height * Scalar{std::sin(angle)}};
  };
  draw_list.lines.push_back({{left, center.y()},
                             {left + width, center.y()},
                             sample_color::kDarkGrey});
  for (int i = 0; i < kDrawPoints; ++i) {
    draw_list.lines.push_back(
        {point(i, kDrawPoints), point(i + 1, kDrawPoints), sample_color::kSky});
  }
  for (const int i : wrong_angles_) {
    draw_list.circles.push_back(
        {point(i, kAngleCount), Ratio(1, 20), sample_color::kRed});
  }
}

}  // namespace neko::tutorial
