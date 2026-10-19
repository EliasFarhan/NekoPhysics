#include "tutorials/12_aiming/solutions/error_table_sample.h"

#include <cmath>
#include <cstdlib>
#include <numbers>

#include "neko/math/fixed.h"
#include "neko/math/scalar_traits.h"

namespace neko::tutorial {
namespace {
using Fixed16Traits = ScalarTraits<Fixed16>;

constexpr double kStep = 1.0 / 65536.0;
constexpr double kMicro = 1e6;
constexpr double kGravity = 9.81;
constexpr std::int32_t kOne = 65536;
// The Atan2 grid: every 1/64 m (1024 raw steps) from -4 to 4 m
constexpr std::int32_t kGridStep = 1024;
constexpr std::int32_t kGridHalfWidth = 4 * kOne;
// Plot: one Fixed16 step of error is this many meters up
constexpr Scalar kMetersPerStep{2};

// The launch angle, once for every number type: ScalarTraits is the switch
template <typename T>
T LaunchAngle(int distance_tenths, int speed_tenths) {
  using Traits = ScalarTraits<T>;
  const T gravity = Traits::FromRatio(981, 100);
  const T distance = Traits::FromRatio(distance_tenths, 10);
  const T speed = Traits::FromRatio(speed_tenths, 10);
  return Traits::Asin(gravity * distance / (speed * speed)) /
         Traits::FromInt(2);
}

double ToDouble(Fixed16 value) {
  return static_cast<double>(value.underlying_value()) * kStep;
}

int Rounded(double value) { return static_cast<int>(std::lround(value)); }
}  // namespace

void ErrorTableSample::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;
  // Every Fixed16 input of Asin
  asin_max_error_ = 0.0;
  buckets_ = {};
  for (std::int32_t raw = -kOne; raw <= kOne; ++raw) {
    const double exact = std::asin(static_cast<double>(raw) * kStep);
    const double error =
        ToDouble(Fixed16Traits::Asin(Fixed16::FromUnderlyingValue(raw))) -
        exact;
    asin_max_error_ = std::fmax(asin_max_error_, std::abs(error) * kMicro);
    const auto bucket =
        static_cast<std::size_t>(raw + kOne) * (kBucketCount - 1) /
        (2 * kOne);
    const int hundredths = Rounded(error / kStep * 100.0);
    if (std::abs(hundredths) > std::abs(buckets_[bucket])) {
      buckets_[bucket] = hundredths;
    }
  }
  atan2_max_error_ = 0.0;
  for (std::int32_t y = -kGridHalfWidth; y <= kGridHalfWidth; y += kGridStep) {
    for (std::int32_t x = -kGridHalfWidth; x <= kGridHalfWidth;
         x += kGridStep) {
      const double exact =
          std::atan2(static_cast<double>(y), static_cast<double>(x));
      const double angle = ToDouble(Fixed16Traits::Atan2(
          Fixed16::FromUnderlyingValue(y), Fixed16::FromUnderlyingValue(x)));
      atan2_max_error_ =
          std::fmax(atan2_max_error_, std::abs(angle - exact) * kMicro);
    }
  }

  // The launch angle with both types, and what the difference does to the
  // landing point
  const int distance = parameters_[kDistance].value;
  const int speed = parameters_[kSpeed].value;
  fixed_angle_ = ToDouble(LaunchAngle<Fixed16>(distance, speed));
  float_angle_ = static_cast<double>(LaunchAngle<float>(distance, speed));
  const double v = static_cast<double>(speed) / 10.0;
  const double range_slope =
      2.0 * v * v * std::cos(2.0 * float_angle_) / kGravity;
  range_shift_ = range_slope * (fixed_angle_ - float_angle_) * kMicro;
  const double input =
      kGravity * (static_cast<double>(distance) / 10.0) / (v * v);
  const double exact_angle = std::asin(std::fmin(input, 1.0)) / 2.0;

  constexpr double to_degrees = 180.0 / std::numbers::pi;
  const std::array<double, 7> values{
      asin_max_error_,
      atan2_max_error_,
      fixed_angle_ * to_degrees,
      float_angle_ * to_degrees,
      (fixed_angle_ - float_angle_) * kMicro,
      range_shift_,
      (float_angle_ - exact_angle) * 2.0 * kMicro};
  for (std::size_t i = 0; i < values.size(); ++i) {
    readouts_[i].value =
        Rounded(values[i] * static_cast<double>(readouts_[i].scale));
    readouts_[i].is_valid = true;
  }
}

void ErrorTableSample::Draw(DrawList& draw_list) const {
  const Scalar left = bounds_.min_bound().x() + Scalar{1};
  const Scalar width = bounds_.GetHalfSize().x() * Scalar{2} - Scalar{2};
  const Scalar middle = bounds_.GetCenter().y();
  const Scalar right = left + width;
  draw_list.lines.push_back(
      {Vec2f{left, middle}, Vec2f{right, middle}, sample_color::kGrey});
  for (const Scalar side : {kMetersPerStep, -kMetersPerStep}) {
    draw_list.lines.push_back({Vec2f{left, middle + side},
                               Vec2f{right, middle + side},
                               sample_color::kRed});
  }
  const auto point = [&](std::size_t bucket) {
    const Scalar x = left + width * Ratio(static_cast<int>(bucket),
                                          static_cast<int>(kBucketCount - 1));
    return Vec2f{x, middle + kMetersPerStep * Ratio(buckets_[bucket], 100)};
  };
  for (std::size_t bucket = 1; bucket < kBucketCount; ++bucket) {
    draw_list.lines.push_back(
        {point(bucket - 1), point(bucket), sample_color::kOrange});
  }
}

void ErrorTableSample::End() {
  buckets_ = {};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

std::uint32_t ErrorTableSample::GenerateChecksum() const {
  // FNV-1a over the readouts and the plot
  std::uint32_t hash = 2166136261u;
  const auto mix = [&hash](int value) {
    hash = (hash ^ static_cast<std::uint32_t>(value)) * 16777619u;
  };
  for (const auto& readout : readouts_) {
    mix(readout.value);
  }
  for (const int bucket : buckets_) {
    mix(bucket);
  }
  return hash;
}

}  // namespace neko::tutorial
