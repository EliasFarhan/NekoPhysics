#ifndef TUTORIALS_12_AIMING_SOLUTIONS_ERROR_TABLE_SAMPLE_H_
#define TUTORIALS_12_AIMING_SOLUTIONS_ERROR_TABLE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 12, exercise "error table": what the Fixed16 inverse trigonometry
// costs against float.
//
// The launch angle is written once, as a template over the number type,
// with ScalarTraits<T> as the only switch: ScalarTraits<Fixed16>::Asin is
// the integer-only arcsine, ScalarTraits<float>::Asin is std::asin. Both are
// computed here whatever NEKO_SCALAR is, and measured against std::asin and
// std::atan2 in double:
//
// - the largest Asin error over all 131073 Fixed16 inputs from -1 to 1, and
//   the largest Atan2 error over a grid of vectors (every 1/64 m from -4 to
//   4 m on both axes), in microradians (one Fixed16 step is 15.3 urad);
// - the launch angle for the distance and speed of the parameters, with
//   each type, their difference, and how far it moves the landing point
//   (dR = 2 v^2 cos(2 theta) / g d(theta), in micrometers).
//
// The Fixed16 error is a few microradians, far below what a player sees;
// what float can't give is the same last bit on another machine.
//
// This exercise runs on the PC only: it uses float and double to measure.
//
// Readouts: the two largest errors; both launch angles; their difference;
// the landing shift; the float arcsine's own error at that input. Drawn: the
// signed Asin error over the inputs from -1 (left) to 1 (right), in Fixed16
// steps (the red lines are +1 and -1 step).
class ErrorTableSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override {}
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // In microradians
  [[nodiscard]] double asin_max_error() const { return asin_max_error_; }
  [[nodiscard]] double atan2_max_error() const { return atan2_max_error_; }
  // Launch angles in radians
  [[nodiscard]] double fixed_angle() const { return fixed_angle_; }
  [[nodiscard]] double float_angle() const { return float_angle_; }
  // In micrometers
  [[nodiscard]] double range_shift() const { return range_shift_; }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kDistance = 1;
  static constexpr std::size_t kBucketCount = 256;

 private:
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 50,
       .max = 140,
       .step = 1,
       .scale = 10,
       .value = 100},
      {.name = "Distance",
       .unit = "m",
       .min = 1,
       .max = 100,
       .step = 1,
       .scale = 10,
       .value = 60},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Asin max err.", .unit = "urad", .scale = 100},
      {.name = "Atan2 max err.", .unit = "urad", .scale = 100},
      {.name = "Fixed16 angle", .unit = "deg", .scale = 10000},
      {.name = "float angle", .unit = "deg", .scale = 10000},
      {.name = "Angle diff.", .unit = "urad", .scale = 100},
      {.name = "Landing shift", .unit = "um", .scale = 10},
      {.name = "float asin err.", .unit = "urad", .scale = 1000},
  }};
  Aabbf bounds_{};
  // Signed Asin error with the largest magnitude in each bucket of inputs,
  // in hundredths of a Fixed16 step
  std::array<int, kBucketCount> buckets_{};
  double asin_max_error_ = 0.0;
  double atan2_max_error_ = 0.0;
  double fixed_angle_ = 0.0;
  double float_angle_ = 0.0;
  double range_shift_ = 0.0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_12_AIMING_SOLUTIONS_ERROR_TABLE_SAMPLE_H_
