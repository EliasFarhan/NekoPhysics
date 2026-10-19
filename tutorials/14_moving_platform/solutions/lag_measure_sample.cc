#include "tutorials/14_moving_platform/solutions/lag_measure_sample.h"

#include <algorithm>

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "tutorials/14_moving_platform/moving_platform_sample.h"

namespace neko::tutorial {
namespace {
constexpr int kStepsPerSecond = 60;
constexpr Vec2f kPlatformHalfSize{Ratio(4, 5), Ratio(3, 20)};
constexpr Scalar kAmplitude = Ratio(6, 5);
}  // namespace

Vec2f LagMeasureSample::TargetPath(int step) const {
  return PatrolPoint(target_center_, kAmplitude, step, period_steps_);
}

Vec2f LagMeasureSample::LagPath(int step) const {
  return PatrolPoint(lag_center_, kAmplitude, step, period_steps_);
}

void LagMeasureSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();
  target_center_ = center + Vec2f{Scalar{0}, Ratio(6, 5)};
  lag_center_ = center - Vec2f{Scalar{0}, Ratio(6, 5)};
  period_steps_ = parameters_[kPeriod].value * kStepsPerSecond;
  // max(5 tau, 2 T) in steps, rounded up to whole periods: tau is in tenths
  // of a second, 6 steps each
  const int forget_steps =
      std::max(5 * parameters_[kLagTau].value * 6, 2 * period_steps_);
  settle_steps_ =
      (forget_steps + period_steps_ - 1) / period_steps_ * period_steps_;
  target_ = AddKinematicPlatform(world_, TargetPath(0), kPlatformHalfSize);
  lag_ = AddKinematicPlatform(world_, LagPath(0), kPlatformHalfSize);
  // The continuous first-order lag: 1 / sqrt(1 + (w tau)^2), atan(w tau)
  const Scalar w_tau = Scalar{2} * Pi<Scalar>() *
                       parameters_[kLagTau].ToScalar() /
                       parameters_[kPeriod].ToScalar();
  predicted_ratio_ = Scalar{1} / Sqrt(Scalar{1} + w_tau * w_tau);
  predicted_phase_ = Degree{Atan2(w_tau, Scalar{1})}.value();
  readouts_[1].value = ToDecimal(predicted_ratio_ * Scalar{100}, 10);
  readouts_[3].value = ToDecimal(predicted_phase_, 10);
  readouts_[1].is_valid = true;
  readouts_[3].is_valid = true;
}

void LagMeasureSample::FixedUpdate() {
  world_.SetKinematicTarget(world_.collider(target_).body_index,
                            TargetPath(step_ + 1), Radian{});
  DriveLagging(world_, world_.collider(lag_).body_index, LagPath(step_ + 1),
               parameters_[kLagTau].ToScalar());
  world_.Step();
  ++step_;
  max_target_error_ =
      std::max(max_target_error_,
               (world_.ColliderCenter(target_) - TargetPath(step_)).Length());
  // One period after the start is forgotten
  const Scalar x = world_.ColliderCenter(lag_).x();
  if (step_ == settle_steps_ + 1) {
    max_x_ = x;
    min_x_ = x;
    max_step_ = step_;
  } else if (step_ > settle_steps_ && step_ <= measure_end_step()) {
    if (x > max_x_) {
      max_x_ = x;
      max_step_ = step_;
    }
    min_x_ = std::min(min_x_, x);
  }
  if (step_ == measure_end_step()) {
    is_measured_ = true;
    measured_ratio_ = (max_x_ - min_x_) / (Scalar{2} * kAmplitude);
    // The target (center - A cos) peaks half a period into each period
    const int target_peak = settle_steps_ + period_steps_ / 2;
    // Divided first: (steps * 360) overflows Fixed16 for long periods
    measured_phase_ = static_cast<Scalar>(max_step_ - target_peak) /
                      static_cast<Scalar>(period_steps_) * Scalar{360};
    readouts_[0].value = ToDecimal(measured_ratio_ * Scalar{100}, 10);
    readouts_[2].value = ToDecimal(measured_phase_, 10);
    readouts_[0].is_valid = true;
    readouts_[2].is_valid = true;
  }
  readouts_[4].value = ToDecimal(max_target_error_ * Scalar{1000}, 10);
  readouts_[4].is_valid = true;
}

void LagMeasureSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {world_.ColliderCenter(target_), kPlatformHalfSize, sample_color::kTeal});
  draw_list.boxes.push_back(
      {world_.ColliderCenter(lag_), kPlatformHalfSize, sample_color::kPurple});
  DrawBoxOutline(LagPath(step_), kPlatformHalfSize, Rotation{},
                 sample_color::kYellow, draw_list);
}

void LagMeasureSample::End() {
  world_.Clear();
  step_ = 0;
  is_measured_ = false;
  max_target_error_ = Scalar{0};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
