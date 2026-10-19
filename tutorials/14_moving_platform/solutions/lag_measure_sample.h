#ifndef TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_LAG_MEASURE_SAMPLE_H_
#define TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_LAG_MEASURE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 14, exercise "lag measure": how far the Unity PatrolPlatform
// script runs behind its path.
//
// Two kinematic platforms follow the chapter's patrol path (amplitude 1.2 m,
// parameter "Period" T). The top one is driven by SetKinematicTarget; the
// bottom one by the PatrolPlatform script, velocity = (target - position) /
// tau (parameter "Lag tau"). That is the first-order lag dx/dt = (x_target -
// x) / tau. For a sine of angular frequency w = 2 pi / T, once the start is
// forgotten (a few tau), the platform swings with the amplitude ratio
// 1 / sqrt(1 + (w tau)^2) and runs behind by atan(w tau): 71 % and 45
// degrees for w tau = 1. Splash's platforms have w tau between about 0.8
// and 1.6: 54 to 79 % of the amplitude, 40 to 50 degrees behind.
//
// The sample waits max(5 tau, 2 T), rounded up to whole periods, then
// measures one period: the amplitude as half the swing, the phase as the
// steps from the target's peak to the platform's peak (so to 360 / (60 T)
// degrees). The prediction uses Sqrt and Atan2 on Scalar. The engine's step
// is the discrete filter x(n+1) = x(n) + (dt / tau) (x_target(n+1) - x(n)),
// close to the continuous one for dt << tau.
//
// Readouts: the measured and predicted amplitude ratio and phase, and the
// largest distance of the SetKinematicTarget platform to its path (0).
// Drawn: both platforms (teal for the target one, purple for the lagging
// one) and the lagging one's target as a yellow outline.
class LagMeasureSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // Once the measured period is over
  [[nodiscard]] bool is_measured() const { return is_measured_; }
  // The steps until is_measured()
  [[nodiscard]] int measure_end_step() const {
    return settle_steps_ + period_steps_;
  }
  [[nodiscard]] Scalar measured_ratio() const { return measured_ratio_; }
  [[nodiscard]] Scalar predicted_ratio() const { return predicted_ratio_; }
  [[nodiscard]] Scalar measured_phase_degrees() const {
    return measured_phase_;
  }
  [[nodiscard]] Scalar predicted_phase_degrees() const {
    return predicted_phase_;
  }
  [[nodiscard]] Scalar max_target_error() const { return max_target_error_; }
  [[nodiscard]] int period_steps() const { return period_steps_; }

  static constexpr std::size_t kPeriod = 0;
  static constexpr std::size_t kLagTau = 1;

 private:
  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 0,
                       .max_passing_pairs = 0}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Period",
       .unit = "s",
       .min = 2,
       .max = 12,
       .step = 1,
       .scale = 1,
       .value = 6},
      {.name = "Lag tau",
       .unit = "s",
       .min = 2,
       .max = 30,
       .step = 1,
       .scale = 10,
       .value = 10},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Meas. amplitude", .unit = "%", .scale = 10},
      {.name = "Pred. amplitude", .unit = "%", .scale = 10},
      {.name = "Meas. phase", .unit = "deg", .scale = 10},
      {.name = "Pred. phase", .unit = "deg", .scale = 10},
      {.name = "Target error", .unit = "mm", .scale = 10},
  }};
  [[nodiscard]] Vec2f TargetPath(int step) const;
  [[nodiscard]] Vec2f LagPath(int step) const;

  ColliderIndex target_ = kInvalidColliderIndex;
  ColliderIndex lag_ = kInvalidColliderIndex;
  Vec2f target_center_{};
  Vec2f lag_center_{};
  int period_steps_ = 0;
  int settle_steps_ = 0;
  int step_ = 0;
  // During the measured period: the platform's extremes and the step of its
  // highest x
  Scalar max_x_{};
  Scalar min_x_{};
  int max_step_ = 0;
  bool is_measured_ = false;
  Scalar measured_ratio_{};
  Scalar predicted_ratio_{};
  Scalar measured_phase_{};
  Scalar predicted_phase_{};
  Scalar max_target_error_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_LAG_MEASURE_SAMPLE_H_
