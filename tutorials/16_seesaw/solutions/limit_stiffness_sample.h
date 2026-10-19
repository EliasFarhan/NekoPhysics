#ifndef TUTORIALS_16_SEESAW_SOLUTIONS_LIMIT_STIFFNESS_SAMPLE_H_
#define TUTORIALS_16_SEESAW_SOLUTIONS_LIMIT_STIFFNESS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 16, exercise "limit stiffness": how far a limit gives under a
// steady torque.
//
// No gravity. A 1 kg plank 2 m long turns about a static axle at its middle,
// limited to plus or minus 30 degrees, and a game-side torque ("Torque", N m)
// turns it counterclockwise, into the upper limit. The limit is a soft step
// of frequency f ("Hertz", PhysicsWorldConfig::joint_hertz) like every joint:
// at rest it acts as an angular spring of stiffness w^2 I (w = 2 pi f, I the
// plank's inertia about the axle, m (w^2 + h^2) / 12), so the plank rests
// past the limit by
//
//   sink = torque / (w^2 I)
//
// (0.10 degree for 5 N m at 15 Hz, 0.86 degree at 5 Hz). The world is
// rebuilt with the new frequency when the parameter changes.
//
// Readouts: the predicted and measured sink (the angle past the limit, the
// average of the last second), the largest overshoot past the limit since
// the start (the plank hits it at speed), and the torque the limit pushes
// back with (its impulse over dt), which settles to the applied torque.
//
// Drawn: the axle in grey, the plank in white, the limits in dark grey.
//
// The world is y-up, in meters and seconds, without gravity.
class LimitStiffnessSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_->GenerateChecksum();
  }
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // torque / (w^2 I), in radians
  [[nodiscard]] Scalar predicted_sink() const;
  // Average angle past the limit over the last kAverageSteps steps, in
  // radians
  [[nodiscard]] Scalar measured_sink() const;
  [[nodiscard]] Scalar max_overshoot() const { return max_overshoot_; }
  // (upper - lower) impulse over dt, in N m
  [[nodiscard]] Scalar limit_torque() const;

  static constexpr std::size_t kHertz = 0;
  static constexpr std::size_t kTorque = 1;
  static constexpr int kAverageSteps = 60;

 private:
  std::optional<PhysicsWorld> world_;
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Hertz",
       .unit = "Hz",
       .min = 5,
       .max = 30,
       .step = 5,
       .scale = 1,
       .value = 15},
      {.name = "Torque",
       .unit = "N m",
       .min = 1,
       .max = 20,
       .step = 1,
       .scale = 1,
       .value = 5},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Pred. sink", .unit = "deg", .scale = 1000},
      {.name = "Measured sink", .unit = "deg", .scale = 1000},
      {.name = "Max overshoot", .unit = "deg", .scale = 1000},
      {.name = "Limit torque", .unit = "N m", .scale = 100},
  }};
  BodyIndex axle_ = kInvalidBodyIndex;
  BodyIndex plank_ = kInvalidBodyIndex;
  RevoluteJointIndex hinge_ = kInvalidRevoluteJointIndex;
  std::array<Scalar, kAverageSteps> sinks_{};
  int step_ = 0;
  Scalar max_overshoot_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_16_SEESAW_SOLUTIONS_LIMIT_STIFFNESS_SAMPLE_H_
