#ifndef TUTORIALS_01_ROTATION_SOLUTIONS_HALF_STEP_SAMPLE_H_
#define TUTORIALS_01_ROTATION_SOLUTIONS_HALF_STEP_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 01, exercise "half step": shrink the time step and watch the
// angle's error shrink with it while the spin doesn't change.
//
// The couple and the 2 x 1 m box of chapter 01, in a world whose time step
// is dt = (1/60 s) / n (parameter "Substeps"): every frame steps it n times,
// applying the couple before each step (Step clears forces and torques).
// After the push of T seconds:
//
//   spin            omega = alpha T                (exact for any dt)
//   angle           alpha T^2 / 2 + alpha T dt / 2 (semi-implicit Euler)
//   excess          alpha T dt / 2: halves when n doubles
//
// With Fixed16, 1/60 s is stored as 1092/65536 s, and 1092 is divisible by
// 1, 2, 3 and 4: every substep dt is exact, so the n substeps of a frame
// last exactly one frame.
//
// Drawn like chapter 01: box outline, orientation, lever arms and forces;
// after the push the predicted direction alpha T^2 / 2 in green and the
// measured one in yellow.
class HalfStepSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // The world's time step, 1/60 s divided by the substeps
  [[nodiscard]] Scalar dt() const;
  // Frames of 1/60 s in the push
  [[nodiscard]] int push_frames() const { return push_frames_; }
  // tau / I = 2 r F / (m (w^2 + h^2) / 12), in rad/s^2
  [[nodiscard]] Scalar angular_acceleration() const;
  // Push duration, in seconds
  [[nodiscard]] Scalar push_time() const;
  // alpha T^2 / 2, in radians
  [[nodiscard]] Scalar predicted_turn() const;
  // alpha T dt / 2, in radians
  [[nodiscard]] Scalar predicted_excess() const;
  // Angle turned during the push; empty until the push is over
  [[nodiscard]] std::optional<Scalar> measured_turn() const {
    return measured_turn_;
  }
  // The box's angular velocity now, in rad/s
  [[nodiscard]] Scalar spin() const;

  static constexpr std::size_t kForce = 0;
  static constexpr std::size_t kLeverArm = 1;
  static constexpr std::size_t kMass = 2;
  static constexpr std::size_t kPushTime = 3;
  static constexpr std::size_t kSubsteps = 4;

 private:
  [[nodiscard]] bool is_pushing() const { return frame_ < push_frames_; }
  void Push();

  // Built in Begin: the time step depends on a parameter
  std::optional<PhysicsWorld> world_;
  std::array<SampleParameter, 5> parameters_{{
      {.name = "Force",
       .unit = "N",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 10,
       .value = 20},
      {.name = "Lever arm",
       .unit = "m",
       .min = 10,
       .max = 150,
       .step = 10,
       .scale = 100,
       .value = 50},
      {.name = "Mass",
       .unit = "kg",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 10,
       .value = 10},
      {.name = "Push time",
       .unit = "s",
       .min = 25,
       .max = 200,
       .step = 25,
       .scale = 100,
       .value = 100},
      {.name = "Substeps",
       .unit = "",
       .min = 1,
       .max = 4,
       .step = 1,
       .scale = 1,
       .value = 2},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Time step", .unit = "ms", .scale = 100},
      {.name = "Predicted turn", .unit = "rad", .scale = 1000},
      {.name = "Measured turn", .unit = "rad", .scale = 1000},
      {.name = "Excess", .unit = "rad", .scale = 1000},
      {.name = "alpha T dt / 2", .unit = "rad", .scale = 1000},
  }};
  BodyIndex box_index_ = kInvalidBodyIndex;
  int push_frames_ = 0;
  int frame_ = 0;
  Scalar turned_{};
  Scalar previous_angle_{};
  std::optional<Scalar> measured_turn_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_01_ROTATION_SOLUTIONS_HALF_STEP_SAMPLE_H_
