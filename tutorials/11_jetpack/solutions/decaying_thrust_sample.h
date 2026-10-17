#ifndef TUTORIALS_11_JETPACK_SOLUTIONS_DECAYING_THRUST_SAMPLE_H_
#define TUTORIALS_11_JETPACK_SOLUTIONS_DECAYING_THRUST_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 11, exercise "decaying thrust": the Splash jetpack of the
// kinematics_02 case study, run in the engine.
//
// A 1 kg player starts at rest in the air. At the first step the burst
// sets its upward speed to v0 = B h / m (parameter "Burst", B = 320 N, for
// one step of h = 1/60 s: 5.33 m/s), here an impulse of B h. From then on
// the jetpack pushes with the decaying thrust 22 r c D(t) (parameter
// "Input", r from 0.1 to 1), where c = 0.75 while rising and 1 otherwise,
// and D(t) = exp(-k t) (parameter "Decay", k = 5 per second) for the first
// second, then 1. The input is released after 1.5 s.
//
// While the player rises, the slide's continuous solution is, with
// A = 22 r 0.75 / m,
//
//   y(t) = y0 + v0 t + (A / k) (t - (1 - exp(-k t)) / k) - g t^2 / 2
//
// The engine applies each step's thrust with D at the start of the step,
// a little more than the exact integral of the decaying force.
//
// Fixed16 has no exponential: D is a factor per step, q = exp(-k h), from
// its Taylor series to x^4 (x = k h), and D after n steps is q^n,
// multiplied step by step like the chapter's drift prediction.
//
// Readouts: the burst speed, the apex time and height (above the start),
// the slide's y at that time, and the difference. Drawn: the ground, the
// player in orange, the flame in red, and in white the height over the
// first three seconds, drawn to the right at 5 cm per step.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class DecayingThrustSample final : public Sample {
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

  // B h / m, the speed right after the burst
  [[nodiscard]] Scalar burst_speed() const;
  // exp(-k h) from its Taylor series
  [[nodiscard]] Scalar decay_per_step() const;
  // Apex step (the first step that ends with v <= 0) and its height above
  // the start, once there
  [[nodiscard]] std::optional<int> apex_step() const { return apex_step_; }
  [[nodiscard]] Scalar apex_height() const { return apex_height_; }
  // The slide's continuous y(t) - y0 at the apex time
  [[nodiscard]] Scalar slide_height() const { return slide_height_; }

  static constexpr std::size_t kInput = 0;
  static constexpr std::size_t kDecay = 1;
  static constexpr std::size_t kBurst = 2;
  // The decay lasts one second, the input 1.5 s
  static constexpr int kDecaySteps = 60;
  static constexpr int kInputSteps = 90;
  static constexpr int kTrailLength = 180;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 2,
                       .max_broadphase_pairs = 2,
                       .max_joints = 0}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Input",
       .unit = "",
       .min = 1,
       .max = 10,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Decay",
       .unit = "1/s",
       .min = 1,
       .max = 10,
       .step = 1,
       .scale = 1,
       .value = 5},
      {.name = "Burst",
       .unit = "N",
       .min = 0,
       .max = 400,
       .step = 10,
       .scale = 1,
       .value = 320},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Burst v", .unit = "m/s", .scale = 1000},
      {.name = "Apex time", .unit = "s", .scale = 1000},
      {.name = "Apex height", .unit = "m", .scale = 1000},
      {.name = "Slide height", .unit = "m", .scale = 1000},
      {.name = "Difference", .unit = "mm", .scale = 10},
  }};
  Aabbf ground_{};
  BodyIndex player_ = kInvalidBodyIndex;
  Scalar start_y_{};
  Scalar decay_{1};
  Scalar thrust_{};
  int step_ = 0;
  std::optional<int> apex_step_;
  Scalar apex_height_{};
  Scalar slide_height_{};
  std::array<Vec2f, kTrailLength> trail_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_11_JETPACK_SOLUTIONS_DECAYING_THRUST_SAMPLE_H_
