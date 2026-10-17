#ifndef TUTORIALS_11_JETPACK_SOLUTIONS_STEP_RATE_SAMPLE_H_
#define TUTORIALS_11_JETPACK_SOLUTIONS_STEP_RATE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 11, exercise "step rate": forces and impulses at 30, 60 and 120
// steps per second.
//
// Four 1 kg bodies, without gravity, in a world stepped at the rate of
// parameter "Rate" (Hz; a frame of the host is 1/60 s, so 30 Hz steps every
// other frame and 120 Hz twice per frame). For one second of world time:
//
// - "thrust": ApplyForce(T) every step (parameter "Thrust"): v = T t / m,
//   whatever the rate;
// - "impulse": ApplyImpulse(J) once (parameter "Impulse"): v = J / m,
//   whatever the rate;
// - "burst": the Splash jetpack's burst of the kinematics_02 case study, a
//   force F (parameter "Burst", 320 N) for one step: v = F h / m, which
//   halves when the rate doubles (5.33 m/s at 60 Hz);
// - "every step": ApplyImpulse(J) every step, the frame-dependent pattern:
//   v = J n / m for n steps, 240 m/s at 60 Hz.
//
// The thrust's distance also changes a little with the rate: semi-implicit
// Euler moves by T dt^2 n (n + 1) / (2 m), (1 + 1 / n) times T t^2 / (2 m).
//
// Readouts after one second: the four speeds, the thrust's distance and the
// number of steps. Drawn: the four bodies, one row each, their velocities.
//
// The world is y-up, in meters and seconds.
class StepRateSample final : public Sample {
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

  static constexpr std::size_t kRate = 0;
  static constexpr std::size_t kThrust = 1;
  static constexpr std::size_t kImpulse = 2;
  static constexpr std::size_t kBurst = 3;
  // Body order
  static constexpr std::size_t kThrustBody = 0;
  static constexpr std::size_t kImpulseBody = 1;
  static constexpr std::size_t kBurstBody = 2;
  static constexpr std::size_t kEveryStepBody = 3;
  static constexpr std::size_t kBodyCount = 4;

  [[nodiscard]] Vec2f velocity(std::size_t body) const {
    return world_.body(bodies_[body]).velocity;
  }
  [[nodiscard]] Scalar distance(std::size_t body) const {
    return world_.body(bodies_[body]).position.x() - kStartX;
  }
  [[nodiscard]] int steps() const { return steps_; }
  [[nodiscard]] bool done() const { return steps_ >= rate(); }
  [[nodiscard]] Scalar dt() const { return world_.dt(); }

  static constexpr Scalar kStartX{1};

 private:
  [[nodiscard]] int rate() const { return parameters_[kRate].value; }
  void StepOnce();

  // Rebuilt in Begin: the time step comes from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Rate",
       .unit = "Hz",
       .min = 30,
       .max = 120,
       .step = 30,
       .scale = 1,
       .value = 60},
      {.name = "Thrust",
       .unit = "N",
       .min = 0,
       .max = 30,
       .step = 1,
       .scale = 1,
       .value = 10},
      {.name = "Impulse",
       .unit = "N s",
       .min = 0,
       .max = 10,
       .step = 1,
       .scale = 1,
       .value = 4},
      {.name = "Burst",
       .unit = "N",
       .min = 0,
       .max = 400,
       .step = 10,
       .scale = 1,
       .value = 320},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Thrust v", .unit = "m/s", .scale = 1000},
      {.name = "Thrust x", .unit = "m", .scale = 1000},
      {.name = "Impulse v", .unit = "m/s", .scale = 1000},
      {.name = "Burst v", .unit = "m/s", .scale = 1000},
      {.name = "Every step v", .unit = "m/s", .scale = 10},
      {.name = "Steps", .unit = "", .scale = 1},
  }};
  std::array<BodyIndex, kBodyCount> bodies_{};
  int frame_ = 0;
  int steps_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_11_JETPACK_SOLUTIONS_STEP_RATE_SAMPLE_H_
