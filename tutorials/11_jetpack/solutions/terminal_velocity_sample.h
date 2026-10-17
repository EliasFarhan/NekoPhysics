#ifndef TUTORIALS_11_JETPACK_SOLUTIONS_TERMINAL_VELOCITY_SAMPLE_H_
#define TUTORIALS_11_JETPACK_SOLUTIONS_TERMINAL_VELOCITY_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 11, exercise "terminal velocity": a falling player with drag.
//
// A player of mass m (parameter "Mass") falls from rest, with no ground.
// Every step the game adds Millington's drag (Game Physics Engine
// Development, section 5.2.4), against the velocity v with the strength
// k1 |v| + k2 |v|^2:
//
//   F = -v (k1 + k2 |v|)
//
// (parameters "k1", in N s/m, and "k2", in N s^2/m^2). The fall speeds up
// until the drag equals the weight, k1 v + k2 v^2 = m g: the terminal
// velocity
//
//   v_t = (sqrt(k1^2 + 4 k2 m g) - k1) / (2 k2), or m g / k1 when k2 = 0
//
// 9.905 m/s for the defaults (k2 = 0.1, 1 kg): sqrt(m g / k2).
//
// Readouts: the predicted terminal velocity, the speed, the time it first
// reaches 99 % of v_t, and the distance fallen. Drawn: the speed over ten
// seconds in white, v_t in green.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class TerminalVelocitySample final : public Sample {
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

  [[nodiscard]] Scalar predicted_terminal() const;
  [[nodiscard]] Scalar speed() const;
  [[nodiscard]] std::optional<Scalar> time_to_99() const { return time_to_99_; }
  [[nodiscard]] Scalar fallen() const;
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kLinear = 0;
  static constexpr std::size_t kQuadratic = 1;
  static constexpr std::size_t kMass = 2;
  // Ten seconds, a point of the graph every six steps
  static constexpr int kMeasuredSteps = 600;
  static constexpr int kStepsPerPoint = 6;
  static constexpr std::size_t kPoints = kMeasuredSteps / kStepsPerPoint + 1;

 private:
  [[nodiscard]] Scalar mass() const { return parameters_[kMass].ToScalar(); }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 1,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 0}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "k1",
       .unit = "N s/m",
       .min = 0,
       .max = 200,
       .step = 5,
       .scale = 100,
       .value = 0},
      {.name = "k2",
       .unit = "N s2/m2",
       .min = 0,
       .max = 1000,
       .step = 10,
       .scale = 1000,
       .value = 100},
      {.name = "Mass",
       .unit = "kg",
       .min = 5,
       .max = 40,
       .step = 1,
       .scale = 10,
       .value = 10},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Pred. terminal", .unit = "m/s", .scale = 1000},
      {.name = "Speed", .unit = "m/s", .scale = 1000},
      {.name = "99 % at", .unit = "s", .scale = 100},
      {.name = "Fallen", .unit = "m", .scale = 10},
  }};
  BodyIndex player_ = kInvalidBodyIndex;
  Vec2f origin_{};
  Scalar start_y_{};
  int step_ = 0;
  std::optional<Scalar> time_to_99_;
  std::array<Scalar, kPoints> speeds_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_11_JETPACK_SOLUTIONS_TERMINAL_VELOCITY_SAMPLE_H_
