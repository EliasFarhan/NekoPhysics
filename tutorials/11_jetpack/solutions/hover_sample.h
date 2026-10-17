#ifndef TUTORIALS_11_JETPACK_SOLUTIONS_HOVER_SAMPLE_H_
#define TUTORIALS_11_JETPACK_SOLUTIONS_HOVER_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 11, exercise "hover": a jetpack controller that holds a height.
//
// The chapter's player (a 0.3 m circle of mass m, parameter "Mass") starts
// on the ground. Every step the controller reads the height y and the
// vertical speed v and sets the thrust
//
//   T = m (g + w^2 (h - y) - 2 z w v)
//
// clamped between 0 (a jetpack doesn't pull down) and parameter "Max
// thrust". h is parameter "Height", w the natural frequency (parameter
// "Omega", rad/s) and z the damping ratio (parameter "Damping ratio"). The
// m g term cancels gravity, the rest makes the height error a damped spring
// of frequency w: it overshoots by exp(-z pi / sqrt(1 - z^2)) of the start
// error below z = 1, not at all from z = 1 on.
//
// Readouts: the thrust, the height, the largest overshoot above h, and the
// settling time (the last time the player was more than 1 cm from h, over
// ten seconds). Drawn: the ground, the target height in green, the player
// in orange, the flame in red, as long as the thrust.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class HoverSample final : public Sample {
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

  [[nodiscard]] Scalar height() const {
    return world_.body(player_).position.y();
  }
  [[nodiscard]] Scalar target() const {
    return parameters_[kHeight].ToScalar();
  }
  [[nodiscard]] Scalar start_height() const { return start_height_; }
  [[nodiscard]] Scalar overshoot() const { return overshoot_; }
  // Time of the last step that ended more than 1 cm from the target
  [[nodiscard]] Scalar settling_time() const;
  [[nodiscard]] Scalar thrust() const { return thrust_; }
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kHeight = 0;
  static constexpr std::size_t kOmega = 1;
  static constexpr std::size_t kDampingRatio = 2;
  static constexpr std::size_t kMaxThrust = 3;
  static constexpr std::size_t kMass = 4;
  // The measurement lasts ten seconds
  static constexpr int kMeasuredSteps = 600;

 private:
  [[nodiscard]] Scalar mass() const { return parameters_[kMass].ToScalar(); }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 2,
                       .max_broadphase_pairs = 2,
                       .max_joints = 0}};
  std::array<SampleParameter, 5> parameters_{{
      {.name = "Height",
       .unit = "m",
       .min = 10,
       .max = 60,
       .step = 1,
       .scale = 10,
       .value = 30},
      {.name = "Omega",
       .unit = "rad/s",
       .min = 10,
       .max = 100,
       .step = 1,
       .scale = 10,
       .value = 40},
      {.name = "Damping ratio",
       .unit = "",
       .min = 0,
       .max = 30,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Max thrust",
       .unit = "N",
       .min = 10,
       .max = 100,
       .step = 1,
       .scale = 1,
       .value = 60},
      {.name = "Mass",
       .unit = "kg",
       .min = 5,
       .max = 40,
       .step = 1,
       .scale = 10,
       .value = 10},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Thrust", .unit = "N", .scale = 100},
      {.name = "Height", .unit = "m", .scale = 1000},
      {.name = "Overshoot", .unit = "mm", .scale = 10},
      {.name = "Settled at", .unit = "s", .scale = 100},
  }};
  Aabbf ground_{};
  BodyIndex player_ = kInvalidBodyIndex;
  Scalar start_height_{};
  Scalar thrust_{};
  Scalar overshoot_{};
  int step_ = 0;
  int last_outside_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_11_JETPACK_SOLUTIONS_HOVER_SAMPLE_H_
