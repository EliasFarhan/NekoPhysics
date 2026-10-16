#ifndef TUTORIALS_05_FRICTION_FRICTION_SAMPLE_H_
#define TUTORIALS_05_FRICTION_FRICTION_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 05, friction: does a crate hold on a ramp, and how fast does it
// slide?
//
// A 1 m crate rests on a static ramp turned by `a` degrees (parameter "Ramp
// angle"); both surfaces have the friction coefficient `mu` (parameter
// "Friction"), so the contact uses sqrt(mu * mu) = mu. Along the slope,
// gravity pulls with m g sin a and the ramp pushes back with m g cos a, so
// Coulomb friction can hold up to mu m g cos a:
//
//   holds            tan a <= mu          (static friction is enough)
//   else slides at   g (sin a - mu cos a) (kinetic friction at its limit)
//
// The sample measures the crate's speed down the slope after one second (the
// average acceleration) and how far it went, and compares them with the
// prediction. Drawn: the ramp in grey, the crate in orange with its
// orientation in white and its velocity in sky blue.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class FrictionSample final : public Sample {
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

  // True when tan a <= mu
  [[nodiscard]] bool predicted_holds() const;
  // g (sin a - mu cos a) when it slides, 0 when it holds, in m/s^2
  [[nodiscard]] Scalar predicted_acceleration() const;
  // Speed down the slope after one second divided by that second, in m/s^2;
  // empty before
  [[nodiscard]] std::optional<Scalar> measured_acceleration() const {
    return measured_acceleration_;
  }
  // Distance down the slope since the start, in meters
  [[nodiscard]] Scalar distance() const;

  static constexpr std::size_t kRampAngle = 0;
  static constexpr std::size_t kFriction = 1;
  // Steps of the measurement, one second at 60 Hz
  static constexpr int kMeasureSteps = 60;

 private:
  [[nodiscard]] Scalar speed_down_slope() const;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Ramp angle",
       .unit = "deg",
       .min = 5,
       .max = 45,
       .step = 5,
       .scale = 1,
       .value = 30},
      {.name = "Friction",
       .unit = "",
       .min = 0,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 40},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "tan(angle)", .unit = "", .scale = 1000},
      {.name = "Holds (predicted)", .unit = "", .scale = 1},
      {.name = "Predicted accel.", .unit = "m/s2", .scale = 1000},
      {.name = "Measured accel.", .unit = "m/s2", .scale = 1000},
      {.name = "Distance", .unit = "m", .scale = 1000},
  }};
  BodyIndex ramp_index_ = kInvalidBodyIndex;
  BodyIndex crate_index_ = kInvalidBodyIndex;
  Vec2f down_slope_{};
  Vec2f start_{};
  int step_ = 0;
  std::optional<Scalar> measured_acceleration_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_05_FRICTION_FRICTION_SAMPLE_H_
