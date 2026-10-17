#ifndef TUTORIALS_05_FRICTION_SOLUTIONS_MIXED_SURFACES_SAMPLE_H_
#define TUTORIALS_05_FRICTION_SOLUTIONS_MIXED_SURFACES_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 05, exercise "mixed surfaces": a crate and a ramp with different
// friction coefficients.
//
// The ramp of chapter 05, turned by `a` degrees (parameter "Ramp angle"),
// with its own coefficient mu_r (parameter "Ramp friction"), and the crate
// with mu_c (parameter "Crate friction"). A contact uses the geometric mean
// of the two:
//
//   mu = sqrt(mu_c mu_r)       (ice 0.05 on rubber 1.0 gives 0.224)
//
// so the crate holds while tan a <= mu, at most atan(mu) degrees, and
// otherwise slides at g (sin a - mu cos a). The sample shows the contact's
// coefficient, the steepest whole angle that holds, and the acceleration
// over the first second, measured as in chapter 05. Drawn: the ramp in grey,
// the crate in orange with its orientation in white and its velocity in sky
// blue.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class MixedSurfacesSample final : public Sample {
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

  // sqrt(mu_c mu_r), the coefficient of the contact
  [[nodiscard]] Scalar contact_friction() const;
  // Largest whole angle, in degrees, with tan a <= mu
  [[nodiscard]] int steepest_holding_angle() const;
  [[nodiscard]] bool predicted_holds() const;
  // g (sin a - mu cos a) when it slides, 0 when it holds, in m/s^2
  [[nodiscard]] Scalar predicted_acceleration() const;
  // Speed down the slope after one second divided by that second, in m/s^2;
  // empty before
  [[nodiscard]] std::optional<Scalar> measured_acceleration() const {
    return measured_acceleration_;
  }

  static constexpr std::size_t kRampAngle = 0;
  static constexpr std::size_t kCrateFriction = 1;
  static constexpr std::size_t kRampFriction = 2;
  // Steps of the measurement, one second at 60 Hz
  static constexpr int kMeasureSteps = 60;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Ramp angle",
       .unit = "deg",
       .min = 1,
       .max = 45,
       .step = 1,
       .scale = 1,
       .value = 15},
      {.name = "Crate friction",
       .unit = "",
       .min = 5,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 5},
      {.name = "Ramp friction",
       .unit = "",
       .min = 5,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 100},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Contact friction", .unit = "", .scale = 1000},
      {.name = "Steepest holding", .unit = "deg", .scale = 1},
      {.name = "Holds (predicted)", .unit = "", .scale = 1},
      {.name = "Predicted accel.", .unit = "m/s2", .scale = 1000},
      {.name = "Measured accel.", .unit = "m/s2", .scale = 1000},
  }};
  BodyIndex ramp_index_ = kInvalidBodyIndex;
  BodyIndex crate_index_ = kInvalidBodyIndex;
  Vec2f down_slope_{};
  int step_ = 0;
  std::optional<Scalar> measured_acceleration_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_05_FRICTION_SOLUTIONS_MIXED_SURFACES_SAMPLE_H_
