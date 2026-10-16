#ifndef TUTORIALS_04_OFF_CENTER_IMPACT_OFF_CENTER_IMPACT_SAMPLE_H_
#define TUTORIALS_04_OFF_CENTER_IMPACT_OFF_CENTER_IMPACT_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 04, make an off-center impact spin a body.
//
// A box of mass 1 kg (parameter "Width", 1 m tall), turned by `a` degrees
// (parameter "Tilt"), moves down at `v` m/s (parameter "Speed") without
// gravity and hits a static, frictionless ground with its lowest corner. The
// contact impulse `j` acts at that corner, at `r` from the center, so it
// changes both the velocity and the spin:
//
//   lever arm            r x n = r.x                (n = (0, 1), up)
//   inertia              I = m (w^2 + h^2) / 12
//   effective inv. mass  k = 1/m + (r x n)^2 / I
//   impulse              j = -(1 + e) v_n / k       (v_n = -v, approaching)
//   after the hit        v' = v + j / m,   w' = (r x n) j / I
//
// `k` is larger than 1/m: part of the impulse goes into spin, so the box
// bounces less than a box hitting flat, and the corner (not the center)
// leaves at -e times its approach speed (parameter "Restitution"). A
// corner left of the center makes it spin clockwise, back toward flat. The
// longer the lever arm, the less the box bounces; the spin
// (1 + e) v l / (1/m + l^2 / I) is fastest when the lever arm l equals the
// radius of gyration sqrt(I / m).
//
// Readouts compare the prediction with the world, measured over the impact
// step. Drawn: the ground in dark grey, the box in orange with its
// orientation in white and velocity in sky blue; after the hit, the lever arm
// in grey, the corner in yellow and the impulse in red.
//
// The world is y-up, in meters and seconds, with no gravity.
class OffCenterImpactSample final : public Sample {
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

  // The values below are empty until the box hit the ground.
  // Lever arm r x n of the corner that hit, in meters
  [[nodiscard]] std::optional<Scalar> lever_arm() const { return lever_arm_; }
  // The predicted impulse j, in N s
  [[nodiscard]] std::optional<Scalar> predicted_impulse() const;
  [[nodiscard]] std::optional<Scalar> predicted_velocity() const;
  [[nodiscard]] std::optional<Scalar> predicted_spin() const;
  // The box's vertical velocity and spin right after the impact step
  [[nodiscard]] std::optional<Scalar> measured_velocity() const {
    return measured_velocity_;
  }
  [[nodiscard]] std::optional<Scalar> measured_spin() const {
    return measured_spin_;
  }
  // -(corner velocity after) / (corner velocity before), predicted to be
  // the restitution
  [[nodiscard]] std::optional<Scalar> corner_rebound() const;
  [[nodiscard]] Scalar restitution() const {
    return parameters_[kRestitution].ToScalar();
  }
  // m (w^2 + h^2) / 12, in kg m^2
  [[nodiscard]] Scalar inertia() const;

  static constexpr std::size_t kTilt = 0;
  static constexpr std::size_t kRestitution = 1;
  static constexpr std::size_t kSpeed = 2;
  static constexpr std::size_t kWidth = 3;

 private:
  [[nodiscard]] Vec2f half_size() const;

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Tilt",
       .unit = "deg",
       .min = 5,
       .max = 40,
       .step = 5,
       .scale = 1,
       .value = 20},
      {.name = "Restitution",
       .unit = "",
       .min = 0,
       .max = 100,
       .step = 10,
       .scale = 100,
       .value = 50},
      {.name = "Speed",
       .unit = "m/s",
       .min = 50,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 200},
      {.name = "Width",
       .unit = "m",
       .min = 100,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 200},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Lever arm", .unit = "m", .scale = 1000},
      {.name = "Predicted spin", .unit = "rad/s", .scale = 1000},
      {.name = "Measured spin", .unit = "rad/s", .scale = 1000},
      {.name = "Predicted vy", .unit = "m/s", .scale = 1000},
      {.name = "Measured vy", .unit = "m/s", .scale = 1000},
      {.name = "Corner rebound", .unit = "", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  // Set by the impact step
  std::optional<Scalar> lever_arm_;
  std::optional<Scalar> approach_velocity_;
  std::optional<Scalar> measured_velocity_;
  std::optional<Scalar> measured_spin_;
  Vec2f impact_center_{};
  Vec2f impact_point_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_04_OFF_CENTER_IMPACT_OFF_CENTER_IMPACT_SAMPLE_H_
