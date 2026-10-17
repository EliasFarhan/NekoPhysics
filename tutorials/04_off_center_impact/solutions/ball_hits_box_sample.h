#ifndef TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_BALL_HITS_BOX_SAMPLE_H_
#define TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_BALL_HITS_BOX_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 04, exercise "ball hits box": an impact between two moving bodies.
//
// No gravity. A free 2 x 1 m box of 1 kg rests, unturned. A ball of radius
// 0.25 m and mass `m` (parameter "Ball mass") comes down at `v` m/s
// (parameter "Speed") and hits the box's top face `x` meters right of its
// center (parameter "Offset"), with restitution `e` (parameter
// "Restitution"); no friction. The contact point is on the top face, at
// r = (x, 0.5) from the box's center and straight below the ball's center,
// so only the box gets a lever arm:
//
//   effective inv. mass  k = 1/M + 1/m + x^2 / I      (r x n = -x, n down)
//   impulse              j = (1 + e) v / k
//   ball                 v_ball' = -v + j / m
//   box                  v_box' = -j / M,  w' = -x j / I
//
// and the contact point's relative speed along the normal is reversed and
// scaled by e (the parameters keep the bounce faster than the solver's
// push-out of the overlap). Drawn: the box in orange with its velocity in sky
// blue, the ball in pink; after the hit, the box's lever arm in grey and the
// impulse on the box in red.
class BallHitsBoxSample final : public Sample {
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

  // m (w^2 + h^2) / 12 of the box, in kg m^2
  [[nodiscard]] Scalar box_inertia() const;
  [[nodiscard]] Scalar predicted_impulse() const;
  [[nodiscard]] Scalar predicted_ball_velocity() const;
  [[nodiscard]] Scalar predicted_box_velocity() const;
  [[nodiscard]] Scalar predicted_box_spin() const;
  // The values below are empty until the ball hit the box.
  [[nodiscard]] std::optional<Scalar> measured_ball_velocity() const {
    return ball_velocity_;
  }
  [[nodiscard]] std::optional<Scalar> measured_box_velocity() const {
    return box_velocity_;
  }
  [[nodiscard]] std::optional<Scalar> measured_box_spin() const {
    return box_spin_;
  }
  // The box's horizontal velocity after the hit (the impulse is vertical)
  [[nodiscard]] std::optional<Scalar> measured_box_drift() const {
    return box_drift_;
  }
  // Relative speed of the contact point after / before, predicted to be e
  [[nodiscard]] std::optional<Scalar> point_rebound() const;

  static constexpr std::size_t kOffset = 0;
  static constexpr std::size_t kBallMass = 1;
  static constexpr std::size_t kRestitution = 2;
  static constexpr std::size_t kSpeed = 3;

 private:
  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Offset",
       .unit = "m",
       .min = -90,
       .max = 90,
       .step = 10,
       .scale = 100,
       .value = 60},
      {.name = "Ball mass",
       .unit = "kg",
       .min = 25,
       .max = 200,
       .step = 25,
       .scale = 100,
       .value = 50},
      {.name = "Restitution",
       .unit = "",
       .min = 20,
       .max = 100,
       .step = 10,
       .scale = 100,
       .value = 50},
      {.name = "Speed",
       .unit = "m/s",
       .min = 200,
       .max = 400,
       .step = 50,
       .scale = 100,
       .value = 300},
  }};
  std::array<SampleReadout, 8> readouts_{{
      {.name = "Impulse", .unit = "N s", .scale = 1000},
      {.name = "Ball vy", .unit = "m/s", .scale = 1000},
      {.name = "Measured", .unit = "m/s", .scale = 1000},
      {.name = "Box vy", .unit = "m/s", .scale = 1000},
      {.name = "Measured", .unit = "m/s", .scale = 1000},
      {.name = "Box spin", .unit = "rad/s", .scale = 1000},
      {.name = "Measured", .unit = "rad/s", .scale = 1000},
      {.name = "Point rebound", .unit = "", .scale = 1000},
  }};
  BodyIndex box_index_ = kInvalidBodyIndex;
  BodyIndex ball_index_ = kInvalidBodyIndex;
  // Set by the impact step
  std::optional<Scalar> ball_velocity_;
  std::optional<Scalar> box_velocity_;
  std::optional<Scalar> box_spin_;
  std::optional<Scalar> box_drift_;
  Vec2f impact_center_{};
  Vec2f impact_point_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_BALL_HITS_BOX_SAMPLE_H_
