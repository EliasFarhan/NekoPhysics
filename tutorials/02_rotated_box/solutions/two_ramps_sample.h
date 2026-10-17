#ifndef TUTORIALS_02_ROTATED_BOX_SOLUTIONS_TWO_RAMPS_SAMPLE_H_
#define TUTORIALS_02_ROTATED_BOX_SOLUTIONS_TWO_RAMPS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 02, exercise "two ramps": two reflections in a row, and the flight
// between them.
//
// Two static ramps face each other: ramp A on the right is turned by `a`
// (its face normal n_A = (-sin a, cos a) points up and to the left), ramp B
// on the left, half a meter higher, by -a (n_B = (sin a, cos a)). The ball
// falls from rest 2 m above A's center, bounces toward B, flies under gravity
// and bounces again. With restitution
// `e`, w the velocity at an impact and p1 the ball center after the first
// bounce step:
//
//   first bounce      w1' = w1 - (1 + e) (w1 . n_A) n_A
//   flight            p(t) = p1 + w1' t + g t^2 / 2
//   second contact    n_B . (p(t) - c_B) = h_B + R  (the center is one radius
//                     above B's top face; c_B is B's center, h_B its half
//                     height): a quadratic in t, its first root is the
//                     flight time
//   second bounce     w2' = w2 - (1 + e) (w2 . n_B) n_B
//
// The world finds the second contact at the first step whose start position
// is within R of the face, so the measured flight time (steps since the
// first bounce times dt) differs from the root by at most a step or so.
//
// Drawn: both ramps in grey with their normals in white, the ball in orange
// with its velocity in sky blue, the predicted contact point on B in green.
class TwoRampsSample final : public Sample {
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

  // w1 - (1 + e) (w1 . n_A) n_A; empty until the first bounce
  [[nodiscard]] std::optional<Vec2f> predicted_bounce() const {
    return predicted_bounce_;
  }
  // The ball's velocity right after the first bounce step
  [[nodiscard]] std::optional<Vec2f> measured_bounce() const {
    return measured_bounce_;
  }
  // First root of the second contact's quadratic, in seconds
  [[nodiscard]] std::optional<Scalar> predicted_flight() const {
    return predicted_flight_;
  }
  // Steps from the first bounce to the second contact, times dt
  [[nodiscard]] std::optional<Scalar> measured_flight() const {
    return measured_flight_;
  }
  // Direction of the velocity change of the second bounce step, gravity
  // removed
  [[nodiscard]] std::optional<Vec2f> second_normal() const {
    return second_normal_;
  }
  // -(w2' . n_B) / (w2 . n_B)
  [[nodiscard]] std::optional<Scalar> second_ratio() const {
    return second_ratio_;
  }
  // B's face normal, (sin a, cos a)
  [[nodiscard]] Vec2f normal_b() const;
  [[nodiscard]] Scalar restitution() const {
    return parameters_[kRestitution].ToScalar();
  }

  static constexpr std::size_t kRampAngle = 0;
  static constexpr std::size_t kRestitution = 1;

 private:
  [[nodiscard]] Vec2f normal_a() const;
  [[nodiscard]] bool Touches(ColliderIndex collider) const;
  void PredictFlight(Vec2f start, Vec2f velocity);

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 3,
                       .max_colliders = 3,
                       .max_touching_pairs = 2,
                       .max_broadphase_pairs = 3}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Ramp angle",
       .unit = "deg",
       .min = 15,
       .max = 30,
       .step = 5,
       .scale = 1,
       .value = 20},
      {.name = "Restitution",
       .unit = "",
       .min = 50,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 75},
  }};
  std::array<SampleReadout, 9> readouts_{{
      {.name = "Bounce x", .unit = "m/s", .scale = 100},
      {.name = "Bounce y", .unit = "m/s", .scale = 100},
      {.name = "Measured x", .unit = "m/s", .scale = 100},
      {.name = "Measured y", .unit = "m/s", .scale = 100},
      {.name = "Flight", .unit = "s", .scale = 1000},
      {.name = "Measured flight", .unit = "s", .scale = 1000},
      {.name = "Normal 2 x", .unit = "", .scale = 1000},
      {.name = "Normal 2 y", .unit = "", .scale = 1000},
      {.name = "Ratio 2", .unit = "", .scale = 1000},
  }};
  BodyIndex ramp_a_index_ = kInvalidBodyIndex;
  BodyIndex ramp_b_index_ = kInvalidBodyIndex;
  BodyIndex ball_index_ = kInvalidBodyIndex;
  ColliderIndex ramp_a_collider_ = kInvalidColliderIndex;
  ColliderIndex ramp_b_collider_ = kInvalidColliderIndex;
  int steps_since_bounce_ = 0;
  std::optional<Vec2f> predicted_bounce_;
  std::optional<Vec2f> measured_bounce_;
  std::optional<Scalar> predicted_flight_;
  std::optional<Scalar> measured_flight_;
  std::optional<Vec2f> predicted_contact_;
  std::optional<Vec2f> second_normal_;
  std::optional<Scalar> second_ratio_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_02_ROTATED_BOX_SOLUTIONS_TWO_RAMPS_SAMPLE_H_
