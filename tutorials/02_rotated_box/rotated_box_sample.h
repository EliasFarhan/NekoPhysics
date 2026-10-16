#ifndef TUTORIALS_02_ROTATED_BOX_ROTATED_BOX_SAMPLE_H_
#define TUTORIALS_02_ROTATED_BOX_ROTATED_BOX_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 02, collide with a rotated box: drop a ball on a tilted ramp and
// measure the contact normal.
//
// The ramp is a static oriented box turned by `a` degrees (parameter "Ramp
// angle"); its top face points along its local y axis, the normal
// n = (-sin a, cos a). A ball falls from rest (parameter "Drop height", from
// the ramp's center to the ball's center) and bounces with restitution `e`
// (parameter "Restitution"). The separating axis test finds the contact: the
// ramp's face normal is the axis of smallest overlap, and the contact
// impulse pushes the ball along it. With w the ball's velocity when it hits
// (the velocity before the contact step plus one step of gravity):
//
//   impulse direction     n                          (frictionless ramp)
//   bounce velocity       w' = w - (1 + e) (w . n) n (reflection about the
//                                                     face, scaled by e)
//   rebound ratio         -(w' . n) / (w . n) = e
//
// The sample measures the velocity change of the bounce step and shows the
// predicted and measured normal and rebound ratio. Drawn: the ramp outline
// in grey with its normal in white, the ball in orange with its velocity in
// sky blue; after the bounce, the predicted bounce direction in green and
// the measured one in yellow.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class RotatedBoxSample final : public Sample {
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

  // The ramp's face normal, its local y axis in the world
  [[nodiscard]] Vec2f predicted_normal() const;
  // Direction of the velocity change of the bounce step, gravity removed;
  // empty until the ball bounced
  [[nodiscard]] std::optional<Vec2f> measured_normal() const {
    return measured_normal_;
  }
  // w - (1 + e) (w . n) n with the measured impact velocity w; empty until
  // the ball bounced
  [[nodiscard]] std::optional<Vec2f> predicted_bounce() const;
  // The ball's velocity right after the bounce step
  [[nodiscard]] std::optional<Vec2f> measured_bounce() const {
    return bounce_velocity_;
  }
  // -(w' . n) / (w . n), predicted to be the restitution
  [[nodiscard]] std::optional<Scalar> rebound_ratio() const;
  [[nodiscard]] Scalar restitution() const {
    return parameters_[kRestitution].ToScalar();
  }

  static constexpr std::size_t kRampAngle = 0;
  static constexpr std::size_t kRestitution = 1;
  static constexpr std::size_t kDropHeight = 2;

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
       .min = -45,
       .max = 45,
       .step = 5,
       .scale = 1,
       .value = 20},
      {.name = "Restitution",
       .unit = "",
       .min = 0,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 75},
      {.name = "Drop height",
       .unit = "m",
       .min = 100,
       .max = 400,
       .step = 25,
       .scale = 100,
       .value = 250},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Normal x", .unit = "", .scale = 1000},
      {.name = "Normal y", .unit = "", .scale = 1000},
      {.name = "Measured x", .unit = "", .scale = 1000},
      {.name = "Measured y", .unit = "", .scale = 1000},
      {.name = "Rebound ratio", .unit = "", .scale = 1000},
  }};
  BodyIndex ramp_index_ = kInvalidBodyIndex;
  BodyIndex ball_index_ = kInvalidBodyIndex;
  // Ball velocity at the impact (before the bounce step plus one step of
  // gravity), right after the bounce step, and where it bounced
  std::optional<Vec2f> impact_velocity_;
  std::optional<Vec2f> bounce_velocity_;
  std::optional<Vec2f> measured_normal_;
  Vec2f bounce_position_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_02_ROTATED_BOX_ROTATED_BOX_SAMPLE_H_
