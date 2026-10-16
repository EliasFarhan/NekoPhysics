#ifndef TUTORIALS_01_ROTATION_ROTATION_SAMPLE_H_
#define TUTORIALS_01_ROTATION_ROTATION_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 01, make a body rotate: spin a box with a couple, measure its spin.
//
// A box of mass `m`, width `w` and height `h` floats without gravity. For
// `T` seconds (parameter "Push time") two opposite forces of `F` newtons push
// it at `r` meters on each side of its center (parameters "Force" and "Lever
// arm"), perpendicular to the lever arm and turning with the box. Such a
// couple cancels out on the center of mass, which stays put, but its two
// torques add up:
//
//   torque               tau = 2 r F            (Cross(lever arm, force) x 2)
//   inertia              I = m (w^2 + h^2) / 12
//   angular acceleration alpha = tau / I
//   spin after T         omega = alpha T
//   angle turned in T    theta = alpha T^2 / 2
//
// After the push the box keeps turning at omega: nothing slows it down.
// Readouts compare the predictions with the world. The measured angle is
// a little ahead of alpha T^2 / 2: semi-implicit Euler updates the angular
// velocity before the angle, so it turns by alpha dt^2 (1 + 2 + ... + N)
// = alpha T^2 / 2 + alpha T dt / 2 after N = T / dt steps.
//
// Drawn: the box outline in orange, its orientation (local x axis) in white,
// the lever arms in grey and the forces in red while they push. After the
// push, the direction predicted at the end of the push is green and the
// measured one yellow.
//
// The world is y-up, in meters, seconds, kilograms and radians, and has no
// gravity.
class RotationSample final : public Sample {
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

  // m (w^2 + h^2) / 12, in kg m^2
  [[nodiscard]] Scalar predicted_inertia() const;
  // 1 / inverse inertia of the box as the world computes it, in kg m^2
  [[nodiscard]] Scalar measured_inertia() const;
  // 2 r F, in N m
  [[nodiscard]] Scalar torque() const;
  // Push duration: push_steps() steps of dt, in seconds
  [[nodiscard]] Scalar push_time() const;
  [[nodiscard]] int push_steps() const { return push_steps_; }
  // alpha T, in rad/s
  [[nodiscard]] Scalar predicted_spin() const;
  // alpha T^2 / 2, in radians
  [[nodiscard]] Scalar predicted_turn() const;
  // The box's angular velocity now, in rad/s
  [[nodiscard]] Scalar spin() const;
  // Angle turned during the push, in radians; empty until the push is over.
  // Summed from the wrapped angle step by step, so it is only right while
  // the box turns by less than half a turn per step (under pi / dt, about
  // 188 rad/s)
  [[nodiscard]] std::optional<Scalar> measured_turn() const {
    return measured_turn_;
  }
  [[nodiscard]] const Body& box() const { return world_.body(box_index_); }

  static constexpr std::size_t kForce = 0;
  static constexpr std::size_t kLeverArm = 1;
  static constexpr std::size_t kMass = 2;
  static constexpr std::size_t kWidth = 3;
  static constexpr std::size_t kHeight = 4;
  static constexpr std::size_t kPushTime = 5;

 private:
  [[nodiscard]] Vec2f half_size() const;
  [[nodiscard]] bool is_pushing() const { return step_ < push_steps_; }

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 1,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 6> parameters_{{
      {.name = "Force",
       .unit = "N",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 10,
       .value = 20},
      {.name = "Lever arm",
       .unit = "m",
       .min = 10,
       .max = 150,
       .step = 10,
       .scale = 100,
       .value = 50},
      {.name = "Mass",
       .unit = "kg",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 10,
       .value = 10},
      {.name = "Width",
       .unit = "m",
       .min = 25,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 200},
      {.name = "Height",
       .unit = "m",
       .min = 25,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 100},
      {.name = "Push time",
       .unit = "s",
       .min = 25,
       .max = 200,
       .step = 25,
       .scale = 100,
       .value = 100},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Inertia", .unit = "kg m2", .scale = 1000},
      {.name = "Torque", .unit = "N m", .scale = 100},
      {.name = "Predicted spin", .unit = "rad/s", .scale = 100},
      {.name = "Spin", .unit = "rad/s", .scale = 100},
      {.name = "Predicted turn", .unit = "rad", .scale = 100},
      {.name = "Measured turn", .unit = "rad", .scale = 100},
  }};
  BodyIndex box_index_ = kInvalidBodyIndex;
  int push_steps_ = 0;
  int step_ = 0;
  // Angle turned since Begin, summed from the wrapped angle of each step
  Scalar turned_{};
  Scalar previous_angle_{};
  std::optional<Scalar> measured_turn_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_01_ROTATION_ROTATION_SAMPLE_H_
