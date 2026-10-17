#ifndef TUTORIALS_01_ROTATION_SOLUTIONS_SINGLE_FORCE_SAMPLE_H_
#define TUTORIALS_01_ROTATION_SOLUTIONS_SINGLE_FORCE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 01, exercise "single force": push the box with one force instead of
// a couple, and predict both its motion and its spin.
//
// The 2 x 1 m box of chapter 01 floats without gravity. For `T` seconds one
// force of `F` newtons along the world y axis pushes it at `r` meters to the
// right of its center. The push point follows the center but doesn't turn
// with the box, so the lever arm stays (r, 0) and the torque stays r F:
//
//   center velocity after T   v = F T / m            (along y)
//   spin after T              omega = r F T / I      (half the couple's)
//   inertia                   I = m (w^2 + h^2) / 12
//
// A force applied away from the center does two things at once: all of it
// accelerates the center (as if it were applied there), and its torque about
// the center spins the body. The couple of chapter 01 is two such forces
// whose pushes on the center cancel out.
//
// Drawn: the box outline in orange, its orientation in white, its velocity in
// sky blue, the lever arm in grey and the force in red while it pushes.
class SingleForceSample final : public Sample {
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
  // Push duration: push_steps() steps of dt, in seconds
  [[nodiscard]] Scalar push_time() const;
  [[nodiscard]] int push_steps() const { return push_steps_; }
  // F T / m, in m/s
  [[nodiscard]] Scalar predicted_speed() const;
  // r F T / I, in rad/s
  [[nodiscard]] Scalar predicted_spin() const;
  [[nodiscard]] const Body& box() const { return world_.body(box_index_); }

  static constexpr std::size_t kForce = 0;
  static constexpr std::size_t kLeverArm = 1;
  static constexpr std::size_t kMass = 2;
  static constexpr std::size_t kPushTime = 3;

 private:
  [[nodiscard]] bool is_pushing() const { return step_ < push_steps_; }

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 1,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Force",
       .unit = "N",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 10,
       .value = 20},
      {.name = "Lever arm",
       .unit = "m",
       .min = 0,
       .max = 100,
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
      {.name = "Push time",
       .unit = "s",
       .min = 25,
       .max = 100,
       .step = 25,
       .scale = 100,
       .value = 100},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Predicted speed", .unit = "m/s", .scale = 100},
      {.name = "Speed", .unit = "m/s", .scale = 100},
      {.name = "Predicted spin", .unit = "rad/s", .scale = 100},
      {.name = "Spin", .unit = "rad/s", .scale = 100},
  }};
  BodyIndex box_index_ = kInvalidBodyIndex;
  int push_steps_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_01_ROTATION_SOLUTIONS_SINGLE_FORCE_SAMPLE_H_
