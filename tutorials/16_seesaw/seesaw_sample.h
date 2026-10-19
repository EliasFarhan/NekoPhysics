#ifndef TUTORIALS_16_SEESAW_SEESAW_SAMPLE_H_
#define TUTORIALS_16_SEESAW_SEESAW_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko::tutorial {

// A static body at `position`, the axle of a hinge
BodyIndex AddAxle(PhysicsWorld& world, Vec2f position);

// A dynamic body with a solid oriented box collider: a plank, a pan, a crate.
// Its inertia comes from the collider (a box of mass m turns with m (w^2 +
// h^2) / 12).
BodyIndex AddPlank(PhysicsWorld& world, Vec2f center, Vec2f half_size,
                   Scalar inverse_mass, Scalar friction);

// Sets the limits of a revolute joint to plus or minus `limit`
void SetSymmetricLimit(PhysicsWorld& world, RevoluteJointIndex joint,
                       Radian limit);

// Chapter 16, seesaw: revolute joints with angle limits.
//
// A LuchaSteps-style balance (Splash): an 8 kg plank 8 m long turns about a
// static axle at its middle, a revolute joint limited to plus or minus
// "Limit" degrees. Two 4 kg pans hang 1.6 m under its ends, each from a
// revolute joint limited to 45 degrees relative to the plank. Crates of 1 kg
// are loaded into the pans, alternately left and right, "Left" and "Right"
// of them, one every 40 steps. A game-side damper on each hinge (a torque
// against the relative spin, "Damping" N m s) stands for the friction of a
// real axle: without it the pans swing for minutes.
//
// With M_L and M_R the masses in the left and the right pan, the weights
// turn the plank about the axle with the torque
//
//   tau = g (M_L - M_R) a cos(theta)
//
// (a = 4 m, theta the plank angle: the pans hang from the ends, their
// weights act at a cos(theta) from the axle). It is the same at any angle up
// to cos(theta): the heavier side goes down until the limit stops it, and
// the limit then carries tau ("Limit torque", from its accumulated impulse
// over dt). Equal crates leave the plank where it is.
//
// The axle carries the weight of everything, g (8 + 8 + crates) N ("Axle
// force", from the point impulse over dt), and a pin is soft: its anchors
// part by about the force over (2 pi 15 Hz)^2 per kilogram of effective
// mass ("Max pin gap").
//
// Drawn: the axle and the hinges in grey, the plank in white, the pans in
// sky blue with their hangers, the crates in orange, the two limits as grey
// lines from the axle.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class SeesawSample final : public Sample {
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

  // The plank's joint angle, counterclockwise (the left end down) positive
  [[nodiscard]] Radian plank_angle() const {
    return world_.RevoluteJointAngle(axle_joint_);
  }
  // g (M_L - M_R) a cos(theta) of the crates dropped so far, in N m
  [[nodiscard]] Scalar predicted_torque() const;
  // The torque the limits hold the plank with, (upper - lower) impulse over
  // dt, in N m: positive when the upper limit holds the left side up
  [[nodiscard]] Scalar limit_torque() const;
  // The force on the axle, |impulse| / dt, in N
  [[nodiscard]] Scalar axle_force() const;
  // g times the mass of the plank, the pans and the crates dropped, in N
  [[nodiscard]] Scalar weight() const;
  // Largest distance between the two anchors of any hinge, in meters
  [[nodiscard]] Scalar max_pin_gap() const { return max_pin_gap_; }
  [[nodiscard]] int dropped(int side) const {
    return dropped_[static_cast<std::size_t>(side)];
  }
  // Steps after which every crate has dropped and had time to settle
  [[nodiscard]] int settled_steps() const;

  static constexpr std::size_t kLimit = 0;
  static constexpr std::size_t kLeft = 1;
  static constexpr std::size_t kRight = 2;
  static constexpr std::size_t kDamping = 3;
  static constexpr int kDropPeriod = 40;
  static constexpr int kMaxCrates = 5;

 private:
  void DropCrate(int side);
  // The game-side damper of a hinge: a torque against the relative spin
  void DampHinge(RevoluteJointIndex joint);

  PhysicsWorld world_{
      {.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
       .dt = kFixedDeltaTime,
       .max_bodies = 4 + 2 * kMaxCrates,
       .max_colliders = 1 + 3 * 2 + 2 * kMaxCrates,
       .max_touching_pairs = 64,
       .max_broadphase_pairs = 128,
       .max_joints = 0,
       .max_revolute_joints = 3,
       .velocity_iterations = sample_config::kVelocityIterations}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Limit",
       .unit = "deg",
       .min = 5,
       .max = 45,
       .step = 5,
       .scale = 1,
       .value = 30},
      {.name = "Left",
       .unit = "crates",
       .min = 0,
       .max = kMaxCrates,
       .step = 1,
       .scale = 1,
       .value = 3},
      {.name = "Right",
       .unit = "crates",
       .min = 0,
       .max = kMaxCrates,
       .step = 1,
       .scale = 1,
       .value = 1},
      {.name = "Damping",
       .unit = "N m s",
       .min = 0,
       .max = 40,
       .step = 5,
       .scale = 1,
       .value = 20},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Plank angle", .unit = "deg", .scale = 10},
      {.name = "Load torque", .unit = "N m", .scale = 10},
      {.name = "Limit torque", .unit = "N m", .scale = 10},
      {.name = "Axle force", .unit = "N", .scale = 10},
      {.name = "Weight", .unit = "N", .scale = 10},
      {.name = "Max pin gap", .unit = "mm", .scale = 10},
  }};
  BodyIndex axle_ = kInvalidBodyIndex;
  BodyIndex plank_ = kInvalidBodyIndex;
  std::array<BodyIndex, 2> pans_{kInvalidBodyIndex, kInvalidBodyIndex};
  RevoluteJointIndex axle_joint_ = kInvalidRevoluteJointIndex;
  std::array<RevoluteJointIndex, 2> pan_joints_{kInvalidRevoluteJointIndex,
                                                kInvalidRevoluteJointIndex};
  std::array<BodyIndex, 2 * kMaxCrates> crates_{};
  int crate_count_ = 0;
  std::array<int, 2> dropped_{};
  int next_side_ = 0;
  int step_ = 0;
  Scalar max_pin_gap_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_16_SEESAW_SEESAW_SAMPLE_H_
