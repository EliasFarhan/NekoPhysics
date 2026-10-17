#ifndef TUTORIALS_01_ROTATION_SOLUTIONS_DISK_SAMPLE_H_
#define TUTORIALS_01_ROTATION_SOLUTIONS_DISK_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 01, exercise "disk": spin a disk of the same mass as the box, and
// find the radius that spins like the 2 x 1 m box.
//
// The couple of chapter 01 (two forces of `F` newtons at `r` meters on each
// side of the center, turning with the body) spins a solid disk of mass `m`
// and radius `R` for `T` seconds, without gravity:
//
//   inertia            I = m R^2 / 2
//   spin after T       omega = 2 r F T / I
//   same spin as box   m R^2 / 2 = m (w^2 + h^2) / 12,
//                      so R = sqrt((w^2 + h^2) / 6), 0.913 m for 2 x 1 m
//
// The mass doesn't change the matching radius: both inertias are the mass
// times a length squared. Readouts compare the disk's spin with the box's
// predicted spin for the same push.
//
// Drawn: the disk in orange, its orientation in white, the lever arms in grey
// and the forces in red while they push.
class DiskSample final : public Sample {
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

  // m R^2 / 2, in kg m^2
  [[nodiscard]] Scalar predicted_inertia() const;
  // 1 / inverse inertia of the disk as the world computes it, in kg m^2
  [[nodiscard]] Scalar measured_inertia() const;
  [[nodiscard]] int push_steps() const { return push_steps_; }
  // 2 r F T / (m R^2 / 2), in rad/s
  [[nodiscard]] Scalar predicted_spin() const;
  // 2 r F T / (m (w^2 + h^2) / 12) for the 2 x 1 m box, in rad/s
  [[nodiscard]] Scalar box_spin() const;
  // sqrt((w^2 + h^2) / 6) for the 2 x 1 m box, in meters
  [[nodiscard]] static Scalar matching_radius();
  [[nodiscard]] const Body& disk() const { return world_.body(disk_index_); }

  static constexpr std::size_t kForce = 0;
  static constexpr std::size_t kLeverArm = 1;
  static constexpr std::size_t kMass = 2;
  static constexpr std::size_t kRadius = 3;
  static constexpr std::size_t kPushTime = 4;

 private:
  [[nodiscard]] Scalar push_time() const;
  [[nodiscard]] Scalar torque() const;
  [[nodiscard]] bool is_pushing() const { return step_ < push_steps_; }

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 1,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 5> parameters_{{
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
      {.name = "Radius",
       .unit = "m",
       .min = 25,
       .max = 150,
       .step = 1,
       .scale = 100,
       .value = 50},
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
      {.name = "World inertia", .unit = "kg m2", .scale = 1000},
      {.name = "Predicted spin", .unit = "rad/s", .scale = 100},
      {.name = "Spin", .unit = "rad/s", .scale = 100},
      {.name = "Box spin", .unit = "rad/s", .scale = 100},
      {.name = "Matching radius", .unit = "m", .scale = 1000},
  }};
  BodyIndex disk_index_ = kInvalidBodyIndex;
  int push_steps_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_01_ROTATION_SOLUTIONS_DISK_SAMPLE_H_
