#ifndef TUTORIALS_01_ROTATION_SOLUTIONS_DUMBBELL_SAMPLE_H_
#define TUTORIALS_01_ROTATION_SOLUTIONS_DUMBBELL_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 01, exercise "dumbbell": one body made of two boxes, and the
// parallel axis theorem.
//
// A body of mass `m` has two square box colliders of side `s`, at `d` meters
// on each side of its center (offsets (-d, 0) and (d, 0)). The world spreads
// the mass over the colliders by area, so each box carries m / 2. A box's
// inertia about the body's center is its inertia about its own center plus
// its mass times the squared distance between the two centers (the parallel
// axis theorem):
//
//   one box     (m / 2) (s^2 + s^2) / 12 + (m / 2) d^2
//   both        I = m (s^2 / 6 + d^2)
//   spin        omega = 2 r F T / I     (the couple of chapter 01)
//
// Moving the boxes apart raises the inertia with d^2, so the same push spins
// the dumbbell more slowly. The colliders are axis-aligned boxes, as in
// chapter 01: for the inertia the world counts them as boxes of the same
// size, and their offsets turn with the body.
//
// Drawn: both boxes in orange, the bar between them in grey, the orientation
// in white, the lever arms in grey and the forces in red while they push.
class DumbbellSample final : public Sample {
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

  // m (s^2 / 6 + d^2), in kg m^2
  [[nodiscard]] Scalar predicted_inertia() const;
  // m d^2, the part of the inertia that comes from the offsets
  [[nodiscard]] Scalar offset_inertia() const;
  // 1 / inverse inertia of the body as the world computes it, in kg m^2
  [[nodiscard]] Scalar measured_inertia() const;
  [[nodiscard]] int push_steps() const { return push_steps_; }
  // 2 r F T / I, in rad/s
  [[nodiscard]] Scalar predicted_spin() const;
  [[nodiscard]] const Body& dumbbell() const {
    return world_.body(body_index_);
  }

  static constexpr std::size_t kForce = 0;
  static constexpr std::size_t kLeverArm = 1;
  static constexpr std::size_t kMass = 2;
  static constexpr std::size_t kSide = 3;
  static constexpr std::size_t kOffset = 4;
  static constexpr std::size_t kPushTime = 5;

 private:
  [[nodiscard]] Scalar push_time() const;
  [[nodiscard]] bool is_pushing() const { return step_ < push_steps_; }

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 1,
                       .max_colliders = 2,
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
      {.name = "Box side",
       .unit = "m",
       .min = 25,
       .max = 100,
       .step = 25,
       .scale = 100,
       .value = 50},
      {.name = "Offset",
       .unit = "m",
       .min = 50,
       .max = 200,
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
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Inertia", .unit = "kg m2", .scale = 1000},
      {.name = "Of which m d2", .unit = "kg m2", .scale = 1000},
      {.name = "World inertia", .unit = "kg m2", .scale = 1000},
      {.name = "Predicted spin", .unit = "rad/s", .scale = 100},
      {.name = "Spin", .unit = "rad/s", .scale = 100},
  }};
  BodyIndex body_index_ = kInvalidBodyIndex;
  int push_steps_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_01_ROTATION_SOLUTIONS_DUMBBELL_SAMPLE_H_
