#ifndef TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_POINTS_OVER_A_TURN_SAMPLE_H_
#define TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_POINTS_OVER_A_TURN_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 03, exercise "points over a turn": the box of chapter 03 spins
// through a whole turn, sunk `d` meters (parameter "Sink depth"); for which
// fraction of the turn does the manifold have two points?
//
// The incident face is the box face most opposed to the ground's normal, at
// most 45 degrees from flat: the bottom or top face (half length hx = half
// the width, half thickness hy = 0.5 m) for half of the turn, a side face
// (half length hy, half thickness hx) for the other half. A face tilted by
// phi, whose center is c = 0.5 - d above the ground's top when flat, has its
// higher corner at c + L |sin phi| - N cos phi, with L its half length and N
// the half thickness behind it. Both corners are below the top while
//
//   c + L sin phi - N cos phi <= 0,   phi from 0 up to phi*
//
// so each face type contributes 4 phi* degrees per turn (two faces, each
// tilted from -phi* to +phi*). The sample finds phi* for both face types by
// bisection, predicts the fraction (4 phi*_bottom + 4 phi*_side) / 360 and
// counts the steps of one turn that have two points. The box spins at 60
// degrees per second (parameter "Spin"): one turn is 360 steps.
//
// Drawn: the ground in dark grey, the box in orange, the manifold points in
// yellow (green while there are two).
class PointsOverATurnSample final : public Sample {
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

  // phi*, in degrees, for the bottom and top faces and for the side faces:
  // 0 when the face never has both corners down, 45 when it always does
  [[nodiscard]] Scalar bottom_threshold() const { return bottom_threshold_; }
  [[nodiscard]] Scalar side_threshold() const { return side_threshold_; }
  // Predicted number of steps of one turn with two points
  [[nodiscard]] Scalar predicted_two_point_steps() const;
  [[nodiscard]] int steps_per_turn() const;
  // Counted over the first turn
  [[nodiscard]] int two_point_steps() const { return two_point_steps_; }
  [[nodiscard]] int one_point_steps() const { return one_point_steps_; }
  [[nodiscard]] int no_point_steps() const { return no_point_steps_; }
  [[nodiscard]] bool turn_done() const { return steps_ >= steps_per_turn(); }

  static constexpr std::size_t kSinkDepth = 0;
  static constexpr std::size_t kSpin = 1;

 private:
  void UpdateContact();
  [[nodiscard]] Scalar ground_top() const;

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Sink depth",
       .unit = "m",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 100,
       .value = 30},
      {.name = "Spin",
       .unit = "deg/s",
       .min = 30,
       .max = 90,
       .step = 30,
       .scale = 1,
       .value = 60},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Bottom phi*", .unit = "deg", .scale = 100},
      {.name = "Side phi*", .unit = "deg", .scale = 100},
      {.name = "Two points", .unit = "steps", .scale = 10},
      {.name = "Measured", .unit = "steps", .scale = 1},
      {.name = "One point", .unit = "steps", .scale = 1},
      {.name = "No point", .unit = "steps", .scale = 1},
      {.name = "Steps per turn", .unit = "", .scale = 1},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  ColliderIndex ground_collider_{};
  ColliderIndex box_collider_{};
  Contact contact_{};
  Scalar bottom_threshold_{};
  Scalar side_threshold_{};
  int steps_ = 0;
  int two_point_steps_ = 0;
  int one_point_steps_ = 0;
  int no_point_steps_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_POINTS_OVER_A_TURN_SAMPLE_H_
