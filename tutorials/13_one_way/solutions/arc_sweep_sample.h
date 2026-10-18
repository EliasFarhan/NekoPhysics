#ifndef TUTORIALS_13_ONE_WAY_SOLUTIONS_ARC_SWEEP_SAMPLE_H_
#define TUTORIALS_13_ONE_WAY_SOLUTIONS_ARC_SWEEP_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 13, exercise "arc sweep": how steep a slope still counts as a
// floor.
//
// Seven tilted one-way planks (OrientedBoxCollider) per row, tilted by 50,
// 60, 70, 74, 76, 80 and 86 degrees. Their one-way normal is kept straight up
// in the world (the local normal is turned back by the tilt), like a
// platform effector whose direction doesn't follow the sprite. A ball is
// thrown at the middle of each plank's top face, along the face's normal
// and without gravity, so the contact normal is the face normal: it is the
// tilt away from the one-way normal. The top row uses the one-way arc
// parameter "Arc A" (150 degrees by default, Splash's value), the bottom
// row "Arc B" (180, a half plane).
//
// A ball lands when the tilt is strictly below half the arc (with the
// scalar's rounding tolerance), and passes through otherwise: with 150
// degrees the 76 degree plank is no floor any more, with 180 every one of
// these planks is.
//
// Readouts: for each row, the steepest plank a ball landed on and the
// predicted one (the steepest listed tilt below half the arc). Drawn: the
// planks in teal with their one-way normals in yellow, the balls in green
// once landed, red once passing, white before.
//
// The world is y-up, in meters and seconds, without gravity.
class ArcSweepSample final : public Sample {
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

  static constexpr std::size_t kTiltCount = 7;
  static constexpr std::array<int, kTiltCount> kTilts = {50, 60, 70, 74,
                                                         76, 80, 86};
  // 1 landed, -1 passed through, 0 not decided yet; row 0 is arc A
  [[nodiscard]] int outcome(std::size_t row, std::size_t tilt) const {
    return outcomes_[row][tilt];
  }
  // The steepest tilt a ball landed on, -1 when none did
  [[nodiscard]] int max_landed_tilt(std::size_t row) const;
  // The steepest listed tilt below half the arc, -1 when none is
  [[nodiscard]] int predicted_max_tilt(std::size_t row) const;
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kArcA = 0;
  static constexpr std::size_t kArcB = 1;
  // The balls reach the planks in half a second
  static constexpr int kMeasuredSteps = 60;

 private:
  struct Shot {
    ColliderIndex plank = kInvalidColliderIndex;
    ColliderIndex ball = kInvalidColliderIndex;
  };

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2 * 2 * kTiltCount,
                       .max_colliders = 2 * 2 * kTiltCount,
                       .max_touching_pairs = 32,
                       .max_broadphase_pairs = 64,
                       .max_joints = 0,
                       .max_passing_pairs = 2 * kTiltCount}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Arc A",
       .unit = "deg",
       .min = 90,
       .max = 180,
       .step = 2,
       .scale = 1,
       .value = 150},
      {.name = "Arc B",
       .unit = "deg",
       .min = 90,
       .max = 180,
       .step = 2,
       .scale = 1,
       .value = 180},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Max landed A", .unit = "deg", .scale = 1},
      {.name = "Predicted A", .unit = "deg", .scale = 1},
      {.name = "Max landed B", .unit = "deg", .scale = 1},
      {.name = "Predicted B", .unit = "deg", .scale = 1},
  }};
  std::array<std::array<Shot, kTiltCount>, 2> shots_{};
  std::array<std::array<int, kTiltCount>, 2> outcomes_{};
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_13_ONE_WAY_SOLUTIONS_ARC_SWEEP_SAMPLE_H_
