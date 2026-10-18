#ifndef TUTORIALS_13_ONE_WAY_SOLUTIONS_STACKED_ONE_WAY_SAMPLE_H_
#define TUTORIALS_13_ONE_WAY_SOLUTIONS_STACKED_ONE_WAY_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 13, exercise "stacked one-way": dropping a whole stack through a
// platform.
//
// Three 0.5 m crates stand in a stack on a one-way platform above the
// ground. At step kDropStep the game drops them, in the way chosen by
// parameter "Mode":
// - 0: DropThrough on the bottom crate, once;
// - 1: DropThrough on every crate, once;
// - 2: DropThrough on every crate, every step for kRepeatSteps steps.
//
// DropThrough only turns the one-way pairs that touch right now into passing
// pairs. In modes 0 and 1 only the bottom crate touches the platform: it
// falls through, and the crates above it land on the platform from above,
// a new pair the one-way rule finds solid. Mode 2 catches each crate on the
// step it reaches the platform: that pair is solid for one step (a small
// bump), then passing, and the whole stack ends on the ground.
//
// Readouts: the crates below the platform at the end, the pairs that became
// passing (the sum of DropThrough's results). Drawn: the ground, the
// platform in teal with its normal, the crates in blue (magenta outline
// while passing).
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class StackedOneWaySample final : public Sample {
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

  static constexpr std::size_t kCrateCount = 3;
  [[nodiscard]] int crates_below() const;
  [[nodiscard]] int passing_made() const { return passing_made_; }
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kMode = 0;
  static constexpr int kDropStep = 60;
  static constexpr int kRepeatSteps = 90;
  static constexpr int kMeasuredSteps = 240;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = kCrateCount + 2,
                       .max_colliders = kCrateCount + 2,
                       .max_touching_pairs = 16,
                       .max_broadphase_pairs = 16,
                       .max_joints = 0,
                       .max_passing_pairs = kCrateCount}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Mode",
       .unit = "",
       .min = 0,
       .max = 2,
       .step = 1,
       .scale = 1,
       .value = 0},
  }};
  std::array<SampleReadout, 2> readouts_{{
      {.name = "Crates below", .unit = "", .scale = 1},
      {.name = "Made passing", .unit = "pairs", .scale = 1},
  }};
  Aabbf ground_{};
  ColliderIndex platform_ = kInvalidColliderIndex;
  std::array<ColliderIndex, kCrateCount> crates_{};
  int passing_made_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_13_ONE_WAY_SOLUTIONS_STACKED_ONE_WAY_SAMPLE_H_
