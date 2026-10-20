#ifndef TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_SYSTEM_WORDS_SAMPLE_H_
#define TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_SYSTEM_WORDS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "tutorials/17_same_numbers/same_numbers_sample.h"

namespace neko::tutorial {

// One checksum word per part of a board's world, like the per-system words
// Splash sends in its desync report
struct SystemWords {
  // Every collider: shape, material, filter (what the level built)
  std::uint32_t colliders = 0;
  // Every body: position, velocity, angle (what moves)
  std::uint32_t bodies = 0;
  // The touching pairs (who touches whom)
  std::uint32_t touching = 0;
  // PhysicsWorld::GenerateChecksum, everything
  std::uint32_t world = 0;
};
[[nodiscard]] SystemWords ComputeSystemWords(const Board& board);

// Chapter 17, exercise "system words": which word names the culprit?
//
// A world checksum says that two peers diverged, not why. Splaying it into
// one word per part of the state (Splash's desync report shows players,
// bullets, physics, level...) tells where. But the first word to differ in
// the moving state is usually not where the difference came from.
//
// Board B is board A with one collider changed: the restitution of the
// top peg under the drop point, "Nudge" units in the last place higher (the
// kind of difference a level table read with another float parser makes).
// No ball is nudged. The readouts give the first step each word differs:
// colliders at once (the change itself), bodies only when a ball bounces
// off that peg, touching pairs later still (when the balls' paths part).
// Comparing words only when a desync is detected, a game would see bodies,
// touching and world all differ and blame the bodies; the colliders word,
// different since step 0, is the one that names the cause.
class SystemWordsSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  static constexpr std::size_t kNudge = 0;
  [[nodiscard]] std::optional<int> colliders_step() const {
    return colliders_step_;
  }
  [[nodiscard]] std::optional<int> bodies_step() const { return bodies_step_; }
  [[nodiscard]] std::optional<int> touching_step() const {
    return touching_step_;
  }
  [[nodiscard]] std::optional<int> world_step() const { return world_step_; }
  [[nodiscard]] const TwinWatch& watch() const { return watch_; }

 private:
  void Compare(int step);

  Board board_a_;
  Board board_b_;
  TwinWatch watch_;
  std::optional<int> colliders_step_;
  std::optional<int> bodies_step_;
  std::optional<int> touching_step_;
  std::optional<int> world_step_;
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Nudge",
       .unit = "ulp",
       .min = 0,
       .max = 64,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Colliders differ", .unit = "step", .scale = 1},
      {.name = "Bodies differ", .unit = "step", .scale = 1},
      {.name = "Touching differ", .unit = "step", .scale = 1},
      {.name = "World differs", .unit = "step", .scale = 1},
      {.name = "Visible (1 mm)", .unit = "step", .scale = 1},
  }};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_SYSTEM_WORDS_SAMPLE_H_
