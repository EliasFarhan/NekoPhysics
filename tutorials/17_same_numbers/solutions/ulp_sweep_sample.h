#ifndef TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_ULP_SWEEP_SAMPLE_H_
#define TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_ULP_SWEEP_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "tutorials/17_same_numbers/same_numbers_sample.h"

namespace neko::tutorial {

// Chapter 17, exercise "ulp sweep": how fast does the board amplify?
//
// A reference board and kNudgeCount nudged copies run together. Copy k
// starts its first ball 2^k units in the last place to the right (1, 2,
// 4... up to 32: still under 1 mm with Fixed16, whose unit is 1/65536 m).
// For each copy the readouts give the first step a ball is more than 1 mm
// from its twin on the reference board.
//
// If every bounce multiplied the difference by the same factor, the gap
// would grow like nudge * factor^bounces, and doubling the nudge would
// bring the visible step earlier by a constant number of steps (the
// steps the board takes to double a gap). The readouts show how far
// that holds: the steps shrink with the nudge, but the board is not that
// regular (a ball can miss a peg by a hair, or not).
//
// Drawn: the reference board, and the first ball of each copy as a small
// dot of its own color.
class UlpSweepSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  static constexpr std::size_t kNudgeCount = 6;
  [[nodiscard]] static constexpr int Nudge(std::size_t copy) {
    int nudge = 1;
    for (std::size_t i = 0; i < copy; ++i) {
      nudge *= 2;
    }
    return nudge;
  }
  [[nodiscard]] std::optional<int> visible_step(std::size_t copy) const {
    return watches_[copy].first_visible_step();
  }
  [[nodiscard]] const Board& copy(std::size_t index) const {
    return copies_[index];
  }

 private:
  Board reference_;
  std::array<Board, kNudgeCount> copies_;
  std::array<TwinWatch, kNudgeCount> watches_;
  std::array<SampleReadout, kNudgeCount> readouts_{{
      {.name = "1 ulp visible", .unit = "step", .scale = 1},
      {.name = "2 ulp visible", .unit = "step", .scale = 1},
      {.name = "4 ulp visible", .unit = "step", .scale = 1},
      {.name = "8 ulp visible", .unit = "step", .scale = 1},
      {.name = "16 ulp visible", .unit = "step", .scale = 1},
      {.name = "32 ulp visible", .unit = "step", .scale = 1},
  }};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_ULP_SWEEP_SAMPLE_H_
