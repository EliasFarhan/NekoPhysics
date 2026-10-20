#ifndef TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_FUSED_UPDATE_SAMPLE_H_
#define TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_FUSED_UPDATE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "tutorials/17_same_numbers/same_numbers_sample.h"

namespace neko::tutorial {

// Chapter 17, exercise "fused update": what one contracted line does.
//
// The integrator's position line is `position += velocity * dt`: a multiply
// rounded to float, then an add rounded to float. A compiler allowed to
// contract (clang on arm64 by default, GCC and clang on x64 with -mfma)
// turns it into one fused multiply-add, rounded once: std::fma(velocity, dt,
// position). Most of the time both give the same float; sometimes they
// differ in the last bit.
//
// Board A steps as the engine does. After each of board B's steps, the
// sample recomputes each ball's position with std::fma from its position
// before the step and its velocity after the contacts (what the step
// integrated with) and writes it back: board B is the board of a machine
// that contracts that one line.
//
// Readouts: the position updates of board A computed both ways in float
// (two per ball per step), how many differ, which fraction; then the first
// step the checksums of A and B differ and the first step a ball is 1 mm
// from its twin. With Fixed16 there is no fused operation (an integer
// multiply and add are exact up to the one rounding of the product), so
// board B is left alone and the last two readouts stay empty; the first
// three still show what float would do with these numbers.
class FusedUpdateSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  [[nodiscard]] int updates() const { return updates_; }
  [[nodiscard]] int differing_updates() const { return differing_updates_; }
  [[nodiscard]] const TwinWatch& watch() const { return watch_; }
  // Whether board B is integrated with fused updates (not with Fixed16)
  [[nodiscard]] static bool fuses();

 private:
  Board board_a_;
  Board board_b_;
  TwinWatch watch_;
  std::array<Vec2f, Board::kBallCount> before_{};
  int updates_ = 0;
  int differing_updates_ = 0;
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Updates", .unit = "", .scale = 1},
      {.name = "Fused differs", .unit = "", .scale = 1},
      {.name = "Differ", .unit = "%", .scale = 100},
      {.name = "Checksums differ", .unit = "step", .scale = 1},
      {.name = "Visible (1 mm)", .unit = "step", .scale = 1},
  }};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_FUSED_UPDATE_SAMPLE_H_
