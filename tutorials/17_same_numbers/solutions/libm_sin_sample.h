#ifndef TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_LIBM_SIN_SAMPLE_H_
#define TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_LIBM_SIN_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 17, exercise "libm sin": does this machine's sin give the right
// float?
//
// IEEE 754 requires +, -, *, / and sqrt to be correctly rounded: the exact
// result rounded to the nearest float, so every conforming machine gives
// the same bits. It only recommends it for sin, cos, exp...: each C library
// (glibc, the Microsoft CRT, newlib, the Switch's) brings its own sinf, and
// they may disagree in the last bit.
//
// The sample takes kAngleCount floats spread over [0, 2 pi) and compares
// std::sin(float) with a reference: the double sin of the same angle,
// rounded to float (right except in very rare double-rounding cases). The
// readouts give how many angles this C library gets wrong and by how many
// units in the last place at most, then whether the engine's own Sin for
// its scalar type (ScalarTraits) is exact across platforms by construction
// (Fixed16's table, the sixit float's own code) or forwards to this sinf.
//
// The numbers depend on the C library: run it on two machines and compare.
// The world checksum doesn't (there is no world): the sample stream stays
// the same everywhere.
//
// Drawn: one period of sin across the view, and a red dot at every angle
// this sinf gets wrong.
class LibmSinSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override {}
  void Draw(DrawList& draw_list) const override;
  void End() override { wrong_angles_.clear(); }
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return kChecksum;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  static constexpr int kAngleCount = 1 << 16;
  [[nodiscard]] int wrong_count() const { return wrong_count_; }
  [[nodiscard]] int max_ulp_error() const { return max_ulp_error_; }

 private:
  static constexpr std::uint32_t kChecksum = 17u;

  Aabbf bounds_{};
  // Index of every angle this sinf gets wrong (for the drawing)
  std::vector<int> wrong_angles_;
  int wrong_count_ = 0;
  int max_ulp_error_ = 0;
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Angles", .unit = "", .scale = 1},
      {.name = "sinf wrong", .unit = "", .scale = 1},
      {.name = "Max error", .unit = "ulp", .scale = 1},
      {.name = "Traits exact", .unit = "", .scale = 1},
  }};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_17_SAME_NUMBERS_SOLUTIONS_LIBM_SIN_SAMPLE_H_
