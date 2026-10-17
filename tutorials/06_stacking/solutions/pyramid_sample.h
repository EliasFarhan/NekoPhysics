#ifndef TUTORIALS_06_STACKING_SOLUTIONS_PYRAMID_SAMPLE_H_
#define TUTORIALS_06_STACKING_SOLUTIONS_PYRAMID_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 06, exercise "pyramid": a pyramid of turning boxes.
//
// Rows of 1 m oriented boxes of 1 kg on a static ground, 4, 3, 2 and 1 from
// the bottom up (parameter "Rows" sets the bottom row's count), 0.2 m apart
// within a row, each box centered over the gap between the two below it, so
// it rests 0.4 m on each. The contacts are solved with `P` passes (parameter
// "Passes") with or without warm starting (parameter "Warm start").
//
// Unlike a column, every box above the bottom row stands on two others, and
// its weight splits between them: the bottom row's inner boxes carry more
// than the outer ones. The boxes can turn, so impulses that come out
// unequal tip them. The sample measures how far the top box sank below a
// perfect pyramid, the largest sideways drift of any box from where it
// started, and how many boxes fell (dropped more than half a meter). Drawn:
// the ground in dark grey and the boxes in orange, a fallen box in red.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class PyramidSample final : public Sample {
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

  // How far the top box is below a perfect pyramid, in meters
  [[nodiscard]] Scalar top_sink() const;
  // Largest horizontal distance of a box from its start, in meters
  [[nodiscard]] Scalar drift() const;
  // Boxes more than half a meter below their start
  [[nodiscard]] int fallen() const;
  [[nodiscard]] int box_count() const;

  static constexpr std::size_t kRows = 0;
  static constexpr std::size_t kPasses = 1;
  static constexpr std::size_t kWarmStart = 2;
  static constexpr int kMaxRows = 5;
  static constexpr int kMaxBoxes = kMaxRows * (kMaxRows + 1) / 2;

 private:
  void UpdateReadouts();
  [[nodiscard]] bool has_fallen(int box) const;

  // Rebuilt in Begin: its passes and warm starting come from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Rows",
       .unit = "",
       .min = 2,
       .max = kMaxRows,
       .step = 1,
       .scale = 1,
       .value = 4},
      {.name = "Passes",
       .unit = "",
       .min = 1,
       .max = 16,
       .step = 1,
       .scale = 1,
       .value = 4},
      {.name = "Warm start",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 3> readouts_{{
      {.name = "Top sank", .unit = "m", .scale = 1000},
      {.name = "Drift", .unit = "m", .scale = 1000},
      {.name = "Fallen", .unit = "", .scale = 1},
  }};
  std::array<Vec2f, kMaxBoxes> starts_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_06_STACKING_SOLUTIONS_PYRAMID_SAMPLE_H_
