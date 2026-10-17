#ifndef TUTORIALS_06_STACKING_SOLUTIONS_HEAVY_ON_LIGHT_SAMPLE_H_
#define TUTORIALS_06_STACKING_SOLUTIONS_HEAVY_ON_LIGHT_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 06, exercise "heavy on light": a heavy box on a stack of light
// ones.
//
// Four 1 m AABB boxes rest on a static ground: three of 1 kg and, on top,
// one of M kg (parameter "Top mass"), solved with `P` passes (parameter
// "Passes") with or without warm starting (parameter "Warm start"). AABBs
// only sink, they never tip. A second, identical stack of four 1 kg boxes
// runs beside it with the same settings, for comparison.
//
// At rest each contact carries the weight of everything above it over one
// step, so the heavy box only makes the impulses larger. The passes see it
// differently: a pass shares each correction between two boxes in
// proportion to their inverse masses, so the heavy box barely moves while
// the light box under it takes almost all of every correction, and the
// error travels down the stack slowly. Without warm starting the stack makes
// up the missing impulse by sinking deeper, until the Baumgarte push-out
// adds it. Warm starting helps up to a point: with a ratio of 20, or 10 with
// two passes, the warm-started stack bounces. Drawn: the ground in dark
// grey, the stack in orange (the heavy box in red), and the reference stack
// in grey.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class HeavyOnLightSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum() ^ reference_.GenerateChecksum();
  }
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // How far the top box is below a perfect stack, in meters: with the heavy
  // box, and in the reference stack of equal boxes
  [[nodiscard]] Scalar top_sink() const;
  [[nodiscard]] Scalar reference_top_sink() const;

  static constexpr std::size_t kTopMass = 0;
  static constexpr std::size_t kPasses = 1;
  static constexpr std::size_t kWarmStart = 2;
  static constexpr int kBoxes = 4;

 private:
  void UpdateReadouts();

  // Rebuilt in Begin: their passes and warm starting come from the
  // parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  PhysicsWorld reference_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Top mass",
       .unit = "kg",
       .min = 1,
       .max = 20,
       .step = 1,
       .scale = 1,
       .value = 10},
      {.name = "Passes",
       .unit = "",
       .min = 1,
       .max = 16,
       .step = 1,
       .scale = 1,
       .value = 8},
      {.name = "Warm start",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 0},
  }};
  std::array<SampleReadout, 2> readouts_{{
      {.name = "Top sank", .unit = "m", .scale = 1000},
      {.name = "Sank, all 1 kg", .unit = "m", .scale = 1000},
  }};
  Scalar ground_top_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_06_STACKING_SOLUTIONS_HEAVY_ON_LIGHT_SAMPLE_H_
