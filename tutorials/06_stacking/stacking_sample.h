#ifndef TUTORIALS_06_STACKING_STACKING_SAMPLE_H_
#define TUTORIALS_06_STACKING_STACKING_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 06, stacking: why the contact solver iterates and warm starts.
//
// A stack of 1 m boxes (parameter "Boxes") rests on a static ground. Every
// box presses on the one below with the weight of everything above it, so
// the contact impulses must grow from one box at the top to the whole stack
// at the bottom. The solver finds them with passes over all the contacts
// (parameter "Passes"): each pass corrects every point a little and the
// corrections disturb each other, so with too few passes the impulses come
// out too small and the stack sinks into itself. Warm starting (parameter
// "Warm start", 0 or 1) starts each step from last step's impulses, so the
// passes only fix the difference. Turning boxes (parameter "Oriented", 0 for
// AABBs, 1 for oriented boxes) can also tip and fall.
//
// Readouts: how far the top box sank below the height of a perfect stack
// (each contact may keep the 5 mm slop: up to boxes * 5 mm is fine), and the
// largest sideways drift. Drawn: the ground in dark grey, the boxes in
// orange.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class StackingSample final : public Sample {
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

  // How far the top box is below a perfect stack, in meters
  [[nodiscard]] Scalar top_sink() const;
  // Largest horizontal distance of a box from the stack's axis, in meters
  [[nodiscard]] Scalar drift() const;

  static constexpr std::size_t kBoxes = 0;
  static constexpr std::size_t kPasses = 1;
  static constexpr std::size_t kWarmStart = 2;
  static constexpr std::size_t kOriented = 3;
  static constexpr int kMaxBoxes = 10;

 private:
  void UpdateReadouts();

  // Rebuilt in Begin: its passes and warm starting come from the parameters,
  // and a world's config is fixed when it is built
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Boxes",
       .unit = "",
       .min = 2,
       .max = kMaxBoxes,
       .step = 1,
       .scale = 1,
       .value = 6},
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
       .value = 1},
      {.name = "Oriented",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 2> readouts_{{
      {.name = "Top sank", .unit = "m", .scale = 1000},
      {.name = "Drift", .unit = "m", .scale = 1000},
  }};
  Scalar ground_top_{};
  Scalar center_x_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_06_STACKING_STACKING_SAMPLE_H_
