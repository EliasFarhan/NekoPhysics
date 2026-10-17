#ifndef TUTORIALS_06_STACKING_SOLUTIONS_IMPULSE_PER_BOX_SAMPLE_H_
#define TUTORIALS_06_STACKING_SOLUTIONS_IMPULSE_PER_BOX_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 06, exercise "impulse per box": what each contact of a stack
// carries.
//
// A stack of 1 m boxes of 1 kg (parameter "Boxes", n) rests on a static
// ground, solved with `P` passes (parameter "Passes") with or without warm
// starting (parameter "Warm start"). The boxes are AABBs, so they only sink
// and never tip. At rest, nothing changes momentum, so the contact under box
// k (k = 0 at the bottom) must carry the weight of every box from k up, over
// one step:
//
//   J_k = (n - k) m g dt
//
// The world doesn't publish its impulses, but momentum does: in one step,
// box k gains m (v' - v) = J_k - J_{k+1} - m g dt (gravity and the contacts
// below and above), so from the top down
//
//   J_top = m (v'_top - v_top) + m g dt,   J_k = m (v'_k - v_k) + m g dt +
//   J_{k+1}
//
// The sample measures every J_k after each step, and keeps those of the
// first step, when the solver starts from zero. Drawn: the ground in dark
// grey, the boxes in orange, and each contact's impulse as a red bar to the
// right of the stack, 0.25 m per box weight (m g dt), with a white tick at the
// predicted length.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class ImpulsePerBoxSample final : public Sample {
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

  // (n - k) m g dt, in N s
  [[nodiscard]] Scalar predicted_impulse(int contact) const;
  // J_k measured over the last step, in N s
  [[nodiscard]] Scalar measured_impulse(int contact) const {
    return impulses_[static_cast<std::size_t>(contact)];
  }
  // J_k measured over the first step, in N s
  [[nodiscard]] Scalar first_step_impulse(int contact) const {
    return first_impulses_[static_cast<std::size_t>(contact)];
  }
  [[nodiscard]] int box_count() const { return parameters_[kBoxes].value; }
  // How far the top box is below a perfect stack, in meters
  [[nodiscard]] Scalar top_sink() const;

  static constexpr std::size_t kBoxes = 0;
  static constexpr std::size_t kPasses = 1;
  static constexpr std::size_t kWarmStart = 2;
  static constexpr int kMaxBoxes = 10;

 private:
  void UpdateReadouts();

  // Rebuilt in Begin: its passes and warm starting come from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Boxes",
       .unit = "",
       .min = 2,
       .max = kMaxBoxes,
       .step = 1,
       .scale = 1,
       .value = 4},
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
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Bottom predicted", .unit = "N s", .scale = 10000},
      {.name = "Bottom, step 1", .unit = "N s", .scale = 10000},
      {.name = "Bottom, now", .unit = "N s", .scale = 10000},
      {.name = "Top predicted", .unit = "N s", .scale = 10000},
      {.name = "Top, step 1", .unit = "N s", .scale = 10000},
      {.name = "Top, now", .unit = "N s", .scale = 10000},
      {.name = "Top sank", .unit = "m", .scale = 1000},
  }};
  std::array<Scalar, kMaxBoxes> impulses_{};
  std::array<Scalar, kMaxBoxes> first_impulses_{};
  Scalar ground_top_{};
  Scalar center_x_{};
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_06_STACKING_SOLUTIONS_IMPULSE_PER_BOX_SAMPLE_H_
