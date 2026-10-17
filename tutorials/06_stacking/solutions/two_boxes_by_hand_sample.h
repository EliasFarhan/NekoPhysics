#ifndef TUTORIALS_06_STACKING_SOLUTIONS_TWO_BOXES_BY_HAND_SAMPLE_H_
#define TUTORIALS_06_STACKING_SOLUTIONS_TWO_BOXES_BY_HAND_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 06, exercise "two boxes by hand": the first step of a two-box
// stack, solved pass by pass.
//
// Two 1 m AABB boxes of 1 kg rest on a static ground, just touching (no
// overlap, so no push-out and no bounce: every target speed is 0). The step
// first adds gravity, so both boxes start the solver at v = -g dt. The world
// solves the contact under the bottom box (A, k = 1/m = 1 per kg), then the
// contact between the boxes (B, k = 1/m + 1/m = 2), each over its two points,
// for `P` passes (parameter "Passes"). The boxes don't turn, so a contact's
// second point finds the job done by its first. One pass:
//
//   A: vn = v1          dj = -v1           v1 += dj
//   B: vn = v2 - v1     dj = -(v2 - v1)/2  v2 += dj, v1 -= dj
//
// From v1 = v2 = -g dt: A stops the bottom box, B then shares the top box's
// speed with it, and both end the pass at -g dt / 2. Each pass halves the
// error, so after P passes both move at -g dt / 2^P, and the impulses are
// short of the weights they must carry, 2 m g dt under the bottom box and
// m g dt between them. The sample runs that loop by hand and compares it with
// the world after the first step. Drawn: the ground in dark grey, the boxes
// in orange with their velocities in sky blue.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class TwoBoxesByHandSample final : public Sample {
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

  // Vertical velocities of the two boxes and the total impulses of the two
  // contacts after one step
  struct State {
    Scalar bottom_velocity;
    Scalar top_velocity;
    Scalar bottom_impulse;  // contact A, under the bottom box
    Scalar top_impulse;     // contact B, between the boxes
  };
  // The loop above by hand, for `passes` passes
  [[nodiscard]] State predicted(int passes) const;
  // The world after its first step (impulses from the momentum changes);
  // empty before
  [[nodiscard]] std::optional<State> measured() const { return measured_; }

  static constexpr std::size_t kPasses = 0;

 private:
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Passes",
       .unit = "",
       .min = 1,
       .max = 8,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 8> readouts_{{
      {.name = "Bottom vy, hand", .unit = "m/s", .scale = 10000},
      {.name = "Bottom vy, world", .unit = "m/s", .scale = 10000},
      {.name = "Top vy, hand", .unit = "m/s", .scale = 10000},
      {.name = "Top vy, world", .unit = "m/s", .scale = 10000},
      {.name = "Impulse A, hand", .unit = "N s", .scale = 10000},
      {.name = "Impulse A, world", .unit = "N s", .scale = 10000},
      {.name = "Impulse B, hand", .unit = "N s", .scale = 10000},
      {.name = "Impulse B, world", .unit = "N s", .scale = 10000},
  }};
  std::optional<State> measured_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_06_STACKING_SOLUTIONS_TWO_BOXES_BY_HAND_SAMPLE_H_
