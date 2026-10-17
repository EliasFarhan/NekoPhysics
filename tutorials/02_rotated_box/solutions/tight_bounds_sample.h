#ifndef TUTORIALS_02_ROTATED_BOX_SOLUTIONS_TIGHT_BOUNDS_SAMPLE_H_
#define TUTORIALS_02_ROTATED_BOX_SOLUTIONS_TIGHT_BOUNDS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 02, exercise "tight bounds": the broadphase box of a turning
// oriented box, and the pairs it reports that aren't contacts.
//
// An oriented box of half size (hx, hy) spins at a constant angular velocity
// (no gravity, no torque). The broadphase sees it as the axis-aligned box
// around it, recomputed from the cached sine and cosine every step:
//
//   half width     |cos a| hx + |sin a| hy
//   half height    |sin a| hx + |cos a| hy
//   widest         sqrt(hx^2 + hy^2), the half diagonal, when the diagonal
//                  lies along the axis (a = atan(hy / hx) for the width)
//
// The half width is also the farthest corner's x: the bounds are tight along
// both axes, but not in their own corners. A small trigger circle sits on the
// diagonal, just out of the box's reach (half diagonal + 0.2 m from its
// center): the turning box never touches it, yet the bounds' corner sweeps
// over it, and the broadphase reports a pair for those steps. The sample
// predicts each step's pair from the formula and counts the steps where the
// world disagrees ("Mismatches", expected 0).
//
// Drawn: the box in orange, its bounds in grey, the probe in red while the
// broadphase pairs it with the box and in green otherwise.
class TightBoundsSample final : public Sample {
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

  // The bounds' half size predicted from the box's rotation
  [[nodiscard]] Vec2f predicted_extents() const;
  // The farthest corner's x and y from the center, from the four rotated
  // corners
  [[nodiscard]] Vec2f corner_extents() const;
  // sqrt(hx^2 + hy^2)
  [[nodiscard]] Scalar half_diagonal() const;
  // Largest predicted half width seen so far
  [[nodiscard]] Scalar widest() const { return widest_; }
  [[nodiscard]] int steps() const { return steps_; }
  // Steps where the broadphase paired the probe with the box
  [[nodiscard]] int paired_steps() const { return paired_steps_; }
  // Steps where the world reported a touching pair (expected 0)
  [[nodiscard]] int touching_steps() const { return touching_steps_; }
  // Steps where the prediction and the broadphase disagreed (expected 0)
  [[nodiscard]] int mismatches() const { return mismatches_; }

  static constexpr std::size_t kWidth = 0;
  static constexpr std::size_t kHeight = 1;
  static constexpr std::size_t kSpin = 2;

 private:
  [[nodiscard]] Vec2f half_size() const;
  [[nodiscard]] Vec2f ExtentsFor(const Rotation& rotation) const;

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Width",
       .unit = "m",
       .min = 100,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 200},
      {.name = "Height",
       .unit = "m",
       .min = 100,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 100},
      {.name = "Spin",
       .unit = "rad/s",
       .min = 10,
       .max = 300,
       .step = 10,
       .scale = 100,
       .value = 100},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Half width", .unit = "m", .scale = 1000},
      {.name = "Half height", .unit = "m", .scale = 1000},
      {.name = "Corner x", .unit = "m", .scale = 1000},
      {.name = "Half diagonal", .unit = "m", .scale = 1000},
      {.name = "Paired steps", .unit = "", .scale = 1},
      {.name = "Touching steps", .unit = "", .scale = 1},
      {.name = "Mismatches", .unit = "", .scale = 1},
  }};
  BodyIndex box_index_ = kInvalidBodyIndex;
  BodyIndex probe_index_ = kInvalidBodyIndex;
  Scalar widest_{};
  bool is_paired_ = false;
  int steps_ = 0;
  int paired_steps_ = 0;
  int touching_steps_ = 0;
  int mismatches_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_02_ROTATED_BOX_SOLUTIONS_TIGHT_BOUNDS_SAMPLE_H_
