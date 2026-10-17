#ifndef TUTORIALS_02_ROTATED_BOX_SOLUTIONS_BOX_ON_RAMP_SAMPLE_H_
#define TUTORIALS_02_ROTATED_BOX_SOLUTIONS_BOX_ON_RAMP_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 02, exercise "box on ramp": drop an oriented box on the ramp and
// predict which of the four separating axes becomes the contact normal.
//
// The ramp of chapter 02 (static, turned by `a`, half size 5 x 0.25 m) and a
// 1 x 0.5 m box turned by `b`, dropped from rest. When the world first finds
// the contact, the sample runs the separating axis test by hand on the
// positions of that step: for each face axis u of both boxes (the ramp's x
// and y, then the box's x and y),
//
//   shadow radius    r(box, u) = hx |u . x_box| + hy |u . y_box|
//   overlap          r(ramp, u) + r(box, u) - |u . (c_box - c_ramp)|
//
// The smallest overlap is the penetration and its axis, turned to point from
// the ramp toward the box, is the contact normal. Both boxes are
// frictionless, so the contact impulse is along that normal and the velocity
// change of the step (gravity removed) measures it, whatever the impulse
// does to the box's spin (an off-center impulse spins it: chapter 04).
//
// Drawn: the ramp in grey, the box in orange with its orientation in white;
// after the contact the predicted normal in green and the measured one in
// yellow, from the box's center.
class BoxOnRampSample final : public Sample {
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

  // Overlaps along the ramp's x and y axes, then the box's x and y axes, at
  // the contact; empty until then
  [[nodiscard]] std::optional<std::array<Scalar, 4>> overlaps() const {
    return overlaps_;
  }
  // Which of the four axes had the smallest overlap (0 to 3, same order)
  [[nodiscard]] int best_axis() const { return best_axis_; }
  // The smallest overlap's axis, from the ramp toward the box
  [[nodiscard]] std::optional<Vec2f> predicted_normal() const {
    return predicted_normal_;
  }
  // Direction of the velocity change of the contact step, gravity removed
  [[nodiscard]] std::optional<Vec2f> measured_normal() const {
    return measured_normal_;
  }
  // The ramp's face normal, (-sin a, cos a)
  [[nodiscard]] Vec2f ramp_normal() const;

  static constexpr std::size_t kRampAngle = 0;
  static constexpr std::size_t kBoxAngle = 1;
  static constexpr std::size_t kDropHeight = 2;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Ramp angle",
       .unit = "deg",
       .min = -30,
       .max = 30,
       .step = 5,
       .scale = 1,
       .value = 20},
      {.name = "Box angle",
       .unit = "deg",
       .min = -90,
       .max = 90,
       .step = 15,
       .scale = 1,
       .value = 0},
      {.name = "Drop height",
       .unit = "m",
       .min = 100,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 200},
  }};
  std::array<SampleReadout, 8> readouts_{{
      {.name = "Ramp x overlap", .unit = "m", .scale = 1000},
      {.name = "Ramp y overlap", .unit = "m", .scale = 1000},
      {.name = "Box x overlap", .unit = "m", .scale = 1000},
      {.name = "Box y overlap", .unit = "m", .scale = 1000},
      {.name = "Normal x", .unit = "", .scale = 1000},
      {.name = "Normal y", .unit = "", .scale = 1000},
      {.name = "Measured x", .unit = "", .scale = 1000},
      {.name = "Measured y", .unit = "", .scale = 1000},
  }};
  BodyIndex ramp_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  std::optional<std::array<Scalar, 4>> overlaps_;
  int best_axis_ = -1;
  std::optional<Vec2f> predicted_normal_;
  std::optional<Vec2f> measured_normal_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_02_ROTATED_BOX_SOLUTIONS_BOX_ON_RAMP_SAMPLE_H_
