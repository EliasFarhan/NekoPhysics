#ifndef TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_BOX_ON_TILTED_BOX_SAMPLE_H_
#define TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_BOX_ON_TILTED_BOX_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 03, exercise "box on a tilted box": the experiment of chapter 03 on
// the ramp of chapter 02. Both are oriented boxes.
//
// A static ramp (half size 5 x 0.25 m) turned by `r` degrees (parameter
// "Ramp angle") and a kinematic 2 x 1 m box turned by `r + t` (parameter
// "Relative tilt"), slid `s` meters along the ramp from its center
// (parameter "Slide") and sunk `d` meters into it (parameter "Sink depth").
// The ramp's top face is the reference face, so the prediction is chapter
// 03's, written in the ramp's frame (x along the ramp, y along its normal):
//
//   box center   (s, 0.25 + 0.5 - d)
//   corners      center + R(t) (+/- 1, -0.5)   (the bottom face, |t| < 45)
//   depth        0.25 - corner.y, kept when >= 0
//   normal       from the box toward the ramp: -(-sin r, cos r)
//
// The measured points are turned into the ramp's frame with two dot products
// (chapter 02) and matched to the corners. Drawn: the ramp in grey, the box
// in orange, the predicted corners as green rings, the measured points in
// yellow with their depth in red.
class BoxOnTiltedBoxSample final : public Sample {
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

  // The bottom corners of the box in the ramp's frame, left then right
  [[nodiscard]] std::array<Vec2f, 2> local_corners() const;
  // Depth of a corner below the ramp's face, empty when it is above
  [[nodiscard]] std::optional<Scalar> predicted_depth(std::size_t corner) const;
  [[nodiscard]] int predicted_point_count() const;
  // Depth of the manifold point at a corner, empty when no point is there
  [[nodiscard]] std::optional<Scalar> measured_depth(std::size_t corner) const;
  [[nodiscard]] Vec2f predicted_normal() const;
  [[nodiscard]] const Contact& contact() const { return contact_; }
  // A world point in the ramp's frame
  [[nodiscard]] Vec2f ToRampFrame(Vec2f point) const;

  static constexpr std::size_t kRampAngle = 0;
  static constexpr std::size_t kRelativeTilt = 1;
  static constexpr std::size_t kSlide = 2;
  static constexpr std::size_t kSinkDepth = 3;

 private:
  void UpdateContact();

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Ramp angle",
       .unit = "deg",
       .min = -30,
       .max = 30,
       .step = 5,
       .scale = 1,
       .value = 20},
      {.name = "Relative tilt",
       .unit = "deg",
       .min = -20,
       .max = 20,
       .step = 1,
       .scale = 1,
       .value = 3},
      {.name = "Slide",
       .unit = "m",
       .min = -300,
       .max = 300,
       .step = 50,
       .scale = 100,
       .value = 0},
      {.name = "Sink depth",
       .unit = "m",
       .min = 0,
       .max = 40,
       .step = 5,
       .scale = 100,
       .value = 20},
  }};
  std::array<SampleReadout, 10> readouts_{{
      {.name = "Points", .unit = "", .scale = 1},
      {.name = "Measured", .unit = "", .scale = 1},
      {.name = "Corner 1 depth", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
      {.name = "Corner 2 depth", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
      {.name = "Normal x", .unit = "", .scale = 1000},
      {.name = "Measured", .unit = "", .scale = 1000},
      {.name = "Normal y", .unit = "", .scale = 1000},
      {.name = "Measured", .unit = "", .scale = 1000},
  }};
  BodyIndex ramp_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  ColliderIndex ramp_collider_{};
  ColliderIndex box_collider_{};
  Contact contact_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_BOX_ON_TILTED_BOX_SAMPLE_H_
