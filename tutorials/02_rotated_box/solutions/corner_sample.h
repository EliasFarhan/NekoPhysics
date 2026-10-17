#ifndef TUTORIALS_02_ROTATED_BOX_SOLUTIONS_CORNER_SAMPLE_H_
#define TUTORIALS_02_ROTATED_BOX_SOLUTIONS_CORNER_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 02, exercise "corner": drop the ball on the ramp's upper corner and
// predict the contact normal, which is no longer the face normal.
//
// The ramp of chapter 02 (a static oriented box turned by `a` degrees, here
// 4 x 0.5 m) has its upper corner c at its right end. The ball falls from
// rest at `dx` meters to the right of the corner (parameter "Offset") and
// `h` meters above it. The box-circle test takes the ball center p in the
// ramp's frame and clamps it to the box: when both coordinates are clamped,
// the closest point q is the corner and the normal points from it to the
// center:
//
//   normal               n = (p - q) / |p - q|    (q = c in the corner region)
//   without sinking      n = (dx, sqrt(R^2 - dx^2)) / R  (a vertical fall
//                                                         touching c)
//
// The world finds the contact one step late, with the ball already sunk by
// up to one step of motion, so the measured normal follows the first formula
// with the position of that step, not the second. "Sink" is R - |p - q|.
//
// Drawn: the ramp in grey with its face normal in white, the corner in red,
// the ball in orange with its velocity in sky blue; after the bounce the
// predicted normal in green and the measured one in yellow, from the corner.
class CornerSample final : public Sample {
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

  // The ramp's face normal, (-sin a, cos a)
  [[nodiscard]] Vec2f face_normal() const;
  // (p - q) / |p - q| at the position where the world found the contact;
  // empty until the bounce
  [[nodiscard]] std::optional<Vec2f> predicted_normal() const {
    return predicted_normal_;
  }
  // Direction of the velocity change of the bounce step, gravity removed
  [[nodiscard]] std::optional<Vec2f> measured_normal() const {
    return measured_normal_;
  }
  // (dx, sqrt(R^2 - dx^2)) / R
  [[nodiscard]] Vec2f ideal_normal() const;
  // Whether the closest point was the corner (both coordinates at the
  // box's half size)
  [[nodiscard]] bool hit_corner() const { return hit_corner_; }
  // R - |p - q| when the contact was found, in meters
  [[nodiscard]] std::optional<Scalar> sink() const { return sink_; }

  static constexpr std::size_t kRampAngle = 0;
  static constexpr std::size_t kOffset = 1;
  static constexpr std::size_t kDropHeight = 2;
  static constexpr std::size_t kRestitution = 3;

 private:
  [[nodiscard]] Vec2f corner() const;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Ramp angle",
       .unit = "deg",
       .min = 0,
       .max = 30,
       .step = 5,
       .scale = 1,
       .value = 20},
      {.name = "Offset",
       .unit = "m",
       .min = 0,
       .max = 20,
       .step = 1,
       .scale = 100,
       .value = 10},
      {.name = "Drop height",
       .unit = "m",
       .min = 50,
       .max = 250,
       .step = 25,
       .scale = 100,
       .value = 100},
      {.name = "Restitution",
       .unit = "",
       .min = 0,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 75},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Normal x", .unit = "", .scale = 1000},
      {.name = "Normal y", .unit = "", .scale = 1000},
      {.name = "Measured x", .unit = "", .scale = 1000},
      {.name = "Measured y", .unit = "", .scale = 1000},
      {.name = "No sink x", .unit = "", .scale = 1000},
      {.name = "No sink y", .unit = "", .scale = 1000},
      {.name = "Sink", .unit = "m", .scale = 1000},
  }};
  BodyIndex ramp_index_ = kInvalidBodyIndex;
  BodyIndex ball_index_ = kInvalidBodyIndex;
  bool has_bounced_ = false;
  bool hit_corner_ = false;
  std::optional<Vec2f> predicted_normal_;
  std::optional<Vec2f> measured_normal_;
  std::optional<Scalar> sink_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_02_ROTATED_BOX_SOLUTIONS_CORNER_SAMPLE_H_
