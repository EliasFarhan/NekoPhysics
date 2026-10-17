#ifndef TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_NARROW_PILLAR_SAMPLE_H_
#define TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_NARROW_PILLAR_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 03, exercise "narrow pillar": a 3 x 1 m box held on a 1 m wide
// pillar, overhanging it, so the side planes of the reference face clip the
// incident face.
//
// The pillar is a static AABB (half size 0.5 x 1 m), the box a kinematic
// oriented box turned by `a` degrees (parameter "Angle"), its center `dx`
// meters right of the pillar's (parameter "Offset") and low enough that,
// unturned, its bottom would be `d` meters deep (parameter "Sink depth").
// For these angles the pillar's top face is the reference face (its overlap
// is the smallest of the four axes, see chapter 02), and the box's bottom
// face is the incident face. The prediction clips it by hand:
//
//   incident edge  from c + R(a) (-1.5, -0.5) to c + R(a) (1.5, -0.5)
//   side planes    pillar left <= x <= pillar right: an end past a plane
//                  moves along the edge to the plane, the depth with it
//   kept           the clipped ends below the pillar's top, with depth
//                  top - y
//
// So with the default offset one point is the box's corner and the other is
// on the pillar's right edge, between the corners' depths. When the box's own
// face wins the separating axis test instead, the readout "Reference" says 2
// and there is no prediction. Drawn: the pillar in dark grey, the box in
// orange, the predicted points as green rings and the measured ones in
// yellow, with their depth in red.
class NarrowPillarSample final : public Sample {
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

  // A point of the predicted manifold: position and depth below the
  // pillar's top
  struct PredictedPoint {
    Vec2f position;
    Scalar depth;
  };
  // The clipped incident edge, in clipping order; count is 0 to 2
  struct Prediction {
    std::array<PredictedPoint, 2> points{};
    int count = 0;
  };
  [[nodiscard]] Prediction prediction() const;
  [[nodiscard]] const Contact& contact() const { return contact_; }
  // Whether the pillar's top face is the reference face: the contact normal
  // is (0, -1), from the box toward the pillar
  [[nodiscard]] bool pillar_is_reference() const;
  [[nodiscard]] Vec2f pillar_center() const;

  static constexpr std::size_t kAngle = 0;
  static constexpr std::size_t kOffset = 1;
  static constexpr std::size_t kSinkDepth = 2;

 private:
  void UpdateContact();
  [[nodiscard]] Scalar pillar_top() const;

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Angle",
       .unit = "deg",
       .min = -10,
       .max = 10,
       .step = 1,
       .scale = 1,
       .value = 5},
      {.name = "Offset",
       .unit = "m",
       .min = 0,
       .max = 180,
       .step = 10,
       .scale = 100,
       .value = 120},
      {.name = "Sink depth",
       .unit = "m",
       .min = 5,
       .max = 30,
       .step = 5,
       .scale = 100,
       .value = 10},
  }};
  std::array<SampleReadout, 11> readouts_{{
      {.name = "Reference", .unit = "", .scale = 1},
      {.name = "Points", .unit = "", .scale = 1},
      {.name = "Measured", .unit = "", .scale = 1},
      {.name = "Point 1 x", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
      {.name = "Point 1 depth", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
      {.name = "Point 2 x", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
      {.name = "Point 2 depth", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
  }};
  BodyIndex pillar_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  ColliderIndex pillar_collider_{};
  ColliderIndex box_collider_{};
  Contact contact_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_NARROW_PILLAR_SAMPLE_H_
