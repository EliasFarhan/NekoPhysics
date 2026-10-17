#ifndef TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_TWO_AABBS_SAMPLE_H_
#define TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_TWO_AABBS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 03, exercise "two AABBs": a 1 m block (a kinematic AABB) held over
// the right edge of a static AABB ledge (4 x 1 m), with `o` meters of its
// width on the ledge (parameter "On the ledge") and its bottom `d` meters
// below the ledge's top (parameter "Sink depth").
//
// Two AABBs need no clipping: their faces are parallel. The world compares
// the overlaps along x and y and keeps the smaller (y on a tie):
//
//   overlap x = o,  overlap y = d
//   normal     from the block (body 2) toward the ledge (body 1): down when
//              d <= o, left when o < d
//   points     the two ends of the overlap along the face, on the block's
//              face toward the ledge, both at the penetration
//
// So a block sunk less than it overlaps rests on the ledge, with its points
// at its own left corner and above the ledge's corner; a block sunk deeper
// than its overlap is pushed out sideways, off the ledge. Positions are
// relative to the ledge's top right corner. Drawn: the ledge in dark grey,
// the block in orange, the predicted points as green rings, the measured
// ones in yellow and the normal in white.
class TwoAabbsSample final : public Sample {
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

  [[nodiscard]] Vec2f predicted_normal() const;
  [[nodiscard]] Scalar predicted_penetration() const;
  // In world space, in the order of the overlap interval (low end first)
  [[nodiscard]] std::array<Vec2f, 2> predicted_points() const;
  [[nodiscard]] const Contact& contact() const { return contact_; }
  [[nodiscard]] Vec2f ledge_corner() const;

  static constexpr std::size_t kOnLedge = 0;
  static constexpr std::size_t kSinkDepth = 1;

 private:
  void UpdateContact();

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "On the ledge",
       .unit = "m",
       .min = 5,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 60},
      {.name = "Sink depth",
       .unit = "m",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 100,
       .value = 20},
  }};
  std::array<SampleReadout, 14> readouts_{{
      {.name = "Normal x", .unit = "", .scale = 1},
      {.name = "Measured", .unit = "", .scale = 1},
      {.name = "Normal y", .unit = "", .scale = 1},
      {.name = "Measured", .unit = "", .scale = 1},
      {.name = "Depth", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
      {.name = "Point 1 x", .unit = "m", .scale = 1000},
      {.name = "Point 1 y", .unit = "m", .scale = 1000},
      {.name = "Measured x", .unit = "m", .scale = 1000},
      {.name = "Measured y", .unit = "m", .scale = 1000},
      {.name = "Point 2 x", .unit = "m", .scale = 1000},
      {.name = "Point 2 y", .unit = "m", .scale = 1000},
      {.name = "Measured x", .unit = "m", .scale = 1000},
      {.name = "Measured y", .unit = "m", .scale = 1000},
  }};
  BodyIndex ledge_index_ = kInvalidBodyIndex;
  BodyIndex block_index_ = kInvalidBodyIndex;
  ColliderIndex ledge_collider_{};
  ColliderIndex block_collider_{};
  Contact contact_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_03_CONTACT_MANIFOLD_SOLUTIONS_TWO_AABBS_SAMPLE_H_
