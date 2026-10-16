#ifndef TUTORIALS_03_CONTACT_MANIFOLD_CONTACT_MANIFOLD_SAMPLE_H_
#define TUTORIALS_03_CONTACT_MANIFOLD_CONTACT_MANIFOLD_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 03, contact manifolds: where does a box touch the ground?
//
// A box (parameter "Width", 1 m tall) is held sunk into a static ground: its
// center is placed so that, unturned, its bottom would be `d` meters deep
// (parameter "Sink depth"), and it is turned by `a` degrees (parameter
// "Angle"). It is kinematic, so contacts don't move it, and it can turn at a
// steady speed (parameter "Spin") to watch corners dip in and out.
//
// The ground's top face is the reference face (it is the separating axis:
// the box's own axes overlap the wide ground much more). The incident face is
// the box face pointing most downward. Its two corners are the candidate
// points, and each one is in the manifold when it is below the ground, with
// its depth below the ground's top:
//
//   corner    c = position + R(a) (+/- half width, -half height), for the
//               bottom face when |cos a| >= |sin a| (else a side face)
//   depth     ground top - c.y, kept when >= 0
//
// So a flat box touches at two points at the same depth, a slightly tilted
// box at two points at different depths, and a box tilted far enough at one
// corner. Readouts compare the predicted and measured number of points and
// the depth of each corner. Drawn: the ground in dark grey, the box in orange
// with its orientation in white, the manifold points in yellow with their
// depth in red, and the contact normal in white.
//
// The world is y-up, in meters and seconds, with no gravity.
class ContactManifoldSample final : public Sample {
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

  // The two corners of the incident face, in the order the clipping visits
  // them
  [[nodiscard]] std::array<Vec2f, 2> incident_corners() const;
  // Depth of an incident corner below the ground's top, empty when it is
  // above
  [[nodiscard]] std::optional<Scalar> predicted_depth(std::size_t corner) const;
  [[nodiscard]] int predicted_point_count() const;
  // Depth of the manifold point at an incident corner, empty when no point is
  // there
  [[nodiscard]] std::optional<Scalar> measured_depth(std::size_t corner) const;
  [[nodiscard]] int measured_point_count() const {
    return contact_.point_count;
  }
  [[nodiscard]] const Contact& contact() const { return contact_; }

  static constexpr std::size_t kAngle = 0;
  static constexpr std::size_t kSinkDepth = 1;
  static constexpr std::size_t kWidth = 2;
  static constexpr std::size_t kSpin = 3;

 private:
  void UpdateContact();
  [[nodiscard]] Scalar ground_top() const;

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Angle",
       .unit = "deg",
       .min = -40,
       .max = 40,
       .step = 5,
       .scale = 1,
       .value = 5},
      {.name = "Sink depth",
       .unit = "m",
       .min = 0,
       .max = 50,
       .step = 5,
       .scale = 100,
       .value = 20},
      {.name = "Width",
       .unit = "m",
       .min = 100,
       .max = 400,
       .step = 25,
       .scale = 100,
       .value = 200},
      {.name = "Spin",
       .unit = "deg/s",
       .min = 0,
       .max = 60,
       .step = 5,
       .scale = 1,
       .value = 0},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Points", .unit = "", .scale = 1},
      {.name = "Measured", .unit = "", .scale = 1},
      {.name = "Corner 1 depth", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
      {.name = "Corner 2 depth", .unit = "m", .scale = 1000},
      {.name = "Measured", .unit = "m", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  ColliderIndex ground_collider_{};
  ColliderIndex box_collider_{};
  Contact contact_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_03_CONTACT_MANIFOLD_CONTACT_MANIFOLD_SAMPLE_H_
