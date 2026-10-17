#ifndef TUTORIALS_09_SPRING_SOLUTIONS_SOFT_VS_EXPLICIT_SAMPLE_H_
#define TUTORIALS_09_SPRING_SOLUTIONS_SOFT_VS_EXPLICIT_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 09, exercise "soft vs explicit": the soft step is a spring; the
// same spring as an explicit force.
//
// Two 1 kg bobs hang at rest from static anchors, 2 m below them, released
// with no stretch. The left one is on a rigid joint, whose length correction
// is the soft step of the world: a spring of frequency f (parameter "Hertz",
// the world's joint_hertz) and damping ratio 2, solved implicitly. The right
// one is on a spring joint with the same stiffness, k = m w^2 (w = 2 pi f),
// and a damping c = 2 zeta m w with its own ratio zeta (parameter "Damping
// ratio"), applied as an explicit force. Both sag by the same m g / k =
// g / w^2 at rest (1.1 mm at 15 Hz).
//
// The explicit spring is stable only while (w dt)^2 + 4 zeta w dt < 4: at
// 15 Hz, w dt = 1.57, so only below zeta = 0.24. Without damping it rings
// around the sag forever; with the soft step's zeta = 2 it explodes in a few
// steps. The rod, the same spring solved implicitly, settles at any ratio.
//
// Readouts: the predicted sag, the rod's and the spring's stretch now, the
// spring's constants, the stability number (w dt)^2 + 4 zeta w dt and
// whether the spring exploded (past 1 m; it then stops). Drawn: the anchors
// in grey, the rod in white, the spring in sky blue, the bobs in orange.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class SoftVsExplicitSample final : public Sample {
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

  // g / w^2, in meters
  [[nodiscard]] Scalar predicted_sag() const;
  // Distance minus length now, in meters
  [[nodiscard]] Scalar rod_stretch() const;
  [[nodiscard]] Scalar spring_stretch() const;
  // Largest |stretch - sag| of the spring since the start, in meters
  [[nodiscard]] Scalar spring_ringing() const { return spring_ringing_; }
  // (w dt)^2 + 4 zeta w dt: below 4 the explicit spring is stable
  [[nodiscard]] Scalar stability_number() const;
  [[nodiscard]] bool exploded() const { return exploded_; }

  static constexpr std::size_t kHertz = 0;
  static constexpr std::size_t kDampingRatio = 1;

 private:
  [[nodiscard]] Scalar omega() const;

  // Rebuilt in Begin: the joint frequency comes from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Hertz",
       .unit = "Hz",
       .min = 5,
       .max = 20,
       .step = 5,
       .scale = 1,
       .value = 15},
      {.name = "Damping ratio",
       .unit = "",
       .min = 0,
       .max = 20,
       .step = 1,
       .scale = 10,
       .value = 0},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Predicted sag", .unit = "mm", .scale = 100},
      {.name = "Rod stretch", .unit = "mm", .scale = 100},
      {.name = "Spring stretch", .unit = "mm", .scale = 100},
      {.name = "Spring k", .unit = "N/m", .scale = 1},
      {.name = "Spring c", .unit = "N s/m", .scale = 10},
      {.name = "Stability num.", .unit = "", .scale = 1000},
      {.name = "Exploded", .unit = "", .scale = 1},
  }};
  JointIndex rod_ = kInvalidJointIndex;
  JointIndex spring_ = kInvalidJointIndex;
  Scalar spring_ringing_{};
  bool exploded_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_09_SPRING_SOLUTIONS_SOFT_VS_EXPLICIT_SAMPLE_H_
