#ifndef TUTORIALS_08_CHAIN_SOLUTIONS_PER_LINK_STRETCH_SAMPLE_H_
#define TUTORIALS_08_CHAIN_SOLUTIONS_PER_LINK_STRETCH_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 08, exercise "per link stretch": the stretch of each joint of a
// hanging chain.
//
// The chapter's chain, hanging at rest: N links (parameter "Links") of 1 kg,
// 0.5 m apart, from a static pivot, the last one of M kg (parameter "End
// mass"), solved with P passes (parameter "Passes"). At rest each joint is a
// soft step: a spring of stiffness w^2 / k with w = 2 pi 15 Hz and k the
// joint's inverse effective mass. Joint i (above link i) carries the weight
// of the links i to N, and its k is 1/m(i-1) + 1/m(i) (only 1/m(1) for the
// first joint, against the static pivot):
//
//   stretch(i) = g (N - i + M) (1/m(i-1) + 1/m(i)) / w^2
//
// So the stretch is not largest at the top, where the load is: the first
// joint pulls against the static pivot and has half the inverse effective
// mass of the others. The second joint carries almost the same load between
// two moving links, and stretches most.
//
// Readouts: predicted and measured stretch of the top joint, of the second
// one and of the bottom one, and the largest gap between prediction and
// measurement over all joints. Drawn: the pivot in grey, each joint in
// white, yellow or red as its stretch passes 1/3 and 2/3 of the largest
// predicted one, the links in orange and the end in red.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class PerLinkStretchSample final : public Sample {
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

  // Joint i holds link i, from 1 (top) to links() (bottom); in meters
  [[nodiscard]] Scalar predicted_stretch(int joint) const;
  [[nodiscard]] Scalar measured_stretch(int joint) const;
  [[nodiscard]] int links() const { return parameters_[kLinks].value; }

  static constexpr std::size_t kLinks = 0;
  static constexpr std::size_t kEndMass = 1;
  static constexpr std::size_t kPasses = 2;
  static constexpr int kMaxLinks = 12;

 private:
  void UpdateReadouts();

  // Rebuilt in Begin: the passes come from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Links",
       .unit = "",
       .min = 3,
       .max = kMaxLinks,
       .step = 1,
       .scale = 1,
       .value = 8},
      {.name = "End mass",
       .unit = "kg",
       .min = 1,
       .max = 50,
       .step = 1,
       .scale = 1,
       .value = 10},
      {.name = "Passes",
       .unit = "",
       .min = 1,
       .max = 16,
       .step = 1,
       .scale = 1,
       .value = 8},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Top, predicted", .unit = "mm", .scale = 10},
      {.name = "Top, measured", .unit = "mm", .scale = 10},
      {.name = "Second, predicted", .unit = "mm", .scale = 10},
      {.name = "Second, measured", .unit = "mm", .scale = 10},
      {.name = "Bottom, predicted", .unit = "mm", .scale = 10},
      {.name = "Bottom, measured", .unit = "mm", .scale = 10},
      {.name = "Largest gap", .unit = "mm", .scale = 100},
  }};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_08_CHAIN_SOLUTIONS_PER_LINK_STRETCH_SAMPLE_H_
