#ifndef TUTORIALS_08_CHAIN_SOLUTIONS_JOINT_ORDER_SAMPLE_H_
#define TUTORIALS_08_CHAIN_SOLUTIONS_JOINT_ORDER_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 08, exercise "joint order": the same chain, solved top-down or
// bottom-up.
//
// Two copies of the chapter's chain (N links of 1 kg from parameter "Links",
// 0.5 m apart, the last one of M kg from parameter "End mass"), in two
// worlds with P passes (parameter "Passes"), with or without warm starting
// (parameter "Warm start"), hanging (parameter "Start" 0) or released
// horizontally (1). They differ only in the order of their
// joint tables: the left chain adds its joints from the pivot down, as the
// chapter does, the right one from the end up. A pass solves the joints in
// table order, each with its neighbors' latest velocities, so a correction
// travels down the whole chain in one top-down pass, and up it in one
// bottom-up pass. With warm starting each step starts from last step's
// answer and the order hardly matters. Without it the passes start from
// zero every step and don't finish: the chain hangs longer than the soft
// step's rest stretch, and less so in the order that carries the weight's
// correction from the end to the pivot in fewer passes.
//
// Readouts: the predicted rest stretch, then for each chain the stretch now
// (the sum of the length errors) and its largest value since the start.
// Drawn: the pivots in grey, the joints in white, the links in orange and
// the ends in red; the bottom-up chain's joints in sky blue.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class JointOrderSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return top_down_.GenerateChecksum() ^ bottom_up_.GenerateChecksum();
  }
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // The chapter's rest stretch, in meters
  [[nodiscard]] Scalar predicted_rest_stretch() const;
  // Sum of the length errors now, in meters
  [[nodiscard]] Scalar top_down_stretch() const;
  [[nodiscard]] Scalar bottom_up_stretch() const;
  [[nodiscard]] Scalar top_down_max() const { return top_down_max_; }
  [[nodiscard]] Scalar bottom_up_max() const { return bottom_up_max_; }

  static constexpr std::size_t kLinks = 0;
  static constexpr std::size_t kEndMass = 1;
  static constexpr std::size_t kPasses = 2;
  static constexpr std::size_t kStart = 3;
  static constexpr std::size_t kWarmStart = 4;
  static constexpr int kMaxLinks = 12;

 private:
  void UpdateReadouts();
  [[nodiscard]] int links() const { return parameters_[kLinks].value; }

  // Rebuilt in Begin: the passes come from the parameters
  PhysicsWorld top_down_{
      {.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  PhysicsWorld bottom_up_{
      {.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 5> parameters_{{
      {.name = "Links",
       .unit = "",
       .min = 2,
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
       .value = 1},
      {.name = "Passes",
       .unit = "",
       .min = 1,
       .max = 16,
       .step = 1,
       .scale = 1,
       .value = 4},
      {.name = "Start",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 0},
      {.name = "Warm start",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 0},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Pred. rest str.", .unit = "mm", .scale = 10},
      {.name = "Top-down str.", .unit = "mm", .scale = 10},
      {.name = "Top-down max", .unit = "mm", .scale = 10},
      {.name = "Bottom-up str.", .unit = "mm", .scale = 10},
      {.name = "Bottom-up max", .unit = "mm", .scale = 10},
  }};
  Scalar top_down_max_{};
  Scalar bottom_up_max_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_08_CHAIN_SOLUTIONS_JOINT_ORDER_SAMPLE_H_
