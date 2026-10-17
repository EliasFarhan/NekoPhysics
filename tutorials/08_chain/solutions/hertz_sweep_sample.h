#ifndef TUTORIALS_08_CHAIN_SOLUTIONS_HERTZ_SWEEP_SAMPLE_H_
#define TUTORIALS_08_CHAIN_SOLUTIONS_HERTZ_SWEEP_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 08, exercise "hertz sweep": a stiffer soft step, and its price.
//
// The chapter's chain (8 links of 1 kg, 0.5 m apart, from a static pivot,
// the last one of M kg from parameter "End mass"), with the soft step's
// frequency f (parameter "Hertz") and damping ratio zeta (parameter
// "Damping") as parameters of the world, solved with P passes (parameter
// "Passes"), hanging at rest (parameter "Start" 0) or released horizontally
// (1). At rest the stretch is the chapter's prediction, which scales as
// 1 / f^2: twice the frequency, a quarter of the stretch. A stiffer soft
// step is a less damped correction per step, and few passes stop resolving
// the chain before they resolve the step: at some frequency the chain
// rings, and beyond it the step diverges.
//
// Readouts: the predicted rest stretch, the stretch now (the sum of the
// length errors) and its largest value since the start, and whether the
// chain diverged (a total stretch past 10 m; the sample then stops
// stepping). Drawn: the pivot in grey, the joints in white, the links in
// orange and the end in red.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class HertzSweepSample final : public Sample {
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

  // sum over i of g (N - i + M) k_i / (2 pi f)^2, in meters
  [[nodiscard]] Scalar predicted_rest_stretch() const;
  // Sum of the length errors now, in meters
  [[nodiscard]] Scalar stretch() const;
  // Largest stretch since the start, in meters
  [[nodiscard]] Scalar max_stretch() const { return max_stretch_; }
  [[nodiscard]] bool diverged() const { return diverged_; }

  static constexpr std::size_t kHertz = 0;
  static constexpr std::size_t kDamping = 1;
  static constexpr std::size_t kPasses = 2;
  static constexpr std::size_t kEndMass = 3;
  static constexpr std::size_t kStart = 4;
  static constexpr int kLinks = 8;

 private:
  void UpdateReadouts();

  // Rebuilt in Begin: the softness and passes come from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 5> parameters_{{
      {.name = "Hertz",
       .unit = "Hz",
       .min = 5,
       .max = 60,
       .step = 5,
       .scale = 1,
       .value = 15},
      {.name = "Damping",
       .unit = "",
       .min = 5,
       .max = 40,
       .step = 5,
       .scale = 10,
       .value = 20},
      {.name = "Passes",
       .unit = "",
       .min = 1,
       .max = 16,
       .step = 1,
       .scale = 1,
       .value = 4},
      {.name = "End mass",
       .unit = "kg",
       .min = 1,
       .max = 50,
       .step = 1,
       .scale = 1,
       .value = 1},
      {.name = "Start",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 0},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Pred. rest str.", .unit = "mm", .scale = 10},
      {.name = "Stretch", .unit = "mm", .scale = 10},
      {.name = "Max stretch", .unit = "mm", .scale = 10},
      {.name = "Diverged", .unit = "", .scale = 1},
  }};
  Scalar max_stretch_{};
  bool diverged_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_08_CHAIN_SOLUTIONS_HERTZ_SWEEP_SAMPLE_H_
