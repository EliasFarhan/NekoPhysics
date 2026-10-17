#ifndef TUTORIALS_08_CHAIN_CHAIN_SAMPLE_H_
#define TUTORIALS_08_CHAIN_CHAIN_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 08, chain: many joints solved together, and a heavy end.
//
// A chain of N links (parameter "Links") of 1 kg point masses, 0.5 m apart,
// hangs from a static pivot by rigid distance joints; the last link weighs
// M kg (parameter "End mass"). It starts hanging at rest (parameter "Start"
// 0) or stretched out horizontally (1), and falls and swings. The solver
// handles one joint at a time, in passes (parameter "Passes"): each joint
// fixes its own length and disturbs its neighbors, and with a heavy end the
// light links in between have to pass a large pull along, which takes more
// passes to converge.
//
// At rest each joint is a soft step of 15 Hz (see DistanceJoint): it
// stretches by its load times its inverse effective mass over
// w^2 = (2 pi 15)^2. Joint i holds the links i to N, a load of g (N - i +
// M) newtons, and its inverse effective mass is 1/m(i-1) + 1/m(i) (only
// 1/m(1) for the first, against the static pivot):
//
//   stretch = sum over i of g (N - i + M) (1/m(i-1) + 1/m(i)) / w^2
//
// Readouts: that prediction, the measured total stretch (the sum of the N
// length errors) and its largest value since the start. With warm starting
// a hanging chain converges to the prediction with a few passes; a swinging
// one stretches more, the more so with a heavy end and few passes.
//
// Drawn: the pivot in grey, the joints in white, the links in orange and
// the end mass in red.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class ChainSample final : public Sample {
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

  // The rest stretch above, in meters
  [[nodiscard]] Scalar predicted_rest_stretch() const;
  // Sum of the length errors now, in meters
  [[nodiscard]] Scalar stretch() const;
  // Largest stretch since the start, in meters
  [[nodiscard]] Scalar max_stretch() const { return max_stretch_; }

  static constexpr std::size_t kLinks = 0;
  static constexpr std::size_t kEndMass = 1;
  static constexpr std::size_t kPasses = 2;
  static constexpr std::size_t kStart = 3;
  static constexpr int kMaxLinks = 12;

 private:
  void UpdateReadouts();

  // Rebuilt in Begin: the passes come from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 4> parameters_{{
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
  }};
  std::array<SampleReadout, 3> readouts_{{
      {.name = "Pred. rest str.", .unit = "mm", .scale = 10},
      {.name = "Stretch", .unit = "mm", .scale = 10},
      {.name = "Max stretch", .unit = "mm", .scale = 10},
  }};
  Scalar max_stretch_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_08_CHAIN_CHAIN_SAMPLE_H_
