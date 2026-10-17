#ifndef TUTORIALS_08_CHAIN_SOLUTIONS_ONE_STEP_BY_HAND_SAMPLE_H_
#define TUTORIALS_08_CHAIN_SOLUTIONS_ONE_STEP_BY_HAND_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 08, exercise "one step by hand": the soft step of one joint,
// computed by hand and compared with the world.
//
// A 1 kg bob is placed at rest a distance e (parameter "Offset") beyond the
// 1 m length of a rigid joint hanging from a static pivot. The world uses
// the joint frequency f (parameter "Hertz") and damping ratio zeta
// (parameter "Damping") and P passes (parameter "Passes"). The first step,
// by hand, with h the time step and w = 2 pi f:
//
//   gravity first:  v = -g h (down), so the anchors separate at Cdot = g h
//   a1 = 2 zeta + h w,  a2 = h w a1,  a3 = 1 / (1 + a2)
//   bias rate b = w / a1,  mass scale s = a2 a3,  leak = a3
//   impulse = s (-b e - Cdot) / k  (k = 1/m = 1 per kg, nothing to leak yet)
//   v' = v - impulse (the pull is along u, from the pivot down to the bob)
//   e' = e - v' h  (the bob moves up by v' h)
//
// One pass is the whole answer for a single joint: a second pass finds the
// separation speed it left and the leak exactly cancel. Readouts: the
// coefficients, then the impulse, the velocity after the step and the error
// after the step, by hand and from the world (the first step only).
//
// Drawn: the pivot in grey, the rod in white, the bob in orange, the rod's
// length as a green tick.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class OneStepByHandSample final : public Sample {
 public:
  // The first step's numbers, by hand or measured
  struct Step {
    Scalar impulse{};
    Scalar velocity{};
    Scalar error{};
  };
  // The soft step's coefficients
  struct Softness {
    Scalar bias_rate{};
    Scalar mass_scale{};
    Scalar impulse_scale{};
  };

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

  [[nodiscard]] Softness softness() const;
  [[nodiscard]] Step predicted() const;
  // The world's first step; empty before it
  [[nodiscard]] std::optional<Step> measured() const { return measured_; }

  static constexpr std::size_t kOffset = 0;
  static constexpr std::size_t kHertz = 1;
  static constexpr std::size_t kDamping = 2;
  static constexpr std::size_t kPasses = 3;

 private:
  [[nodiscard]] Scalar length_error() const;

  // Rebuilt in Begin: the softness and passes come from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Offset",
       .unit = "mm",
       .min = -50,
       .max = 50,
       .step = 5,
       .scale = 1,
       .value = 10},
      {.name = "Hertz",
       .unit = "Hz",
       .min = 5,
       .max = 30,
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
       .max = 8,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 9> readouts_{{
      {.name = "Bias rate", .unit = "1/s", .scale = 1000},
      {.name = "Mass scale", .unit = "", .scale = 10000},
      {.name = "Leak", .unit = "", .scale = 10000},
      {.name = "Hand impulse", .unit = "N s", .scale = 10000},
      {.name = "World impulse", .unit = "N s", .scale = 10000},
      {.name = "Hand velocity", .unit = "m/s", .scale = 10000},
      {.name = "World velocity", .unit = "m/s", .scale = 10000},
      {.name = "Hand error", .unit = "mm", .scale = 100},
      {.name = "World error", .unit = "mm", .scale = 100},
  }};
  BodyIndex pivot_ = kInvalidBodyIndex;
  BodyIndex bob_ = kInvalidBodyIndex;
  JointIndex rod_ = kInvalidJointIndex;
  std::optional<Step> measured_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_08_CHAIN_SOLUTIONS_ONE_STEP_BY_HAND_SAMPLE_H_
