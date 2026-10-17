#ifndef TUTORIALS_07_PENDULUM_SOLUTIONS_TENSION_SAMPLE_H_
#define TUTORIALS_07_PENDULUM_SOLUTIONS_TENSION_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 07, exercise "tension": the rod's pull, read from its impulse.
//
// The chapter's pendulum: a 1 kg bob on a rigid rod of length L (parameter
// "Length"), released at rest at theta0 (parameter "Amplitude"). The joint's
// accumulated impulse is the step's total along the rod, from the pivot
// toward the bob, so the rod's tension is -impulse / dt (positive when it
// pulls). At the angle theta the bob needs the centripetal force m v^2 / L
// on top of the weight's part along the rod, m g cos theta, and energy
// gives v^2 = 2 g L (cos theta - cos theta0):
//
//   T = m g (3 cos theta - 2 cos theta0)
//
// independent of L: m g at the bottom of a tiny swing, 3 m g for a release
// at 90 degrees, and negative (the rod pushes) near the ends of a swing
// released past 90 degrees, wherever 3 cos theta < 2 cos theta0.
//
// The soft step needs a few steps to take the load: released with no
// stretch, the rod pulls with only 0.9 of m g cos theta0 in the first step
// (its mass scale), so the sample compares the two after 0.5 s.
//
// Readouts: the predicted tension at the bottom, the largest measured
// tension, the first step's tension, the tension now and its prediction at
// the bob's angle now, and the largest gap between the two after 0.5 s.
// Drawn: the pivot in grey,
// the rod in white (red while it pushes), the bob in orange with its
// velocity in sky blue.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class TensionSample final : public Sample {
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

  // m g (3 - 2 cos theta0), in newtons
  [[nodiscard]] Scalar predicted_bottom_tension() const;
  // -impulse / dt of the last step, in newtons
  [[nodiscard]] Scalar tension() const;
  // m g (3 cos theta - 2 cos theta0) at the bob's angle now, in newtons
  [[nodiscard]] Scalar predicted_tension() const;
  [[nodiscard]] Scalar max_tension() const { return max_tension_; }
  [[nodiscard]] Scalar min_tension() const { return min_tension_; }
  [[nodiscard]] Scalar first_tension() const { return first_tension_; }
  // Largest |tension - predicted| after 0.5 s, in newtons
  [[nodiscard]] Scalar max_error() const { return max_error_; }

  static constexpr std::size_t kLength = 0;
  static constexpr std::size_t kAmplitude = 1;

 private:
  [[nodiscard]] Scalar length() const {
    return parameters_[kLength].ToScalar();
  }
  [[nodiscard]] Scalar cos_amplitude() const;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Length",
       .unit = "m",
       .min = 5,
       .max = 40,
       .step = 5,
       .scale = 10,
       .value = 20},
      {.name = "Amplitude",
       .unit = "deg",
       .min = 5,
       .max = 170,
       .step = 5,
       .scale = 1,
       .value = 90},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Pred. bottom T", .unit = "N", .scale = 1000},
      {.name = "Max tension", .unit = "N", .scale = 1000},
      {.name = "First step T", .unit = "N", .scale = 1000},
      {.name = "Tension", .unit = "N", .scale = 1000},
      {.name = "Predicted now", .unit = "N", .scale = 1000},
      {.name = "Max gap", .unit = "N", .scale = 1000},
  }};
  BodyIndex pivot_ = kInvalidBodyIndex;
  BodyIndex bob_ = kInvalidBodyIndex;
  JointIndex rod_ = kInvalidJointIndex;
  Scalar max_tension_{};
  Scalar min_tension_{};
  Scalar first_tension_{};
  Scalar max_error_{};
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_07_PENDULUM_SOLUTIONS_TENSION_SAMPLE_H_
