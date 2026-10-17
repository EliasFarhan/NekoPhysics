#ifndef TUTORIALS_07_PENDULUM_PENDULUM_SAMPLE_H_
#define TUTORIALS_07_PENDULUM_PENDULUM_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 07, pendulum: a distance joint holds a bob on a circle.
//
// A 1 kg bob hangs from a static pivot on a rod of length L (parameter
// "Length") and is released at rest at an angle theta0 from the vertical
// (parameter "Amplitude"). The rod is a rigid distance joint, or a rope
// (parameter "Rope", 0 or 1) that only pulls. For a small swing the period
// is
//
//   T = 2 pi sqrt(L / g) (1 + theta0^2 / 16 + 11 theta0^4 / 3072)
//
// (the series of the exact elliptic integral: 0.0004 % low at 30 degrees,
// 0.4 % at 90, 9 % at 150 where the exact period of a 2 m pendulum is
// 5.00 s and the series says 4.53). The sample measures the period from the
// times the bob crosses the vertical below the pivot, and the length error
// |distance - L|. Two effects make the rod longer than L, both largest at the
// bottom, where the bob is fastest (v^2 = 2 g L (1 - cos theta0)):
//
// - the joint is a soft step of w = 2 pi 15 Hz: it stretches by its tension
//   over w^2 per kilogram, and the tension at the bottom is
//   m g (3 - 2 cos theta0) (1.1 mm per g);
// - each step moves the bob v dt along the tangent, which takes it
//   (v dt)^2 / (2 L) off the circle, and the soft step pulls back
//   h w / (2 zeta + h w) = 0.28 of the error per step (zeta = 2), so the
//   error settles at drift / 0.28.
//
// The predicted largest stretch is the sum (readout "Pred. stretch").
//
// Past 90 degrees a rope goes slack near the top: the tension
// m g (3 cos theta - 2 cos theta0) would have to be negative, a rope can't
// push, so the bob falls inside the circle until the rope catches it (the
// "Slack steps" readout counts the steps with the rope shorter than L by
// more than 1 cm). A rod keeps it on the circle.
//
// Drawn: the pivot in grey, the rod in white (the rope in yellow, dark grey
// while slack), the bob in orange with its velocity in sky blue.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class PendulumSample final : public Sample {
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

  // The series above, in seconds
  [[nodiscard]] Scalar predicted_period() const;
  // From the vertical crossings, once the bob crossed three times; empty
  // before
  [[nodiscard]] std::optional<Scalar> measured_period() const;
  // Tension sag plus swing drift (see above), in meters
  [[nodiscard]] Scalar predicted_max_stretch() const;
  // Largest |distance - L| since the start, in meters
  [[nodiscard]] Scalar max_length_error() const { return max_length_error_; }
  [[nodiscard]] int slack_steps() const { return slack_steps_; }

  static constexpr std::size_t kLength = 0;
  static constexpr std::size_t kAmplitude = 1;
  static constexpr std::size_t kRope = 2;

 private:
  [[nodiscard]] Scalar length() const {
    return parameters_[kLength].ToScalar();
  }
  [[nodiscard]] Radian amplitude() const;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 1}};
  std::array<SampleParameter, 3> parameters_{{
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
       .value = 10},
      {.name = "Rope",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 0},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Predicted period", .unit = "s", .scale = 1000},
      {.name = "Measured period", .unit = "s", .scale = 1000},
      {.name = "Length error", .unit = "mm", .scale = 10},
      {.name = "Max length err.", .unit = "mm", .scale = 10},
      {.name = "Pred. stretch", .unit = "mm", .scale = 10},
      {.name = "Slack steps", .unit = "", .scale = 1},
  }};
  BodyIndex pivot_ = kInvalidBodyIndex;
  BodyIndex bob_ = kInvalidBodyIndex;
  JointIndex joint_ = kInvalidJointIndex;
  int step_ = 0;
  Scalar previous_x_{};
  // Times of the first and last vertical crossings, and their count
  Scalar first_crossing_{};
  Scalar last_crossing_{};
  int crossings_ = 0;
  Scalar max_length_error_{};
  int slack_steps_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_07_PENDULUM_PENDULUM_SAMPLE_H_
