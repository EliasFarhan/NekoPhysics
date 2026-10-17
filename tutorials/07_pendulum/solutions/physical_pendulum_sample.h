#ifndef TUTORIALS_07_PENDULUM_SOLUTIONS_PHYSICAL_PENDULUM_SAMPLE_H_
#define TUTORIALS_07_PENDULUM_SOLUTIONS_PHYSICAL_PENDULUM_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 07, exercise "physical pendulum": a box hung from its corner.
//
// A 1 kg square box of side a (parameter "Size") hangs from a static pivot
// by a zero-length rigid joint anchored at its top-left corner, released at
// rest with its center at theta0 from the vertical (parameter
// "Amplitude"). The anchor is off-center, so the joint turns the box: it
// swings as a physical pendulum, with the moment of inertia about the pivot
// I_p = I_c + m d^2 (parallel axes), d = a / sqrt(2) the corner-to-center
// distance and I_c = m a^2 / 6 for a square. For a small swing
//
//   T = 2 pi sqrt(I_p / (m g d)) = 2 pi sqrt((4 d / 3) / g)
//
// the period of a point mass on a rod of 4 d / 3, the "equivalent length"
// (the same for any rectangle hung from a corner: I_p = m (w^2 + h^2) / 3).
// The sample uses the chapter's amplitude series on top of it.
//
// The box has no collider: its inertia is set by hand
// (InertiaMode::kOverride). Readouts: the predicted and measured periods
// (from the center's crossings of the vertical below the pivot), the
// equivalent length, and the largest distance between the corner and the
// pivot (a zero-length distance joint pulls the corner back along the line
// between them, like the rod of the chapter). Drawn: the pivot in grey, the
// box in orange, its center and the line to the pivot in white.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class PhysicalPendulumSample final : public Sample {
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

  // 4 d / 3, in meters
  [[nodiscard]] Scalar equivalent_length() const;
  // 2 pi sqrt(4 d / (3 g)) times the amplitude series, in seconds
  [[nodiscard]] Scalar predicted_period() const;
  // From the center's crossings of the vertical, once there were three
  [[nodiscard]] std::optional<Scalar> measured_period() const;
  // Largest corner-to-pivot distance since the start, in meters
  [[nodiscard]] Scalar max_pin_error() const { return max_pin_error_; }

  static constexpr std::size_t kSize = 0;
  static constexpr std::size_t kAmplitude = 1;

 private:
  [[nodiscard]] Scalar size() const { return parameters_[kSize].ToScalar(); }
  [[nodiscard]] Radian amplitude() const;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Size",
       .unit = "m",
       .min = 2,
       .max = 20,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Amplitude",
       .unit = "deg",
       .min = 5,
       .max = 60,
       .step = 5,
       .scale = 1,
       .value = 10},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Predicted period", .unit = "s", .scale = 1000},
      {.name = "Measured period", .unit = "s", .scale = 1000},
      {.name = "Equiv. length", .unit = "m", .scale = 1000},
      {.name = "Max pin error", .unit = "mm", .scale = 10},
  }};
  BodyIndex pivot_ = kInvalidBodyIndex;
  BodyIndex box_ = kInvalidBodyIndex;
  JointIndex pin_ = kInvalidJointIndex;
  int step_ = 0;
  Scalar previous_x_{};
  Scalar first_crossing_{};
  Scalar last_crossing_{};
  int crossings_ = 0;
  Scalar max_pin_error_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_07_PENDULUM_SOLUTIONS_PHYSICAL_PENDULUM_SAMPLE_H_
