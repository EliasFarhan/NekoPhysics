#ifndef TUTORIALS_07_PENDULUM_SOLUTIONS_ENERGY_SAMPLE_H_
#define TUTORIALS_07_PENDULUM_SOLUTIONS_ENERGY_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 07, exercise "energy": what the joint does to the swing's energy.
//
// The chapter's rod pendulum (1 kg bob, length L from parameter "Length",
// released at rest at theta0 from parameter "Amplitude"), with the world's
// joint frequency as a parameter ("Joint hertz", 0 for no length
// correction). Its energy, measured from the lowest point of the circle,
//
//   E = m v^2 / 2 + m g (y - y_bottom)
//
// starts at m g L (1 - cos theta0). A rod does no work on a bob that moves
// along the circle: its pull is perpendicular to the velocity. The engine's
// joint is a soft step instead, a stiff and heavily damped spring along the
// rod, and the bob moves off the circle and back every step.
//
// Semi-implicit Euler's energy wobbles within a swing, so the sample
// compares the energy at the same point of every swing: each time the bob
// crosses the vertical below the pivot (interpolated between the two
// steps). Readouts: the start energy, the energy at the first and at the
// last crossing, the loss per period between them, and the rod's length
// error now. Drawn: the pivot in grey, the rod in white, the bob in orange
// with its velocity in sky blue, the lowest point of the circle in green.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class EnergySample final : public Sample {
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

  // m g L (1 - cos theta0), in joules
  [[nodiscard]] Scalar start_energy() const { return start_energy_; }
  // m v^2 / 2 + m g (y - y_bottom) now, in joules
  [[nodiscard]] Scalar energy() const;
  // At the first and the last crossing of the vertical, in joules
  [[nodiscard]] Scalar first_bottom_energy() const {
    return first_bottom_energy_;
  }
  [[nodiscard]] Scalar last_bottom_energy() const {
    return last_bottom_energy_;
  }
  // Percent of the first crossing's energy lost per period since, once the
  // bob crossed three times; empty before
  [[nodiscard]] std::optional<Scalar> loss_per_period() const;
  // Distance minus L now, in meters
  [[nodiscard]] Scalar length_error() const;

  static constexpr std::size_t kLength = 0;
  static constexpr std::size_t kAmplitude = 1;
  static constexpr std::size_t kJointHertz = 2;

 private:
  [[nodiscard]] Scalar length() const {
    return parameters_[kLength].ToScalar();
  }

  // Rebuilt in Begin: the joint frequency comes from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
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
       .value = 90},
      {.name = "Joint hertz",
       .unit = "Hz",
       .min = 0,
       .max = 30,
       .step = 5,
       .scale = 1,
       .value = 15},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Start energy", .unit = "J", .scale = 1000},
      {.name = "First bottom", .unit = "J", .scale = 1000},
      {.name = "Last bottom", .unit = "J", .scale = 1000},
      {.name = "Lost per period", .unit = "%", .scale = 1000},
      {.name = "Length error", .unit = "mm", .scale = 10},
  }};
  BodyIndex pivot_ = kInvalidBodyIndex;
  BodyIndex bob_ = kInvalidBodyIndex;
  JointIndex rod_ = kInvalidJointIndex;
  Scalar start_energy_{};
  Scalar previous_x_{};
  Scalar previous_energy_{};
  Scalar first_bottom_energy_{};
  Scalar last_bottom_energy_{};
  int crossings_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_07_PENDULUM_SOLUTIONS_ENERGY_SAMPLE_H_
