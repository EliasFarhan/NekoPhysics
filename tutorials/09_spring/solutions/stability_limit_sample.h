#ifndef TUTORIALS_09_SPRING_SOLUTIONS_STABILITY_LIMIT_SAMPLE_H_
#define TUTORIALS_09_SPRING_SOLUTIONS_STABILITY_LIMIT_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 09, exercise "stability limit": the stiffest spring a time step
// can hold.
//
// A mass m (parameter "Mass") on an undamped spring, as in the chapter, but
// released only 1 cm below its equilibrium. Semi-implicit Euler keeps the
// oscillation bounded while w dt < 2, w = sqrt(k / m), so the stiffest
// spring the time step dt can hold is
//
//   k_max = 4 m / dt^2
//
// In Begin the sample finds the measured limit by bisection: it runs the
// spring for 10 s with a candidate k, calls it stable if the mass stays
// within 1 m of its equilibrium (100 times the release), and halves the
// interval down to 1 N/m. Fixed16 shapes the code: the time step is
// 1092/65536 s, so the limit is 4 m (65536/1092)^2, and dt^2 (18 units of
// 1/65536) is too coarse to divide by, so the prediction squares 1/dt
// instead. The small release keeps k x within Fixed16's 32767 N, and the
// bisection's midpoint is low + (high - low) / 2: the sum low + high passes
// 32767 for 1 kg.
//
// Readouts: the predicted and measured limits and w dt at the measured one.
// Drawn: two springs, at 95 % (left) and 105 % (right) of the measured
// limit; the right one explodes and stops at 1 m. The anchors in grey, the
// springs in white, the masses in orange, the equilibria in green.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class StabilityLimitSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return below_.GenerateChecksum() ^ above_.GenerateChecksum();
  }
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // 4 m (1 / dt)^2, in N/m
  [[nodiscard]] Scalar predicted_limit() const;
  // Largest stable stiffness found by bisection, in N/m
  [[nodiscard]] Scalar measured_limit() const { return measured_limit_; }
  // Whether a spring of this stiffness stays within 1 m for 10 s
  [[nodiscard]] bool IsStable(Scalar stiffness) const;
  // Distance below the equilibrium of the spring past the limit, in meters
  [[nodiscard]] Scalar above_displacement() const;

  static constexpr std::size_t kMass = 0;
  static constexpr Scalar kRelease = Ratio(1, 100);
  static constexpr Scalar kBound{1};

 private:
  [[nodiscard]] Scalar mass() const { return parameters_[kMass].ToScalar(); }

  PhysicsWorld below_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  PhysicsWorld above_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Mass",
       .unit = "kg",
       .min = 1,
       .max = 10,
       .step = 1,
       .scale = 10,
       .value = 10},
  }};
  std::array<SampleReadout, 3> readouts_{{
      {.name = "Pred. limit", .unit = "N/m", .scale = 1},
      {.name = "Measured limit", .unit = "N/m", .scale = 1},
      {.name = "w dt at limit", .unit = "", .scale = 10000},
  }};
  Scalar measured_limit_{};
  Scalar below_equilibrium_y_{};
  Scalar above_equilibrium_y_{};
  bool above_exploded_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_09_SPRING_SOLUTIONS_STABILITY_LIMIT_SAMPLE_H_
