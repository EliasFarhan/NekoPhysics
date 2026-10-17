#ifndef TUTORIALS_09_SPRING_SPRING_SAMPLE_H_
#define TUTORIALS_09_SPRING_SPRING_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 09, spring: a Hooke force, its period, its damping and its limit.
//
// A mass m (parameter "Mass") hangs from a static anchor on a spring joint
// of rest length 2 m, stiffness k (parameter "Stiffness") and damping c
// (parameter "Damping"). A spring is not a constraint: before each step it
// adds the force k x + c v along the spring, with x the stretch and v the
// stretching speed. Gravity pulls it down to the equilibrium stretch
//
//   x_eq = m g / k
//
// and the sample releases it at rest 0.5 m below that. It then oscillates
// around the equilibrium with the period and decay of a mass on a spring:
//
//   T = 2 pi sqrt(m / k)        amplitude ~ exp(-c / (2 m) t)
//
// and the damping ratio c / (2 sqrt(k m)) says how fast it settles: below 1
// it rings, 1 is the fastest return without overshoot (critical damping).
// Damping also slows the ringing a little: the damped period is
// T / sqrt(1 - zeta^2) (k = 40 N/m, m = 1 kg, c = 4 N s/m: zeta = 0.32 and
// 1.047 s instead of 0.993 s).
//
// The force is explicit: computed from the positions at the start of the
// step. Semi-implicit Euler keeps the oscillation bounded only while
// w dt < 2 (w = sqrt(k / m)): a stiff spring on a light mass gains energy
// every step and explodes (the readout "Exploded" turns to 1 and the sample
// stops stepping past 100 m). Even below the limit the measured period is a
// little shorter than T: the discrete motion turns by acos(1 - (w dt)^2 / 2)
// per step instead of w dt.
//
// Readouts: predicted and measured periods (from the crossings of the
// equilibrium), the equilibrium stretch, w dt, the damping ratio and whether
// it exploded. Drawn: the anchor in grey, the spring as a white zigzag, the
// mass in orange, the equilibrium as a short green line.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class SpringSample final : public Sample {
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

  // 2 pi sqrt(m / k), in seconds
  [[nodiscard]] Scalar predicted_period() const;
  // From the crossings of the equilibrium, once there were three; empty
  // before
  [[nodiscard]] std::optional<Scalar> measured_period() const;
  // m g / k, in meters
  [[nodiscard]] Scalar equilibrium_stretch() const;
  // sqrt(k / m) dt
  [[nodiscard]] Scalar omega_dt() const;
  // c / (2 sqrt(k m))
  [[nodiscard]] Scalar damping_ratio() const;
  // Distance of the mass below the equilibrium, in meters
  [[nodiscard]] Scalar displacement() const;
  [[nodiscard]] bool exploded() const { return exploded_; }

  static constexpr std::size_t kStiffness = 0;
  static constexpr std::size_t kMass = 1;
  static constexpr std::size_t kDamping = 2;
  // Initial displacement below the equilibrium, in meters
  static constexpr Scalar kRelease = Ratio(1, 2);

 private:
  [[nodiscard]] Scalar stiffness() const {
    return parameters_[kStiffness].ToScalar();
  }
  [[nodiscard]] Scalar mass() const { return parameters_[kMass].ToScalar(); }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Stiffness",
       .unit = "N/m",
       .min = 20,
       .max = 2000,
       .step = 20,
       .scale = 1,
       .value = 40},
      {.name = "Mass",
       .unit = "kg",
       .min = 1,
       .max = 20,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Damping",
       .unit = "N s/m",
       .min = 0,
       .max = 100,
       .step = 1,
       .scale = 10,
       .value = 0},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Predicted period", .unit = "s", .scale = 1000},
      {.name = "Measured period", .unit = "s", .scale = 1000},
      {.name = "Equil. stretch", .unit = "m", .scale = 1000},
      {.name = "w dt", .unit = "", .scale = 1000},
      {.name = "Damping ratio", .unit = "", .scale = 1000},
      {.name = "Exploded", .unit = "", .scale = 1},
  }};
  BodyIndex anchor_ = kInvalidBodyIndex;
  BodyIndex mass_ = kInvalidBodyIndex;
  JointIndex spring_ = kInvalidJointIndex;
  Scalar equilibrium_y_{};
  int step_ = 0;
  Scalar previous_displacement_{};
  Scalar first_crossing_{};
  Scalar last_crossing_{};
  int crossings_ = 0;
  bool exploded_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_09_SPRING_SPRING_SAMPLE_H_
