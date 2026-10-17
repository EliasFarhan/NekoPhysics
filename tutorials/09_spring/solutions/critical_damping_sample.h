#ifndef TUTORIALS_09_SPRING_SOLUTIONS_CRITICAL_DAMPING_SAMPLE_H_
#define TUTORIALS_09_SPRING_SOLUTIONS_CRITICAL_DAMPING_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 09, exercise "critical damping": how fast a damped spring settles.
//
// The chapter's spring (rest length 2 m, stiffness k from parameter
// "Stiffness", mass m from parameter "Mass", released at rest 0.5 m below
// its equilibrium), with its damping given as a damping ratio zeta
// (parameter "Damping ratio") instead of a coefficient:
//
//   c = 2 zeta sqrt(k m)
//
// Below zeta = 1 the mass rings around the equilibrium: its first overshoot
// is 0.5 m * exp(-zeta pi / sqrt(1 - zeta^2)) past it. At zeta = 1, critical
// damping, it returns as fast as it can without overshooting:
// x(t) = 0.5 m (1 + w t) exp(-w t), w = sqrt(k / m). Above 1 it creeps back
// more and more slowly, at the rate w (zeta - sqrt(zeta^2 - 1)). The 1 %
// band is crossed soonest a little below zeta = 1, where a small overshoot
// stays inside it.
//
// The damping force is explicit too, computed from the speed at the start
// of the step: it removes a little more than the exact motion would, the
// amplitude shrinking by sqrt(1 - c dt / m) per step instead of
// exp(-c dt / (2 m)), so the overshoots are a little smaller than the
// formula's.
//
// Readouts: the damping coefficient, the settling time (the last time the
// mass was more than 1 % of the release, 5 mm, from the equilibrium, over
// 10 s) and the overshoot (how far past the equilibrium it went). Drawn: the
// anchor in grey, the spring as a white zigzag, the mass in orange, the
// equilibrium as a green line and the 5 mm band as two grey ticks.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class CriticalDampingSample final : public Sample {
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

  // 2 zeta sqrt(k m), in N s/m
  [[nodiscard]] Scalar damping() const;
  // Last time outside the band, in seconds since the release
  [[nodiscard]] Scalar settling_time() const { return settling_time_; }
  // Largest distance past the equilibrium (above it), in meters
  [[nodiscard]] Scalar overshoot() const { return overshoot_; }
  // Distance of the mass below the equilibrium, in meters
  [[nodiscard]] Scalar displacement() const;

  static constexpr std::size_t kStiffness = 0;
  static constexpr std::size_t kMass = 1;
  static constexpr std::size_t kDampingRatio = 2;
  static constexpr Scalar kRelease = Ratio(1, 2);
  // 1 % of the release
  static constexpr Scalar kBand = Ratio(1, 200);

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
       .max = 400,
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
      {.name = "Damping ratio",
       .unit = "",
       .min = 0,
       .max = 30,
       .step = 1,
       .scale = 10,
       .value = 10},
  }};
  std::array<SampleReadout, 3> readouts_{{
      {.name = "Damping", .unit = "N s/m", .scale = 1000},
      {.name = "Settling time", .unit = "s", .scale = 1000},
      {.name = "Overshoot", .unit = "mm", .scale = 10},
  }};
  BodyIndex anchor_ = kInvalidBodyIndex;
  BodyIndex mass_ = kInvalidBodyIndex;
  Scalar equilibrium_y_{};
  int step_ = 0;
  Scalar settling_time_{};
  Scalar overshoot_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_09_SPRING_SOLUTIONS_CRITICAL_DAMPING_SAMPLE_H_
