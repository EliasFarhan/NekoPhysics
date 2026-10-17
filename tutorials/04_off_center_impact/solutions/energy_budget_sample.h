#ifndef TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_ENERGY_BUDGET_SAMPLE_H_
#define TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_ENERGY_BUDGET_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 04, exercise "energy budget": where does the energy of a corner
// hit go?
//
// The experiment of chapter 04: a 1 kg box (parameter "Width", 1 m tall)
// turned by `a` degrees (parameter "Tilt") falls at 2 m/s without gravity
// onto a static, frictionless ground with restitution `e` (parameter
// "Restitution"). Before the hit its energy is all linear, m v^2 / 2. After,
// it is split between the fall and the spin:
//
//   kinetic energy    m v'^2 / 2 + I w'^2 / 2
//   lost              (1 - e^2) vn^2 / (2 k),   k = 1/m + l^2 / I
//
// where vn = -v is the corner's approach speed and l the corner's lever arm.
// An elastic hit (e = 1) keeps all the energy; the spin takes part of it.
// A flat box (k = 1/m) would lose (1 - e^2) m v^2 / 2: more, because the
// corner's effective mass 1/k is smaller than m. The formula holds when the
// bounce `e |vn|` is faster than the solver's push-out of the overlap (true
// here for e >= 0.2). Drawn as chapter 04.
class EnergyBudgetSample final : public Sample {
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

  // m (w^2 + h^2) / 12, in kg m^2
  [[nodiscard]] Scalar inertia() const;
  // m v^2 / 2 before the hit, in joules
  [[nodiscard]] Scalar energy_before() const;
  // The values below are empty until the box hit the ground.
  [[nodiscard]] std::optional<Scalar> linear_energy_after() const;
  [[nodiscard]] std::optional<Scalar> rotation_energy_after() const;
  [[nodiscard]] std::optional<Scalar> predicted_loss() const;
  [[nodiscard]] std::optional<Scalar> measured_loss() const;
  // (1 - e^2) m v^2 / 2: what a flat landing would lose
  [[nodiscard]] Scalar flat_loss() const;

  static constexpr std::size_t kTilt = 0;
  static constexpr std::size_t kRestitution = 1;
  static constexpr std::size_t kWidth = 2;

 private:
  [[nodiscard]] Vec2f half_size() const;

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Tilt",
       .unit = "deg",
       .min = 5,
       .max = 40,
       .step = 5,
       .scale = 1,
       .value = 20},
      {.name = "Restitution",
       .unit = "",
       .min = 20,
       .max = 100,
       .step = 10,
       .scale = 100,
       .value = 50},
      {.name = "Width",
       .unit = "m",
       .min = 100,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 200},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Energy before", .unit = "J", .scale = 1000},
      {.name = "Linear after", .unit = "J", .scale = 1000},
      {.name = "Spin after", .unit = "J", .scale = 1000},
      {.name = "Total after", .unit = "J", .scale = 1000},
      {.name = "Predicted loss", .unit = "J", .scale = 1000},
      {.name = "Measured loss", .unit = "J", .scale = 1000},
      {.name = "Loss if flat", .unit = "J", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  // Set by the impact step
  std::optional<Scalar> lever_arm_;
  std::optional<Scalar> velocity_after_;
  std::optional<Scalar> spin_after_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_ENERGY_BUDGET_SAMPLE_H_
