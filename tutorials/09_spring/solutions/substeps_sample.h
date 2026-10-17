#ifndef TUTORIALS_09_SPRING_SOLUTIONS_SUBSTEPS_SAMPLE_H_
#define TUTORIALS_09_SPRING_SOLUTIONS_SUBSTEPS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 09, exercise "substeps": a smaller time step for a stiffer spring.
//
// The chapter's spring (rest length 2 m, stiffness k from parameter
// "Stiffness", mass m from parameter "Mass", undamped, released at rest
// 0.5 m below its equilibrium), in a world whose time step is dt / n, with
// n world steps per frame (parameter "Substeps"). The stability condition
// w (dt / n) < 2 allows n^2 times the stiffness for n times the steps, and
// the period error of semi-implicit Euler, about (w dt / n)^2 / 24, shrinks
// by n^2 too. The chapter's 2000 N/m on 0.1 kg (w dt = 2.36) explodes with
// one step per frame and swings with two.
//
// In Fixed16 the frame's 1092/65536 s splits exactly into 2, 3, 4 and 6
// substeps; 5 rounds each substep to 218/65536 s. The prediction takes
// sqrt(m) / sqrt(k): m / k itself (0.1 / 2000) is 3 units of 1/65536.
//
// Readouts: w times the substep, the predicted and measured periods (from
// the crossings of the equilibrium), and whether it exploded (past 1.5 m,
// before k x / m overflows Fixed16's 32767 m/s^2; the sample then stops).
// Drawn as in the chapter.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class SubstepsSample final : public Sample {
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
  [[nodiscard]] std::optional<Scalar> measured_period() const;
  // sqrt(k / m) times the world's time step
  [[nodiscard]] Scalar omega_dt() const;
  [[nodiscard]] Scalar substep() const { return world_.dt(); }
  [[nodiscard]] bool exploded() const { return exploded_; }

  static constexpr std::size_t kStiffness = 0;
  static constexpr std::size_t kMass = 1;
  static constexpr std::size_t kSubsteps = 2;
  static constexpr Scalar kRelease = Ratio(1, 2);

 private:
  [[nodiscard]] Scalar stiffness() const {
    return parameters_[kStiffness].ToScalar();
  }
  [[nodiscard]] Scalar mass() const { return parameters_[kMass].ToScalar(); }
  [[nodiscard]] Scalar displacement() const;

  // Rebuilt in Begin: the time step comes from the parameters
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1, .max_joints = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Stiffness",
       .unit = "N/m",
       .min = 20,
       .max = 2000,
       .step = 20,
       .scale = 1,
       .value = 2000},
      {.name = "Mass",
       .unit = "kg",
       .min = 1,
       .max = 20,
       .step = 1,
       .scale = 10,
       .value = 1},
      {.name = "Substeps",
       .unit = "",
       .min = 1,
       .max = 6,
       .step = 1,
       .scale = 1,
       .value = 2},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "w dt / n", .unit = "", .scale = 1000},
      {.name = "Predicted period", .unit = "s", .scale = 10000},
      {.name = "Measured period", .unit = "s", .scale = 10000},
      {.name = "Exploded", .unit = "", .scale = 1},
  }};
  BodyIndex anchor_ = kInvalidBodyIndex;
  BodyIndex mass_ = kInvalidBodyIndex;
  Scalar equilibrium_y_{};
  int step_ = 0;
  Scalar previous_displacement_{};
  Scalar first_crossing_{};
  Scalar last_crossing_{};
  int crossings_ = 0;
  bool exploded_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_09_SPRING_SOLUTIONS_SUBSTEPS_SAMPLE_H_
