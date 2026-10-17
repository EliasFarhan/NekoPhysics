#ifndef TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_PASSES_SAMPLE_H_
#define TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_PASSES_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 04, exercise "one pass vs eight": a flat landing on two points,
// solved with `P` passes (parameter "Passes", the world's
// velocity_iterations).
//
// The box of chapter 04 (2 x 1 m, 1 kg, I = 5/12 kg m^2) lands flat at 2 m/s
// without gravity, with restitution `e` (parameter "Restitution"). Its
// manifold has two points, at x = -1 (point 1) and x = +1 (point 2) from the
// center, each with k = 1/m + x^2 / I = 3.4 per kg and the target speed
// e * 2 m/s. The solver visits point 1, then point 2, and repeats:
//
//   vn_i = v + w x_i                    (the point's vertical velocity)
//   dj   = (target - vn_i) / k          (clamped so the total stays >= 0)
//   v   += dj / m,   w += x_i dj / I
//
// One pass leaves the box spinning: point 1's impulse turns the box, which
// drives point 2 down faster, and point 2 then pushes harder. The exact
// answer, the same impulse at both points, is v = 2 e and no spin; each pass
// gets closer. The sample runs that loop by hand for `P` passes and compares
// it with the world. Drawn as chapter 04, with the manifold's two corners in
// yellow after the hit.
class PassesSample final : public Sample {
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

  // Velocity and spin of the box after `passes` passes of the loop above
  struct Motion {
    Scalar velocity;
    Scalar spin;
  };
  [[nodiscard]] Motion predicted(int passes) const;
  // The box's vertical velocity and spin after the impact step, empty
  // before
  [[nodiscard]] std::optional<Motion> measured() const { return measured_; }
  // The exact answer: the box leaves at e times its speed, without spin
  [[nodiscard]] Scalar exact_velocity() const;

  static constexpr std::size_t kPasses = 0;
  static constexpr std::size_t kRestitution = 1;

 private:
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Passes",
       .unit = "",
       .min = 1,
       .max = 8,
       .step = 1,
       .scale = 1,
       .value = 1},
      {.name = "Restitution",
       .unit = "",
       .min = 20,
       .max = 100,
       .step = 10,
       .scale = 100,
       .value = 50},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Predicted vy", .unit = "m/s", .scale = 1000},
      {.name = "Measured", .unit = "m/s", .scale = 1000},
      {.name = "Predicted spin", .unit = "rad/s", .scale = 1000},
      {.name = "Measured", .unit = "rad/s", .scale = 1000},
      {.name = "Exact vy", .unit = "m/s", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  std::optional<Motion> measured_;
  Vec2f impact_center_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_PASSES_SAMPLE_H_
