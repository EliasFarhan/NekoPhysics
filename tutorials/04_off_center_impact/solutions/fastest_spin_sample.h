#ifndef TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_FASTEST_SPIN_SAMPLE_H_
#define TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_FASTEST_SPIN_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 04, exercise "fastest spin": which tilt makes a 3 m wide box spin
// fastest when its corner hits the ground?
//
// The experiment of chapter 04 (a 1 kg box, 1 m tall, falling at 2 m/s
// without gravity onto a static, frictionless ground), with the width
// (parameter "Width"), the tilt in steps of one degree (parameter "Tilt")
// and the restitution (parameter "Restitution"). The corner at lever arm l
// gets the impulse j = (1 + e) v / (1/m + l^2 / I) and the box spins at
//
//   |w| = |l| j / I = (1 + e) v |l| / (I/m + l^2)
//
// which is largest when |l| equals the radius of gyration k = sqrt(I / m),
// at |w| = (1 + e) v / (2 k). The lever arm of the lowest corner is
// l = hx cos a - hy sin a (in size), so the best tilt solves
// hx cos a - hy sin a = k. The sample scans the integer tilts from 5 to 40
// degrees with that formula ("Fastest tilt"), and measures the spin of the
// current tilt. Drawn as chapter 04: the ground in dark grey, the box in
// orange, after the hit the lever arm in grey and the corner in yellow.
class FastestSpinSample final : public Sample {
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
  // sqrt(I / m), in meters
  [[nodiscard]] Scalar gyration_radius() const;
  // Size of the lowest corner's lever arm at a tilt, hx cos a - hy sin a
  [[nodiscard]] Scalar lever_arm(int tilt_degrees) const;
  // Predicted spin after the hit at a tilt (negative: clockwise)
  [[nodiscard]] Scalar predicted_spin(int tilt_degrees) const;
  // The integer tilt from 5 to 40 degrees with the fastest predicted spin
  [[nodiscard]] int fastest_tilt() const;
  // (1 + e) v / (2 k), the fastest spin any lever arm can give
  [[nodiscard]] Scalar spin_bound() const;
  // The box's spin right after the impact step, empty before
  [[nodiscard]] std::optional<Scalar> measured_spin() const {
    return measured_spin_;
  }

  static constexpr std::size_t kTilt = 0;
  static constexpr std::size_t kWidth = 1;
  static constexpr std::size_t kRestitution = 2;

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
       .step = 1,
       .scale = 1,
       .value = 36},
      {.name = "Width",
       .unit = "m",
       .min = 100,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 300},
      {.name = "Restitution",
       .unit = "",
       .min = 10,
       .max = 100,
       .step = 10,
       .scale = 100,
       .value = 100},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Lever arm", .unit = "m", .scale = 1000},
      {.name = "Gyration radius", .unit = "m", .scale = 1000},
      {.name = "Predicted spin", .unit = "rad/s", .scale = 1000},
      {.name = "Measured spin", .unit = "rad/s", .scale = 1000},
      {.name = "Fastest tilt", .unit = "deg", .scale = 1},
      {.name = "Spin bound", .unit = "rad/s", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  std::optional<Scalar> measured_spin_;
  Vec2f impact_center_{};
  Vec2f impact_point_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_04_OFF_CENTER_IMPACT_SOLUTIONS_FASTEST_SPIN_SAMPLE_H_
