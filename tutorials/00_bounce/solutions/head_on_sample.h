#ifndef TUTORIALS_00_BOUNCE_SOLUTIONS_HEAD_ON_SAMPLE_H_
#define TUTORIALS_00_BOUNCE_SOLUTIONS_HEAD_ON_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 00, exercise "head on": two balls collide, without gravity.
//
// Ball A (1 kg) moves right at v (parameter "Speed A") toward ball B, at rest,
// of mass m_B (parameter "Mass B"). Both have the restitution e (parameter
// "Restitution"). The contact's impulse is the same on both, opposite, so the
// momentum m_A v_A + m_B v_B doesn't change; the restitution sets the
// separating speed to e times the approach speed. Together:
//
//   v_A' = (m_A v - e m_B v) / (m_A + m_B)
//   v_B' = (1 + e) m_A v / (m_A + m_B)
//
// Equal masses with e = 1 swap their velocities (A stops, B leaves at v). An
// approach slower than the world's restitution threshold, 1 m/s, doesn't
// bounce: the two leave together at the common velocity m_A v / (m_A + m_B)
// (and the push-out of the overlap on top).
//
// Readouts: the predicted and measured velocities after the contact (one
// second after it starts), and the momentum before and after. Drawn: the two
// balls and their velocities.
//
// The world has no gravity, in meters and seconds.
class HeadOnSample final : public Sample {
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

  [[nodiscard]] Scalar predicted_velocity_a() const;
  [[nodiscard]] Scalar predicted_velocity_b() const;
  [[nodiscard]] Scalar velocity_a() const;
  [[nodiscard]] Scalar velocity_b() const;
  [[nodiscard]] Scalar momentum() const;
  [[nodiscard]] Scalar initial_momentum() const;
  // Set one second after the first contact
  [[nodiscard]] bool is_measured() const { return is_measured_; }
  [[nodiscard]] std::optional<int> contact_step() const {
    return contact_step_;
  }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kMassB = 1;
  static constexpr std::size_t kRestitution = 2;

 private:
  [[nodiscard]] Scalar speed() const { return parameters_[kSpeed].ToScalar(); }
  [[nodiscard]] Scalar mass_b() const { return parameters_[kMassB].ToScalar(); }
  [[nodiscard]] Scalar restitution() const {
    return parameters_[kRestitution].ToScalar();
  }

  PhysicsWorld world_{{.gravity = Vec2f::Zero(),
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 0}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Speed A",
       .unit = "m/s",
       .min = 5,
       .max = 50,
       .step = 1,
       .scale = 10,
       .value = 20},
      {.name = "Mass B",
       .unit = "kg",
       .min = 5,
       .max = 40,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Restitution",
       .unit = "",
       .min = 0,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 100},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Pred. A after", .unit = "m/s", .scale = 1000},
      {.name = "A after", .unit = "m/s", .scale = 1000},
      {.name = "Pred. B after", .unit = "m/s", .scale = 1000},
      {.name = "B after", .unit = "m/s", .scale = 1000},
      {.name = "Momentum before", .unit = "kg m/s", .scale = 1000},
      {.name = "Momentum after", .unit = "kg m/s", .scale = 1000},
  }};
  BodyIndex ball_a_ = kInvalidBodyIndex;
  BodyIndex ball_b_ = kInvalidBodyIndex;
  int step_ = 0;
  std::optional<int> contact_step_;
  bool is_measured_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_00_BOUNCE_SOLUTIONS_HEAD_ON_SAMPLE_H_
