#ifndef TUTORIALS_11_JETPACK_JETPACK_SAMPLE_H_
#define TUTORIALS_11_JETPACK_JETPACK_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 11, jetpack: forces push over time, impulses push at once.
//
// A player of mass m (parameter "Mass") stands on the ground. A scripted
// input, counted in fixed steps so every run is the same, drives three
// gameplay pushes written here, in the game, over the physics API:
//
// - the jetpack: a force T up (parameter "Thrust") from step 30 to step 210,
//   applied every step with ApplyForce. The player rises with the
//   acceleration a = T / m - g s, s being the body's gravity scale
//   (parameter "Grav. scale"), and hovers when T = m g s;
// - the wind: above 2 m, a drag-like force c (v_wind - v_x) toward the wind
//   speed (parameter "Wind"), c = 2 N s/m. Step by step the gap to the wind
//   speed shrinks by (1 - c dt / m), so after n steps in the wind
//   v_x = v_wind (1 - (1 - c dt / m)^n): an exponential approach with time
//   constant m / c;
// - a knockback at step 150: one impulse J (parameter "Knockback") to the
//   left with ApplyImpulse. The velocity jumps by J / m in that step, a
//   heavier player is kicked less, and it doesn't depend on the step rate.
//
// Forces from several sources add up, so the game decides who wins: here
// the knockback cuts the jetpack for 0.3 s (18 steps) so the hit is felt.
// That rule lives in the game's controller, not in the physics.
//
// Readouts: the hover thrust m g s; the predicted and measured
// acceleration under thrust (measured between steps 35 and 45); the
// predicted and measured drift speed just before the knockback; the
// predicted and measured velocity jump of the knockback (measured: the
// change over that step minus the step's wind force). Drawn: the ground,
// the wind line at 2 m, the player in orange, its velocity, and the
// jetpack flame in red while it pushes.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class JetpackSample final : public Sample {
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

  // m g s, in newtons
  [[nodiscard]] Scalar hover_thrust() const;
  // T / m - g s, in m/s^2
  [[nodiscard]] Scalar predicted_acceleration() const;
  // Between steps 35 and 45, once there
  [[nodiscard]] std::optional<Scalar> measured_acceleration() const {
    return measured_acceleration_;
  }
  // v_wind (1 - (1 - c dt / m)^n) for the n wind steps before the knockback
  [[nodiscard]] Scalar predicted_drift() const;
  // The horizontal speed just before the knockback, once there
  [[nodiscard]] std::optional<Scalar> measured_drift() const {
    return measured_drift_;
  }
  // -J / m, in m/s
  [[nodiscard]] Scalar predicted_kick() const;
  [[nodiscard]] std::optional<Scalar> measured_kick() const {
    return measured_kick_;
  }
  [[nodiscard]] Vec2f player_velocity() const {
    return world_.body(player_).velocity;
  }
  [[nodiscard]] Vec2f player_position() const {
    return world_.body(player_).position;
  }
  [[nodiscard]] bool thrusting() const { return thrusting_; }
  [[nodiscard]] int wind_steps() const { return wind_steps_; }

  static constexpr std::size_t kThrust = 0;
  static constexpr std::size_t kMass = 1;
  static constexpr std::size_t kGravityScale = 2;
  static constexpr std::size_t kWind = 3;
  static constexpr std::size_t kKnockback = 4;
  // The script, in steps of 1/60 s
  static constexpr int kThrustStart = 30;
  static constexpr int kThrustEnd = 210;
  static constexpr int kKnockbackStep = 150;
  static constexpr int kKnockbackLock = 18;
  static constexpr int kAccelerationFrom = 35;
  static constexpr int kAccelerationTo = 45;

 private:
  [[nodiscard]] Scalar mass() const { return parameters_[kMass].ToScalar(); }
  [[nodiscard]] Scalar gravity_scale() const {
    return parameters_[kGravityScale].ToScalar();
  }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 2,
                       .max_broadphase_pairs = 2,
                       .max_joints = 0}};
  std::array<SampleParameter, 5> parameters_{{
      {.name = "Thrust",
       .unit = "N",
       .min = 0,
       .max = 60,
       .step = 1,
       .scale = 1,
       .value = 30},
      {.name = "Mass",
       .unit = "kg",
       .min = 5,
       .max = 40,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Grav. scale",
       .unit = "",
       .min = -10,
       .max = 20,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Wind",
       .unit = "m/s",
       .min = -50,
       .max = 50,
       .step = 1,
       .scale = 10,
       .value = 30},
      {.name = "Knockback",
       .unit = "N s",
       .min = 0,
       .max = 100,
       .step = 1,
       .scale = 10,
       .value = 40},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Hover thrust", .unit = "N", .scale = 100},
      {.name = "Pred. accel", .unit = "m/s2", .scale = 100},
      {.name = "Meas. accel", .unit = "m/s2", .scale = 100},
      {.name = "Pred. drift", .unit = "m/s", .scale = 1000},
      {.name = "Meas. drift", .unit = "m/s", .scale = 1000},
      {.name = "Pred. kick", .unit = "m/s", .scale = 1000},
      {.name = "Meas. kick", .unit = "m/s", .scale = 1000},
  }};
  Aabbf ground_{};
  Scalar wind_height_{};
  BodyIndex player_ = kInvalidBodyIndex;
  int step_ = 0;
  int wind_steps_ = 0;
  // Wind steps before the knockback, for the drift prediction
  int drift_steps_ = 0;
  // Last step the jetpack is cut by the knockback
  int lock_until_ = -1;
  bool thrusting_ = false;
  Scalar velocity_at_from_{};
  std::optional<Scalar> measured_acceleration_;
  std::optional<Scalar> measured_drift_;
  std::optional<Scalar> measured_kick_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_11_JETPACK_JETPACK_SAMPLE_H_
