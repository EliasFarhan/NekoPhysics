#ifndef TUTORIALS_12_AIMING_AIMING_SAMPLE_H_
#define TUTORIALS_12_AIMING_AIMING_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/math/angle.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Ballistics on flat ground, shared by the chapter and its solutions. A
// projectile leaves at speed v and angle theta above the horizontal, gravity
// g pulls it down, and it comes back to its launch height after the flight
// time T = 2 v sin(theta) / g, at the range R = v^2 sin(2 theta) / g.

// The low-arc launch angle that reaches the horizontal distance d (not
// negative): theta = asin(g d / v^2) / 2. Beyond the largest range v^2 / g the
// arcsine's input passes 1, Asin clamps it, and theta is 45 degrees: the shot
// goes as far as it can and falls short.
[[nodiscard]] Radian LowArcAngle(Scalar distance, Scalar speed, Scalar gravity);
// T = 2 v sin(theta) / g
[[nodiscard]] Scalar FlightTime(Radian angle, Scalar speed, Scalar gravity);

// The aim at a target moving at a constant horizontal speed u, starting at
// the distance d0: the projectile must go to d0 + u T, but T depends on the
// angle, which depends on the distance. A fixed-point iteration solves it:
// t0 = 0 (aim where the target is now), then t(k+1) = T(theta(d0 + u t(k))).
// Each iteration shrinks the error by about u dT/dd, so a few are enough.
struct LeadSolution {
  // Angle for the distance d0 + u lead_time
  Radian angle{};
  // The last flight time guess, used for the aim point
  Scalar lead_time{};
  // Where the shot is aimed, from the launch point
  Scalar aim_distance{};
};
[[nodiscard]] LeadSolution SolveLead(Scalar distance, Scalar target_speed,
                                     Scalar speed, Scalar gravity,
                                     int iterations);

// Watches a projectile come back down through its launch height, and where a
// target moving horizontally is at that moment. Both positions are known only
// after each step, so the crossing is interpolated linearly between the
// last step above and the first step below.
struct Crossing {
  // Since the launch, in seconds
  Scalar time{};
  Scalar projectile_x{};
  Scalar target_x{};
  // The projectile's velocity during the step that crossed
  Vec2f velocity{};
};

class CrossingWatch {
 public:
  // At the launch: the projectile is at the launch height
  void Start(Vec2f projectile, Scalar target_x);
  // After each step; returns the crossing once, at the step it happens
  [[nodiscard]] std::optional<Crossing> Update(Vec2f projectile, Vec2f velocity,
                                               Scalar target_x, Scalar dt);

 private:
  Vec2f previous_{};
  Scalar previous_target_x_{};
  Scalar height_{};
  int steps_ = 0;
  bool is_watching_ = false;
};

// Chapter 12, aiming: a turret hits a moving target with inverse
// trigonometry that replays.
//
// The turret's muzzle and a drone (the target, a kinematic box) are at the
// same height. The drone starts at distance d (parameter "Distance") and
// flies at u (parameter "Target speed"). The turret solves the lead with
// SolveLead (4 iterations), whose angle comes from Asin, and fires a ball at
// v (parameter "Speed"). Every 3 seconds the round starts again from the
// same state.
//
// The engine integrates with semi-implicit Euler (velocity first, then
// position), and that integrator lands one step early: its heights are
// y(n dt) = v_y t - g t (t + dt) / 2, which is back to 0 at T - dt instead of
// T. So the hit error, the ball's x minus the drone's x when the ball comes
// back to the launch height, is predicted to be (v_x - u) (T - dt) - d: about
// -(v_x - u) dt for a target in range. The measured error matches it within
// 2 mm: the crossing is read on the straight line between two steps, which
// cuts the parabola's curve, most at flat angles.
//
// With Fixed16, Asin is integer-only, so the angle, and every round, is the
// same bit for bit on any machine (the PC and the DS); std::asin promises
// that only on one machine with one build. "Same rounds" counts the rounds
// whose hit error has exactly the bits of the first round.
//
// Readouts: the launch angle; the lead time; the predicted (T - dt) and
// measured flight time; the predicted and measured hit error; the impact
// angle (Atan2 of the ball's velocity when it crosses, close to minus the
// launch angle); the rounds identical to the first. Drawn: the ground, the
// launch height, the turret and its barrel, the drone in green, the aim
// point in yellow, the ball in orange and its path.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class AimingSample final : public Sample {
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

  [[nodiscard]] const LeadSolution& lead() const { return lead_; }
  // T - dt, where the integrator brings the ball back to its launch height
  [[nodiscard]] Scalar predicted_flight_time() const;
  // (v_x - u) (T - dt) - d
  [[nodiscard]] Scalar predicted_error() const;
  // Of the last round, once the ball has come down
  [[nodiscard]] std::optional<Scalar> measured_flight_time() const {
    return measured_flight_time_;
  }
  [[nodiscard]] std::optional<Scalar> measured_error() const {
    return measured_error_;
  }
  [[nodiscard]] std::optional<Radian> impact_angle() const {
    return impact_angle_;
  }
  [[nodiscard]] int rounds() const { return rounds_; }
  // Rounds after the first whose hit error has the first one's bits
  [[nodiscard]] int same_rounds() const { return same_rounds_; }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kDistance = 1;
  static constexpr std::size_t kTargetSpeed = 2;
  static constexpr int kLeadIterations = 4;
  static constexpr int kRoundSteps = 180;

 private:
  void StartRound();

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 3,
                       .max_colliders = 3,
                       .max_touching_pairs = 3,
                       .max_broadphase_pairs = 3,
                       .max_joints = 0}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 50,
       .max = 140,
       .step = 1,
       .scale = 10,
       .value = 100},
      {.name = "Distance",
       .unit = "m",
       .min = 10,
       .max = 90,
       .step = 1,
       .scale = 10,
       .value = 60},
      {.name = "Target speed",
       .unit = "m/s",
       .min = -20,
       .max = 20,
       .step = 1,
       .scale = 10,
       .value = 10},
  }};
  std::array<SampleReadout, 8> readouts_{{
      {.name = "Launch angle", .unit = "deg", .scale = 100},
      {.name = "Lead time", .unit = "s", .scale = 1000},
      {.name = "Pred. flight", .unit = "s", .scale = 1000},
      {.name = "Meas. flight", .unit = "s", .scale = 1000},
      {.name = "Pred. error", .unit = "m", .scale = 1000},
      {.name = "Meas. error", .unit = "m", .scale = 1000},
      {.name = "Impact angle", .unit = "deg", .scale = 100},
      {.name = "Same rounds", .unit = "", .scale = 1},
  }};
  Aabbf ground_{};
  Vec2f muzzle_{};
  Vec2f target_start_{};
  BodyIndex ball_ = kInvalidBodyIndex;
  BodyIndex target_ = kInvalidBodyIndex;
  LeadSolution lead_{};
  CrossingWatch watch_{};
  // The ball's position after each step of the round, until it crosses
  std::array<Vec2f, kRoundSteps + 1> path_{};
  std::size_t path_size_ = 0;
  int step_in_round_ = 0;
  int rounds_ = 0;
  int same_rounds_ = 0;
  bool has_crossed_ = false;
  Scalar first_error_{};
  std::optional<Scalar> measured_flight_time_;
  std::optional<Scalar> measured_error_;
  std::optional<Radian> impact_angle_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_12_AIMING_AIMING_SAMPLE_H_
