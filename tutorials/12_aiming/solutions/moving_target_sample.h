#ifndef TUTORIALS_12_AIMING_SOLUTIONS_MOVING_TARGET_SAMPLE_H_
#define TUTORIALS_12_AIMING_SOLUTIONS_MOVING_TARGET_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/math/angle.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "tutorials/12_aiming/aiming_sample.h"

namespace neko::tutorial {

// Chapter 12, exercise "moving target": how many lead iterations are enough.
//
// A drone starts at the distance d (parameter "Distance") and flies at u
// (parameter "Target speed"). Four balls leave together at v (parameter
// "Speed"), aimed by SolveLead with 0, 1, 2 and 3 iterations. With 0 the
// turret aims where the drone is now and misses by about u T. Each
// iteration multiplies the remaining miss by about u dT/dd (dT/dd: how much
// longer the flight gets per meter of range), so the misses shrink
// geometrically, down to the integrator's own error, (v_x - u) (T - dt) - d,
// about -(v_x - u) dt.
//
// Readouts: the measured miss of each ball (its x minus the drone's x when
// it comes back to the launch height), the lead time and the flight time T
// of the 3-iteration aim, and its predicted miss. Drawn: the ground, the
// launch height, the drone in green, the balls in red, orange, yellow and
// lime (0 to 3 iterations), with their paths.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class MovingTargetSample final : public Sample {
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

  static constexpr std::size_t kShotCount = 4;
  // The aim of the ball with that many iterations
  [[nodiscard]] const LeadSolution& lead(std::size_t iterations) const {
    return leads_[iterations];
  }
  [[nodiscard]] std::optional<Scalar> miss(std::size_t iterations) const {
    return misses_[iterations];
  }
  // (v_x - u) (T - dt) - d, for that many iterations
  [[nodiscard]] Scalar predicted_miss(std::size_t iterations) const;

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kDistance = 1;
  static constexpr std::size_t kTargetSpeed = 2;

 private:
  static constexpr std::size_t kMaxPath = 240;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = kShotCount + 1,
                       .max_colliders = kShotCount + 1,
                       .max_touching_pairs = 16,
                       .max_broadphase_pairs = 16,
                       .max_joints = 0}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 60,
       .max = 140,
       .step = 1,
       .scale = 10,
       .value = 100},
      {.name = "Distance",
       .unit = "m",
       .min = 10,
       .max = 80,
       .step = 1,
       .scale = 10,
       .value = 50},
      {.name = "Target speed",
       .unit = "m/s",
       .min = -30,
       .max = 30,
       .step = 1,
       .scale = 10,
       .value = 20},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Miss, 0 it.", .unit = "m", .scale = 1000},
      {.name = "Miss, 1 it.", .unit = "m", .scale = 1000},
      {.name = "Miss, 2 it.", .unit = "m", .scale = 1000},
      {.name = "Miss, 3 it.", .unit = "m", .scale = 1000},
      {.name = "Lead, 3 it.", .unit = "s", .scale = 1000},
      {.name = "Flight T, 3 it.", .unit = "s", .scale = 1000},
      {.name = "Pred. miss, 3 it.", .unit = "m", .scale = 1000},
  }};
  Aabbf ground_{};
  Vec2f muzzle_{};
  BodyIndex target_ = kInvalidBodyIndex;
  std::array<BodyIndex, kShotCount> balls_{};
  std::array<LeadSolution, kShotCount> leads_{};
  std::array<CrossingWatch, kShotCount> watches_{};
  std::array<std::array<Vec2f, kMaxPath>, kShotCount> paths_{};
  std::array<std::size_t, kShotCount> path_sizes_{};
  std::array<std::optional<Scalar>, kShotCount> misses_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_12_AIMING_SOLUTIONS_MOVING_TARGET_SAMPLE_H_
