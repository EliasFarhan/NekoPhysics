#ifndef TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_RIDER_CARRY_SAMPLE_H_
#define TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_RIDER_CARRY_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 14, exercise "rider carry": three ways a player rides a platform.
//
// Three kinematic platforms side by side follow the same patrol (amplitude
// 1.2 m, parameter "Period" T), each with the chapter's player box on it:
// - left, a frictionless player and no carry: the platform slides away
//   under it and it falls to the ground;
// - middle, a frictionless player with the game-side carry: while the pair
//   touches, the player's horizontal velocity is the platform's (Splash's
//   script). It rides exactly, whatever the acceleration;
// - right, a player with friction 0.5 and no carry: friction carries it as
//   long as the platform's acceleration stays under mu g = 4.9 m/s^2. The
//   patrol's largest acceleration is w^2 A (w = 2 pi / T): 3.0 m/s^2 at
//   T = 4 s, 11.8 m/s^2 at T = 2 s, where the player slips and may fall off.
//
// Readouts: each player's offset from its platform's center, the patrol's
// peak acceleration w^2 A and mu g. Drawn: the ground, the platforms in teal,
// the players in orange (red once they fell off).
class RiderCarrySample final : public Sample {
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

  // The player's offset from its platform's center, and whether it still
  // stands on it (0 no carry, 1 carry, 2 friction)
  [[nodiscard]] Scalar offset(std::size_t rider) const;
  [[nodiscard]] Scalar max_offset(std::size_t rider) const {
    return max_offset_[rider];
  }
  [[nodiscard]] bool is_on_platform(std::size_t rider) const;
  [[nodiscard]] Scalar peak_acceleration() const { return peak_acceleration_; }

  static constexpr std::size_t kPeriod = 0;
  static constexpr std::size_t kRiderCount = 3;
  static constexpr int kMeasuredSteps = 600;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 8,
                       .max_colliders = 8,
                       .max_touching_pairs = 8,
                       .max_broadphase_pairs = 16,
                       .max_joints = 0,
                       .max_passing_pairs = 0}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Period",
       .unit = "s",
       .min = 2,
       .max = 10,
       .step = 1,
       .scale = 1,
       .value = 4},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "No carry", .unit = "mm", .scale = 1},
      {.name = "Carry", .unit = "mm", .scale = 10},
      {.name = "Friction", .unit = "mm", .scale = 1},
      {.name = "Peak w^2 A", .unit = "m/s^2", .scale = 100},
      {.name = "mu g", .unit = "m/s^2", .scale = 100},
  }};
  Aabbf ground_{};
  std::array<ColliderIndex, kRiderCount> platforms_{};
  std::array<ColliderIndex, kRiderCount> riders_{};
  std::array<Vec2f, kRiderCount> centers_{};
  std::array<Scalar, kRiderCount> max_offset_{};
  Scalar peak_acceleration_{};
  int period_steps_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_RIDER_CARRY_SAMPLE_H_
