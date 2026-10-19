#ifndef TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_SPEED_THRESHOLD_SAMPLE_H_
#define TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_SPEED_THRESHOLD_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 15, exercise "speed threshold": the speed where a plain shot (no
// continuous collision) starts to miss a wall.
//
// Eight lanes of plain solid shots (radius r = 0.1 m) against walls of
// thickness t (parameter "Wall", 0.2 m). Rounds at 5, 6, 7... 60 m/s: each
// round fires the eight shots from 2 m before the wall, each one an eighth
// of a step v dt farther back, so the steps of the eight lanes fall at eight
// evenly spread places on the way. A shot misses when it ends past the
// wall's middle.
//
// The first step that overlaps the wall finds the shot's center up to v dt
// past the place where it touches; the contact pushes it back while the
// center is before the wall's middle, r + t / 2 deep, and pushes it out
// through the far side past it. So no shot misses while
//
//   v dt <= r + t / 2          v <= (r + t / 2) / dt
//
// (12 m/s for the default wall), and just above, the lane whose step lands
// the deepest misses first: the first miss is within a lane spacing, a
// factor 8 / 7, of that speed. The usual rule of thumb, "fast when it moves
// more than its size in a step", (2 r + t) / dt = 24 m/s, is twice too
// optimistic: the contact itself throws the shot through.
//
// Readouts: the speed of the last round, its misses (of 8), the first
// speed that missed, (r + t / 2) / dt and (2 r + t) / dt. Drawn: the walls
// in grey, the shots in orange.
//
// The world is y-up, in meters and seconds; shots ignore gravity.
class SpeedThresholdSample final : public Sample {
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

  // (r + t / 2) / dt and (2 r + t) / dt, in m/s
  [[nodiscard]] Scalar safe_speed() const;
  [[nodiscard]] Scalar size_speed() const;
  // The first speed (m/s) where a lane missed, once a round found one
  [[nodiscard]] std::optional<int> first_miss_speed() const {
    return first_miss_speed_;
  }
  [[nodiscard]] bool is_done() const { return speed_ > kLastSpeed; }
  // Misses of the round at `speed` m/s, once it ran
  [[nodiscard]] int misses_at(int speed) const {
    return misses_[static_cast<std::size_t>(speed - kFirstSpeed)];
  }

  static constexpr std::size_t kWall = 0;
  static constexpr int kFirstSpeed = 5;
  static constexpr int kLastSpeed = 60;
  static constexpr std::size_t kLaneCount = 8;

 private:
  [[nodiscard]] Scalar thickness() const {
    return parameters_[kWall].ToScalar();
  }
  void StartRound();
  void EndRound();
  void UpdateReadouts();

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 16,
                       .max_colliders = 16,
                       .max_touching_pairs = 16,
                       .max_broadphase_pairs = 32,
                       .max_joints = 0,
                       .max_passing_pairs = 0}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Wall",
       .unit = "m",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 100,
       .value = 20},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Speed", .unit = "m/s", .scale = 1},
      {.name = "Misses of 8", .unit = "", .scale = 1},
      {.name = "First miss", .unit = "m/s", .scale = 1},
      {.name = "(r + t/2) / dt", .unit = "m/s", .scale = 10},
      {.name = "(2r + t) / dt", .unit = "m/s", .scale = 10},
  }};
  std::array<ColliderIndex, kLaneCount> shots_{};
  std::array<ColliderIndex, kLaneCount> walls_{};
  std::array<Scalar, kLaneCount> lane_y_{};
  std::array<int, kLastSpeed - kFirstSpeed + 1> misses_{};
  std::optional<int> first_miss_speed_;
  Scalar wall_x_{};
  int speed_ = kFirstSpeed;
  // The speed of the last round that ended, 0 before the first
  int last_speed_ = 0;
  int round_steps_ = 0;
  int round_step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_SPEED_THRESHOLD_SAMPLE_H_
