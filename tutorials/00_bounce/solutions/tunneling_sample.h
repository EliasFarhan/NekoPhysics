#ifndef TUTORIALS_00_BOUNCE_SOLUTIONS_TUNNELING_SAMPLE_H_
#define TUTORIALS_00_BOUNCE_SOLUTIONS_TUNNELING_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 00, exercise "tunneling": a fast ball through a thin ground.
//
// The ball (radius r = 0.25 m, gravity scale 0) is shot straight down at v
// (parameter "Speed") from 2 m above a ground of thickness t (parameter
// "Thickness"). The world only looks at the positions of each step: the
// ball's center moves by v dt per step, and the first step that finds an
// overlap decides. While the center is above the middle of the ground, the
// contact normal pushes the ball back up; below the middle it pushes it down,
// out through the bottom; and a step longer than 2 r + t can jump over the
// ground without any overlap. The center enters the overlap band (r above the
// top) at most v dt deep, so the ball is always caught while
//
//   v dt <= r + t / 2          v <= (r + t / 2) / dt
//
// 30.0 m/s for the default 0.5 m ground. Faster, it depends on where the
// steps fall: the first overlap is found
//
//   depth = ceil(2 / (v dt)) v dt - 2
//
// below the band's top, and the ball passes when that is more than r + t / 2.
//
// Readouts: the safe speed (r + t / 2) / dt, the step length v dt, the depth
// of the first overlap, and whether the ball passed (1) or was caught (0),
// one second after the start. Drawn: the ground, the band of the overlaps
// (grey lines r above and below it), its middle (yellow), the ball and its
// velocity.
//
// The world is y-up, in meters and seconds.
class TunnelingSample final : public Sample {
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

  [[nodiscard]] Scalar safe_speed() const;
  [[nodiscard]] Scalar step_length() const;
  // How far below the band's top (r above the ground) the center was at the
  // first overlap; empty when the ball never overlapped the ground
  [[nodiscard]] std::optional<Scalar> first_depth() const {
    return first_depth_;
  }
  // After one second: whether the ball is below the ground's middle
  [[nodiscard]] std::optional<bool> passed() const { return passed_; }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kThickness = 1;
  static constexpr int kMeasuredSteps = 60;

 private:
  [[nodiscard]] Scalar thickness() const {
    return parameters_[kThickness].ToScalar();
  }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 0}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 5,
       .max = 100,
       .step = 1,
       .scale = 1,
       .value = 38},
      {.name = "Thickness",
       .unit = "m",
       .min = 10,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 50},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Safe up to", .unit = "m/s", .scale = 100},
      {.name = "Step v dt", .unit = "m", .scale = 1000},
      {.name = "First depth", .unit = "m", .scale = 1000},
      {.name = "Passed (1 yes)", .unit = "", .scale = 1},
  }};
  BodyIndex ground_ = kInvalidBodyIndex;
  BodyIndex ball_ = kInvalidBodyIndex;
  Scalar ground_top_{};
  int step_ = 0;
  std::optional<Scalar> first_depth_;
  std::optional<bool> passed_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_00_BOUNCE_SOLUTIONS_TUNNELING_SAMPLE_H_
