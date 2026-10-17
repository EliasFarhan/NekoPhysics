#ifndef TUTORIALS_05_FRICTION_SOLUTIONS_ROLLING_DISK_SAMPLE_H_
#define TUTORIALS_05_FRICTION_SOLUTIONS_ROLLING_DISK_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 05, exercise "rolling disk": a disk thrown without spin slides,
// and friction turns the slide into rolling.
//
// A disk of 1 kg and radius `r` (parameter "Radius") starts at `v0` m/s
// (parameter "Speed") along a static ground, without spin; both surfaces have
// the friction coefficient `mu` (parameter "Friction"). While its bottom
// point slides, kinetic friction mu m g acts there, backward: it slows the
// center at mu g and spins the disk up through its lever arm r, with
// I = m r^2 / 2, at 2 mu g / r. The bottom point moves at v + w r (w < 0
// turns clockwise), and it stops sliding when v = -w r:
//
//   v0 - mu g t = 2 mu g t     =>   t = v0 / (3 mu g)
//   v = 2/3 v0,   w = -v / r,   slid d = v0 t - mu g t^2 / 2
//                                       = 5 v0^2 / (18 mu g)
//
// From then on it rolls: the bottom point is at rest, static friction has
// nothing to do, and the disk keeps 2/3 v0 (no rolling resistance). The
// sample measures the first step at which the bottom point moves slower than
// 1 cm/s. Drawn: the ground in dark grey, the disk in orange with its
// orientation in white and its velocity in sky blue, and where the slip
// should end as a red line.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class RollingDiskSample final : public Sample {
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

  // v0 / (3 mu g), in seconds
  [[nodiscard]] Scalar predicted_time() const;
  // 2/3 v0, in m/s
  [[nodiscard]] Scalar predicted_speed() const;
  // 5 v0^2 / (18 mu g), in meters
  [[nodiscard]] Scalar predicted_distance() const;
  // When the slip ended, and the disk's state then; empty before
  struct Rolling {
    Scalar time;
    Scalar distance;
    Scalar speed;
    Scalar spin;
  };
  [[nodiscard]] std::optional<Rolling> measured() const { return measured_; }
  // The disk's current velocity and spin
  [[nodiscard]] const Body& disk() const { return world_.body(disk_index_); }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kFriction = 1;
  static constexpr std::size_t kRadius = 2;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 100,
       .max = 600,
       .step = 50,
       .scale = 100,
       .value = 300},
      {.name = "Friction",
       .unit = "",
       .min = 10,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 30},
      {.name = "Radius",
       .unit = "m",
       .min = 25,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 50},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Predicted time", .unit = "s", .scale = 1000},
      {.name = "Measured time", .unit = "s", .scale = 1000},
      {.name = "Predicted speed", .unit = "m/s", .scale = 1000},
      {.name = "Measured speed", .unit = "m/s", .scale = 1000},
      {.name = "Predicted dist.", .unit = "m", .scale = 1000},
      {.name = "Measured dist.", .unit = "m", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex disk_index_ = kInvalidBodyIndex;
  Vec2f start_{};
  int step_ = 0;
  std::optional<Rolling> measured_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_05_FRICTION_SOLUTIONS_ROLLING_DISK_SAMPLE_H_
