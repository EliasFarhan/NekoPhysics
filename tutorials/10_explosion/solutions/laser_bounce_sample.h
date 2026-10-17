#ifndef TUTORIALS_10_EXPLOSION_SOLUTIONS_LASER_BOUNCE_SAMPLE_H_
#define TUTORIALS_10_EXPLOSION_SOLUTIONS_LASER_BOUNCE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 10, exercise "laser bounce": a laser reflected at every hit.
//
// A room of four static boxes (floor top at 0.5 m, ceiling at 6.7 m, walls
// at 0.3 and 12.5 m), a round post and a tilted mirror (an oriented box).
// The laser starts at (1, 4) at the angle of parameter "Angle" (degrees,
// positive up). Each segment is a RaycastClosest of 30 m; at the hit, the
// direction d is reflected about the normal n:
//
//   r = d - 2 (d . n) n
//
// and the next segment starts at the hit point moved off the surface along
// r by parameter "Push off" (in mm). With 0, the next segment starts on the
// boundary, and a segment starting inside a shape (boundary included) hits
// it at t = 0 with a zero normal: the reflection keeps the direction and
// every later segment stops at the same point.
//
// With the default -30 degrees, by hand: the floor at (7.062, 0.5), then
// the right wall at (12.5, 3.640).
//
// Readouts: the first two hit points, the number of segments that hit, and
// the length of the path. Drawn: the room, the laser in white, the normals
// at the hits in yellow.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class LaserBounceSample final : public Sample {
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

  static constexpr std::size_t kAngle = 0;
  static constexpr std::size_t kBounces = 1;
  static constexpr std::size_t kPushOff = 2;
  static constexpr std::size_t kMaxBounces = 10;

  struct Bounce {
    Vec2f point{};
    Vec2f normal{};
    Scalar fraction{};
  };
  // The hits of the path, in order
  [[nodiscard]] std::span<const Bounce> bounces() const {
    return std::span<const Bounce>{bounces_}.first(bounce_count_);
  }
  [[nodiscard]] Vec2f start() const;
  [[nodiscard]] Scalar path_length() const { return path_length_; }

 private:
  void Trace();
  void AddBox(Vec2f center, Vec2f half_size, Scalar degrees);

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 8,
                       .max_colliders = 8,
                       .max_touching_pairs = 16,
                       .max_broadphase_pairs = 32,
                       .max_joints = 0}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Angle",
       .unit = "deg",
       .min = -80,
       .max = 80,
       .step = 5,
       .scale = 1,
       .value = -30},
      {.name = "Bounces",
       .unit = "",
       .min = 1,
       .max = 10,
       .step = 1,
       .scale = 1,
       .value = 6},
      {.name = "Push off",
       .unit = "mm",
       .min = 0,
       .max = 10,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Hit 1 x", .unit = "m", .scale = 1000},
      {.name = "Hit 1 y", .unit = "m", .scale = 1000},
      {.name = "Hit 2 x", .unit = "m", .scale = 1000},
      {.name = "Hit 2 y", .unit = "m", .scale = 1000},
      {.name = "Segments", .unit = "", .scale = 1},
      {.name = "Path", .unit = "m", .scale = 100},
  }};
  std::array<Bounce, kMaxBounces> bounces_{};
  std::size_t bounce_count_ = 0;
  Vec2f last_end_{};
  Scalar path_length_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_10_EXPLOSION_SOLUTIONS_LASER_BOUNCE_SAMPLE_H_
