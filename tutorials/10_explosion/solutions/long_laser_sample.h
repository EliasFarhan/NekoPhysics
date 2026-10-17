#ifndef TUTORIALS_10_EXPLOSION_SOLUTIONS_LONG_LASER_SAMPLE_H_
#define TUTORIALS_10_EXPLOSION_SOLUTIONS_LONG_LASER_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 10, exercise "long laser": what the length of a segment costs in
// Fixed16.
//
// Two horizontal lasers start at x = 0.4 m: the upper one, at the chapter's
// height 6.4 m, hits the chapter's round post (center 9.5 m, radius 0.4 m)
// on its left side at x = 9.1 m; the lower one, at 3 m, hits a box whose
// left face is at x = 9.1 m too. Both obstacles are 8.7 m away whatever the
// length L of the segments (parameter "Length", in meters).
//
// A hit's fraction is t = 8.7 / L. Fixed16 stores t to 1/65536, so t L,
// the distance it stands for, is known only to L / 65536: 0.13 mm for 12 m,
// 15 mm for 1 km. The box's hit point is start + t (end - start), so its
// error grows like that; the circle's is start + direction * distance,
// computed before the division, and stays at the resolution of a position.
// With float, t has 24 significant bits whatever its size.
//
// Readouts, for each laser: t L and the error of the hit point's x, both
// against 8.7 m and 9.1 m, and L / 65536. Drawn: the lasers in white up to
// their hits, the post and the box in grey.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class LongLaserSample final : public Sample {
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

  [[nodiscard]] std::optional<RaycastHit> circle_hit() const {
    return circle_hit_;
  }
  [[nodiscard]] std::optional<RaycastHit> box_hit() const { return box_hit_; }
  [[nodiscard]] Scalar length() const {
    return parameters_[kLength].ToScalar();
  }
  // Where both obstacles start: 9.1 m, computed like the shapes are
  [[nodiscard]] Scalar face_x() const;

  static constexpr std::size_t kLength = 0;
  static constexpr Scalar kStartX = Ratio(2, 5);

 private:
  void Cast();

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 2,
                       .max_broadphase_pairs = 2,
                       .max_joints = 0}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Length",
       .unit = "m",
       .min = 12,
       .max = 3000,
       .step = 4,
       .scale = 1,
       .value = 12},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Circle t L", .unit = "m", .scale = 1000},
      {.name = "Circle x err", .unit = "mm", .scale = 100},
      {.name = "Box t L", .unit = "m", .scale = 1000},
      {.name = "Box x err", .unit = "mm", .scale = 100},
      {.name = "L / 65536", .unit = "mm", .scale = 100},
  }};
  std::optional<RaycastHit> circle_hit_;
  std::optional<RaycastHit> box_hit_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_10_EXPLOSION_SOLUTIONS_LONG_LASER_SAMPLE_H_
