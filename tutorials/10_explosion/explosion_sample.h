#ifndef TUTORIALS_10_EXPLOSION_EXPLOSION_SAMPLE_H_
#define TUTORIALS_10_EXPLOSION_EXPLOSION_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 10, explosion: ask the world what is there, without stepping.
//
// A bomb goes off in the air above a row of targets standing on the ground.
// Two world queries decide who is pushed:
//
// - an overlap query, OverlapCircle(bomb, radius), finds every target whose
//   shape reaches into the blast radius (parameter "Radius"). Its filter
//   mask says which categories the blast affects: crates and the barrel,
//   plus the shielded crate (cyan) only when "Hit shielded" is 1;
// - a raycast from the bomb to each of them, RaycastClosest, is the line of
//   sight: the first collider on the segment is the target itself when
//   nothing is in the way, or the wall (parameter "Wall") when it hides the
//   target. Every solid category blocks the view, the ground excepted.
//
// Visible targets get an impulse away from the bomb that falls off with the
// distance d to their center, J (1 - d / radius). Hidden ones don't move.
// The ground is frictionless: from above, the blast also pushes the targets
// into the ground, and friction would eat most of the push along it.
//
// Categories are bits of a 16-bit mask, chosen by the game: here ground 1,
// wall 2, crate 4, shielded 8. A query sees a collider when its category
// shares a bit with the filter's mask, and never sees triggers unless asked.
//
// A second ray, the laser from the top left (parameter "Laser angle", in
// degrees below horizontal), shows what a raycast returns: the fraction t
// of the segment where it enters the first shape (0 at the start, 1 at the
// end) and the outward normal there. At 0 degrees it hits the round post
// 8.7 m along a 12 m segment, t = 0.725, normal (-1, 0); at 20 degrees the
// tilted plank's top face, whose normal is the plank's up axis
// (-sin 15, cos 15); at 60 degrees the ground, normal (0, 1).
//
// Readouts: targets in the blast radius, visible, blocked; the laser's t and
// normal. Drawn: the blast circle, the sight lines (green: visible, red: up
// to what blocked it), the laser in white with its normal in yellow.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class ExplosionSample final : public Sample {
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

  // Categories of the scene, one bit each
  static constexpr std::uint16_t kGround = 1 << 0;
  static constexpr std::uint16_t kWall = 1 << 1;
  static constexpr std::uint16_t kCrate = 1 << 2;
  static constexpr std::uint16_t kShielded = 1 << 3;

  // A target of the blast and what the queries said about it
  struct Target {
    BodyIndex body{};
    ColliderIndex collider{};
    bool in_radius = false;
    bool visible = false;
    // Where the sight line stopped: the target's center, or the blocker
    Vec2f sight_end{};
  };

  [[nodiscard]] bool exploded() const { return exploded_; }
  [[nodiscard]] std::span<const Target> targets() const { return targets_; }
  [[nodiscard]] int in_radius_count() const { return in_radius_count_; }
  [[nodiscard]] int visible_count() const { return visible_count_; }
  // The laser's first hit, if any, at the last step
  [[nodiscard]] std::optional<RaycastHit> laser_hit() const {
    return laser_hit_;
  }
  [[nodiscard]] Vec2f bomb_position() const { return bomb_; }
  [[nodiscard]] Vec2f laser_start() const;
  [[nodiscard]] Vec2f laser_end() const;
  [[nodiscard]] Vec2f target_velocity(std::size_t i) const {
    return world_.body(targets_[i].body).velocity;
  }

  static constexpr std::size_t kRadius = 0;
  static constexpr std::size_t kWallOn = 1;
  static constexpr std::size_t kHitShielded = 2;
  static constexpr std::size_t kLaserAngle = 3;
  // The step of the blast: 0.5 s, once the targets have settled
  static constexpr int kBlastStep = 30;
  // Target order in targets(): crate, barrel, shielded crate, crate behind
  // the wall, far crate
  static constexpr std::size_t kTargetCount = 5;

 private:
  void Explode();
  ColliderIndex AddStaticBox(Vec2f center, Vec2f half_size,
                             std::uint16_t category);

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 16,
                       .max_colliders = 16,
                       .max_touching_pairs = 32,
                       .max_broadphase_pairs = 64,
                       .max_joints = 0}};
  std::array<SampleParameter, 4> parameters_{{
      {.name = "Radius",
       .unit = "m",
       .min = 10,
       .max = 60,
       .step = 1,
       .scale = 10,
       .value = 35},
      {.name = "Wall",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 1},
      {.name = "Hit shielded",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 0},
      {.name = "Laser angle",
       .unit = "deg",
       .min = 0,
       .max = 60,
       .step = 5,
       .scale = 1,
       .value = 20},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "In radius", .unit = "", .scale = 1},
      {.name = "Visible", .unit = "", .scale = 1},
      {.name = "Blocked", .unit = "", .scale = 1},
      {.name = "Laser t", .unit = "", .scale = 1000},
      {.name = "Normal x", .unit = "", .scale = 1000},
      {.name = "Normal y", .unit = "", .scale = 1000},
  }};
  std::array<Target, kTargetCount> targets_{};
  Aabbf ground_{};
  ColliderIndex wall_ = kInvalidColliderIndex;
  BodyIndex plank_ = kInvalidBodyIndex;
  Vec2f bomb_{};
  Vec2f laser_origin_{};
  std::optional<RaycastHit> laser_hit_;
  int step_ = 0;
  int in_radius_count_ = 0;
  int visible_count_ = 0;
  bool exploded_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_10_EXPLOSION_EXPLOSION_SAMPLE_H_
