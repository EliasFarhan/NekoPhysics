#ifndef TUTORIALS_15_FAST_PROJECTILE_FAST_PROJECTILE_SAMPLE_H_
#define TUTORIALS_15_FAST_PROJECTILE_FAST_PROJECTILE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/draw_list.h"
#include "main/samples/sample.h"
#include "main/samples/sample_rng.h"

namespace neko::tutorial {

// Shared by the chapter and its solutions.
//
// The radius of every shot of the chapter: a water ball of 0.1 m
inline constexpr Scalar kShotRadius = Ratio(1, 10);
// A shot: a circle of kShotRadius at `center`, dynamic, no gravity (it flies
// straight), no bounce and no friction; `is_bullet` sets Body::is_bullet,
// `is_trigger` makes it a trigger (a water shot that only reports hits).
// Returns its collider.
ColliderIndex AddShot(PhysicsWorld& world, Vec2f center, bool is_bullet,
                      bool is_trigger);
// A static solid wall: an AabbCollider of `half_size` centered on `center`,
// no bounce. Returns its collider.
ColliderIndex AddWall(PhysicsWorld& world, Vec2f center, Vec2f half_size);
// Fires the shot of `collider` from `from` at `velocity`: Teleport (no stale
// contact from where it stopped) and the new velocity
void Fire(PhysicsWorld& world, ColliderIndex collider, Vec2f from,
          Vec2f velocity);

// Chapter 15, fast projectiles: the same shot with and without continuous
// collision (Body::is_bullet).
//
// Four lanes, all fired together every second, to the right, at the speed of
// the parameter "Speed" (50 m/s by default: 0.83 m per step at 60 Hz). Each
// firing starts from a seeded random place along the first half meter, so
// the steps fall at a different place on the way every time.
//
// - Top lane, plain: a solid shot (radius 0.1 m) against a wall of the
//   thickness of the parameter "Wall" (0.2 m). The world only looks at the
//   positions of each step: the shot is caught while its first overlap is
//   less than r + t / 2 deep (chapter 0's tunneling exercise), always below
//   (r + t / 2) / dt = 12 m/s; faster, it depends on where the steps fall,
//   and the shot ends past the wall (pushed out through the far side, or
//   never overlapping it).
// - Second lane, bullet: the same with Body::is_bullet. After the step the
//   engine sweeps the shot from its start to its end position against the
//   wall, moves it back to the time of impact, and the next step's contact
//   stops it. It never misses.
// - Third and fourth lanes, water: trigger shots (bullet in the third, plain
//   in the fourth) at a player box (0.38 m by 0.6 m, kinematic) that walks
//   up and down across the lane. The bullet water shot gets a trigger enter
//   for every crossing of the player, even when no step lands on it; the
//   plain one only when a step happens to overlap the player.
//
// Readouts: the step length v dt, the hits (the shot stopped before the
// wall's middle) and misses of the two solid lanes, the hits of the two
// water lanes and the number of firings. Drawn: the walls in grey, the plain
// shot in orange, the bullet in sky blue, the water shots in cyan, the
// players in purple (lime for a frame after a hit) and each shot's last step
// as a yellow line.
//
// The world is y-up, in meters and seconds; shots ignore gravity.
class FastProjectileSample final : public Sample {
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

  // Lane order of the counters
  static constexpr std::size_t kPlainLane = 0;
  static constexpr std::size_t kBulletLane = 1;
  static constexpr std::size_t kWaterBulletLane = 2;
  static constexpr std::size_t kWaterPlainLane = 3;

  // Firings whose outcome is counted (the last one is still flying)
  [[nodiscard]] int firings() const { return firings_; }
  // The solid lanes: shots stopped before the wall's middle, and the others
  [[nodiscard]] int hits(std::size_t lane) const { return hits_[lane]; }
  [[nodiscard]] int misses(std::size_t lane) const { return misses_[lane]; }
  [[nodiscard]] Vec2f shot_position(std::size_t lane) const {
    return world_.ColliderCenter(shots_[lane]);
  }
  [[nodiscard]] Scalar wall_x() const { return wall_x_; }
  [[nodiscard]] Scalar speed() const { return parameters_[kSpeed].ToScalar(); }
  [[nodiscard]] Scalar thickness() const {
    return parameters_[kWall].ToScalar();
  }
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kWall = 1;
  static constexpr int kFirePeriod = 60;
  static constexpr std::size_t kLaneCount = 4;

 private:
  // Counts the hits of the water shots on their players
  class HitListener final : public ContactListener {
   public:
    void OnTriggerEnter(const ColliderPair& pair) override;
    void OnTriggerExit(const ColliderPair&) override {}
    void OnCollisionEnter(const ColliderPair&) override {}
    void OnCollisionExit(const ColliderPair&) override {}
    FastProjectileSample* sample = nullptr;
  };

  void FireAll();
  void MovePlayers();
  void UpdateReadouts();

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 8,
                       .max_colliders = 8,
                       .max_touching_pairs = 16,
                       .max_broadphase_pairs = 32,
                       .max_joints = 0,
                       .max_passing_pairs = 0}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 5,
       .max = 100,
       .step = 5,
       .scale = 1,
       .value = 50},
      {.name = "Wall",
       .unit = "m",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 100,
       .value = 20},
  }};
  std::array<SampleReadout, 8> readouts_{{
      {.name = "Step v dt", .unit = "m", .scale = 1000},
      {.name = "Plain hits", .unit = "", .scale = 1},
      {.name = "Plain misses", .unit = "", .scale = 1},
      {.name = "Bullet hits", .unit = "", .scale = 1},
      {.name = "Bullet misses", .unit = "", .scale = 1},
      {.name = "Water hits bullet", .unit = "", .scale = 1},
      {.name = "Water hits plain", .unit = "", .scale = 1},
      {.name = "Firings", .unit = "", .scale = 1},
  }};

  HitListener listener_;
  SampleRng rng_{15};
  std::array<ColliderIndex, kLaneCount> shots_{};
  std::array<ColliderIndex, 2> walls_{};
  std::array<ColliderIndex, 2> players_{};
  std::array<Vec2f, kLaneCount> previous_{};
  std::array<int, kLaneCount> hits_{};
  std::array<int, kLaneCount> misses_{};
  // Step of the last hit of each player, for the drawing
  std::array<int, 2> hit_step_{};
  std::array<Scalar, kLaneCount> lane_y_{};
  Scalar muzzle_x_{};
  Scalar wall_x_{};
  int firings_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_15_FAST_PROJECTILE_FAST_PROJECTILE_SAMPLE_H_
