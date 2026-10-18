#ifndef TUTORIALS_13_ONE_WAY_ONE_WAY_SAMPLE_H_
#define TUTORIALS_13_ONE_WAY_ONE_WAY_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/draw_list.h"
#include "main/samples/sample.h"

namespace neko::tutorial {

// Shared by the chapter and its solutions.
//
// A static one-way platform: an AabbCollider of `half_size` centered on
// `center`, solid, no bounce, its one-way normal up (the default) and the
// one-way arc `arc`. Returns its collider.
ColliderIndex AddOneWayPlatform(PhysicsWorld& world, Vec2f center,
                                Vec2f half_size, Radian arc);
// A dynamic box of 1 kg (an AabbCollider) that doesn't turn: solid, no
// bounce, friction 0.5, `user_id` set. Returns its collider.
ColliderIndex AddCrate(PhysicsWorld& world, Vec2f center, Vec2f half_size,
                       std::uint32_t user_id);
// The debug view of the one-way colliders: the world one-way normal of
// `collider`, 0.3 m long from the middle of its top, in yellow
void DrawOneWayNormal(const PhysicsWorld& world, ColliderIndex collider,
                      DrawList& draw_list);
// Every passing pair as a magenta line between the two shape centers
void DrawPassingPairs(const PhysicsWorld& world, DrawList& draw_list);

// Chapter 13, one-way platforms: floors you jump through from below and
// land on from above, in the style of Haunted's floors.
//
// On the left, a player-like box (0.38 m wide, 0.6 m tall) stands on the
// ground under a stack of three one-way platforms 1.6 m apart. A scripted
// input, counted in fixed steps, jumps three times (setting the vertical
// velocity to parameter "Jump speed") and then drops through three times
// with PhysicsWorld::DropThrough: up through each floor, then back down to
// the ground, every 75 steps, forever. A jump of speed v rises v^2 / (2 g),
// so 6.5 m/s clears a 1.6 m floor spacing plus the player's height.
//
// On the right, six crates fall from the top onto a high one-way shelf and
// a lower ghost platform. The ghost platform toggles Collider::is_enabled every
// parameter "Ghost period" steps: when it vanishes, the crates on it fall to
// the ground with a collision exit event; when it comes back, it is a new
// collider for the narrowphase and the one-way rule decides again.
//
// The rule (Collider::is_one_way): when a pair starts touching, it is solid
// if the contact normal, from the platform toward the other shape, is
// within half of the one-way arc (parameter "Arc") of the platform's normal
// and the other shape doesn't move along that normal. Otherwise it is
// passing: not solved, no events, until the shapes stop overlapping.
//
// Readouts: the number of passing pairs now and the most seen at once, the
// floor the player stands on (0 for the ground, invalid in the air), the
// player's landings on one-way floors (collision enter events, found by the
// colliders' user_id), the drop-throughs that took effect, and whether the
// ghost is on. Drawn: the ground in grey, the one-way platforms in teal
// (the ghost dark grey while off) with their normals in yellow, the player
// in orange, the crates in blue, and the passing pairs in magenta.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class OneWaySample final : public Sample {
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

  // The floor the player stands on (0 ground, 1 to 3), -1 in the air
  [[nodiscard]] int player_floor() const;
  [[nodiscard]] Vec2f player_position() const {
    return world_.body(world_.collider(player_).body_index).position;
  }
  [[nodiscard]] int landings() const { return listener_.landings; }
  [[nodiscard]] int drops() const { return drops_; }
  [[nodiscard]] int steps() const { return step_; }
  [[nodiscard]] std::size_t max_passing() const { return max_passing_; }
  [[nodiscard]] bool ghost_enabled() const {
    return world_.collider(ghost_).is_enabled;
  }
  [[nodiscard]] const PhysicsWorld& world() const { return world_; }
  [[nodiscard]] ColliderIndex player() const { return player_; }
  [[nodiscard]] ColliderIndex ghost() const { return ghost_; }
  [[nodiscard]] std::span<const ColliderIndex> crates() const {
    return crates_;
  }
  [[nodiscard]] std::span<const ColliderIndex> floors() const {
    return floors_;
  }

  static constexpr std::size_t kJumpSpeed = 0;
  static constexpr std::size_t kGhostPeriod = 1;
  static constexpr std::size_t kArc = 2;
  // One scripted action every kActionSteps steps, kActionOffset steps into
  // the period: jump, jump, jump, drop, drop, drop
  static constexpr int kActionSteps = 75;
  static constexpr int kActionOffset = 30;
  static constexpr int kCycleSteps = 6 * kActionSteps;
  static constexpr std::size_t kCrateCount = 6;
  static constexpr std::uint32_t kPlayerId = 100;

 private:
  // Counts the player's landings on the floors of the stack
  class LandingListener final : public ContactListener {
   public:
    void OnTriggerEnter(const ColliderPair&) override {}
    void OnTriggerExit(const ColliderPair&) override {}
    void OnCollisionEnter(const ColliderPair& pair) override;
    void OnCollisionExit(const ColliderPair&) override {}
    const PhysicsWorld* world = nullptr;
    int landings = 0;
  };

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 16,
                       .max_colliders = 16,
                       .max_touching_pairs = 32,
                       .max_broadphase_pairs = 64,
                       .max_joints = 0,
                       .max_passing_pairs = 16}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Jump speed",
       .unit = "m/s",
       .min = 40,
       .max = 90,
       .step = 1,
       .scale = 10,
       .value = 65},
      {.name = "Ghost period",
       .unit = "steps",
       .min = 30,
       .max = 240,
       .step = 10,
       .scale = 1,
       .value = 120},
      {.name = "Arc",
       .unit = "deg",
       .min = 90,
       .max = 180,
       .step = 5,
       .scale = 1,
       .value = 180},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Passing", .unit = "pairs", .scale = 1},
      {.name = "Max passing", .unit = "pairs", .scale = 1},
      {.name = "Player floor", .unit = "", .scale = 1},
      {.name = "Landings", .unit = "", .scale = 1},
      {.name = "Drops", .unit = "", .scale = 1},
      {.name = "Ghost on", .unit = "", .scale = 1},
  }};
  void UpdateReadouts();

  LandingListener listener_;
  Aabbf ground_{};
  ColliderIndex player_ = kInvalidColliderIndex;
  ColliderIndex ghost_ = kInvalidColliderIndex;
  std::array<ColliderIndex, 3> floors_{};
  ColliderIndex shelf_ = kInvalidColliderIndex;
  std::array<ColliderIndex, kCrateCount> crates_{};
  int step_ = 0;
  int drops_ = 0;
  std::size_t max_passing_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_13_ONE_WAY_ONE_WAY_SAMPLE_H_
