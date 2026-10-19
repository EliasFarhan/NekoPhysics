#ifndef TUTORIALS_14_MOVING_PLATFORM_MOVING_PLATFORM_SAMPLE_H_
#define TUTORIALS_14_MOVING_PLATFORM_MOVING_PLATFORM_SAMPLE_H_

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
// A kinematic platform: an AabbCollider of `half_size` centered on `center`,
// solid, no bounce, friction 0.5. Returns its collider.
ColliderIndex AddKinematicPlatform(PhysicsWorld& world, Vec2f center,
                                   Vec2f half_size);
// The patrol path: `center - (amplitude cos(2 pi step / period_steps), 0)`,
// where the platform must be after `step` steps: it starts at rest at the
// left end and swings to the right end and back. The phase is computed from
// `step % period_steps`, so it stays small in Fixed16 however long the run.
[[nodiscard]] Vec2f PatrolPoint(Vec2f center, Scalar amplitude, int step,
                                int period_steps);
// Whether `pair` is in the world's touching pairs (since the last Step)
[[nodiscard]] bool IsTouching(const PhysicsWorld& world, ColliderPair pair);
// The PatrolPlatform script of Splash, ported from Unity: each step the
// platform's velocity is set to the distance to the target divided by `tau`
// (Unity's `velocity = deltaPos`, tau = 1 s). That is a first-order lag, not
// a path follower: the platform runs behind the target and swings less.
void DriveLagging(PhysicsWorld& world, BodyIndex platform, Vec2f target,
                  Scalar tau);

// Chapter 14, moving platforms: kinematic bodies that follow a path and
// carry what stands on them, and a platform that wraps around the screen.
//
// Bottom left, the patrol lift: a kinematic platform driven every step by
// PhysicsWorld::SetKinematicTarget along a horizontal sine (amplitude 1.2 m,
// parameter "Period"). The engine sets the velocity that reaches the target
// in one step, so the contact solver sees a moving surface and friction
// carries the crate on it; at the end of the step the platform is put
// exactly on the target, so its error is zero in Fixed16 too.
//
// Bottom right, the lagging platform: the same path, driven by the Unity
// PatrolPlatform script of Splash (DriveLagging, parameter "Lag tau"). Its
// target is drawn as a yellow outline: the platform runs behind it and
// swings less, the first-order lag atan(w tau) and 1 / sqrt(1 + (w tau)^2):
// 46 degrees and 69 % of the amplitude at the defaults (T = 6 s, tau = 1 s,
// w tau = 1.05).
//
// Middle, a frictionless player box on a patrolling platform. Friction can't
// carry it: the game does, as Splash does by script. While the pair is
// touching, it copies the platform's horizontal velocity into the player's
// (parameter "Rider carry"; without it the platform slides away under the
// player, which falls).
//
// Top, the TicTac platform: it crosses the screen to the right at 1.6 m/s
// (SetKinematicTarget with a constant step) and, 19.2 m right of the middle
// of the view (Splash's value: the level is wider than the view), wraps to
// 19.2 m left of it at a new height from the seeded generator with
// PhysicsWorld::Teleport. The crate riding it is teleported by the same
// offset, found through the touching pairs: the pair stays touching, so no
// exit or enter event, and Teleport drops the cached contact impulses of
// both. It is off screen two thirds of its 24.75 s cycle.
//
// Readouts: the lift's distance to its path (0), the lagging platform's
// distance to its path, how far the lift's crate slipped on it since the
// start, the rider's offset on its platform, the wraps, and the TicTac
// crate's collision exits. Drawn: the ground in grey, the lift and the rider
// platform in teal, the lagging platform in purple with its target in
// yellow, the TicTac platform in lime, the crates in blue and the player in
// orange.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class MovingPlatformSample final : public Sample {
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

  // Where the patrol and the lagging platform must be after `step` steps
  [[nodiscard]] Vec2f LiftPath(int step) const;
  [[nodiscard]] Vec2f LagPath(int step) const;
  [[nodiscard]] Vec2f position(ColliderIndex collider) const {
    return world_.ColliderCenter(collider);
  }
  // How far the lift's crate moved on the lift since the start
  [[nodiscard]] Scalar crate_slip() const;
  // The player's horizontal offset from its platform's center
  [[nodiscard]] Scalar rider_offset() const;
  [[nodiscard]] int wraps() const { return wraps_; }
  [[nodiscard]] int tictac_exits() const { return listener_.exits; }
  [[nodiscard]] int steps() const { return step_; }
  [[nodiscard]] const PhysicsWorld& world() const { return world_; }
  [[nodiscard]] ColliderIndex lift() const { return lift_; }
  [[nodiscard]] ColliderIndex lift_crate() const { return lift_crate_; }
  [[nodiscard]] ColliderIndex lag() const { return lag_; }
  [[nodiscard]] ColliderIndex rider_platform() const { return rider_platform_; }
  [[nodiscard]] ColliderIndex player() const { return player_; }
  [[nodiscard]] ColliderIndex tictac() const { return tictac_; }
  [[nodiscard]] ColliderIndex tictac_crate() const { return tictac_crate_; }

  static constexpr std::size_t kPeriod = 0;
  static constexpr std::size_t kLagTau = 1;
  static constexpr std::size_t kRiderCarry = 2;
  static constexpr std::uint32_t kPlayerId = 100;
  static constexpr std::uint32_t kTicTacCrateId = 2;

 private:
  // Counts the collision exits of the TicTac crate
  class ExitListener final : public ContactListener {
   public:
    void OnTriggerEnter(const ColliderPair&) override {}
    void OnTriggerExit(const ColliderPair&) override {}
    void OnCollisionEnter(const ColliderPair&) override {}
    void OnCollisionExit(const ColliderPair& pair) override;
    const PhysicsWorld* world = nullptr;
    int exits = 0;
  };

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 16,
                       .max_colliders = 16,
                       .max_touching_pairs = 32,
                       .max_broadphase_pairs = 64,
                       .max_joints = 0,
                       .max_passing_pairs = 0}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Period",
       .unit = "s",
       .min = 2,
       .max = 12,
       .step = 1,
       .scale = 1,
       .value = 6},
      {.name = "Lag tau",
       .unit = "s",
       .min = 2,
       .max = 30,
       .step = 1,
       .scale = 10,
       .value = 10},
      {.name = "Rider carry",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Lift error", .unit = "mm", .scale = 10},
      {.name = "Lag error", .unit = "mm", .scale = 1},
      {.name = "Crate slip", .unit = "mm", .scale = 10},
      {.name = "Rider offset", .unit = "mm", .scale = 10},
      {.name = "Wraps", .unit = "", .scale = 1},
      {.name = "TicTac exits", .unit = "", .scale = 1},
  }};
  void WrapTicTac();
  void UpdateReadouts();

  ExitListener listener_;
  SampleRng rng_{14};
  Aabbf ground_{};
  Scalar left_{};
  Scalar right_{};
  // Where the TicTac platform wraps, 19.2 m each side of the middle
  Scalar tictac_left_{};
  Scalar tictac_right_{};
  ColliderIndex lift_ = kInvalidColliderIndex;
  ColliderIndex lift_crate_ = kInvalidColliderIndex;
  ColliderIndex lag_ = kInvalidColliderIndex;
  ColliderIndex lag_crate_ = kInvalidColliderIndex;
  ColliderIndex rider_platform_ = kInvalidColliderIndex;
  ColliderIndex player_ = kInvalidColliderIndex;
  ColliderIndex tictac_ = kInvalidColliderIndex;
  ColliderIndex tictac_crate_ = kInvalidColliderIndex;
  Vec2f lift_center_{};
  Vec2f lag_center_{};
  Vec2f rider_center_{};
  // The lift crate's offset from the lift at the start
  Scalar crate_start_offset_{};
  int period_steps_ = 0;
  int step_ = 0;
  int wraps_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_14_MOVING_PLATFORM_MOVING_PLATFORM_SAMPLE_H_
