#ifndef TUTORIALS_16_SEESAW_SOLUTIONS_RETARGET_CHAIN_SAMPLE_H_
#define TUTORIALS_16_SEESAW_SOLUTIONS_RETARGET_CHAIN_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/contact_listener.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 16, exercise "retarget chain": a TeamVersus chain (Splash) moved
// from one player to another at runtime.
//
// A chain of 8 links (0.2 kg solid circles, rope joints) hangs from a static
// post down to player A, a 1 kg box standing on the ground. Every 2 s the
// joint at its end is moved to the other player with
// PhysicsWorld::SetJointBodies. Three things go with the joint:
//
// - the collision filter: the end link stops colliding with the new player
//   and collides with the old one again, so the next step sends an enter
//   event for the old player's pair if they overlap, and an exit for the new
//   one if it was touching (readouts "Enters" and "Exits");
// - the accumulated impulse, which SetJointBodies resets: last step's pull
//   belongs to the old player and the old direction;
// - the length, which SetJointBodies does not touch: it is the rest length
//   the game chose for the old player. With "Keep length" at 0 the game sets
//   it to the new distance right after the call (the rope starts just taut);
//   at 1 it keeps the old length, the new player stands meters beyond it,
//   and the next step pulls it toward the chain as hard as the soft step
//   allows: "Max kick", the largest change of the new player's speed over
//   the step after a retarget, goes from 0 to more than 10 m/s (18 m/s in
//   FIXED16, 12 m/s in FLOAT over the first 10 s): the player is thrown at
//   the chain.
//
// Drawn: the ground and the post in grey, the links in yellow with their
// ropes, the attached player in orange, the other one in sky blue.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class RetargetChainSample final : public Sample, public ContactListener {
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

  void OnTriggerEnter(const ColliderPair& pair) override;
  void OnTriggerExit(const ColliderPair& pair) override;
  void OnCollisionEnter(const ColliderPair& pair) override;
  void OnCollisionExit(const ColliderPair& pair) override;

  [[nodiscard]] int retargets() const { return retargets_; }
  // Collision events between the end link and a player
  [[nodiscard]] int enters() const { return enters_; }
  [[nodiscard]] int exits() const { return exits_; }
  // Largest speed change of the new player over the step after a retarget,
  // in m/s
  [[nodiscard]] Scalar max_kick() const { return max_kick_; }
  // 0 for player A, 1 for player B
  [[nodiscard]] std::size_t attached() const { return attached_; }

  static constexpr std::size_t kKeepLength = 0;
  static constexpr int kLinkCount = 8;
  static constexpr int kRetargetPeriod = 120;

 private:
  [[nodiscard]] bool IsEndLinkPair(const ColliderPair& pair) const;

  PhysicsWorld world_{
      {.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
       .dt = kFixedDeltaTime,
       .max_bodies = 4 + kLinkCount,
       .max_colliders = 3 + kLinkCount,
       .max_touching_pairs = 64,
       .max_broadphase_pairs = 128,
       .max_joints = kLinkCount + 1,
       .max_revolute_joints = 0,
       .velocity_iterations = sample_config::kVelocityIterations}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Keep length",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 0},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Retargets", .unit = "", .scale = 1},
      {.name = "Enters", .unit = "", .scale = 1},
      {.name = "Exits", .unit = "", .scale = 1},
      {.name = "Max kick", .unit = "m/s", .scale = 1000},
  }};
  BodyIndex ground_ = kInvalidBodyIndex;
  BodyIndex post_ = kInvalidBodyIndex;
  std::array<BodyIndex, 2> players_{};
  std::array<BodyIndex, kLinkCount> links_{};
  ColliderIndex end_link_collider_{};
  std::array<JointIndex, kLinkCount + 1> ropes_{};
  std::size_t attached_ = 0;
  int step_ = 0;
  int retargets_ = 0;
  int enters_ = 0;
  int exits_ = 0;
  // The new player's velocity before the step after a retarget, and whether
  // that step is the next one
  Vec2f velocity_before_{};
  bool is_measuring_kick_ = false;
  Scalar max_kick_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_16_SEESAW_SOLUTIONS_RETARGET_CHAIN_SAMPLE_H_
