#ifndef TUTORIALS_13_ONE_WAY_SOLUTIONS_DROP_THROUGH_SAMPLE_H_
#define TUTORIALS_13_ONE_WAY_SOLUTIONS_DROP_THROUGH_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 13, exercise "drop through": how long a drop-through lasts.
//
// The chapter's player (0.38 m wide, parameter "Player height" tall) stands
// on a one-way platform (parameter "Thickness") above the ground. At step
// kDropStep the game calls PhysicsWorld::DropThrough: the pair becomes
// passing, sends a collision exit event, and stays passing while the shapes
// overlap, that is until the player's top is below the platform's bottom. It
// has to fall its own height plus the thickness (minus the few millimeters
// of resting overlap).
//
// The prediction runs the engine's recurrence (v += g dt, then y += v dt)
// from the player's state at the drop and counts the steps that start with
// the shapes overlapping. The continuous estimate sqrt(2 d / g) / dt, d the
// distance to fall, is close but not exact: the step's discrete fall
// g dt^2 n (n + 1) / 2 runs ahead of g (n dt)^2 / 2 by half a step.
//
// Readouts: the measured passing steps, the predicted ones, the continuous
// estimate, and the collision exit events. Drawn: the ground, the platform
// in teal with its normal, the player in orange (magenta outline while
// passing).
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class DropThroughSample final : public Sample {
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

  [[nodiscard]] int passing_steps() const { return passing_steps_; }
  // From the recurrence, once the drop happened
  [[nodiscard]] int predicted_steps() const { return predicted_steps_; }
  // sqrt(2 d / g) / dt
  [[nodiscard]] Scalar continuous_steps() const { return continuous_steps_; }
  [[nodiscard]] int exit_events() const { return listener_.exits; }
  [[nodiscard]] std::size_t drop_result() const { return drop_result_; }
  [[nodiscard]] Scalar player_height() const {
    return world_.ColliderCenter(player_).y();
  }
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kThickness = 0;
  static constexpr std::size_t kPlayerHeight = 1;
  static constexpr int kDropStep = 30;
  static constexpr int kMeasuredSteps = 120;

 private:
  class ExitListener final : public ContactListener {
   public:
    void OnTriggerEnter(const ColliderPair&) override {}
    void OnTriggerExit(const ColliderPair&) override {}
    void OnCollisionEnter(const ColliderPair&) override {}
    void OnCollisionExit(const ColliderPair&) override { ++exits; }
    int exits = 0;
  };

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 3,
                       .max_colliders = 3,
                       .max_touching_pairs = 4,
                       .max_broadphase_pairs = 4,
                       .max_joints = 0,
                       .max_passing_pairs = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Thickness",
       .unit = "cm",
       .min = 10,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 35},
      {.name = "Player height",
       .unit = "cm",
       .min = 30,
       .max = 120,
       .step = 5,
       .scale = 100,
       .value = 60},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Meas. passing", .unit = "steps", .scale = 1},
      {.name = "Pred. passing", .unit = "steps", .scale = 1},
      {.name = "sqrt(2d/g)/dt", .unit = "steps", .scale = 100},
      {.name = "Exit events", .unit = "", .scale = 1},
  }};
  ExitListener listener_;
  Aabbf ground_{};
  ColliderIndex platform_ = kInvalidColliderIndex;
  ColliderIndex player_ = kInvalidColliderIndex;
  Vec2f platform_half_size_{};
  Vec2f player_half_size_{};
  std::size_t drop_result_ = 0;
  int passing_steps_ = 0;
  int predicted_steps_ = 0;
  Scalar continuous_steps_{};
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_13_ONE_WAY_SOLUTIONS_DROP_THROUGH_SAMPLE_H_
