#ifndef TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_WRAP_EVENTS_SAMPLE_H_
#define TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_WRAP_EVENTS_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_rng.h"

namespace neko::tutorial {

// Chapter 14, exercise "wrap events": what the contact events say when a
// platform wraps around the screen.
//
// The chapter's TicTac platform carries a crate to the right at parameter
// "Speed" and, past the right post of a 10.8 m wide screen (drawn as two
// grey posts), wraps to the left post at a seeded height with
// PhysicsWorld::Teleport. Only the events of the pair (platform, crate)
// are counted. Parameter "Mode":
// - 0, the platform alone: the crate stays at the right post. The next step
//   finds the pair apart: one collision exit. The crate keeps its speed, falls
//   and stops against the wall at the right of the world;
// - 1, the platform and its riders (found by the touching pairs) by the
//   same offset: the pair still touches after the teleport, so no event at
//   all; Teleport only dropped the warm start of the pair.
//
// The crate starts at rest on a platform already at full speed, so it slips
// v^2 / (2 mu g) backward before riding (0.59 m at 2.4 m/s); above 2.8 m/s
// it slips off the 1.2 m platform before the first wrap.
//
// Readouts: the wraps, the pair's collision enters and exits, whether the
// crate is still on the platform. Drawn: the ground, the wall and the posts
// in grey, the platform in lime, the crate in blue.
class WrapEventsSample final : public Sample {
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

  [[nodiscard]] int wraps() const { return wraps_; }
  [[nodiscard]] int enters() const { return listener_.enters; }
  [[nodiscard]] int exits() const { return listener_.exits; }
  // The step of the first wrap, -1 before
  [[nodiscard]] int first_wrap_step() const { return first_wrap_step_; }
  // The step of the crate's first exit event, -1 before
  [[nodiscard]] int first_exit_step() const { return listener_.first_exit; }
  [[nodiscard]] bool is_crate_on_platform() const;
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kMode = 0;
  static constexpr std::size_t kSpeed = 1;
  static constexpr int kMeasuredSteps = 1500;

 private:
  class CrateListener final : public ContactListener {
   public:
    void OnTriggerEnter(const ColliderPair&) override {}
    void OnTriggerExit(const ColliderPair&) override {}
    void OnCollisionEnter(const ColliderPair& pair) override {
      if (pair == watched) {
        ++enters;
      }
    }
    void OnCollisionExit(const ColliderPair& pair) override {
      if (pair == watched) {
        if (first_exit < 0) {
          first_exit = *step;
        }
        ++exits;
      }
    }
    ColliderPair watched{};
    const int* step = nullptr;
    int enters = 0;
    int exits = 0;
    int first_exit = -1;
  };

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 4,
                       .max_colliders = 4,
                       .max_touching_pairs = 4,
                       .max_broadphase_pairs = 8,
                       .max_joints = 0,
                       .max_passing_pairs = 0}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Mode",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 1},
      {.name = "Speed",
       .unit = "m/s",
       .min = 5,
       .max = 28,
       .step = 1,
       .scale = 10,
       .value = 24},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Wraps", .unit = "", .scale = 1},
      {.name = "Enters", .unit = "", .scale = 1},
      {.name = "Exits", .unit = "", .scale = 1},
      {.name = "On platform", .unit = "", .scale = 1},
  }};
  void Wrap();

  CrateListener listener_;
  SampleRng rng_{14};
  Aabbf ground_{};
  Aabbf wall_{};
  Scalar left_post_{};
  Scalar right_post_{};
  ColliderIndex platform_ = kInvalidColliderIndex;
  ColliderIndex crate_ = kInvalidColliderIndex;
  int step_ = 0;
  int wraps_ = 0;
  int first_wrap_step_ = -1;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_WRAP_EVENTS_SAMPLE_H_
