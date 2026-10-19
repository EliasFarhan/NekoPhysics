#ifndef TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_TELEPORT_WARM_START_SAMPLE_H_
#define TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_TELEPORT_WARM_START_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 14, exercise "teleport warm start": what goes wrong when a body is
// moved by hand instead of with PhysicsWorld::Teleport.
//
// A stack of turning boxes (OrientedBoxCollider, 0.5 m, each 3 cm to the
// right of the one below; parameter "Stack" boxes on the bottom one) rests
// on the ground at x = 3.2 m. At step kTeleportStep the bottom box is moved
// to x = 9.6 m on the same ground, keeping its height and angle, the boxes
// above left behind. Parameter "Mode":
// - 0, PhysicsWorld::Teleport: the cached contact impulses of the box are
//   dropped, the next step starts its contacts from zero, like a box
//   created there;
// - 1, by hand (`body.position = ...`): the cache still holds the impulses
//   of the ground pair, which carried the whole stack's weight. Same pair,
//   same feature ids: the next step warm starts with them. The solver
//   (parameter "Iterations" passes) clamps most of it away, but not evenly
//   over the two points, and the box gets a spin it never had.
//
// Readouts: the bottom box's largest angular speed and rise during the
// kWatchSteps steps after the move. Drawn: the ground, the boxes in blue,
// the moved one in orange.
class TeleportWarmStartSample final : public Sample {
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

  // After the move, in rad/s and meters
  [[nodiscard]] Scalar max_spin() const { return max_spin_; }
  [[nodiscard]] Scalar max_rise() const { return max_rise_; }

  static constexpr std::size_t kMode = 0;
  static constexpr std::size_t kStack = 1;
  static constexpr std::size_t kIterations = 2;
  static constexpr int kTeleportStep = 180;
  static constexpr int kWatchSteps = 30;
  static constexpr int kMeasuredSteps = kTeleportStep + kWatchSteps;
  static constexpr std::size_t kMaxBoxes = 6;

 private:
  PhysicsWorld world_{{.max_bodies = 1, .max_colliders = 1}};
  std::array<SampleParameter, 3> parameters_{{
      {.name = "Mode",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 1},
      {.name = "Stack",
       .unit = "boxes",
       .min = 1,
       .max = 5,
       .step = 1,
       .scale = 1,
       .value = 3},
      {.name = "Iterations",
       .unit = "",
       .min = 1,
       .max = 8,
       .step = 1,
       .scale = 1,
       .value = 2},
  }};
  std::array<SampleReadout, 2> readouts_{{
      {.name = "Max spin", .unit = "rad/s", .scale = 1000},
      {.name = "Max rise", .unit = "mm", .scale = 10},
  }};

  Aabbf ground_{};
  std::array<ColliderIndex, kMaxBoxes> boxes_{};
  std::size_t box_count_ = 0;
  Scalar destination_x_{};
  Scalar moved_y_{};
  Scalar max_spin_{};
  Scalar max_rise_{};
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_14_MOVING_PLATFORM_SOLUTIONS_TELEPORT_WARM_START_SAMPLE_H_
