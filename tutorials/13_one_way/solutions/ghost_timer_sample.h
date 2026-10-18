#ifndef TUTORIALS_13_ONE_WAY_SOLUTIONS_GHOST_TIMER_SAMPLE_H_
#define TUTORIALS_13_ONE_WAY_SOLUTIONS_GHOST_TIMER_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 13, exercise "ghost timer": how long must a ghost platform stay
// off for a crate to fall through it?
//
// Six one-way ghost platforms, each with a 0.4 m crate resting on it. At
// step kOffStep all of them switch off (Collider::is_enabled); ghost i comes
// back n_i steps later, n_i = parameter "First off" + i. When a ghost comes
// back, the crate partly inside it is a new pair for the one-way rule:
// - its center is still above the ghost's center: the contact normal points
//   up, the pair is solid, and the crate is pushed back on top (caught);
// - its center is below: the normal points down, the pair is passing, and
//   the crate falls through to the ground.
//
// From rest, the semi-implicit Euler fall after n steps is
// g dt^2 n (n + 1) / 2, so the shortest off time is the smallest n whose
// fall exceeds the rest height of the crate's center above the ghost's
// center (about 0.375 m: half the crate plus half the ghost, minus the few
// millimeters of resting overlap). The prediction runs the same recurrence
// as the engine (v += g dt, then y += v dt) from the crate's state when the
// ghosts switch off, so it matches bit for bit.
//
// Readouts: the rest offset, the predicted and the measured shortest off
// time. Drawn: the ghosts in teal while on, dark grey while off, the crates
// in blue (green once caught, red once fallen).
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class GhostTimerSample final : public Sample {
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

  static constexpr std::size_t kGhostCount = 6;
  static constexpr int kOffStep = 30;
  static constexpr int kMeasuredSteps = 120;
  // Off time of ghost i, in steps
  [[nodiscard]] int off_steps(std::size_t ghost) const {
    return parameters_[kFirstOff].value + static_cast<int>(ghost);
  }
  // 1 caught, -1 fell through, 0 not decided yet (decided at kMeasuredSteps)
  [[nodiscard]] int outcome(std::size_t ghost) const {
    return outcomes_[ghost];
  }
  // The crate center's height above the ghost center when the ghosts switch
  // off, once there
  [[nodiscard]] Scalar rest_offset() const { return rest_offset_; }
  // The smallest off time, in steps, after which the crate's center is
  // below the ghost's center, from the recurrence; 0 before kOffStep
  [[nodiscard]] int predicted_min_off() const { return predicted_min_off_; }
  // The smallest off time whose crate fell, -1 when none did
  [[nodiscard]] int measured_min_off() const;
  [[nodiscard]] int steps() const { return step_; }

  static constexpr std::size_t kFirstOff = 0;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2 * kGhostCount + 1,
                       .max_colliders = 2 * kGhostCount + 1,
                       .max_touching_pairs = 32,
                       .max_broadphase_pairs = 64,
                       .max_joints = 0,
                       .max_passing_pairs = kGhostCount}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "First off",
       .unit = "steps",
       .min = 5,
       .max = 30,
       .step = 1,
       .scale = 1,
       .value = 15},
  }};
  std::array<SampleReadout, 3> readouts_{{
      {.name = "Rest offset", .unit = "mm", .scale = 1},
      {.name = "Pred. min off", .unit = "steps", .scale = 1},
      {.name = "Meas. min off", .unit = "steps", .scale = 1},
  }};
  Aabbf ground_{};
  std::array<ColliderIndex, kGhostCount> ghosts_{};
  std::array<ColliderIndex, kGhostCount> crates_{};
  std::array<int, kGhostCount> outcomes_{};
  Scalar rest_offset_{};
  int predicted_min_off_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_13_ONE_WAY_SOLUTIONS_GHOST_TIMER_SAMPLE_H_
