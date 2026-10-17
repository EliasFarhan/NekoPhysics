#ifndef TUTORIALS_05_FRICTION_SOLUTIONS_STOPPING_DISTANCE_SAMPLE_H_
#define TUTORIALS_05_FRICTION_SOLUTIONS_STOPPING_DISTANCE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 05, exercise "stopping distance": a box thrown along flat ground
// slides to a stop.
//
// A 1 m box of 1 kg starts at `v` m/s (parameter "Speed") on a static
// ground; both surfaces have the friction coefficient `mu` (parameter
// "Friction", at most 0.8: past 1 the braking would tip the square box
// over its front edge). The ground pushes up with m g, so kinetic friction
// brakes with mu m g, a constant deceleration mu g:
//
//   stops after   t = v / (mu g)
//   having slid   d = v^2 / (2 mu g)
//
// The sample measures the time and distance until the box's speed drops
// below 1 mm/s. Drawn: the ground in dark grey, the box in orange with its
// orientation in white and its velocity in sky blue, and the predicted stop
// (where the box's center comes to rest) as a red line.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class StoppingDistanceSample final : public Sample {
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

  // v / (mu g), in seconds
  [[nodiscard]] Scalar predicted_time() const;
  // v^2 / (2 mu g), in meters
  [[nodiscard]] Scalar predicted_distance() const;
  // Time and distance until the box stopped; empty before
  [[nodiscard]] std::optional<Scalar> measured_time() const {
    return measured_time_;
  }
  [[nodiscard]] std::optional<Scalar> measured_distance() const {
    return measured_distance_;
  }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kFriction = 1;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 100,
       .max = 500,
       .step = 50,
       .scale = 100,
       .value = 300},
      {.name = "Friction",
       .unit = "",
       .min = 20,
       .max = 80,
       .step = 5,
       .scale = 100,
       .value = 50},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Predicted time", .unit = "s", .scale = 1000},
      {.name = "Measured time", .unit = "s", .scale = 1000},
      {.name = "Predicted dist.", .unit = "m", .scale = 1000},
      {.name = "Measured dist.", .unit = "m", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  Vec2f start_{};
  int step_ = 0;
  std::optional<Scalar> measured_time_;
  std::optional<Scalar> measured_distance_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_05_FRICTION_SOLUTIONS_STOPPING_DISTANCE_SAMPLE_H_
