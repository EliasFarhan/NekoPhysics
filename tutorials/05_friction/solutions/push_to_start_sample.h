#ifndef TUTORIALS_05_FRICTION_SOLUTIONS_PUSH_TO_START_SAMPLE_H_
#define TUTORIALS_05_FRICTION_SOLUTIONS_PUSH_TO_START_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 05, exercise "push to start": how hard must you push a resting box
// to move it?
//
// A 1 m box of 1 kg rests on a static ground; both surfaces have the
// friction coefficient `mu` (parameter "Friction"). A constant horizontal
// force F (parameter "Push") acts at its center every step. Static friction
// pushes back with exactly F, up to its limit mu m g; past it, the box slides
// and kinetic friction brakes with mu m g:
//
//   holds        F <= mu m g      friction = F
//   else slides  a = F / m - mu g friction = mu m g
//
// The push is at the center, 0.5 m above the floor, and friction acts at
// the floor: past F = m g the square box would tip over its front edge, so
// the push stops at 9 N and the friction at 0.8.
//
// The sample measures the box's speed after one second (the average
// acceleration) and, from it, the average friction force F - m a. Drawn: the
// ground in dark grey, the box in orange with its orientation in white and
// its velocity in sky blue, and the push as a red line (0.1 m per newton).
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class PushToStartSample final : public Sample {
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

  // mu m g, in newtons
  [[nodiscard]] Scalar static_limit() const;
  [[nodiscard]] bool predicted_holds() const;
  // F / m - mu g when it slides, 0 when it holds, in m/s^2
  [[nodiscard]] Scalar predicted_acceleration() const;
  // Speed after one second divided by that second, in m/s^2; empty before
  [[nodiscard]] std::optional<Scalar> measured_acceleration() const {
    return measured_acceleration_;
  }
  // F - m a over that second, in newtons; empty before
  [[nodiscard]] std::optional<Scalar> measured_friction() const;
  [[nodiscard]] const Body& box() const { return world_.body(box_index_); }

  static constexpr std::size_t kPush = 0;
  static constexpr std::size_t kFriction = 1;
  // Steps of the measurement, one second at 60 Hz
  static constexpr int kMeasureSteps = 60;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Push",
       .unit = "N",
       .min = 0,
       .max = 900,
       .step = 50,
       .scale = 100,
       .value = 400},
      {.name = "Friction",
       .unit = "",
       .min = 10,
       .max = 80,
       .step = 5,
       .scale = 100,
       .value = 50},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Static limit", .unit = "N", .scale = 1000},
      {.name = "Holds (predicted)", .unit = "", .scale = 1},
      {.name = "Predicted accel.", .unit = "m/s2", .scale = 1000},
      {.name = "Measured accel.", .unit = "m/s2", .scale = 1000},
      {.name = "Friction force", .unit = "N", .scale = 1000},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex box_index_ = kInvalidBodyIndex;
  int step_ = 0;
  std::optional<Scalar> measured_acceleration_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_05_FRICTION_SOLUTIONS_PUSH_TO_START_SAMPLE_H_
