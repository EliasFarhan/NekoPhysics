#ifndef TUTORIALS_00_BOUNCE_SOLUTIONS_EXPLICIT_EULER_SAMPLE_H_
#define TUTORIALS_00_BOUNCE_SOLUTIONS_EXPLICIT_EULER_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 00, exercise "explicit Euler": three falls from rest, side by side.
//
// The engine's ball falls with semi-implicit Euler, the velocity first:
//
//   v += g dt;  y += v dt      drop after N steps: g dt^2 N (N + 1) / 2
//
// Next to it the sample integrates a particle by hand with explicit Euler,
// the position first, with the velocity at the start of the step:
//
//   y += v dt;  v += g dt      drop after N steps: g dt^2 N (N - 1) / 2
//
// and computes the exact fall g t^2 / 2 at t = N dt. The three velocities are
// the same, N g dt: a constant acceleration has no velocity error. The drops
// are g T dt / 2 apart (T = N dt): semi-implicit falls ahead, explicit
// behind, 0.0817 m each after one second.
//
// Semi-implicit Euler under gravity keeps v^2 - g dt v + 2 g y exactly: the
// "Shadow energy" readout shows it minus its starting value, 0 up to the
// rounding of Fixed16.
//
// Parameter: the fall time T ("Fall time"). Readouts: the three drops and the
// shadow energy's change, after N = 60 T steps. Drawn: the engine's ball
// orange, the explicit particle red, the exact fall green, the three drops
// scaled to the view.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class ExplicitEulerSample final : public Sample {
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

  // Steps of the fall, 60 T
  [[nodiscard]] int fall_steps() const;
  [[nodiscard]] int steps() const { return step_; }
  // Drops below the start, in meters, after steps()
  [[nodiscard]] Scalar engine_drop() const;
  [[nodiscard]] Scalar explicit_drop() const { return -explicit_y_; }
  [[nodiscard]] Scalar exact_drop() const;
  [[nodiscard]] Scalar engine_velocity() const;
  [[nodiscard]] Scalar explicit_velocity() const { return explicit_v_; }
  // v^2 - g dt v + 2 g y of the engine's ball minus its value at the start
  [[nodiscard]] Scalar shadow_energy_change() const;

  static constexpr std::size_t kFallTime = 0;

 private:
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 1,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 0}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Fall time",
       .unit = "s",
       .min = 25,
       .max = 300,
       .step = 25,
       .scale = 100,
       .value = 100},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Engine drop", .unit = "m", .scale = 1000},
      {.name = "Explicit drop", .unit = "m", .scale = 1000},
      {.name = "Exact drop", .unit = "m", .scale = 1000},
      {.name = "Shadow energy", .unit = "m2/s2", .scale = 1000},
  }};
  BodyIndex ball_ = kInvalidBodyIndex;
  Vec2f top_{};
  int step_ = 0;
  // The explicit particle, relative to the start
  Scalar explicit_y_{};
  Scalar explicit_v_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_00_BOUNCE_SOLUTIONS_EXPLICIT_EULER_SAMPLE_H_
