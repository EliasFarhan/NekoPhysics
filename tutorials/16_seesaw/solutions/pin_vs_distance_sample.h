#ifndef TUTORIALS_16_SEESAW_SOLUTIONS_PIN_VS_DISTANCE_SAMPLE_H_
#define TUTORIALS_16_SEESAW_SOLUTIONS_PIN_VS_DISTANCE_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 16, exercise "pin vs distance": a revolute joint is not a distance
// joint of length 0.
//
// Top: two pendulums, a 1 kg disk of radius r ("Radius") 2 m under a static
// pivot, released at 30 degrees. The left one hangs from a revolute joint
// (the pin is 2 m from the disk's center, so the disk turns with the swing),
// the right one from a rigid distance joint to the disk's center (the disk
// never turns). The right one is the point mass of chapter 07,
//
//   T = 2 pi sqrt(L / g) (1 + theta0^2 / 16),
//
// the left one a physical pendulum whose disk adds its own inertia m r^2 / 2
// to m L^2 about the pivot:
//
//   T_pin = T sqrt(1 + r^2 / (2 L^2))
//
// (0.25 % longer at r = 0.2 m, 1.6 % at 0.5 m).
//
// Bottom: two point masses sitting on their pivots, kicked sideways at 2 m/s
// every second (game-side). The pin holds them in every direction (a few mm
// of give, the soft step); the zero-length distance joint has no direction
// while the anchors are on top of each other, and lets the kick through for
// a step (33 mm) before it pulls back.
//
// Drawn: the pivots in grey, the pin pendulum in sky blue (with a white
// spoke showing the disk's turn), the distance pendulum in orange, the
// kicked bobs in the same colors.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class PinVsDistanceSample final : public Sample {
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

  // Index 0 for the pin, 1 for the distance joint
  [[nodiscard]] std::optional<Scalar> measured_period(std::size_t index) const;
  [[nodiscard]] Scalar predicted_period(std::size_t index) const;
  // Largest distance of a kicked bob from its pivot, in meters
  [[nodiscard]] Scalar max_kick_gap(std::size_t index) const {
    return max_kick_gap_[index];
  }

  static constexpr std::size_t kRadius = 0;
  static constexpr int kKickPeriod = 60;

 private:
  [[nodiscard]] Scalar radius() const {
    return parameters_[kRadius].ToScalar();
  }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 8,
                       .max_colliders = 2,
                       .max_touching_pairs = 4,
                       .max_broadphase_pairs = 4,
                       .max_joints = 2,
                       .max_revolute_joints = 2}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Radius",
       .unit = "m",
       .min = 5,
       .max = 50,
       .step = 5,
       .scale = 100,
       .value = 20},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Pin period", .unit = "s", .scale = 10000},
      {.name = "Pred. pin", .unit = "s", .scale = 10000},
      {.name = "Rod period", .unit = "s", .scale = 10000},
      {.name = "Pred. rod", .unit = "s", .scale = 10000},
      {.name = "Pin kick gap", .unit = "mm", .scale = 10},
      {.name = "Rod kick gap", .unit = "mm", .scale = 10},
  }};
  std::array<BodyIndex, 2> pivots_{};
  std::array<BodyIndex, 2> bobs_{};
  std::array<BodyIndex, 2> kick_pivots_{};
  std::array<BodyIndex, 2> kicked_{};
  int step_ = 0;
  std::array<Scalar, 2> previous_x_{};
  std::array<Scalar, 2> first_crossing_{};
  std::array<Scalar, 2> last_crossing_{};
  std::array<int, 2> crossings_{};
  std::array<Scalar, 2> max_kick_gap_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_16_SEESAW_SOLUTIONS_PIN_VS_DISTANCE_SAMPLE_H_
