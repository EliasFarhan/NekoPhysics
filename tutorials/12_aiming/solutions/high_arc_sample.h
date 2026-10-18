#ifndef TUTORIALS_12_AIMING_SOLUTIONS_HIGH_ARC_SAMPLE_H_
#define TUTORIALS_12_AIMING_SOLUTIONS_HIGH_ARC_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/math/angle.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "tutorials/12_aiming/aiming_sample.h"

namespace neko::tutorial {

// Chapter 12, exercise "high arc": the other angle that reaches the target.
//
// sin(2 theta) = sin(pi - 2 theta), so the range v^2 sin(2 theta) / g is the
// same for theta and for pi / 2 - theta: below the largest range, two angles
// hit a target at the distance d (parameter "Distance"), a flat one and a
// lob. Two balls leave the muzzle at v (parameter "Speed") at the same time,
// one at each angle, toward a fixed target at the muzzle's height.
//
// The lob flies longer: T = 2 v sin(theta) / g with the larger sine, which
// makes it worse against a moving target. Its horizontal speed v cos(theta)
// is smaller, so the integrator's one-step-early landing costs it less: each
// error is predicted at -v_x dt.
//
// Readouts: both angles, both measured flight times and both predicted and
// measured hit errors. Drawn: the ground, the launch height, the target in
// green, the flat shot in orange and the lob in sky blue, with their paths.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class HighArcSample final : public Sample {
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

  static constexpr std::size_t kLow = 0;
  static constexpr std::size_t kHigh = 1;
  [[nodiscard]] Radian angle(std::size_t shot) const { return angles_[shot]; }
  // -v cos(theta) dt
  [[nodiscard]] Scalar predicted_error(std::size_t shot) const;
  [[nodiscard]] std::optional<Scalar> measured_error(std::size_t shot) const {
    return errors_[shot];
  }
  [[nodiscard]] std::optional<Scalar> flight_time(std::size_t shot) const {
    return flight_times_[shot];
  }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kDistance = 1;

 private:
  static constexpr std::size_t kMaxPath = 240;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 0}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 50,
       .max = 110,
       .step = 1,
       .scale = 10,
       .value = 100},
      {.name = "Distance",
       .unit = "m",
       .min = 10,
       .max = 90,
       .step = 1,
       .scale = 10,
       .value = 60},
  }};
  std::array<SampleReadout, 8> readouts_{{
      {.name = "Low angle", .unit = "deg", .scale = 100},
      {.name = "High angle", .unit = "deg", .scale = 100},
      {.name = "Low flight", .unit = "s", .scale = 1000},
      {.name = "High flight", .unit = "s", .scale = 1000},
      {.name = "Low pred. err.", .unit = "m", .scale = 1000},
      {.name = "Low meas. err.", .unit = "m", .scale = 1000},
      {.name = "High pred. err.", .unit = "m", .scale = 1000},
      {.name = "High meas. err.", .unit = "m", .scale = 1000},
  }};
  Aabbf ground_{};
  Vec2f muzzle_{};
  Scalar target_x_{};
  std::array<BodyIndex, 2> balls_{kInvalidBodyIndex, kInvalidBodyIndex};
  std::array<Radian, 2> angles_{};
  std::array<CrossingWatch, 2> watches_{};
  std::array<std::array<Vec2f, kMaxPath>, 2> paths_{};
  std::array<std::size_t, 2> path_sizes_{};
  std::array<std::optional<Scalar>, 2> errors_{};
  std::array<std::optional<Scalar>, 2> flight_times_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_12_AIMING_SOLUTIONS_HIGH_ARC_SAMPLE_H_
