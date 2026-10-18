#ifndef TUTORIALS_12_AIMING_SOLUTIONS_OUT_OF_RANGE_SAMPLE_H_
#define TUTORIALS_12_AIMING_SOLUTIONS_OUT_OF_RANGE_SAMPLE_H_

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

// Chapter 12, exercise "out of range": no angle reaches the target.
//
// The largest range of a speed v is v^2 / g, at 45 degrees. A target
// farther than that (parameter "Distance" against parameter "Speed") makes
// the arcsine's input g d / v^2 larger than 1: std::asin returns NaN there,
// and a NaN angle sent to the physics poisons every body it touches. Asin
// clamps its input instead, so LowArcAngle gives 45 degrees, the best the
// turret can do, and the shot falls short at (v cos 45) (T - dt), T - dt
// being where the integrator lands.
//
// The game can then tell the player, or fire faster: the second ball leaves
// at 45 degrees with the smallest speed that reaches d, sqrt(g d), and its
// error is the usual one step, -v_x dt.
//
// Readouts: the arcsine's input; the largest range; the launch angle; the
// predicted and measured landing distance of the first ball; the speed
// needed; the hit error of the second ball. Drawn: the ground, the launch
// height, the target in green, the largest range in red, the first ball in
// orange and the second in sky blue, with their paths.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class OutOfRangeSample final : public Sample {
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

  // g d / v^2: past 1, out of range
  [[nodiscard]] Scalar asin_input() const;
  // v^2 / g
  [[nodiscard]] Scalar max_range() const;
  [[nodiscard]] Radian angle() const { return angle_; }
  // v cos(theta) (T - dt)
  [[nodiscard]] Scalar predicted_landing() const;
  // From the muzzle, once the first ball is down
  [[nodiscard]] std::optional<Scalar> measured_landing() const {
    return landing_;
  }
  // sqrt(g d)
  [[nodiscard]] Scalar needed_speed() const;
  [[nodiscard]] std::optional<Scalar> needed_error() const {
    return needed_error_;
  }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kDistance = 1;

 private:
  static constexpr std::size_t kMaxPath = 240;
  static constexpr std::size_t kGiven = 0;
  static constexpr std::size_t kNeeded = 1;

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
       .min = 40,
       .max = 110,
       .step = 1,
       .scale = 10,
       .value = 80},
      {.name = "Distance",
       .unit = "m",
       .min = 10,
       .max = 110,
       .step = 1,
       .scale = 10,
       .value = 90},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Asin input", .unit = "", .scale = 1000},
      {.name = "Max range", .unit = "m", .scale = 1000},
      {.name = "Launch angle", .unit = "deg", .scale = 100},
      {.name = "Pred. landing", .unit = "m", .scale = 1000},
      {.name = "Meas. landing", .unit = "m", .scale = 1000},
      {.name = "Speed needed", .unit = "m/s", .scale = 1000},
      {.name = "Needed error", .unit = "m", .scale = 1000},
  }};
  Aabbf ground_{};
  Vec2f muzzle_{};
  Scalar target_x_{};
  Radian angle_{};
  std::array<BodyIndex, 2> balls_{kInvalidBodyIndex, kInvalidBodyIndex};
  std::array<CrossingWatch, 2> watches_{};
  std::array<std::array<Vec2f, kMaxPath>, 2> paths_{};
  std::array<std::size_t, 2> path_sizes_{};
  std::array<bool, 2> is_down_{};
  std::optional<Scalar> landing_;
  std::optional<Scalar> needed_error_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_12_AIMING_SOLUTIONS_OUT_OF_RANGE_SAMPLE_H_
