#ifndef TUTORIALS_07_PENDULUM_SOLUTIONS_WHERE_SLACK_SAMPLE_H_
#define TUTORIALS_07_PENDULUM_SOLUTIONS_WHERE_SLACK_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 07, exercise "where slack": where a rope stops pulling.
//
// A 1 kg bob hangs at rest from a static pivot on a rope of length L
// (parameter "Length") and is launched from the bottom at the speed v0
// (parameter "Speed"). On the way up its speed is v^2 = v0^2 - 2 g (h + L),
// with h its height above the pivot, and the rope must pull with the
// centripetal force minus the part of the weight along the rope:
//
//   T = m v^2 / L - m g h / L = m (v0^2 - 2 g L - 3 g h) / L
//
// The rope goes slack where T reaches 0, at the height
//
//   h_slack = (v0^2 - 2 g L) / (3 g)
//
// above the pivot, if the bob gets past the horizontal (v0^2 > 2 g L) but
// can't loop (v0^2 < 5 g L). The bob then flies on a parabola inside the
// circle until the rope catches it. A bob released at rest at an angle
// theta0 past the horizontal reaches the same heights with
// v0^2 = 2 g L (1 - cos theta0): it goes slack at 2/3 of its release height.
//
// Readouts: the predicted height, the bob's height above the pivot at the
// first step the rope's impulse is 0 (it stopped pulling), and the steps
// spent slack (shorter than L by more than 1 cm). Drawn: the pivot in grey,
// the rope in yellow (dark grey while slack), the bob in orange with its
// velocity in sky blue, the predicted height as a green line.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class WhereSlackSample final : public Sample {
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

  // (v0^2 - 2 g L) / (3 g), in meters above the pivot
  [[nodiscard]] Scalar predicted_slack_height() const;
  // Height above the pivot at the first step without a pull; empty before
  [[nodiscard]] std::optional<Scalar> slack_height() const {
    return slack_height_;
  }
  [[nodiscard]] int slack_steps() const { return slack_steps_; }

  static constexpr std::size_t kLength = 0;
  static constexpr std::size_t kSpeed = 1;

 private:
  [[nodiscard]] Scalar length() const {
    return parameters_[kLength].ToScalar();
  }
  [[nodiscard]] Scalar speed() const { return parameters_[kSpeed].ToScalar(); }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 1,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Length",
       .unit = "m",
       .min = 10,
       .max = 30,
       .step = 5,
       .scale = 10,
       .value = 20},
      {.name = "Speed",
       .unit = "m/s",
       .min = 40,
       .max = 120,
       .step = 5,
       .scale = 10,
       .value = 80},
  }};
  std::array<SampleReadout, 3> readouts_{{
      {.name = "Pred. slack h", .unit = "m", .scale = 1000},
      {.name = "Slack height", .unit = "m", .scale = 1000},
      {.name = "Slack steps", .unit = "", .scale = 1},
  }};
  BodyIndex pivot_ = kInvalidBodyIndex;
  BodyIndex bob_ = kInvalidBodyIndex;
  JointIndex rope_ = kInvalidJointIndex;
  std::optional<Scalar> slack_height_;
  int slack_steps_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_07_PENDULUM_SOLUTIONS_WHERE_SLACK_SAMPLE_H_
