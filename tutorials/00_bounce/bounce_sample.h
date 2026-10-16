#ifndef TUTORIALS_00_BOUNCE_BOUNCE_SAMPLE_H_
#define TUTORIALS_00_BOUNCE_BOUNCE_SAMPLE_H_

#include <array>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 00, the experiment harness: drop a ball, measure its bounce.
//
// A ball falls from rest onto a static ground. Its bottom starts `h` meters
// above the ground (parameter "Drop height") and the contact restitution is
// `e` (parameter "Restitution"; the ground has 0 and a contact uses the larger
// of its two colliders, so the contact uses `e`). Energy conservation
// predicts that the ball reaches `e * e * h` after the first bounce:
//
//   impact speed       v = sqrt(2 g h)       (fall from h)
//   rebound speed      e v                   (definition of restitution)
//   apex height        (e v)^2 / (2 g) = e^2 h
//
// The sample measures the real apex (the ball's bottom when its vertical
// velocity turns from up to down after the first bounce) and shows both as
// readouts and as horizontal lines: grey for the drop height, green for the
// prediction, yellow for the measurement. The sky blue line is the velocity,
// drawn as the distance covered in 0.1 s, and the white radius is the ball's
// orientation (it never turns: contacts have no friction yet).
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class BounceSample final : public Sample {
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

  // e^2 h, in meters
  [[nodiscard]] Scalar predicted_apex() const;
  // Height of the ball's bottom above the ground at the first apex after the
  // first bounce, in meters; empty until the ball got there
  [[nodiscard]] std::optional<Scalar> measured_apex() const {
    return measured_apex_;
  }

  static constexpr std::size_t kDropHeight = 0;
  static constexpr std::size_t kRestitution = 1;

 private:
  // Height of the ball's bottom above the ground's top, in meters
  [[nodiscard]] Scalar BallHeight() const;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Drop height",
       .unit = "m",
       .min = 100,
       .max = 600,
       .step = 25,
       .scale = 100,
       .value = 400},
      {.name = "Restitution",
       .unit = "",
       .min = 0,
       .max = 100,
       .step = 5,
       .scale = 100,
       .value = 50},
  }};
  std::array<SampleReadout, 2> readouts_{{
      {.name = "Predicted apex", .unit = "m", .scale = 100},
      {.name = "Measured apex", .unit = "m", .scale = 100},
  }};
  BodyIndex ground_index_ = kInvalidBodyIndex;
  BodyIndex ball_index_ = kInvalidBodyIndex;
  Scalar ground_top_{};
  Aabbf bounds_{};
  // Set once the ball moves up after its first contact
  bool has_bounced_ = false;
  std::optional<Scalar> measured_apex_;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_00_BOUNCE_BOUNCE_SAMPLE_H_
