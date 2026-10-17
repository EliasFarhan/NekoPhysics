#ifndef TUTORIALS_00_BOUNCE_SOLUTIONS_BOUNCES_TO_REST_SAMPLE_H_
#define TUTORIALS_00_BOUNCE_SOLUTIONS_BOUNCES_TO_REST_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 00, exercise "bounces to rest": how many times does the ball bounce?
//
// The chapter's ball, dropped from h (parameter "Drop height") with the
// restitution e (parameter "Restitution"), bounces until an impact is slower
// than the world's restitution threshold, 1 m/s: a contact that approaches
// more slowly doesn't bounce, the push-out only lifts the ball out of the
// ground. Each impact is e times as fast as the one before, so with
// v0 = sqrt(2 g h) the impacts that bounce are those with e^k v0 > 1 m/s:
//
//   bounces          the number of k = 0, 1, 2... with e^k v0 > 1 m/s
//   apex k           e^(2k) h
//   last bounce at   v0 / g + 2 v0 (e + e^2 + ... + e^(n-1)) / g
//
// The sample counts a bounce when the ball's vertical velocity turns up in a
// step that approached the ground faster than the threshold (the approach is
// the velocity after that step's gravity, as the solver sees it), and records
// the apex after each one.
//
// Readouts: the predicted and measured bounces, the predicted and measured
// time of the last bounce, and the first two apexes. Drawn: the ground, the
// ball, the measured apexes in yellow and the predicted e^(2k) h in green.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class BouncesToRestSample final : public Sample {
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

  [[nodiscard]] int predicted_bounces() const;
  [[nodiscard]] Scalar predicted_last_bounce() const;
  [[nodiscard]] int bounces() const { return bounce_count_; }
  // Time of the step of the last bounce so far, in seconds
  [[nodiscard]] std::optional<Scalar> last_bounce() const {
    return last_bounce_;
  }
  // Apex after bounce k (0 is the first bounce), above the ground's top
  [[nodiscard]] std::optional<Scalar> apex(std::size_t k) const;
  // Approach speed of bounce k, m/s
  [[nodiscard]] std::optional<Scalar> approach(std::size_t k) const;

  static constexpr std::size_t kDropHeight = 0;
  static constexpr std::size_t kRestitution = 1;
  static constexpr std::size_t kMaxBounces = 32;

 private:
  [[nodiscard]] Scalar BallHeight() const;
  [[nodiscard]] Scalar drop_height() const {
    return parameters_[kDropHeight].ToScalar();
  }
  [[nodiscard]] Scalar restitution() const {
    return parameters_[kRestitution].ToScalar();
  }

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 2,
                       .max_colliders = 2,
                       .max_touching_pairs = 1,
                       .max_broadphase_pairs = 1,
                       .max_joints = 0}};
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
       .min = 10,
       .max = 95,
       .step = 5,
       .scale = 100,
       .value = 75},
  }};
  std::array<SampleReadout, 6> readouts_{{
      {.name = "Pred. bounces", .unit = "", .scale = 1},
      {.name = "Bounces", .unit = "", .scale = 1},
      {.name = "Pred. last at", .unit = "s", .scale = 100},
      {.name = "Last bounce at", .unit = "s", .scale = 100},
      {.name = "Apex 1", .unit = "m", .scale = 1000},
      {.name = "Apex 2", .unit = "m", .scale = 1000},
  }};
  BodyIndex ground_ = kInvalidBodyIndex;
  BodyIndex ball_ = kInvalidBodyIndex;
  Scalar ground_top_{};
  int step_ = 0;
  int bounce_count_ = 0;
  // Rising after a bounce, until the velocity turns down
  bool is_rising_ = false;
  std::optional<Scalar> last_bounce_;
  std::array<Scalar, kMaxBounces> apexes_{};
  std::array<Scalar, kMaxBounces> approaches_{};
  std::size_t apex_count_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_00_BOUNCE_SOLUTIONS_BOUNCES_TO_REST_SAMPLE_H_
