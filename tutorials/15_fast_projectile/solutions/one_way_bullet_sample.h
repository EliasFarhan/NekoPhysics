#ifndef TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_ONE_WAY_BULLET_SAMPLE_H_
#define TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_ONE_WAY_BULLET_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_rng.h"

namespace neko::tutorial {

// Chapter 15, exercise "one-way bullet": fast shots through a one-way
// platform from below, and onto it from above.
//
// Two columns, fired up together every two seconds at the speed of the
// parameter "Speed" (50 m/s) from a seeded random height along half a meter
// under a one-way platform 0.1 m thick (chapter 13). Above it, a thick
// bumper (restitution 1, 3 m thick so that no shot goes through it) sends
// the shot back down at the same speed. The left shot is a bullet, the
// right one isn't.
//
// On the way up both pass the platform: the bullet's sweep applies the
// one-way rule (the normal of the hit, from the platform toward the shot,
// points down, out of the arc), and the plain shot either jumps over it or
// overlaps it from below, which makes the pair passing. On the way down the
// bullet's sweep finds the top face from the solid side and stops it there:
// it lands every time. The plain shot lands only when a step puts its center
// between the platform's middle and r above its top (0.15 m of the 0.83 m
// step at 50 m/s); below the middle the contact normal points down, so the
// pair starts as passing and the shot falls through.
//
// Readouts: the firings counted, the shots of each column that went up
// through the platform and those that landed on it. Drawn: the platforms in
// teal, the bumpers in grey, the bullet in sky blue, the plain shot in
// orange.
//
// The world is y-up, in meters and seconds; shots ignore gravity.
class OneWayBulletSample final : public Sample {
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

  // 0 for the bullet's column, 1 for the plain shot's
  [[nodiscard]] int went_up(std::size_t column) const {
    return went_up_[column];
  }
  [[nodiscard]] int landed(std::size_t column) const { return landed_[column]; }
  // Firings whose outcome is counted (the last one is still flying)
  [[nodiscard]] int firings() const { return fired_ - 1; }

  static constexpr std::size_t kSpeed = 0;
  static constexpr int kFirePeriod = 120;

 private:
  void FireBoth();
  void UpdateReadouts();

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 8,
                       .max_colliders = 8,
                       .max_touching_pairs = 8,
                       .max_broadphase_pairs = 16,
                       .max_joints = 0,
                       .max_passing_pairs = 4}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 10,
       .max = 100,
       .step = 5,
       .scale = 1,
       .value = 50},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Firings", .unit = "", .scale = 1},
      {.name = "Up through bullet", .unit = "", .scale = 1},
      {.name = "Up through plain", .unit = "", .scale = 1},
      {.name = "Landed bullet", .unit = "", .scale = 1},
      {.name = "Landed plain", .unit = "", .scale = 1},
  }};

  SampleRng rng_{15};
  std::array<ColliderIndex, 2> shots_{};
  std::array<ColliderIndex, 2> platforms_{};
  std::array<ColliderIndex, 2> bumpers_{};
  std::array<Scalar, 2> column_x_{};
  // Whether the shot of this firing went above the platform
  std::array<bool, 2> is_up_{};
  std::array<int, 2> went_up_{};
  std::array<int, 2> landed_{};
  Scalar launch_y_{};
  Scalar platform_top_{};
  Scalar bumper_bottom_{};
  int fired_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_ONE_WAY_BULLET_SAMPLE_H_
