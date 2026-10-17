#ifndef TUTORIALS_10_EXPLOSION_SOLUTIONS_ONE_HIT_SAMPLE_H_
#define TUTORIALS_10_EXPLOSION_SOLUTIONS_ONE_HIT_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 10, exercise "one hit": a body with two colliders is found twice.
//
// The chapter's bomb at (6.4, 2.5) goes off at step 30 over two 1 kg crates
// on frictionless ground, at the same distance on either side: a plain
// crate at x = 4.4 and, at x = 8.4, a crate with a lid (a second, thin
// collider on the same body, 0.3 m above its center). OverlapCircle returns
// colliders, so it lists the lidded crate twice. The blast loop pushes the
// body of every collider it finds, J (1 - d / radius) with J = 8 N s and d
// the distance to the body's center. With parameter "One per body" at 0 the
// lidded crate is pushed twice and leaves twice as fast as the plain one;
// at 1 the loop skips a body it has already pushed, and both get the same
// push, 1.449 m/s.
//
// Readouts: colliders found, bodies pushed, impulses applied, and the speeds
// of the plain and the lidded crate after the blast. Drawn: the blast circle
// in red, the crates in orange, the lid in yellow.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class OneHitSample final : public Sample {
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

  [[nodiscard]] bool exploded() const { return exploded_; }
  [[nodiscard]] int colliders_found() const { return colliders_found_; }
  [[nodiscard]] int bodies_pushed() const { return bodies_pushed_; }
  [[nodiscard]] int impulses() const { return impulses_; }
  [[nodiscard]] Vec2f plain_velocity() const {
    return world_.body(plain_).velocity;
  }
  [[nodiscard]] Vec2f lidded_velocity() const {
    return world_.body(lidded_).velocity;
  }

  static constexpr std::size_t kOnePerBody = 0;
  static constexpr std::size_t kRadius = 1;
  static constexpr int kBlastStep = 30;
  static constexpr std::uint16_t kCrate = 1 << 2;

 private:
  void Explode();
  BodyIndex AddCrate(Scalar x);

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 4,
                       .max_colliders = 4,
                       .max_touching_pairs = 8,
                       .max_broadphase_pairs = 16,
                       .max_joints = 0}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "One per body",
       .unit = "",
       .min = 0,
       .max = 1,
       .step = 1,
       .scale = 1,
       .value = 1},
      {.name = "Radius",
       .unit = "m",
       .min = 10,
       .max = 60,
       .step = 1,
       .scale = 10,
       .value = 35},
  }};
  std::array<SampleReadout, 5> readouts_{{
      {.name = "Colliders found", .unit = "", .scale = 1},
      {.name = "Bodies pushed", .unit = "", .scale = 1},
      {.name = "Impulses", .unit = "", .scale = 1},
      {.name = "Plain speed", .unit = "m/s", .scale = 1000},
      {.name = "Lidded speed", .unit = "m/s", .scale = 1000},
  }};
  Aabbf ground_{};
  BodyIndex plain_ = kInvalidBodyIndex;
  BodyIndex lidded_ = kInvalidBodyIndex;
  int step_ = 0;
  int colliders_found_ = 0;
  int bodies_pushed_ = 0;
  int impulses_ = 0;
  bool exploded_ = false;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_10_EXPLOSION_SOLUTIONS_ONE_HIT_SAMPLE_H_
