#ifndef TUTORIALS_10_EXPLOSION_SOLUTIONS_REACH_SAMPLE_H_
#define TUTORIALS_10_EXPLOSION_SOLUTIONS_REACH_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"

namespace neko::tutorial {

// Chapter 10, exercise "reach": the smallest blast radius that finds each
// target, by bisection.
//
// The chapter's bomb and its five targets (crate, barrel, shielded crate,
// crate behind the wall, far crate), all of the crate category here. For
// each target the sample halves an interval of radii, from 0 (not found) to
// 8 m (found), asking OverlapCircle(bomb, radius) at the midpoint, until the
// interval is under 1 mm: 13 queries. The overlap is exact for the shape,
// so the reach is the distance from the bomb to the closest point of the
// shape, not to its center: the clamp of the bomb to the box for a crate,
// the center distance minus the radius for the barrel.
//
// Readouts, for the target chosen by parameter "Target" (0 to 4, in the
// order above): the measured reach, the closest-point distance, the center
// distance, and the number of queries. Drawn: the targets (the chosen one in
// orange), the bomb, the circle of the measured reach in red, the line to
// the closest point in green and to the center in grey.
//
// The world is y-up, in meters and seconds, gravity (0, -9.81) m/s^2.
class ReachSample final : public Sample {
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

  // Smallest radius found by bisection whose overlap query sees target i
  [[nodiscard]] Scalar measured_reach(std::size_t i) const { return reach_[i]; }
  // Distance from the bomb to the closest point of target i's shape
  [[nodiscard]] Scalar predicted_reach(std::size_t i) const;
  // Distance from the bomb to target i's center
  [[nodiscard]] Scalar center_distance(std::size_t i) const;
  [[nodiscard]] int queries(std::size_t i) const { return queries_[i]; }

  static constexpr std::size_t kTarget = 0;
  static constexpr std::size_t kTargetCount = 5;
  static constexpr std::uint16_t kCrate = 1 << 2;

 private:
  [[nodiscard]] bool Finds(std::size_t i, Scalar radius) const;
  [[nodiscard]] Vec2f closest_point(std::size_t i) const;

  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = 8,
                       .max_colliders = 8,
                       .max_touching_pairs = 16,
                       .max_broadphase_pairs = 32,
                       .max_joints = 0}};
  std::array<SampleParameter, 1> parameters_{{
      {.name = "Target",
       .unit = "",
       .min = 0,
       .max = 4,
       .step = 1,
       .scale = 1,
       .value = 3},
  }};
  std::array<SampleReadout, 4> readouts_{{
      {.name = "Reach", .unit = "m", .scale = 1000},
      {.name = "Closest point", .unit = "m", .scale = 1000},
      {.name = "Center dist.", .unit = "m", .scale = 1000},
      {.name = "Queries", .unit = "", .scale = 1},
  }};
  Aabbf ground_{};
  std::array<BodyIndex, kTargetCount> bodies_{};
  std::array<ColliderIndex, kTargetCount> colliders_{};
  std::array<Scalar, kTargetCount> reach_{};
  std::array<int, kTargetCount> queries_{};
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_10_EXPLOSION_SOLUTIONS_REACH_SAMPLE_H_
