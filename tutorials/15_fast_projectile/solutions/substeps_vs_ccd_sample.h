#ifndef TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_SUBSTEPS_VS_CCD_SAMPLE_H_
#define TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_SUBSTEPS_VS_CCD_SAMPLE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_rng.h"

namespace neko::tutorial {

// Chapter 15, exercise "substeps vs CCD": two cures for tunneling.
//
// Two worlds with the same plain shot (radius r = 0.1 m) fired every second
// at the speed of the parameter "Speed" (80 m/s) at a 0.2 m wall, from a
// seeded random place along the first half meter:
//
// - Top, substeps: the world's time step is dt / N (parameter "Substeps")
//   and each frame steps it N times. The shot moves v dt / N per step, so it
//   never misses once v dt / N <= r + t / 2 (the threshold of the "speed
//   threshold" exercise): N >= v dt / (r + t / 2), 7 at 80 m/s. Every body
//   of the world pays N steps per frame.
// - Bottom, continuous collision: one step per frame and Body::is_bullet on
//   the shot only. It never misses, at the cost of one sweep of one body.
//
// Readouts: the hits and misses of both, the substeps the formula asks for,
// and the steps per frame of each world. Drawn: the walls in grey, the
// substepped shot in orange, the bullet in sky blue.
//
// The world is y-up, in meters and seconds; shots ignore gravity.
class SubstepsVsCcdSample final : public Sample {
 public:
  void Begin(const SampleContext& context) override;
  void FixedUpdate() override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;
  [[nodiscard]] std::span<SampleParameter> parameters() override {
    return parameters_;
  }
  [[nodiscard]] std::span<const SampleReadout> readouts() const override {
    return readouts_;
  }

  // The smallest N with v dt / N <= r + t / 2
  [[nodiscard]] int needed_substeps() const;
  // 0 for the substepped world, 1 for the bullet's
  [[nodiscard]] int hits(std::size_t world) const { return hits_[world]; }
  [[nodiscard]] int misses(std::size_t world) const { return misses_[world]; }

  static constexpr std::size_t kSpeed = 0;
  static constexpr std::size_t kSubsteps = 1;
  static constexpr int kFirePeriod = 60;

 private:
  void FireBoth();

  // Built in Begin: its time step depends on the parameter
  std::optional<PhysicsWorld> substep_world_;
  PhysicsWorld bullet_world_{{.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
                              .dt = kFixedDeltaTime,
                              .max_bodies = 4,
                              .max_colliders = 4,
                              .max_touching_pairs = 4,
                              .max_broadphase_pairs = 4,
                              .max_joints = 0,
                              .max_passing_pairs = 0}};
  std::array<SampleParameter, 2> parameters_{{
      {.name = "Speed",
       .unit = "m/s",
       .min = 10,
       .max = 100,
       .step = 5,
       .scale = 1,
       .value = 80},
      {.name = "Substeps",
       .unit = "",
       .min = 1,
       .max = 16,
       .step = 1,
       .scale = 1,
       .value = 1},
  }};
  std::array<SampleReadout, 7> readouts_{{
      {.name = "Substep hits", .unit = "", .scale = 1},
      {.name = "Substep misses", .unit = "", .scale = 1},
      {.name = "CCD hits", .unit = "", .scale = 1},
      {.name = "CCD misses", .unit = "", .scale = 1},
      {.name = "Substeps needed", .unit = "", .scale = 1},
      {.name = "Steps/frame substep", .unit = "", .scale = 1},
      {.name = "Steps/frame CCD", .unit = "", .scale = 1},
  }};
  void UpdateReadouts();
  [[nodiscard]] PhysicsWorld& world(std::size_t index) {
    return index == 0 ? *substep_world_ : bullet_world_;
  }

  SampleRng rng_{15};
  std::array<ColliderIndex, 2> shots_{};
  std::array<ColliderIndex, 2> walls_{};
  std::array<Scalar, 2> lane_y_{};
  std::array<int, 2> hits_{};
  std::array<int, 2> misses_{};
  Scalar muzzle_x_{};
  Scalar wall_x_{};
  int fired_ = 0;
  int step_ = 0;
};

}  // namespace neko::tutorial

#endif  // TUTORIALS_15_FAST_PROJECTILE_SOLUTIONS_SUBSTEPS_VS_CCD_SAMPLE_H_
