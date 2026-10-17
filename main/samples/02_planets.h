#ifndef MAIN_SAMPLES_02_PLANETS_H_
#define MAIN_SAMPLES_02_PLANETS_H_
#include <vector>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko {

/// Sample 02: circles orbiting a black hole at the center of the world.
///
/// Shows how to drive bodies with forces. The world has no gravity; instead,
/// @ref FixedUpdate adds to each @ref Body::force an attraction toward the
/// center that falls off with the square of the distance, then calls
/// @ref PhysicsWorld::Step, which turns the force into acceleration (times
/// `inverse_mass`) and clears it. @ref Begin gives each planet the speed of a
/// circular orbit, perpendicular to its radius, plus a random disturbance, so
/// the orbits are slightly elliptical.
///
/// What to look at: inner planets go around faster than outer ones, and the
/// orbits stay stable over time. There are no colliders. The number of
/// planets is @ref sample_config::kPlanetCount.
class PlanetSample : public Sample {
 public:
  /// Builds the scene described in the class documentation.
  void Begin(const SampleContext& context) override;
  /// Applies the black hole's attraction to every planet, then steps the world.
  void FixedUpdate() override;
  /// Draws one circle per planet, in its random color.
  void Draw(DrawList& draw_list) const override;
  /// Clears the world and the per-body data.
  void End() override;
  /// Returns the checksum of the sample's physics world.
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }

 private:
  static constexpr Index kCircleCount = sample_config::kPlanetCount;
  PhysicsWorld world_{
      {.gravity = Vec2f::Zero(),
       .dt = kFixedDeltaTime,
       .max_bodies = kCircleCount,
       .max_colliders = 1,
       .max_touching_pairs = 1,
       .max_broadphase_pairs = 1,
       .velocity_iterations = sample_config::kVelocityIterations}};
  std::vector<ColorId> colors_;
  Vec2f center_{};
};
}  // namespace neko

#endif  // MAIN_SAMPLES_02_PLANETS_H_
