#ifndef MAIN_SAMPLES_01_KINEMATIC_H_
#define MAIN_SAMPLES_01_KINEMATIC_H_

#include <vector>

#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko {
/// Sample 01: circles moving at constant speed, bouncing on the world bounds.
///
/// The simplest use of a @ref PhysicsWorld: bodies only, with no collider, no
/// gravity (`gravity =` @ref Vec2f::Zero) and no force. Each
/// @ref PhysicsWorld::Step integrates `position += velocity * dt`, so every
/// circle moves in a straight line at the random velocity it got in
/// @ref Begin. @ref BounceInside flips the velocity when a circle leaves the
/// screen.
///
/// What to look at: the circles cross each other without reacting (there is
/// nothing to collide with), and their speed never changes. The number of
/// circles is @ref sample_config::kKinematicCount.
class KinematicSample : public Sample {
 public:
  /// Builds the scene described in the class documentation.
  void Begin(const SampleContext& context) override;
  /// Steps the world, then bounces every circle inside the world bounds.
  void FixedUpdate() override;
  /// Draws one circle per body, in its random color.
  void Draw(DrawList& draw_list) const override;
  /// Clears the world and the per-body data.
  void End() override;
  /// Returns the checksum of the sample's physics world.
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }

 private:
  static constexpr Index kCircleCount = sample_config::kKinematicCount;
  PhysicsWorld world_{
      {.gravity = Vec2f::Zero(),
       .dt = kFixedDeltaTime,
       .max_bodies = kCircleCount,
       .max_colliders = 1,
       .max_touching_pairs = 1,
       .max_broadphase_pairs = 1,
       .velocity_iterations = sample_config::kVelocityIterations}};
  std::vector<ColorId> colors_;
  Aabbf bounds_{};
};
}  // namespace neko

#endif  // MAIN_SAMPLES_01_KINEMATIC_H_
