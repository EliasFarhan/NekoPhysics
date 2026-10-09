#ifndef MAIN_SAMPLES_05_COLLISIONS_H_
#define MAIN_SAMPLES_05_COLLISIONS_H_

#include <vector>

#include "neko/bsh/quadtree.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko {
/// Sample 05: colliding circles in a quadtree broadphase.
///
/// The same scene as @ref TriggersSample with bigger circles whose colliders
/// are solid (`Collider::is_trigger = false`): the contact solver pushes
/// overlapping circles apart and makes them bounce (default restitution 1),
/// and the listener receives @ref OnCollisionEnter and @ref OnCollisionExit
/// instead of the trigger callbacks.
///
/// What to look at: circles are red alone and green while they touch another
/// one; unlike the triggers sample they bounce off each other instead of
/// passing through. The world keeps the default `PhysicsWorldConfig::gravity`
/// (0, -9.81), which pulls toward the top of the screen since the samples
/// draw with y down; @ref BounceInside keeps the circles on screen. The
/// number of circles is @ref sample_config::kCollisionCount.
class CollisionsSample : public Sample, public ContactListener {
 public:
  /// Builds the scene described in the class documentation.
  void Begin(const SampleContext& context) override;
  /// Draws the circles, colored by contact.
  void Draw(DrawList& draw_list) const override;
  /// Clears the world and the per-body data.
  void End() override;
  /// Returns the checksum of the sample's physics world.
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }
  /// Does nothing: the colliders are solid.
  void OnTriggerEnter(const ColliderPair& p) override;
  /// Does nothing: the colliders are solid.
  void OnTriggerExit(const ColliderPair& p) override;
  /// Steps the world, then bounces every circle inside the world bounds.
  void FixedUpdate() override;
  /// Counts the contact on both bodies of the pair.
  void OnCollisionEnter(const ColliderPair& p) override;
  /// Removes the contact from both bodies of the pair.
  void OnCollisionExit(const ColliderPair& p) override;

 private:
  struct CollidingBody {
    int count = 0;
    Scalar circle_radius{0};
  };
  static constexpr Index kCircleCount = sample_config::kCollisionCount;
  static constexpr std::size_t kMaxPairs = std::size_t{kCircleCount} * 8;
  PhysicsWorld world_{{.dt = kFixedDeltaTime,
                       .max_bodies = kCircleCount,
                       .max_colliders = kCircleCount,
                       .max_touching_pairs = kMaxPairs,
                       .max_broadphase_pairs = 1}};
  QuadTree quad_tree_{kCircleCount, kMaxPairs};
  std::vector<CollidingBody> bodies_;
  Aabbf bounds_{};
};
}  // namespace neko

#endif  // MAIN_SAMPLES_05_COLLISIONS_H_
