#ifndef MAIN_SAMPLES_03_TRIGGERS_H_
#define MAIN_SAMPLES_03_TRIGGERS_H_
#include <vector>

#include "neko/bsh/quadtree.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko {
/// Sample 03: trigger circles in a quadtree broadphase.
///
/// Every body gets a circle collider, left as a trigger (the default
/// `Collider::is_trigger = true`): triggers report overlaps through the
/// `ContactListener` but don't push each other. The sample registers itself
/// with `PhysicsWorld::SetContactListener` and counts, per body, the triggers
/// it overlaps in @ref OnTriggerEnter and @ref OnTriggerExit. The world uses a
/// `QuadTree` as broadphase (`PhysicsWorld::SetBSH`), so only nearby circles
/// are tested against each other.
///
/// What to look at: circles are red alone and green while they overlap
/// another one, and they pass through each other. The grey lines are the
/// quadtree nodes: the tree splits where circles crowd together. The world
/// keeps the default `PhysicsWorldConfig::gravity` (0, -9.81), which pulls
/// toward the top of the screen since the samples draw with y down;
/// @ref BounceInside keeps the circles on screen. The number of circles is
/// @ref sample_config::kTriggerCount.
class TriggersSample : public Sample, public ContactListener {
 public:
  /// Builds the scene described in the class documentation.
  void Begin(const SampleContext& context) override;
  /// Draws the circles, colored by overlap, and the quadtree nodes.
  void Draw(DrawList& draw_list) const override;
  /// Clears the world and the per-body data.
  void End() override;
  /// Returns the checksum of the sample's physics world.
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }
  /// Counts the overlap on both bodies of the pair.
  void OnTriggerEnter(const ColliderPair& p) override;
  /// Removes the overlap from both bodies of the pair.
  void OnTriggerExit(const ColliderPair& p) override;
  /// Steps the world, then bounces every circle inside the world bounds.
  void FixedUpdate() override;
  /// Does nothing: the colliders are triggers.
  void OnCollisionEnter(const ColliderPair& p) override;
  /// Does nothing: the colliders are triggers.
  void OnCollisionExit(const ColliderPair& p) override;

 private:
  struct TriggeredBody {
    int count = 0;
    Scalar circle_radius{0};
  };
  static constexpr Index kCircleCount = sample_config::kTriggerCount;
  static constexpr std::size_t kMaxPairs = std::size_t{kCircleCount} * 8;
  PhysicsWorld world_{{.dt = kFixedDeltaTime,
                       .max_bodies = kCircleCount,
                       .max_colliders = kCircleCount,
                       .max_touching_pairs = kMaxPairs,
                       .max_broadphase_pairs = 1}};
  QuadTree quad_tree_{kCircleCount, kMaxPairs};
  std::vector<TriggeredBody> bodies_;
  Aabbf bounds_{};
};
}  // namespace neko

#endif  // MAIN_SAMPLES_03_TRIGGERS_H_
