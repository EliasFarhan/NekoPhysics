#ifndef MAIN_SAMPLES_05_COLLISIONS_H_
#define MAIN_SAMPLES_05_COLLISIONS_H_

#include <vector>

#include "neko/bsh/quadtree.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko {
// Colliding circles in a quadtree broadphase: red alone, green while touching
// another one
class CollisionsSample : public Sample, public ContactListener {
 public:
  void Begin(const SampleContext& context) override;
  void Draw(DrawList& draw_list) const override;
  void End() override;
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }
  void OnTriggerEnter(const ColliderPair& p) override;
  void OnTriggerExit(const ColliderPair& p) override;
  void FixedUpdate() override;
  void OnCollisionEnter(const ColliderPair& p) override;
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
