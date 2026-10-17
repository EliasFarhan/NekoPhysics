#ifndef MAIN_SAMPLES_04_GROUND_H_
#define MAIN_SAMPLES_04_GROUND_H_
#include "neko/bsh/quadtree.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko {
/// Sample 04: a ball and a box falling on a static ground.
///
/// The first sample with solid colliders (@ref Collider::is_trigger `= false`)
/// and gravity: @ref PhysicsWorldConfig::gravity is (0, -9.81), pointing down
/// on screen. The ground is a @ref BodyType::kStatic body with zero
/// `inverse_mass` and an AABB collider; the ball (circle collider) and the
/// box (AABB collider) are dynamic. It shows circle-vs-AABB and AABB-vs-AABB
/// contacts and @ref Collider::restitution: the ball has 0.5 and bounces a few
/// times, the box has 0 and stops dead.
///
/// What to look at: the ball is blue and turns magenta while at least one
/// collision is in progress, counted in @ref OnCollisionEnter and
/// @ref OnCollisionExit. The counter covers every pair, including the box on
/// the ground, so the ball stays magenta once the box has landed.
class GroundSample : public Sample, public ContactListener {
 public:
  /// Builds the scene described in the class documentation.
  void Begin(const SampleContext& context) override;
  /// Draws the ball, the ground and the box.
  void Draw(DrawList& draw_list) const override;
  /// Clears the world and the per-body data.
  void End() override;
  /// Returns the checksum of the sample's physics world.
  [[nodiscard]] std::uint32_t GenerateChecksum() const override {
    return world_.GenerateChecksum();
  }
  /// Does nothing: the sample has no trigger.
  void OnTriggerEnter(const ColliderPair& p) override;
  /// Does nothing: the sample has no trigger.
  void OnTriggerExit(const ColliderPair& p) override;
  /// Steps the world.
  void FixedUpdate() override;
  /// Counts one more touching pair.
  void OnCollisionEnter(const ColliderPair& p) override;
  /// Counts one less touching pair.
  void OnCollisionExit(const ColliderPair& p) override;

 private:
  static constexpr std::size_t kBodyCount = 3;
  static constexpr std::size_t kMaxPairs = 8;
  PhysicsWorld world_{
      {.gravity = Vec2f{Scalar{0}, -Ratio(981, 100)},
       .dt = kFixedDeltaTime,
       .max_bodies = kBodyCount,
       .max_colliders = kBodyCount,
       .max_touching_pairs = kMaxPairs,
       .max_broadphase_pairs = 1,
       .velocity_iterations = sample_config::kVelocityIterations}};
  QuadTree quad_tree_{kBodyCount, kMaxPairs};
  BodyIndex circle_body_index_ = kInvalidBodyIndex;
  BodyIndex rect_body_index_ = kInvalidBodyIndex;
  BodyIndex ground_body_index_ = kInvalidBodyIndex;
  int collision_count_ = 0;
};
}  // namespace neko

#endif  // MAIN_SAMPLES_04_GROUND_H_
