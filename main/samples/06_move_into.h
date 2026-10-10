#ifndef MAIN_SAMPLES_06_MOVE_INTO_H_
#define MAIN_SAMPLES_06_MOVE_INTO_H_

#include "neko/bsh/quadtree.h"
#include "neko/physics/physics.h"

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

// Box2D comparison, PC only: define USE_BOX2D (and link box2d::box2d to
// NekoPhysicsSamples) to enable it
#if defined(USE_BOX2D) && !defined(NEKO_PLATFORM_NDS)
#define NEKO_SAMPLE_BOX2D
#include <box2d/box2d.h>
#endif

namespace neko {

/// Sample 06: two boxes on a static ground, steered with forces.
///
/// Shows how to control a body with forces, like a character controller:
/// once a box has landed, @ref FixedUpdate computes the force that would
/// reach a wanted velocity in one step, `(wanted - velocity) / dt * mass`,
/// clamps it to a maximum and adds it to @ref Body::force. The left box wants a
/// zero velocity and is held in place; the right box wants 1 m/s to the left
/// and walks into it, so the AABB-vs-AABB contact has to resolve a box pushing
/// another one that resists. Landing is detected in @ref OnCollisionEnter by
/// comparing the @ref ColliderPair with (box, ground).
///
/// What to look at: the boxes are blue in the air and turn magenta once they
/// have touched the ground; then the right box moves left, stops against the
/// held box, and the two push against each other.
///
/// On PC, defining `USE_BOX2D` (and linking `box2d::box2d` to
/// `NekoPhysicsSamples`) runs the same scene with Box2D instead, to compare
/// the behaviors; its checksum is then always 0.
class MoveIntoSample final : public Sample,
#ifndef NEKO_SAMPLE_BOX2D
                             public ContactListener
#else
                             public b2ContactListener
#endif
{
 public:
#ifndef NEKO_SAMPLE_BOX2D
  /// Does nothing: the sample has no trigger.
  void OnTriggerEnter(const ColliderPair& p) override;

  /// Does nothing: the sample has no trigger.
  void OnTriggerExit(const ColliderPair& p) override;

  /// Marks a box as on the ground when it touches the ground.
  void OnCollisionEnter(const ColliderPair& p) override;

  /// Does nothing: a box stays marked as on the ground.
  void OnCollisionExit(const ColliderPair& p) override;
#else
  /// Called when two fixtures begin to touch.
  void BeginContact(b2Contact* contact) override;

  /// Called when two fixtures cease to touch.
  void EndContact(b2Contact* contact) override;
#endif
  /// Builds the ground and the two boxes above it.
  void Begin(const SampleContext& context) override;

  /// Draws the ground and the two boxes, colored by landing state.
  void Draw(DrawList& draw_list) const override;

  /// Clears the world and the landing flags.
  void End() override;

  /// Returns the checksum of the physics world (0 with Box2D).
  [[nodiscard]] std::uint32_t GenerateChecksum() const override;

  /// Applies the steering forces to the landed boxes, then steps the world.
  void FixedUpdate() override;

 private:
#ifndef NEKO_SAMPLE_BOX2D
  static constexpr std::size_t kBodyCount = 3;
  static constexpr std::size_t kMaxPairs = 8;
  PhysicsWorld world_{{.gravity = Vec2f{Scalar{0}, Ratio(981, 100)},
                       .dt = kFixedDeltaTime,
                       .max_bodies = kBodyCount,
                       .max_colliders = kBodyCount,
                       .max_touching_pairs = kMaxPairs,
                       .max_broadphase_pairs = 1}};
  QuadTree quad_tree_{kBodyCount, kMaxPairs};

  BodyIndex static_body_index_ = kInvalidBodyIndex;
  BodyIndex dynamic_body_index_ = kInvalidBodyIndex;
  BodyIndex ground_body_index_ = kInvalidBodyIndex;
  ColliderIndex ground_collider_index_ = kInvalidColliderIndex;
  ColliderIndex static_collider_index_ = kInvalidColliderIndex;
  ColliderIndex dynamic_collider_index_ = kInvalidColliderIndex;
#else
  // Box 2d version
  b2World b2_world_{b2Vec2{0.0f, 9.81f}};
  b2Body* b2_ground_body_;
  b2Body* b2_static_body_;
  b2Body* b2_dynamic_body_;

#endif
  bool static_body_on_ground_ = false;
  bool dynamic_body_on_ground_ = false;
};
}  // namespace neko

#endif  // MAIN_SAMPLES_06_MOVE_INTO_H_
