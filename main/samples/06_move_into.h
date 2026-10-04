#ifndef MAIN_SAMPLES_06_MOVE_INTO_H_
#define MAIN_SAMPLES_06_MOVE_INTO_H_

#include "neko/physics/physics.h"
#include "neko/bsh/quadtree.h"
#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

// Box2D comparison, PC only: define USE_BOX2D (and link box2d::box2d to NekoPhysicsSamples) to enable it
#if defined(USE_BOX2D) && !defined(NEKO_PLATFORM_NDS)
#define NEKO_SAMPLE_BOX2D
#include <box2d/box2d.h>
#endif

namespace neko
{

/**
 * @brief Two boxes on a static ground, pushed with forces once they land: one is held in place, the other moves left
 * into it. Magenta once on the ground.
 */
class MoveIntoSample final : public Sample,
#ifndef NEKO_SAMPLE_BOX2D
	public ContactListener
#else
	public b2ContactListener
#endif
{
public:
#ifndef NEKO_SAMPLE_BOX2D
	void OnTriggerEnter(const ColliderPair& p) override;

	void OnTriggerExit(const ColliderPair& p) override;

	void OnCollisionEnter(const ColliderPair& p) override;

	void OnCollisionExit(const ColliderPair& p) override;
#else
	/// Called when two fixtures begin to touch.
	void BeginContact(b2Contact* contact) override;

	/// Called when two fixtures cease to touch.
	void EndContact(b2Contact* contact) override;
#endif
	void Begin(const SampleContext& context) override;

	void Draw(DrawList& draw_list) const override;

	void End() override;

	[[nodiscard]] std::uint32_t GenerateChecksum() const override;

	void FixedUpdate() override;
private:
#ifndef NEKO_SAMPLE_BOX2D
	static constexpr std::size_t kBodyCount = 3;
	static constexpr std::size_t kMaxPairs = 8;
	PhysicsWorld world_{ {.gravity = Vec2f{Scalar{0}, Ratio(981, 100)}, .dt = kFixedDeltaTime, .max_bodies = kBodyCount,
		.max_colliders = kBodyCount, .max_touching_pairs = kMaxPairs, .max_broadphase_pairs = 1} };
	QuadTree quad_tree_{ kBodyCount, kMaxPairs };

	BodyIndex static_body_index_ = kInvalidBodyIndex;
	BodyIndex dynamic_body_index_ = kInvalidBodyIndex;
	BodyIndex ground_body_index_ = kInvalidBodyIndex;
	ColliderIndex ground_collider_index_ = kInvalidColliderIndex;
	ColliderIndex static_collider_index_ = kInvalidColliderIndex;
	ColliderIndex dynamic_collider_index_ = kInvalidColliderIndex;
#else
	//Box 2d version
	b2World b2_world_{b2Vec2{0.0f,9.81f}};
	b2Body* b2_ground_body_;
	b2Body* b2_static_body_;
	b2Body* b2_dynamic_body_;

#endif
	bool static_body_on_ground_ = false;
	bool dynamic_body_on_ground_ = false;
};
}

#endif  // MAIN_SAMPLES_06_MOVE_INTO_H_
