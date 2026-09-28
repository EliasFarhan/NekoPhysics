#pragma once

#include "physics/physics.h"
#include "bsh/quadtree.h"
#include "sample.h"

// Box2D comparison, PC only: define USE_BOX2D (and link box2d::box2d to NekoPhysicsSamples) to enable it
#if defined(USE_BOX2D) && !defined(NEKO_PLATFORM_NDS)
#define NEKO_SAMPLE_BOX2D
#include <box2d/box2d.h>
#endif

namespace neko
{

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

	void Draw(DrawList& drawList) const override;

	void End() override;

	void FixedUpdate() override;
private:
#ifndef NEKO_SAMPLE_BOX2D
	PhysicsWorld world_{ {.gravity = Vec2f{Scalar{0.0f}, Scalar{9.81f}}, .dt = fixedDeltaTime} };
	QuadTree quadTree_;

	BodyIndex staticBodyIndex = INVALID_BODY_INDEX;
	BodyIndex dynamicBodyIndex = INVALID_BODY_INDEX;
	BodyIndex groundBodyIndex_ = INVALID_BODY_INDEX;
	ColliderIndex groundColliderIndex_ = INVALID_COLLIDER_INDEX;
	ColliderIndex staticColliderIndex_ = INVALID_COLLIDER_INDEX;
	ColliderIndex dynamicColliderIndex_ = INVALID_COLLIDER_INDEX;
#else
	//Box 2d version
	b2World b2World_{b2Vec2{0.0f,9.81f}};
	b2Body* b2groundBody_;
	b2Body* b2staticBody_;
	b2Body* b2dynamicBody_;

#endif
	bool staticBodyOnGround_ = false;
	bool dynamicBodyOnGround_ = false;
};
}
