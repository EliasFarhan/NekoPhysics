#pragma once

#include "physics/physics.h"
#include "sample.h"

#include <box2d/b2_world.h>

namespace neko
{

#define USE_BOX2D
class MoveIntoSample final : public Sample,
#ifndef USE_BOX2D
	public ContactListener
#else
	public b2ContactListener
#endif
{
public:
#ifndef USE_BOX2D
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
	void Begin() override;

	void Update(float dt) override;

	void Draw(SDL_Renderer* renderer) override;

	void End() override;

	void FixedUpdate() override;
private:
#ifndef USE_BOX2D
	PhysicsWorld world_{ Vec2f{Scalar{0.0f}, Scalar{9.81f}} };
	QuadTree quadTree_;

	BodyIndex staticBodyIndex = INVALID_BODY_INDEX;
	BodyIndex dynamicBodyIndex = INVALID_BODY_INDEX;
	BodyIndex groundBodyIndex_ = INVALID_BODY_INDEX;
	ColliderIndex groundColliderIndex_ = INVALID_COLLIDER_INDEX;
	ColliderIndex staticColliderIndex_ = INVALID_COLLIDER_INDEX;
	ColliderIndex dynamicColliderIndex_ = INVALID_COLLIDER_INDEX;
#else
	//Box 2d version
	b2World b2World_{b2Vec2{0.0f,-9.81f}};
	b2Body* b2groundBody_;
	b2Body* b2staticBody_;
	b2Body* b2dynamicBody_;

#endif
	std::vector<SDL_Vertex> vertices_;
	std::vector<int> indices_;


	bool staticBodyOnGround_ = false;
	bool dynamicBodyOnGround_ = false;
};
}
