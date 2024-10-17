#pragma once

#include "physics/physics.h"
#include "sample.h"

namespace neko
{

class MoveIntoSample : public Sample, public ContactListener
{
public:
	void OnTriggerEnter(const ColliderPair& p) override;

	void OnTriggerExit(const ColliderPair& p) override;

	void OnCollisionEnter(const ColliderPair& p) override;

	void OnCollisionExit(const ColliderPair& p) override;

	void Begin() override;

	void Update(float dt) override;

	void Draw(SDL_Renderer* renderer) override;

	void End() override;

	void FixedUpdate() override;
private:
	PhysicsWorld world_{ Vec2f{Scalar{0.0f}, Scalar{9.81f}} };
	QuadTree quadTree_;
	std::vector<SDL_Vertex> vertices_;
	std::vector<int> indices_;
	BodyIndex staticBodyIndex = INVALID_BODY_INDEX;
	BodyIndex dynamicBodyIndex = INVALID_BODY_INDEX;
	BodyIndex groundBodyIndex_ = INVALID_BODY_INDEX;
	ColliderIndex groundColliderIndex_ = INVALID_COLLIDER_INDEX;
	ColliderIndex staticColliderIndex_ = INVALID_COLLIDER_INDEX;
	ColliderIndex dynamicColliderIndex_ = INVALID_COLLIDER_INDEX;

	bool staticBodyOnGround_ = false;
	bool dynamicBodyOnGround_ = false;
};
}
