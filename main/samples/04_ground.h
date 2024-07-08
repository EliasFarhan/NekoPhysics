#pragma once
#include "physics/physics.h"
#include "sample.h"

namespace neko
{
class GroundSample : public Sample, public ContactListener
{
public:
    void Begin() override;
    void Update(float dt) override;
    void Draw(SDL_Renderer* renderer) override;
    void End() override;
    void OnTriggerEnter(const ColliderPair& p) override;
    void OnTriggerExit(const ColliderPair& p) override;
    void FixedUpdate() override;
    void OnCollisionEnter(const ColliderPair& p) override;
    void OnCollisionExit(const ColliderPair& p) override;
private:
    PhysicsWorld world_{ Vec2f{Scalar{0.0f}, Scalar{9.81f}} };
    QuadTree quadTree_;
    std::vector<SDL_Vertex> vertices_;
    std::vector<int> indices_;
    BodyIndex circleBodyIndex_ = INVALID_BODY_INDEX;
    BodyIndex groundBodyIndex_ = INVALID_BODY_INDEX;
    int collisionCount = 0;
};
}
