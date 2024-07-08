#pragma once

#include "physics/physics.h"
#include "sample.h"

namespace neko
{
class CollisionsSample : public Sample, public ContactListener
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
    struct TriggeredBody
    {
        ColliderIndex index{};
        int count = 0;
        Scalar circleRadius{ 0.0f };
    };
    PhysicsWorld world_;
    QuadTree quadTree_;
    std::vector<TriggeredBody> bodies_;
    std::vector<SDL_Vertex> vertices_;
    std::vector<int> indices_;
};
}