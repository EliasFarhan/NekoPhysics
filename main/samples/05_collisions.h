#pragma once

#include "physics/physics.h"
#include "bsh/quadtree.h"
#include "sample.h"

#include <vector>

namespace neko
{
class CollisionsSample : public Sample, public ContactListener
{
public:
    void Begin(const SampleContext& context) override;
    void Draw(DrawList& drawList) const override;
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
    PhysicsWorld world_{ {.dt = fixedDeltaTime} };
    QuadTree quadTree_;
    std::vector<TriggeredBody> bodies_;
};
}