#pragma once
#include "physics/physics.h"
#include "sample.h"

namespace neko
{
class TriggersSample : public Sample, public ContactListener
{
public:
    void Begin() override;
    void Update(float dt) override;
    void Draw(SDL_Renderer* renderer) override;
    void End() override;
    void OnTriggerEnter(const TriggerPair& p) override;
    void OnTriggerExit(const TriggerPair& p) override;
    void FixedUpdate() override;
private:
    struct TriggeredBody
    {
        ColliderIndex index{};
        int count = 0;
        float circleRadius = 0.0f;
    };
    PhysicsWorld world_;
    QuadTree quadTree_;
    std::vector<TriggeredBody> bodies_;
    std::vector<SDL_Vertex> vertices_;
    std::vector<int> indices_;
};
}
