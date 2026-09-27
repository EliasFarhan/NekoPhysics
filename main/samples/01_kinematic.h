#pragma once

#include "sample.h"
#include "physics/physics.h"

namespace neko
{
class KinematicSample : public Sample
{
public:
    void Begin() override;
    void Update(float dt) override;
    void FixedUpdate() override;
    void End() override;
    void Draw(SDL_Renderer* renderer) override;

private:
    static constexpr std::size_t circleCount = 1'000;
    PhysicsWorld world_{ {.gravity = Vec2f::zero(), .dt = fixedDeltaTime, .maxBodies = circleCount} };
    std::vector<SDL_Vertex> vertices_;
    std::vector<int> indices_;
    
};
}
