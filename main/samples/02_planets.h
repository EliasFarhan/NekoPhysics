#pragma once
#include "physics/physics.h"
#include "sample.h"

namespace neko
{

class PlanetSample : public Sample
{
public:
    void Begin() override;
    void Update(float dt) override;
    void FixedUpdate() override;
    void Draw(SDL_Renderer* renderer) override;
    void End() override;
private:
    static constexpr std::size_t circleCount = 1'000;
    PhysicsWorld world_{ {.gravity = Vec2f::zero(), .dt = fixedDeltaTime, .maxBodies = circleCount} };
    std::vector<SDL_Vertex> vertices_;
    std::vector<int> indices_;
};
} // namespace neko
