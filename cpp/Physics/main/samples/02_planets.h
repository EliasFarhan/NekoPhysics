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
    void Draw(SDL_Renderer* renderer) override;
    void End() override;
private:
    PhysicsWorld world_{{0,0}};
    std::vector<SDL_Vertex> vertices_;
    std::vector<int> indices_;
};
} // namespace neko
