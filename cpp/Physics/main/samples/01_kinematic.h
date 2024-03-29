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
    void End() override;
    void Draw(SDL_Renderer* renderer) override;

private:
    PhysicsWorld world_{ {Scalar{0.0f}, Scalar{0.0f}} };
    std::vector<SDL_Vertex> vertices_;
    std::vector<int> indices_;
    
};
}
