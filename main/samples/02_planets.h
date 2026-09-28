#pragma once
#include "physics/physics.h"
#include "sample.h"

#include <vector>

namespace neko
{

class PlanetSample : public Sample
{
public:
    void Begin(const SampleContext& context) override;
    void FixedUpdate() override;
    void Draw(DrawList& drawList) const override;
    void End() override;
private:
    static constexpr std::size_t circleCount = 1'000;
    PhysicsWorld world_{ {.gravity = Vec2f::zero(), .dt = fixedDeltaTime, .maxBodies = circleCount} };
    std::vector<ColorId> colors_;
};
} // namespace neko
