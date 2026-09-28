#pragma once
#include "physics/physics.h"
#include "sample.h"
#include "sample_config.h"

#include <vector>

namespace neko
{

/**
 * @brief Circles without colliders orbiting a black hole at the center of the world
 */
class PlanetSample : public Sample
{
public:
    void Begin(const SampleContext& context) override;
    void FixedUpdate() override;
    void Draw(DrawList& drawList) const override;
    void End() override;
    [[nodiscard]] std::uint32_t GenerateChecksum() const override { return world_.GenerateChecksum(); }
private:
    static constexpr Index circleCount = sample_config::PLANET_COUNT;
    PhysicsWorld world_{ {.gravity = Vec2f::zero(), .dt = fixedDeltaTime, .maxBodies = circleCount,
        .maxColliders = 1, .maxTouchingPairs = 1, .maxBroadphasePairs = 1} };
    std::vector<ColorId> colors_;
    Vec2f center_{};
};
} // namespace neko
