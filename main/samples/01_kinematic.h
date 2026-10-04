#ifndef MAIN_SAMPLES_01_KINEMATIC_H_
#define MAIN_SAMPLES_01_KINEMATIC_H_

#include "main/samples/sample.h"
#include "main/samples/sample_config.h"
#include "neko/physics/physics.h"

#include <vector>

namespace neko
{
/**
 * @brief Circles without colliders moving at constant speed, bouncing on the world bounds
 */
class KinematicSample : public Sample
{
public:
    void Begin(const SampleContext& context) override;
    void FixedUpdate() override;
    void Draw(DrawList& drawList) const override;
    void End() override;
    [[nodiscard]] std::uint32_t GenerateChecksum() const override { return world_.GenerateChecksum(); }

private:
    static constexpr Index circleCount = sample_config::KINEMATIC_COUNT;
    PhysicsWorld world_{ {.gravity = Vec2f::zero(), .dt = fixedDeltaTime, .maxBodies = circleCount,
        .maxColliders = 1, .maxTouchingPairs = 1, .maxBroadphasePairs = 1} };
    std::vector<ColorId> colors_;
    Aabbf bounds_{};
};
}

#endif  // MAIN_SAMPLES_01_KINEMATIC_H_
