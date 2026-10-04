#ifndef MAIN_SAMPLES_02_PLANETS_H_
#define MAIN_SAMPLES_02_PLANETS_H_
#include "neko/physics/physics.h"
#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

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
    void Draw(DrawList& draw_list) const override;
    void End() override;
    [[nodiscard]] std::uint32_t GenerateChecksum() const override { return world_.GenerateChecksum(); }
private:
    static constexpr Index kCircleCount = sample_config::kPlanetCount;
    PhysicsWorld world_{ {.gravity = Vec2f::Zero(), .dt = kFixedDeltaTime, .max_bodies = kCircleCount,
        .max_colliders = 1, .max_touching_pairs = 1, .max_broadphase_pairs = 1} };
    std::vector<ColorId> colors_;
    Vec2f center_{};
};
} // namespace neko

#endif  // MAIN_SAMPLES_02_PLANETS_H_
