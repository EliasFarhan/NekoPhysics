#ifndef MAIN_SAMPLES_03_TRIGGERS_H_
#define MAIN_SAMPLES_03_TRIGGERS_H_
#include "neko/physics/physics.h"
#include "neko/bsh/quadtree.h"
#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

#include <vector>

namespace neko
{
/**
 * @brief Tiny trigger circles in a quadtree broadphase: red alone, green while overlapping another one. Draws the
 * quadtree nodes.
 */
class TriggersSample : public Sample, public ContactListener
{
public:
    void Begin(const SampleContext& context) override;
    void Draw(DrawList& drawList) const override;
    void End() override;
    [[nodiscard]] std::uint32_t GenerateChecksum() const override { return world_.GenerateChecksum(); }
    void OnTriggerEnter(const ColliderPair& p) override;
    void OnTriggerExit(const ColliderPair& p) override;
    void FixedUpdate() override;
    void OnCollisionEnter(const ColliderPair& p) override;
    void OnCollisionExit(const ColliderPair& p) override;

private:
    struct TriggeredBody
    {
        int count = 0;
        Scalar circleRadius{ 0 };
    };
    static constexpr Index circleCount = sample_config::TRIGGER_COUNT;
    static constexpr std::size_t maxPairs = std::size_t{ circleCount } * 8;
    PhysicsWorld world_{ {.dt = fixedDeltaTime, .maxBodies = circleCount, .maxColliders = circleCount,
        .maxTouchingPairs = maxPairs, .maxBroadphasePairs = 1} };
    QuadTree quadTree_{ circleCount, maxPairs };
    std::vector<TriggeredBody> bodies_;
    Aabbf bounds_{};
};
}

#endif  // MAIN_SAMPLES_03_TRIGGERS_H_
