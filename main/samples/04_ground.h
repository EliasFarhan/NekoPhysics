#pragma once
#include "physics/physics.h"
#include "bsh/quadtree.h"
#include "sample.h"
#include "sample_config.h"

namespace neko
{
/**
 * @brief A ball and a box falling on a static ground. The ball turns magenta while it touches something.
 */
class GroundSample : public Sample, public ContactListener
{
public:
    void Begin(const SampleContext& context) override;
    void Draw(DrawList& drawList) const override;
    void End() override;
    void OnTriggerEnter(const ColliderPair& p) override;
    void OnTriggerExit(const ColliderPair& p) override;
    void FixedUpdate() override;
    void OnCollisionEnter(const ColliderPair& p) override;
    void OnCollisionExit(const ColliderPair& p) override;
private:
    static constexpr std::size_t bodyCount = 3;
    static constexpr std::size_t maxPairs = 8;
    PhysicsWorld world_{ {.gravity = Vec2f{Scalar{0}, Ratio(981, 100)}, .dt = fixedDeltaTime, .maxBodies = bodyCount,
        .maxColliders = bodyCount, .maxTouchingPairs = maxPairs, .maxBroadphasePairs = 1} };
    QuadTree quadTree_{ bodyCount, maxPairs };
    BodyIndex circleBodyIndex_ = INVALID_BODY_INDEX;
    BodyIndex rectBodyIndex_ = INVALID_BODY_INDEX;
    BodyIndex groundBodyIndex_ = INVALID_BODY_INDEX;
    int collisionCount = 0;
};
}
