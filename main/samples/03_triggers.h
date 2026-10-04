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
    void Draw(DrawList& draw_list) const override;
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
        Scalar circle_radius{ 0 };
    };
    static constexpr Index kCircleCount = sample_config::kTriggerCount;
    static constexpr std::size_t kMaxPairs = std::size_t{ kCircleCount } * 8;
    PhysicsWorld world_{ {.dt = kFixedDeltaTime, .max_bodies = kCircleCount, .max_colliders = kCircleCount,
        .max_touching_pairs = kMaxPairs, .max_broadphase_pairs = 1} };
    QuadTree quad_tree_{ kCircleCount, kMaxPairs };
    std::vector<TriggeredBody> bodies_;
    Aabbf bounds_{};
};
}

#endif  // MAIN_SAMPLES_03_TRIGGERS_H_
