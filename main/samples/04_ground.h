#ifndef MAIN_SAMPLES_04_GROUND_H_
#define MAIN_SAMPLES_04_GROUND_H_
#include "neko/physics/physics.h"
#include "neko/bsh/quadtree.h"
#include "main/samples/sample.h"
#include "main/samples/sample_config.h"

namespace neko
{
// A ball and a box falling on a static ground. The ball turns magenta while it touches something.
class GroundSample : public Sample, public ContactListener
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
    static constexpr std::size_t kBodyCount = 3;
    static constexpr std::size_t kMaxPairs = 8;
    PhysicsWorld world_{ {.gravity = Vec2f{Scalar{0}, Ratio(981, 100)}, .dt = kFixedDeltaTime, .max_bodies = kBodyCount,
        .max_colliders = kBodyCount, .max_touching_pairs = kMaxPairs, .max_broadphase_pairs = 1} };
    QuadTree quad_tree_{ kBodyCount, kMaxPairs };
    BodyIndex circle_body_index_ = kInvalidBodyIndex;
    BodyIndex rect_body_index_ = kInvalidBodyIndex;
    BodyIndex ground_body_index_ = kInvalidBodyIndex;
    int collision_count_ = 0;
};
}  // namespace neko

#endif  // MAIN_SAMPLES_04_GROUND_H_
