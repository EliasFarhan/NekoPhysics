#include "03_triggers.h"
#include "sample_rng.h"

#include "core/profile.h"

#include <array>

namespace neko
{

constexpr static Scalar maxSpeed = Scalar{ 4 };
constexpr static Scalar maxCircleRadius = Ratio(1, 50);
constexpr static Scalar minCircleRadius = Ratio(1, 100);
constexpr static ColorId triggerColor = sample_color::GREEN;
constexpr static ColorId untriggerColor = sample_color::RED;
constexpr static ColorId quadTreeColor = sample_color::GREY;
constexpr static std::uint32_t seed = 0x03C0FFEEu;

void TriggersSample::Begin(const SampleContext& context)
{
    NEKO_PROFILE_FUNCTION();
    bounds_ = context.worldBounds;
    world_.SetContactListener(this);
    world_.SetBSH(&quadTree_);
    SampleRng rng{ seed };
    bodies_.resize(circleCount);
    for (Index i = 0; i < circleCount; ++i)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { rng.Range(bounds_.minBound.x, bounds_.maxBound.x),
            rng.Range(bounds_.minBound.y, bounds_.maxBound.y) };
        body.velocity = Vec2f{ rng.Range(Scalar{ -1 }, Scalar{ 1 }), rng.Range(Scalar{ -1 }, Scalar{ 1 }) } * maxSpeed;
        body.inverseMass = Scalar{ 1 };

        auto& triggerBody = bodies_[i];
        triggerBody.circleRadius = rng.Range(minCircleRadius, maxCircleRadius);
        const auto colliderIndex = world_.AddCircleCollider(index);
        world_.circle(world_.collider(colliderIndex).shapeIndex).radius = triggerBody.circleRadius;
    }
}

void TriggersSample::Draw(DrawList& drawList) const
{
    NEKO_PROFILE_FUNCTION();
    for (Index i = 0; i < bodies_.size(); ++i)
    {
        const auto& triggerBody = bodies_[i];
        drawList.circles.push_back({ world_.body({ i }).position, triggerBody.circleRadius,
            triggerBody.count == 0 ? untriggerColor : triggerColor });
    }
    // Depth-first walk with an explicit stack: each pop pushes at most 4 children, one level deeper
    std::array<Index, 3 * QuadTree::MAX_DEPTH + 1> stack{};
    std::size_t stackSize = 0;
    stack[stackSize++] = QuadTree::ROOT;
    while (stackSize > 0)
    {
        const auto& node = quadTree_.GetNode(stack[--stackSize]);
        const auto& aabb = node.aabb;
        const std::array<Vec2f, 4> corners
        {
            aabb.minBound,
            Vec2f{ aabb.maxBound.x, aabb.minBound.y },
            aabb.maxBound,
            Vec2f{ aabb.minBound.x, aabb.maxBound.y },
        };
        for (std::size_t i = 0; i < corners.size(); i++)
        {
            drawList.lines.push_back({ corners[i], corners[(i + 1) % corners.size()], quadTreeColor });
        }
        if (!node.IsLeaf())
        {
            for (Index child = node.firstChild; child < node.firstChild + 4; ++child)
            {
                stack[stackSize++] = child;
            }
        }
    }
}

void TriggersSample::End()
{
    world_.Clear();
    bodies_.clear();
    quadTree_.Clear();
}

void TriggersSample::OnTriggerEnter(const ColliderPair& p)
{
    bodies_[world_.collider(p.c1).bodyIndex.index].count++;
    bodies_[world_.collider(p.c2).bodyIndex.index].count++;
}

void TriggersSample::OnTriggerExit(const ColliderPair& p)
{
    bodies_[world_.collider(p.c1).bodyIndex.index].count--;
    bodies_[world_.collider(p.c2).bodyIndex.index].count--;
}

void TriggersSample::FixedUpdate()
{
    NEKO_PROFILE_FUNCTION();
    world_.Step();
    for (Index i = 0; i < bodies_.size(); ++i)
    {
        BounceInside(world_.body({ i }), bounds_);
    }
}

void TriggersSample::OnCollisionEnter([[maybe_unused]] const ColliderPair& p)
{
}

void TriggersSample::OnCollisionExit([[maybe_unused]] const ColliderPair& p)
{
}
}
