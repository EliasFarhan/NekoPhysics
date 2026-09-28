#include "03_triggers.h"

#include "core/profile.h"

#include <functional>

#include "random_utils.h"

namespace neko
{


constexpr static Scalar maxSpeed = Scalar{ 4 };
constexpr static Scalar maxCircleRadius = Scalar{ 0.02f };
constexpr static Scalar minCircleRadius = Scalar{ 0.01f };
constexpr static ColorId triggerColor = sample_color::GREEN;
constexpr static ColorId untriggerColor = sample_color::RED;
constexpr static ColorId quadTreeColor = sample_color::WHITE;

void TriggersSample::Begin([[maybe_unused]] const SampleContext& context)
{
    NEKO_PROFILE_FUNCTION();
    world_.SetContactListener(this);
    world_.SetBSH(&quadTree_);
    bodies_.resize(circleCount);
    for (std::size_t i = 0; i < circleCount; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { Scalar{RandomRange(0.0f, 12.8f)}, Scalar{RandomRange(0.0f, 7.2f)} };
        body.velocity = Vec2f{ Scalar{RandomRange(-1.0f, 1.0f)}, Scalar{RandomRange(-1.0f, 1.0f)} } *maxSpeed;
        body.inverseMass = Scalar{ 1 };

        auto& triggerBody = bodies_[i];

        const auto circleRadius = Scalar{ RandomRange(float{minCircleRadius}, float{maxCircleRadius}) };
        triggerBody.circleRadius = circleRadius;
        triggerBody.index = world_.AddCircleCollider(index);
        const auto& triggerCollider = world_.collider({ triggerBody.index });
        auto& circleCollider = world_.circle(triggerCollider.shapeIndex);
        circleCollider.radius = triggerBody.circleRadius;
    }
}

void TriggersSample::Draw(DrawList& drawList) const
{
    NEKO_PROFILE_FUNCTION();
    for (std::size_t i = 0; i < bodies_.size(); i++)
    {
        const auto& body = world_.body({ static_cast<Index>(i) });
        const auto& triggerBody = bodies_[i];
        drawList.circles.push_back({ body.position, triggerBody.circleRadius,
            triggerBody.count == 0 ? untriggerColor : triggerColor });
    }
    std::function<void(Index)> drawQuad = [this, &drawList, &drawQuad](Index nodeIndex)
    {
        const auto& node = quadTree_.GetNode(nodeIndex);
        const auto& aabb = node.aabb;
        const std::array<Vec2f, 4> corners
        {
            aabb.minBound,
            Vec2f{ aabb.maxBound.x, aabb.minBound.y },
            aabb.maxBound,
            Vec2f{ aabb.minBound.x, aabb.maxBound.y },
        };
        for(std::size_t i = 0; i < 4; i++)
        {
            drawList.lines.push_back({ corners[i], corners[(i + 1) % 4], quadTreeColor });
        }
        if(!node.IsLeaf())
        {
            for(Index child = node.firstChild; child < node.firstChild + 4; ++child)
            {
                drawQuad(child);
            }
        }
    };
    drawQuad(QuadTree::ROOT);
}

void TriggersSample::End()
{
    world_.Clear();
    bodies_.clear();
    quadTree_.Clear();
}

void TriggersSample::OnTriggerEnter(const ColliderPair& p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count++;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count++;
}

void TriggersSample::OnTriggerExit(const ColliderPair& p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count--;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count--;
}

void TriggersSample::FixedUpdate()
{
    NEKO_PROFILE_FUNCTION();
    world_.Step();
    for (std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({ static_cast<Index>(i) });
        if ((body.velocity.x > Scalar{0.0f}&& body.position.x > Scalar{12.8f}) || (body.velocity.x < Scalar{0.0f}&& body.position.x < Scalar{0.0f}))
        {
            body.velocity.x = -body.velocity.x;
        }

        if ((body.velocity.y > Scalar{0.0f} && body.position.y > Scalar{7.2f}) || (body.velocity.y < Scalar{0.0f}&& body.position.y < Scalar{0.0f}))
        {
            body.velocity.y = -body.velocity.y;
        }
    }
}

void TriggersSample::OnCollisionEnter([[maybe_unused]] const ColliderPair& p)
{
}

void TriggersSample::OnCollisionExit([[maybe_unused]] const ColliderPair& p)
{
}
}
