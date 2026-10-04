#include "main/samples/03_triggers.h"
#include "main/samples/sample_rng.h"

#include "neko/core/profile.h"

#include <array>

namespace neko
{

constexpr static Scalar kMaxSpeed = Scalar{ 4 };
constexpr static Scalar kMaxCircleRadius = Ratio(1, 50);
constexpr static Scalar kMinCircleRadius = Ratio(1, 100);
constexpr static ColorId kTriggerColor = sample_color::kGreen;
constexpr static ColorId kUntriggerColor = sample_color::kRed;
constexpr static ColorId kQuadTreeColor = sample_color::kGrey;
constexpr static std::uint32_t kSeed = 0x03C0FFEEu;

void TriggersSample::Begin(const SampleContext& context)
{
    NEKO_PROFILE_FUNCTION();
    bounds_ = context.world_bounds;
    world_.SetContactListener(this);
    world_.SetBSH(&quad_tree_);
    SampleRng rng{ kSeed };
    bodies_.resize(kCircleCount);
    for (Index i = 0; i < kCircleCount; ++i)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { rng.Range(bounds_.min_bound.x, bounds_.max_bound.x),
            rng.Range(bounds_.min_bound.y, bounds_.max_bound.y) };
        body.velocity = Vec2f{ rng.Range(Scalar{ -1 }, Scalar{ 1 }), rng.Range(Scalar{ -1 }, Scalar{ 1 }) } * kMaxSpeed;
        body.inverse_mass = Scalar{ 1 };

        auto& trigger_body = bodies_[i];
        trigger_body.circle_radius = rng.Range(kMinCircleRadius, kMaxCircleRadius);
        const auto collider_index = world_.AddCircleCollider(index);
        world_.circle(world_.collider(collider_index).shape_index).radius = trigger_body.circle_radius;
    }
}

void TriggersSample::Draw(DrawList& draw_list) const
{
    NEKO_PROFILE_FUNCTION();
    for (Index i = 0; i < bodies_.size(); ++i)
    {
        const auto& trigger_body = bodies_[i];
        draw_list.circles.push_back({ world_.body({ i }).position, trigger_body.circle_radius,
            trigger_body.count == 0 ? kUntriggerColor : kTriggerColor });
    }
    // Children are exact quadrants of their parent: the root's outline plus a cross through every split node draws
    // each edge once (the children's outlines would redraw their parent's edges)
    const auto& root = quad_tree_.GetNode(QuadTree::kRoot).aabb;
    const Vec2f top_right{ root.max_bound.x, root.min_bound.y };
    const Vec2f bottom_left{ root.min_bound.x, root.max_bound.y };
    draw_list.lines.push_back({ root.min_bound, top_right, kQuadTreeColor });
    draw_list.lines.push_back({ top_right, root.max_bound, kQuadTreeColor });
    draw_list.lines.push_back({ root.max_bound, bottom_left, kQuadTreeColor });
    draw_list.lines.push_back({ bottom_left, root.min_bound, kQuadTreeColor });
    // Depth-first walk with an explicit stack: each pop pushes at most 4 children, one level deeper
    std::array<Index, 3 * QuadTree::kMaxDepth + 1> stack{};
    std::size_t stack_size = 0;
    stack[stack_size++] = QuadTree::kRoot;
    while (stack_size > 0)
    {
        const auto& node = quad_tree_.GetNode(stack[--stack_size]);
        if (node.IsLeaf())
        {
            continue;
        }
        const auto& aabb = node.aabb;
        const auto center = aabb.GetCenter();
        draw_list.lines.push_back({ Vec2f{ aabb.min_bound.x, center.y }, Vec2f{ aabb.max_bound.x, center.y }, kQuadTreeColor });
        draw_list.lines.push_back({ Vec2f{ center.x, aabb.min_bound.y }, Vec2f{ center.x, aabb.max_bound.y }, kQuadTreeColor });
        for (Index child = node.first_child; child < node.first_child + 4; ++child)
        {
            stack[stack_size++] = child;
        }
    }
}

void TriggersSample::End()
{
    world_.Clear();
    bodies_.clear();
    quad_tree_.Clear();
}

void TriggersSample::OnTriggerEnter(const ColliderPair& p)
{
    bodies_[world_.collider(p.c1).body_index.index].count++;
    bodies_[world_.collider(p.c2).body_index.index].count++;
}

void TriggersSample::OnTriggerExit(const ColliderPair& p)
{
    bodies_[world_.collider(p.c1).body_index.index].count--;
    bodies_[world_.collider(p.c2).body_index.index].count--;
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
