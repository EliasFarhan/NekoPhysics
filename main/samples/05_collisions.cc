#include "main/samples/05_collisions.h"
#include "main/samples/sample_rng.h"

namespace neko
{

constexpr static Scalar maxSpeed = Scalar{ 4 };
constexpr static Scalar maxCircleRadius = Ratio(1, 5);
constexpr static Scalar minCircleRadius = Ratio(1, 10);
constexpr static ColorId collidingColor = sample_color::GREEN;
constexpr static ColorId notCollidingColor = sample_color::RED;
constexpr static std::uint32_t seed = 0x05C0FFEEu;

void CollisionsSample::Begin(const SampleContext& context)
{
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

        auto& collidingBody = bodies_[i];
        collidingBody.circleRadius = rng.Range(minCircleRadius, maxCircleRadius);
        const auto colliderIndex = world_.AddCircleCollider(index);
        auto& collider = world_.collider(colliderIndex);
        collider.isTrigger = false;
        world_.circle(collider.shapeIndex).radius = collidingBody.circleRadius;
    }
}

void CollisionsSample::Draw(DrawList& drawList) const
{
    for (Index i = 0; i < bodies_.size(); ++i)
    {
        const auto& collidingBody = bodies_[i];
        drawList.circles.push_back({ world_.body({ i }).position, collidingBody.circleRadius,
            collidingBody.count == 0 ? notCollidingColor : collidingColor });
    }
}

void CollisionsSample::End()
{
    world_.Clear();
    bodies_.clear();
    quadTree_.Clear();
}

void CollisionsSample::FixedUpdate()
{
    world_.Step();
    for (Index i = 0; i < bodies_.size(); ++i)
    {
        BounceInside(world_.body({ i }), bounds_);
    }
}

void CollisionsSample::OnCollisionEnter(const ColliderPair& p)
{
    bodies_[world_.collider(p.c1).bodyIndex.index].count++;
    bodies_[world_.collider(p.c2).bodyIndex.index].count++;
}

void CollisionsSample::OnCollisionExit(const ColliderPair& p)
{
    bodies_[world_.collider(p.c1).bodyIndex.index].count--;
    bodies_[world_.collider(p.c2).bodyIndex.index].count--;
}

void CollisionsSample::OnTriggerEnter([[maybe_unused]] const ColliderPair& p)
{
}

void CollisionsSample::OnTriggerExit([[maybe_unused]] const ColliderPair& p)
{
}
}
