#include "05_collisions.h"
#include "random_utils.h"

namespace neko
{

constexpr static std::size_t circleCount = 50;
constexpr static Scalar maxSpeed = Scalar{ 4 };
constexpr static Scalar maxCircleRadius = Scalar{ 0.2f };
constexpr static Scalar minCircleRadius = Scalar{ 0.1f };
constexpr static ColorId collidingColor = sample_color::GREEN;
constexpr static ColorId notCollidingColor = sample_color::RED;


void CollisionsSample::Begin([[maybe_unused]] const SampleContext& context)
{
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
        auto& triggerCollider = world_.collider({ triggerBody.index });
        triggerCollider.isTrigger = false;
        auto& circleCollider = world_.circle(triggerCollider.shapeIndex);
        circleCollider.radius = triggerBody.circleRadius;

    }
}

void CollisionsSample::Draw(DrawList& drawList) const
{
    for (std::size_t i = 0; i < bodies_.size(); i++)
    {
        const auto& body = world_.body({ static_cast<Index>(i) });
        const auto& collidingBody = bodies_[i];
        drawList.circles.push_back({ body.position, collidingBody.circleRadius,
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

void CollisionsSample::OnCollisionEnter(const ColliderPair &p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count++;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count++;
}

void CollisionsSample::OnCollisionExit(const ColliderPair &p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count--;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count--;
}

void CollisionsSample::OnTriggerEnter([[maybe_unused]]const ColliderPair &p)
{

}

void CollisionsSample::OnTriggerExit([[maybe_unused]] const ColliderPair &p)
{

}
}