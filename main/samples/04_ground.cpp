#include "04_ground.h"

#include "core/profile.h"

namespace neko
{

constexpr static Scalar circleRadius = Ratio(1, 2);
constexpr static Vec2f groundHalfSize = Vec2f{ Scalar{ 5 }, Scalar{ 1 } };
constexpr static Vec2f rectHalfSize = Vec2f{ Ratio(1, 2), Ratio(1, 2) };
constexpr static ColorId groundColor = sample_color::GREEN;
constexpr static ColorId circleColor = sample_color::BLUE;
constexpr static ColorId collisionCircleColor = sample_color::MAGENTA;
constexpr static ColorId rectColor = sample_color::WHITE;

void GroundSample::Begin(const SampleContext& context)
{
    world_.SetContactListener(this);
    world_.SetBSH(&quadTree_);
    const auto worldCenter = context.worldBounds.GetCenter();

    //Implement ground
    {
        groundBodyIndex_ = world_.AddBody();
        Body& body = world_.body(groundBodyIndex_);
        body.position = worldCenter + Vec2f{ Scalar{ 0 }, Scalar{ 3 } };
        body.type = BodyType::STATIC;
        body.inverseMass = Scalar{ 0 };

        const auto colliderIndex = world_.AddAabbCollider(groundBodyIndex_);
        auto& collider = world_.collider(colliderIndex);
        world_.aabb(collider.shapeIndex).halfSize = groundHalfSize;
        collider.isTrigger = false;
        collider.restitution = Scalar{ 0 };
    }

    //Implement ball
    {
        circleBodyIndex_ = world_.AddBody();
        Body& body = world_.body(circleBodyIndex_);
        body.position = worldCenter - Vec2f{ Scalar{ 2 }, Scalar{ 3 } };
        body.type = BodyType::DYNAMIC;
        body.inverseMass = Scalar{ 1 };

        const auto colliderIndex = world_.AddCircleCollider(circleBodyIndex_);
        auto& collider = world_.collider(colliderIndex);
        world_.circle(collider.shapeIndex).radius = circleRadius;
        collider.isTrigger = false;
        collider.restitution = Ratio(1, 2);
    }

    //Implement box
    {
        rectBodyIndex_ = world_.AddBody();
        Body& body = world_.body(rectBodyIndex_);
        body.position = worldCenter - Vec2f{ Scalar{ -2 }, Scalar{ 3 } };
        body.type = BodyType::DYNAMIC;
        body.inverseMass = Scalar{ 1 };

        const auto colliderIndex = world_.AddAabbCollider(rectBodyIndex_);
        auto& collider = world_.collider(colliderIndex);
        world_.aabb(collider.shapeIndex).halfSize = rectHalfSize;
        collider.isTrigger = false;
        collider.restitution = Scalar{ 0 };
    }
}

void GroundSample::Draw(DrawList& drawList) const
{
    drawList.circles.push_back({ world_.body(circleBodyIndex_).position, circleRadius,
        collisionCount > 0 ? collisionCircleColor : circleColor });
    drawList.boxes.push_back({ world_.body(groundBodyIndex_).position, groundHalfSize, groundColor });
    drawList.boxes.push_back({ world_.body(rectBodyIndex_).position, rectHalfSize, rectColor });
}

void GroundSample::End()
{
    world_.Clear();
    quadTree_.Clear();
    collisionCount = 0;
}

void GroundSample::OnTriggerEnter([[maybe_unused]] const ColliderPair& p)
{
}

void GroundSample::OnTriggerExit([[maybe_unused]] const ColliderPair& p)
{
}

void GroundSample::FixedUpdate()
{
    NEKO_PROFILE_FUNCTION();
    world_.Step();
}

void GroundSample::OnCollisionEnter([[maybe_unused]] const ColliderPair& p)
{
    collisionCount++;
}

void GroundSample::OnCollisionExit([[maybe_unused]] const ColliderPair& p)
{
    collisionCount--;
}
}
