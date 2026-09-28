#include "04_ground.h"

#include "core/profile.h"

namespace neko
{


constexpr static Scalar circleRadius = Scalar{ 0.5f };
constexpr static Vec2f groundHalfSize = Vec2f{ Scalar{5}, Scalar{ 1 } };
constexpr static Vec2f rectHalfSize = Vec2f{ Scalar{0.5f}, Scalar{ 0.5f } };
constexpr static ColorId groundColor = sample_color::GREEN;
constexpr static ColorId circleColor = sample_color::BLUE;
constexpr static ColorId collisionCircleColor = sample_color::MAGENTA;
constexpr static ColorId rectColor = sample_color::WHITE;
constexpr static Vec2f worldCenter = { Scalar{12.8f / 2.0f}, Scalar{7.2f / 2.0f} };
 static Vec2f groundPosition = worldCenter+Vec2f{Scalar{0}, Scalar{3}};
 static Vec2f circlePosition = worldCenter-Vec2f{Scalar{2}, Scalar{3}};
 static Vec2f rectPosition = worldCenter-Vec2f{Scalar{-2}, Scalar{3}};

void GroundSample::Begin([[maybe_unused]] const SampleContext& context)
{
    world_.SetContactListener(this);
    world_.SetBSH(&quadTree_);

    //Implement ground
    {
        groundBodyIndex_ = world_.AddBody();
        Body& body = world_.body(groundBodyIndex_);
        body.position = groundPosition;
        body.velocity = Vec2f{};
        body.type = BodyType::STATIC;
        body.inverseMass = Scalar{ 0 };

        const auto colliderIndex = world_.AddAabbCollider(groundBodyIndex_);
        auto& collider = world_.collider(colliderIndex);
        auto& aabbCollider = world_.aabb(collider.shapeIndex);
        aabbCollider.halfSize = groundHalfSize;
        collider.isTrigger = false;
		collider.restitution = neko::Scalar {0};


    }
    
    //Implement ball
    {
        circleBodyIndex_ = world_.AddBody();
        Body& body = world_.body(circleBodyIndex_);
        body.position = circlePosition;
        body.velocity = Vec2f{};
        body.type = BodyType::DYNAMIC;
        body.inverseMass = Scalar{ 1 };

        const auto colliderIndex = world_.AddCircleCollider(circleBodyIndex_);
        auto& collider = world_.collider(colliderIndex);
        auto& circleCollider = world_.circle(collider.shapeIndex);
        circleCollider.radius = circleRadius;
        collider.isTrigger = false;

		collider.restitution = neko::Scalar {0.5f};

    }

	{
		//rect ball
		rectBodyIndex_ = world_.AddBody();
		Body& body = world_.body(rectBodyIndex_);
		body.position = rectPosition;
		body.velocity = Vec2f{};
		body.type = BodyType::DYNAMIC;
		body.inverseMass = Scalar{ 1 };

		const auto colliderIndex = world_.AddAabbCollider(rectBodyIndex_);
		auto& collider = world_.collider(colliderIndex);
		auto& aabbCollider = world_.aabb(collider.shapeIndex);
		aabbCollider.halfSize = rectHalfSize;
		collider.isTrigger = false;
		collider.restitution = neko::Scalar {0};
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

void GroundSample::OnTriggerEnter([[maybe_unused]]const ColliderPair& p)
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
