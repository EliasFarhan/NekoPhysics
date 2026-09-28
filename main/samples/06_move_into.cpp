#include "06_move_into.h"

#include "core/profile.h"

namespace neko
{

constexpr static Vec2f groundHalfSize = Vec2f{ Scalar{ 5 }, Scalar{ 1 }};
constexpr static Vec2f rectHalfSize = Vec2f{ Ratio(1, 2), Ratio(1, 2) };
constexpr static ColorId groundColor = sample_color::GREEN;
constexpr static ColorId inAirColor = sample_color::BLUE;
constexpr static ColorId onGroundColor = sample_color::MAGENTA;
constexpr static auto maxForce = Scalar{ 10 };

#ifndef NEKO_SAMPLE_BOX2D
void MoveIntoSample::OnTriggerEnter([[maybe_unused]] const ColliderPair& p)
{
}

void MoveIntoSample::OnTriggerExit([[maybe_unused]] const ColliderPair& p)
{
}

void MoveIntoSample::OnCollisionEnter(const ColliderPair& p)
{
	ColliderPair staticPair{staticColliderIndex_, groundColliderIndex_};
	ColliderPair dynamicPair{dynamicColliderIndex_, groundColliderIndex_};

	if(p == staticPair)
	{
		staticBodyOnGround_ = true;
	}
	if(p == dynamicPair)
	{
		dynamicBodyOnGround_ = true;
	}
}

void MoveIntoSample::OnCollisionExit([[maybe_unused]] const ColliderPair& p)
{
}
#else
/// Called when two fixtures begin to touch.
void MoveIntoSample::BeginContact(b2Contact* contact)
{
	const auto userDataA = contact->GetFixtureA()->GetUserData().pointer;
	const auto userDataB = contact->GetFixtureB()->GetUserData().pointer;

	if((b2Body*)userDataA == b2groundBody_ || (b2Body*)userDataB == b2groundBody_)
	{
		if((b2Body*)userDataA == b2staticBody_ || (b2Body*)userDataB == b2staticBody_)
		{
			staticBodyOnGround_ = true;
		}

		if((b2Body*)userDataA == b2dynamicBody_ || (b2Body*)userDataB == b2dynamicBody_)
		{
			dynamicBodyOnGround_ = true;
		}
	}
}

/// Called when two fixtures cease to touch.
void MoveIntoSample::EndContact(b2Contact* contact)
{
}
#endif
void MoveIntoSample::Begin(const SampleContext& context)
{
	const auto worldCenter = context.worldBounds.GetCenter();
	const auto groundPosition = worldCenter + Vec2f{ Scalar{ 0 }, Scalar{ 3 } };
	const auto dynamicRectPosition = worldCenter - Vec2f{ Scalar{ -2 }, Scalar{ 3 } };
	const auto staticRectPosition = worldCenter - Vec2f{ Scalar{ 2 }, Scalar{ 3 } };
#ifndef NEKO_SAMPLE_BOX2D
	world_.SetContactListener(this);
	world_.SetBSH(&quadTree_);
#else
	b2World_.SetContactListener(this);
#endif
	//Implement ground
	{
#ifndef NEKO_SAMPLE_BOX2D
		groundBodyIndex_ = world_.AddBody();
		Body& body = world_.body(groundBodyIndex_);
		body.position = groundPosition;
		body.velocity = Vec2f{};
		body.type = BodyType::STATIC;
		body.inverseMass = Scalar{ 0 };

		groundColliderIndex_ = world_.AddAabbCollider(groundBodyIndex_);
		auto& collider = world_.collider(groundColliderIndex_);
		auto& aabbCollider = world_.aabb(collider.shapeIndex);
		aabbCollider.halfSize = groundHalfSize;
		collider.isTrigger = false;
		collider.restitution = Scalar{ 0 };
#else
		b2BodyDef bodyDef{};
		bodyDef.position = b2Vec2{ float{groundPosition.x}, float{groundPosition.y} };
		bodyDef.type = b2_staticBody;
		bodyDef.fixedRotation = true;
		b2groundBody_ = b2World_.CreateBody(&bodyDef);

		b2PolygonShape shape;
		shape.SetAsBox(float{groundHalfSize.x}, float{groundHalfSize.y});
		b2FixtureDef fixtureDef{};
		fixtureDef.shape = &shape;
		fixtureDef.friction = 0.0f;
		fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(b2groundBody_);
		b2groundBody_->CreateFixture(&fixtureDef);
#endif
	}
	{
#ifndef NEKO_SAMPLE_BOX2D
		//rect ball
		staticBodyIndex = world_.AddBody();
		Body& body = world_.body(staticBodyIndex);
		body.position = staticRectPosition;
		body.velocity = Vec2f{};
		body.type = BodyType::DYNAMIC;
		body.inverseMass = Scalar{ 1 };

		staticColliderIndex_ = world_.AddAabbCollider(staticBodyIndex);
		auto& collider = world_.collider(staticColliderIndex_);
		auto& aabbCollider = world_.aabb(collider.shapeIndex);
		aabbCollider.halfSize = rectHalfSize;
		collider.isTrigger = false;
		collider.restitution = Scalar{ 0 };
#else
		b2BodyDef bodyDef{};
		bodyDef.position = { float{staticRectPosition.x}, float{staticRectPosition.y} };
		bodyDef.type = b2_dynamicBody;
		bodyDef.fixedRotation = true;
		b2staticBody_ = b2World_.CreateBody(&bodyDef);

		b2PolygonShape shape{};
		shape.SetAsBox(float{rectHalfSize.x}, float{rectHalfSize.y});

		b2FixtureDef fixtureDef{};
		fixtureDef.shape = &shape;
		fixtureDef.friction = 0.0f;
		fixtureDef.density = 1.0f;
		fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(b2staticBody_);
		b2staticBody_->CreateFixture(&fixtureDef);
#endif
	}

	{
#ifndef NEKO_SAMPLE_BOX2D
		//rect ball
		dynamicBodyIndex = world_.AddBody();
		Body& body = world_.body(dynamicBodyIndex);
		body.position = dynamicRectPosition;
		body.velocity = Vec2f{};
		body.type = BodyType::DYNAMIC;
		body.inverseMass = Scalar{ 1 };

		dynamicColliderIndex_ = world_.AddAabbCollider(dynamicBodyIndex);
		auto& collider = world_.collider(dynamicColliderIndex_);
		auto& aabbCollider = world_.aabb(collider.shapeIndex);
		aabbCollider.halfSize = rectHalfSize;
		collider.isTrigger = false;
		collider.restitution = Scalar{ 0 };
#else
		b2BodyDef bodyDef{};
		bodyDef.position = { float{dynamicRectPosition.x}, float{dynamicRectPosition.y} };
		bodyDef.type = b2_dynamicBody;
		bodyDef.fixedRotation = true;
		b2dynamicBody_ = b2World_.CreateBody(&bodyDef);

		b2PolygonShape shape{};
		shape.SetAsBox(float{rectHalfSize.x}, float{rectHalfSize.y});

		b2FixtureDef fixtureDef{};
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.0f;
		fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(b2dynamicBody_);
		b2dynamicBody_->CreateFixture(&fixtureDef);
#endif
	}
}

void MoveIntoSample::Draw(DrawList& drawList) const
{
#ifdef NEKO_SAMPLE_BOX2D
	const auto position = [](const b2Body* body)
	{
		const auto pos = body->GetPosition();
		return Vec2f{ Scalar{ pos.x }, Scalar{ pos.y } };
	};
	const Vec2f groundCenter = position(b2groundBody_);
	const Vec2f staticPosition = position(b2staticBody_);
	const Vec2f dynamicPosition = position(b2dynamicBody_);
#else
	const Vec2f groundCenter = world_.body(groundBodyIndex_).position;
	const Vec2f staticPosition = world_.body(staticBodyIndex).position;
	const Vec2f dynamicPosition = world_.body(dynamicBodyIndex).position;
#endif
	drawList.boxes.push_back({ groundCenter, groundHalfSize, groundColor });
	drawList.boxes.push_back({ staticPosition, rectHalfSize, staticBodyOnGround_ ? onGroundColor : inAirColor });
	drawList.boxes.push_back({ dynamicPosition, rectHalfSize, dynamicBodyOnGround_ ? onGroundColor : inAirColor });
}

void MoveIntoSample::End()
{
#ifdef NEKO_SAMPLE_BOX2D
	b2World_ = b2World{{0.0f, -9.81f}};
#else
	world_.Clear();
	quadTree_.Clear();
#endif
	staticBodyOnGround_ = false;
	dynamicBodyOnGround_ = false;
}

void MoveIntoSample::FixedUpdate()
{
	NEKO_PROFILE_FUNCTION();

	if (dynamicBodyOnGround_)
	{
#ifdef NEKO_SAMPLE_BOX2D
		const Scalar mass = (Scalar)b2dynamicBody_->GetMass();
		const Vec2f velocity {b2dynamicBody_->GetLinearVelocity()};
		constexpr neko::Vec2f wantedVelocity{ Scalar{ -1 }, Scalar{}};
		auto force = (wantedVelocity - velocity) / fixedDeltaTime * mass ;
		if (force.SquareLength() > maxForce * maxForce)
		{
			force = force.Normalized() * maxForce;
		}
		b2dynamicBody_->ApplyForceToCenter({float{force.x}, float{force.y}}, true);
#else
		Body& dynamicBody = world_.body(dynamicBodyIndex);
		//Move dynamic object to the right
		constexpr neko::Vec2f wantedVelocity{ Scalar{ -1 }, Scalar{}};
		auto force = (wantedVelocity - dynamicBody.velocity) / fixedDeltaTime / dynamicBody.inverseMass;
		if (force.SquareLength() > maxForce * maxForce)
		{
			force = force.Normalized() * maxForce;
		}
		dynamicBody.force += force;
#endif
	}
	if (staticBodyOnGround_)
	{
#ifdef NEKO_SAMPLE_BOX2D
		const Scalar mass = (Scalar)b2staticBody_->GetMass();
		const Vec2f velocity {b2staticBody_->GetLinearVelocity()};
		constexpr neko::Vec2f wantedVelocity{};
		auto force = (wantedVelocity - velocity) / fixedDeltaTime * mass ;
		if (force.SquareLength() > maxForce * maxForce)
		{
			force = force.Normalized() * maxForce;
		}
		b2staticBody_->ApplyForceToCenter({float{force.x}, float{force.y}}, true);
#else
		//Use force to make object static
		Body& staticBody = world_.body(staticBodyIndex);
		//Move dynamic object to the right
		constexpr neko::Vec2f wantedVelocity{};
		auto force = (wantedVelocity - staticBody.velocity) / fixedDeltaTime / staticBody.inverseMass;
		if (force.SquareLength() > maxForce * maxForce)
		{
			force = force.Normalized() * maxForce;
		}
		staticBody.force += force;
#endif
	}
#ifdef NEKO_SAMPLE_BOX2D
	b2World_.Step(float{fixedDeltaTime}, 10, 3);
#else
	world_.Step();
#endif

}
}
