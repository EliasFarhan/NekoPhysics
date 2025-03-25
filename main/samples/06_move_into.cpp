//
// Created by unite on 17.10.2024.
//

#include "06_move_into.h"

#include <SDL3/SDL_log.h>
#include <fmt/format.h>
#include <box2d/b2_contact.h>
#include <box2d/b2_polygon_shape.h>
#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

namespace neko
{

constexpr static Vec2f groundHalfSize = Vec2f{ Scalar{ 5 }, Scalar{ 1 }};
constexpr static Vec2f rectHalfSize = Vec2f{ Scalar{ 0.5f }, Scalar{ 0.5f }};
constexpr static std::size_t quadResolution = 2;
constexpr static std::size_t quadVertexCount = 4;
constexpr static Scalar pixelPerMeter = Scalar{ 100.0f };

constexpr static SDL_FColor groundColor{ 0, 1, 0, 1 };
constexpr static SDL_FColor inAirColor{ 0, 0, 1, 1 };
constexpr static SDL_FColor onGroundColor{ 1, 0, 1, 1 };
constexpr static Vec2f worldCenter = { Scalar{ 12.8f / 2.0f }, Scalar{ 7.2f / 2.0f }};
static Vec2f groundPosition = worldCenter + Vec2f{ Scalar{ 0 }, Scalar{ 3 }};
static Vec2f dynamicRectPosition = worldCenter - Vec2f{ Scalar{ -2 }, Scalar{ 3 }};
static Vec2f staticRectPosition = worldCenter - Vec2f{ Scalar{ 2 }, Scalar{ 3 }};
constexpr static auto maxForce = neko::Scalar{ 10.0f };

#ifndef USE_BOX2D
void MoveIntoSample::OnTriggerEnter(const ColliderPair& p)
{
}

void MoveIntoSample::OnTriggerExit(const ColliderPair& p)
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

void MoveIntoSample::OnCollisionExit(const ColliderPair& p)
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
void MoveIntoSample::Begin()
{

	indices_.reserve(quadResolution * 3);
	vertices_.reserve(quadVertexCount);
#ifndef USE_BOX2D
	world_.SetContactListener(this);
	world_.SetBSH(&quadTree_);
#else
	b2World_.SetContactListener(this);
#endif
	//Implement ground
	{
#ifndef USE_BOX2D
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
		collider.restitution = neko::Scalar {0};
#else
		b2BodyDef bodyDef{};
		bodyDef.position = b2Vec2{ (float)groundPosition.x, (float)groundPosition.y };
		bodyDef.type = b2_staticBody;
		bodyDef.fixedRotation = true;
		b2groundBody_ = b2World_.CreateBody(&bodyDef);

		b2PolygonShape shape;
		shape.SetAsBox((float)groundHalfSize.x, (float)groundHalfSize.y);
		b2FixtureDef fixtureDef{};
		fixtureDef.shape = &shape;
		fixtureDef.friction = 0.0f;
		fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(b2groundBody_);
		b2groundBody_->CreateFixture(&fixtureDef);
#endif
		std::array<Vec2f, 4> vertices =
			{
				Vec2f{ -groundHalfSize.x, -groundHalfSize.y },
				Vec2f{ -groundHalfSize.x, groundHalfSize.y },
				Vec2f{ groundHalfSize.x, -groundHalfSize.y },
				Vec2f{ groundHalfSize.x, groundHalfSize.y },
			};
		for (const auto vertexOffset : vertices)
		{
			const auto [x, y] = (groundPosition + vertexOffset) * pixelPerMeter;
			SDL_Vertex vertex{};
			vertex.color = groundColor;
			vertex.position.x = float{ x };
			vertex.position.y = float{ y };
			vertices_.push_back(vertex);
		}
		constexpr std::array<int, 6> indices = { 0, 1, 2, 1, 2, 3 };
		for (const auto index : indices)
		{
			indices_.push_back(index);
		}

	}
	{
#ifndef USE_BOX2D
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
		collider.restitution = neko::Scalar {0};
#else
		b2BodyDef bodyDef{};
		bodyDef.position = { (float)staticRectPosition.x, (float)staticRectPosition.y };
		bodyDef.type = b2_dynamicBody;
		bodyDef.fixedRotation = true;
		b2staticBody_ = b2World_.CreateBody(&bodyDef);

		b2PolygonShape shape{};
		shape.SetAsBox((float)rectHalfSize.x, (float)rectHalfSize.y);

		b2FixtureDef fixtureDef{};
		fixtureDef.shape = &shape;
		fixtureDef.friction = 0.0f;
		fixtureDef.density = 1.0f;
		fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(b2staticBody_);
		b2staticBody_->CreateFixture(&fixtureDef);
#endif
	}

	{
#ifndef USE_BOX2D
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
		collider.restitution = neko::Scalar {0};
#else
		b2BodyDef bodyDef{};
		bodyDef.position = { (float)dynamicRectPosition.x, (float)dynamicRectPosition.y };
		bodyDef.type = b2_dynamicBody;
		bodyDef.fixedRotation = true;
		b2dynamicBody_ = b2World_.CreateBody(&bodyDef);

		b2PolygonShape shape{};
		shape.SetAsBox((float)rectHalfSize.x, (float)rectHalfSize.y);

		b2FixtureDef fixtureDef{};
		fixtureDef.shape = &shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.0f;
		fixtureDef.userData.pointer = reinterpret_cast<uintptr_t>(b2dynamicBody_);
		b2dynamicBody_->CreateFixture(&fixtureDef);
#endif
	}
}

void MoveIntoSample::Update(float dt)
{
}

void MoveIntoSample::Draw(SDL_Renderer* renderer)
{
	if (SDL_RenderGeometry(renderer, nullptr,
		vertices_.data(), static_cast<int>(vertices_.size()),
		indices_.data(), static_cast<int>(indices_.size())))
	{
		SDL_Log("%s\n", SDL_GetError());
	}
	{
#ifdef USE_BOX2D
		const auto pos = b2staticBody_->GetPosition();
		const Vec2f bodyPosition{ pos.x, pos.y };
		const auto rect = SDL_Rect{ (int)((bodyPosition.x - rectHalfSize.x) * pixelPerMeter),
									(int)((bodyPosition.y - rectHalfSize.y) * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.x * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.y * pixelPerMeter)
		};
#else
		const auto& body = world_.body(staticBodyIndex);

		const auto rect = SDL_FRect{ (float)((body.position.x-rectHalfSize.x) * pixelPerMeter),
									(float)((body.position.y-rectHalfSize.y) * pixelPerMeter),
									(float)(neko::Scalar{ 2 } * rectHalfSize.x * pixelPerMeter),
									(float)(neko::Scalar{ 2 } * rectHalfSize.y * pixelPerMeter)
		};
#endif
		const auto color = staticBodyOnGround_ ? onGroundColor : inAirColor;
		SDL_SetRenderDrawColorFloat(renderer, color.r, color.g, color.b, color.a);
		SDL_RenderFillRect(renderer, &rect);
	}
	{
#ifdef USE_BOX2D
		const auto pos = b2dynamicBody_->GetPosition();
		const Vec2f bodyPosition{ pos.x, pos.y };
		const auto rect = SDL_Rect{ (int)((bodyPosition.x - rectHalfSize.x) * pixelPerMeter),
									(int)((bodyPosition.y - rectHalfSize.y) * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.x * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.y * pixelPerMeter)
		};
#else
		const auto& body = world_.body(dynamicBodyIndex);
		const auto rect = SDL_FRect{ (float)((body.position.x - rectHalfSize.x) * pixelPerMeter),
									(float)((body.position.y - rectHalfSize.y) * pixelPerMeter),
									(float)(neko::Scalar{ 2 } * rectHalfSize.x * pixelPerMeter),
									(float)(neko::Scalar{ 2 } * rectHalfSize.y * pixelPerMeter)
		};
#endif
		const auto color = dynamicBodyOnGround_ ? onGroundColor : inAirColor;
		SDL_SetRenderDrawColorFloat(renderer, color.r, color.g, color.b, color.a);
		SDL_RenderFillRect(renderer, &rect);
	}
}

void MoveIntoSample::End()
{
#ifdef USE_BOX2D
	b2World_ = b2World{{0.0f, -9.81f}};
#else
	world_.Clear();
	quadTree_.Clear();
#endif
	vertices_.clear();
	indices_.clear();
}

void MoveIntoSample::FixedUpdate()
{
#ifdef TRACY_ENABLE
	ZoneScoped;
#endif

	if (dynamicBodyOnGround_)
	{
#ifdef USE_BOX2D
		const Scalar mass = (Scalar)b2dynamicBody_->GetMass();
		const Vec2f velocity {b2dynamicBody_->GetLinearVelocity()};
		constexpr neko::Vec2f wantedVelocity{ Scalar{ -1 }, Scalar{}};
		auto force = (wantedVelocity - velocity) / fixedDeltaTime * mass ;
		if (force.SquareLength() > maxForce * maxForce)
		{
			force = force.Normalized() * maxForce;
		}
		b2dynamicBody_->ApplyForceToCenter({(float)force.x, (float)force.y}, true);
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

		fmt::print("Dynamic Player Pos: {}, Player Velocity: {}, Player Force: {}\n",
			(float)dynamicBody.position.x,
			(float)dynamicBody.velocity.x,
			(float)dynamicBody.force.x);
#endif
	}
	if (staticBodyOnGround_)
	{
#ifdef USE_BOX2D
		const Scalar mass = (Scalar)b2staticBody_->GetMass();
		const Vec2f velocity {b2staticBody_->GetLinearVelocity()};
		constexpr neko::Vec2f wantedVelocity{};
		auto force = (wantedVelocity - velocity) / fixedDeltaTime * mass ;
		if (force.SquareLength() > maxForce * maxForce)
		{
			force = force.Normalized() * maxForce;
		}
		b2staticBody_->ApplyForceToCenter({(float)force.x, (float)force.y}, true);
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
		fmt::print("Static Player Pos: {}, Player Velocity: {}, Player Force: {}\n",
			(float)staticBody.position.x,
			(float)staticBody.velocity.x,
			(float)staticBody.force.x);
#endif
	}
#ifdef USE_BOX2D
	b2World_.Step((float)fixedDeltaTime, 10, 3);
#else
	world_.Step(fixedDeltaTime);
#endif

}
}
