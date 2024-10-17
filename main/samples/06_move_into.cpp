//
// Created by unite on 17.10.2024.
//

#include "06_move_into.h"

#include <SDL_log.h>
#include <fmt/format.h>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif



namespace neko
{

constexpr static Vec2f groundHalfSize = Vec2f{ Scalar{5}, Scalar{ 1 } };
constexpr static Vec2f rectHalfSize = Vec2f{ Scalar{0.5f}, Scalar{ 0.5f } };
constexpr static std::size_t quadResolution = 2;
constexpr static std::size_t quadVertexCount = 4;
constexpr static Scalar pixelPerMeter = Scalar{ 100.0f };

constexpr static SDL_Color groundColor{ 0, 255, 0, 255 };
constexpr static SDL_Color inAirColor{ 0, 0, 255, 255 };
constexpr static SDL_Color onGroundColor{ 255, 0, 255, 255 };
constexpr static Vec2f worldCenter = { Scalar{12.8f / 2.0f}, Scalar{7.2f / 2.0f} };
constexpr static Vec2f groundPosition = worldCenter+Vec2f{Scalar{0}, Scalar{3}};
constexpr static Vec2f dynamicRectPosition = worldCenter-Vec2f{Scalar{-2}, Scalar{3}};
constexpr static Vec2f staticRectPosition = worldCenter-Vec2f{Scalar{2}, Scalar{3}};

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

void MoveIntoSample::Begin()
{
	world_.SetContactListener(this);
	world_.SetBSH(&quadTree_);
	indices_.reserve( quadResolution*3);
	vertices_.reserve( quadVertexCount);

	//Implement ground
	{
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

		constexpr std::array<Vec2f, 4> vertices =
		{
			Vec2f{-groundHalfSize.x, -groundHalfSize.y},
			Vec2f{-groundHalfSize.x, groundHalfSize.y},
			Vec2f{groundHalfSize.x, -groundHalfSize.y},
			Vec2f{groundHalfSize.x, groundHalfSize.y},
		};
		for(const auto vertexOffset : vertices)
		{
			const auto [x, y] = (groundPosition+vertexOffset) * pixelPerMeter;
			SDL_Vertex vertex{};
			vertex.color = groundColor;
			vertex.position.x = float{ x };
			vertex.position.y = float{ y };
			vertices_.push_back(vertex);
		}
		constexpr std::array<int, 6> indices = {0,1,2,1,2,3};
		for(const auto index : indices)
		{
			indices_.push_back(index);
		}

	}
	{
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
	}

	{
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
		const auto& body = world_.body(staticBodyIndex);
		const auto rect = SDL_Rect{ (int)((body.position.x-rectHalfSize.x) * pixelPerMeter),
									(int)((body.position.y-rectHalfSize.y) * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.x * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.y * pixelPerMeter)
		};
		const SDL_Color color = staticBodyOnGround_?onGroundColor:inAirColor;
		SDL_SetRenderDrawColor(renderer, color.r, color.g,color.b,color.a);
		SDL_RenderDrawRect(renderer, &rect);
	}
	{
		const auto& body = world_.body(dynamicBodyIndex);
		const auto rect = SDL_Rect{ (int)((body.position.x-rectHalfSize.x) * pixelPerMeter),
									(int)((body.position.y-rectHalfSize.y) * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.x * pixelPerMeter),
									(int)(neko::Scalar{ 2 } * rectHalfSize.y * pixelPerMeter)
		};
		const SDL_Color color = dynamicBodyOnGround_?onGroundColor:inAirColor;
		SDL_SetRenderDrawColor(renderer, color.r, color.g,color.b,color.a);
		SDL_RenderDrawRect(renderer, &rect);
	}
}

void MoveIntoSample::End()
{
	world_.Clear();
	vertices_.clear();
	indices_.clear();
	quadTree_.Clear();
}

void MoveIntoSample::FixedUpdate()
{
#ifdef TRACY_ENABLE
	ZoneScoped;
#endif

	if(dynamicBodyOnGround_)
	{
		Body& dynamicBody = world_.body(dynamicBodyIndex);
		//Move dynamic object to the right
		constexpr neko::Vec2f wantedVelocity{Scalar{-1}, Scalar{}};
		dynamicBody.force += (wantedVelocity-dynamicBody.velocity)/fixedDeltaTime/dynamicBody.inverseMass;
		fmt::print("Dynamic Player Pos: {}, Player Velocity: {}\n", (float)dynamicBody.position.x, (float)dynamicBody.velocity.x);

	}
	if(staticBodyOnGround_)
	{
		//Use force to make object static
		Body& staticBody = world_.body(staticBodyIndex);
		//Move dynamic object to the right
		constexpr neko::Vec2f wantedVelocity{};
		staticBody.force += (wantedVelocity-staticBody.velocity)/fixedDeltaTime/staticBody.inverseMass;
		fmt::print("Static Player Pos: {}, Player Velocity: {}\n", (float)staticBody.position.x, (float)staticBody.velocity.x);

	}

	world_.Step(fixedDeltaTime);

}
}
