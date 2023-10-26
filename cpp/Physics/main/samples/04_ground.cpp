#include "04_ground.h"

#include <SDL_log.h>
#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

namespace neko
{


constexpr static std::size_t circleResolution = 30;
constexpr static Scalar circleRadius = Scalar{ 0.5f };
constexpr static Vec2f groundHalfSize = Vec2f{ Scalar{5}, Scalar{ 1 } };
constexpr static std::size_t quadResolution = 2;
constexpr static std::size_t quadVertexCount = 4;
constexpr static Scalar pixelPerMeter = Scalar{ 100.0f };
constexpr static SDL_Color groundColor{ 0, 255, 0, 255 };
constexpr static SDL_Color circleColor{ 0, 0, 255, 255 };
constexpr static SDL_Color collisionCircleColor{ 255, 0, 255, 255 };
constexpr static Vec2f worldCenter = { Scalar{12.8f / 2.0f}, Scalar{7.2f / 2.0f} };
constexpr static Vec2f groundPosition = worldCenter+Vec2f{Scalar{0}, Scalar{3}};
constexpr static Vec2f circlePosition = worldCenter-Vec2f{Scalar{0}, Scalar{3}};

void GroundSample::Begin()
{
    world_.SetContactListener(this);
    world_.SetBSH(&quadTree_);
    indices_.reserve( quadResolution*3+circleResolution * 3);
    vertices_.reserve( quadVertexCount+(circleResolution + 1));

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

        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            SDL_Vertex vertex{};
            vertex.color = circleColor;

            if (j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(Scalar{ 2 } *Pi<Scalar>() * Scalar(j - 1) / Scalar(circleResolution)) * pixelPerMeter;
            }
            vertex.position.x = float{ pos.x };
            vertex.position.y = float{ pos.y };
            vertices_.push_back(vertex);
        }

        for (std::size_t j = 0; j < circleResolution; j++)
        {
            constexpr auto firstIndex = quadVertexCount;
            indices_.push_back(static_cast<int>(firstIndex));
            indices_.push_back(static_cast<int>(firstIndex + j + 1ull));
            indices_.push_back(static_cast<int>(firstIndex + (j == circleResolution - 1ull ? 1ull : 2ull + j)));
        }
    }
    
}

void GroundSample::Update([[maybe_unused]] float dt)
{
    const auto& circleBody = world_.body(circleBodyIndex_);

    for (std::size_t j = 0; j < circleResolution + 1; j++)
    {
        auto pos = circleBody.position * pixelPerMeter;
        auto& vertex = vertices_[quadVertexCount + j];

        if (j != 0)
        {
            pos += (Vec2f::up() * circleRadius).Rotate(Scalar{ 2 } *Pi<Scalar>() * Scalar(j - 1) / Scalar(circleResolution)) * pixelPerMeter;
        }
        vertex.position.x = float{ pos.x };
        vertex.position.y = float{ pos.y };
        vertex.color = collisionCount > 0 ? collisionCircleColor : circleColor;
    }
}

void GroundSample::Draw(SDL_Renderer* renderer)
{
    if (SDL_RenderGeometry(renderer, nullptr,
        vertices_.data(), static_cast<int>(vertices_.size()),
        indices_.data(), static_cast<int>(indices_.size())))
    {
        SDL_Log("%s\n", SDL_GetError());
    }
}

void GroundSample::End()
{
    world_.Clear();
    vertices_.clear();
    indices_.clear();
    quadTree_.Clear();
}

void GroundSample::OnTriggerEnter([[maybe_unused]]const ColliderPair& p)
{
}

void GroundSample::OnTriggerExit([[maybe_unused]] const ColliderPair& p)
{
}

void GroundSample::FixedUpdate()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    world_.Step(fixedDeltaTime);
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
