#include "03_triggers.h"

#include <numbers>
#include <SDL_log.h>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif
#include <iostream>

#include "random_utils.h"

namespace neko
{


constexpr static std::size_t circleCount = 200;
constexpr static std::size_t circleResolution = 30;
constexpr static float maxSpeed = 4.0f;
constexpr static float maxCircleRadius = 0.2f;
constexpr static float minCircleRadius = 0.1f;
constexpr static float pixelPerMeter = 100.0f;
constexpr static auto pi = std::numbers::pi_v<float>;
constexpr static SDL_Color triggerColor{ 0,255,0,255 };
constexpr static SDL_Color untriggerColor{ 255,0,0,255 };

void TriggersSample::Begin()
{
    world_.SetContactListener(this);
    indices_.reserve(circleCount * (circleResolution * 3));
    vertices_.reserve(circleCount * (circleResolution + 1));
    bodies_.resize(circleCount);
    for (std::size_t i = 0; i < circleCount; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { RandomRange(0.0f, 12.8f), RandomRange(0.0f, 7.2f) };
        body.velocity = Vec2f{ RandomRange(-1.0f, 1.0f), RandomRange(-1.0f, 1.0f) }*maxSpeed;
        body.mass = 1.0f;

        auto& triggerBody = bodies_[i];
        

        const auto circleRadius = RandomRange(minCircleRadius, maxCircleRadius);
        triggerBody.circleRadius = circleRadius;
        triggerBody.index = world_.AddCircleCollider(index);
        const auto& triggerCollider = world_.collider({ triggerBody.index });
        auto& circleCollider = world_.circle(triggerCollider.shapeIndex);
        circleCollider.radius = triggerBody.circleRadius;

        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            SDL_Vertex vertex{};
            vertex.color = untriggerColor;

            if (j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(2.0f * pi * static_cast<float>(j - 1) / static_cast<float>(circleResolution)) * pixelPerMeter;
            }
            vertex.position.x = pos.x;
            vertex.position.y = pos.y;
            vertices_.push_back(vertex);
        }

        const auto firstIndex = i * (circleResolution + 1);
        for (std::size_t j = 0; j < circleResolution; j++)
        {
            indices_.push_back(static_cast<int>(firstIndex));
            indices_.push_back(static_cast<int>(firstIndex + j + 1ull));
            indices_.push_back(static_cast<int>(firstIndex + (j == circleResolution - 1ull ? 1ull : 2ull + j)));
        }
    }
}

void TriggersSample::Update(float dt)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    for (std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({ static_cast<int>(i) });
        if ((body.velocity.x > 0.0f && body.position.x > 12.8f) || (body.velocity.x < 0.0f && body.position.x < 0.0f))
        {
            body.velocity.x = -body.velocity.x;
        }

        if ((body.velocity.y > 0.0f && body.position.y > 7.2f) || (body.velocity.y < 0.0f && body.position.y < 0.0f))
        {
            body.velocity.y = -body.velocity.y;
        }

        const auto& triggerBody = bodies_[i];

        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            auto& vertex = vertices_[i * (circleResolution + 1) + j];

            if (j != 0)
            {
                pos += (Vec2f::up() * triggerBody.circleRadius).Rotate(2.0f * pi * static_cast<float>(j - 1) / static_cast<float>(circleResolution))*pixelPerMeter;
            }
            vertex.position.x = pos.x;
            vertex.position.y = pos.y;
            vertex.color = triggerBody.count == 0 ? untriggerColor : triggerColor;
        }

        const auto firstIndex = i * (circleResolution + 1);
        for (std::size_t j = 0; j < circleResolution; j++)
        {
            auto* indices = &indices_[i * circleResolution * 3 + j * 3];
            indices[0] = (static_cast<int>(firstIndex));
            indices[1] = (static_cast<int>(firstIndex + j + 1ull));
            indices[2] = (static_cast<int>(firstIndex + (j == circleResolution - 1ull ? 1ull : 2ull + j)));
        }
    }
}

void TriggersSample::Draw(SDL_Renderer* renderer)
{
    if (SDL_RenderGeometry(renderer, nullptr,
        vertices_.data(), static_cast<int>(vertices_.size()),
        indices_.data(), static_cast<int>(indices_.size())))
    {
        SDL_Log("%s\n", SDL_GetError());
    }
}

void TriggersSample::End()
{
    world_.Clear();
    bodies_.clear();
    vertices_.clear();
    indices_.clear();
}

void TriggersSample::OnTriggerEnter(const TriggerPair& p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count++;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count++;
}

void TriggersSample::OnTriggerExit(const TriggerPair& p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count--;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count--;
}

void TriggersSample::FixedUpdate()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    world_.Step(fixedDeltaTime);
    
}
}
