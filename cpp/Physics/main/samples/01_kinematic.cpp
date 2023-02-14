#include "01_kinematic.h"
#include "random_utils.h"

#include <numbers>
#include <SDL_log.h>
#include <iostream>

namespace neko
{

constexpr static std::size_t circleCount = 1'000;
constexpr static std::size_t circleResolution = 10;
constexpr static float maxSpeed = 10.0f;
constexpr static float circleRadius = 10.0f;
constexpr static float pixelPerMeter = 100.0f;
constexpr static auto pi = std::numbers::pi_v<float>;

void KinematicSample::Begin()
{
    indices_.reserve(circleCount * (circleResolution * 3));
    vertices_.reserve(circleCount * (circleResolution + 1));
    for(std::size_t i = 0; i < circleCount; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { RandomRange(0.0f, 12.8f), RandomRange(0.0f, 7.2f) };
        body.velocity = Vec2f{ RandomRange(-1.0f, 1.0f), RandomRange(-1.0f, 1.0f) }*maxSpeed;
        body.mass = 1.0f;

        const SDL_Color color{
            static_cast<Uint8>(RandomRange(100u,255u)),
            static_cast<Uint8>(RandomRange(100u,255u)),
            static_cast<Uint8>(RandomRange(100u,255u)),
            255u
        };

        for(std::size_t j = 0; j < circleResolution+1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            SDL_Vertex vertex{};
            vertex.color = color;
            
            if(j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(2.0f * pi * static_cast<float>(j - 1)/ static_cast<float>(circleResolution));
            }
            vertex.position.x = pos.x;
            vertex.position.y = pos.y;
            vertices_.push_back(vertex);
        }

        const auto firstIndex = i * (circleResolution + 1);
        for(std::size_t j = 0; j < circleResolution; j++)
        {
            indices_.push_back(static_cast<int>(firstIndex));
            indices_.push_back(static_cast<int>(firstIndex + j + 1ull));
            indices_.push_back(static_cast<int>(firstIndex + (j == circleResolution-1ull?1ull:2ull + j) ));
        }
    }
}

void KinematicSample::Update(float dt)
{
    world_.Step(dt);
    for (std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({static_cast<int>(i)});
        if((body.velocity.x > 0.0f && body.position.x > 12.8f) || (body.velocity.x < 0.0f && body.position.x < 0.0f))
        {
            body.velocity.x = -body.velocity.x;
        }

        if ((body.velocity.y > 0.0f && body.position.y > 7.2f) || (body.velocity.y < 0.0f && body.position.y < 0.0f))
        {
            body.velocity.y = -body.velocity.y;
        }


        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            auto& vertex = vertices_[i * (circleResolution + 1) + j];

            if (j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(2.0f * pi * static_cast<float>(j - 1) / static_cast<float>(circleResolution));
            }
            vertex.position.x = pos.x;
            vertex.position.y = pos.y;
        }

        const auto firstIndex = i * (circleResolution + 1);
        for (std::size_t j = 0; j < circleResolution; j++)
        {
            auto* indices = &indices_[i * circleResolution * 3 + j*3];
            indices[0]=(static_cast<int>(firstIndex));
            indices[1]=(static_cast<int>(firstIndex + j + 1ull));
            indices[2]=(static_cast<int>(firstIndex + (j == circleResolution - 1ull ? 1ull : 2ull + j)));
        }
    }
}

void KinematicSample::End()
{
    indices_.clear();
    vertices_.clear();
    world_.Clear();
}

void KinematicSample::Draw(SDL_Renderer* renderer)
{
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    if(SDL_RenderGeometry(renderer, nullptr, 
        vertices_.data(), static_cast<int>(vertices_.size()), 
        indices_.data(), static_cast<int>(indices_.size())))
    {
        SDL_Log("%s\n", SDL_GetError());
    }
}
}
