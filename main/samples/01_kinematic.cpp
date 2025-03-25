#include "01_kinematic.h"
#include "random_utils.h"

#include <SDL3/SDL_log.h>

namespace neko
{

constexpr static std::size_t circleCount = 1'000;
constexpr static std::size_t circleResolution = 10;
constexpr static auto maxSpeed = Scalar{ 10.0f };
constexpr static auto circleRadius = Scalar{10.0f };
constexpr static auto pixelPerMeter = Scalar{ 100.0f };

void KinematicSample::Begin()
{
    indices_.reserve(circleCount * (circleResolution * 3));
    vertices_.reserve(circleCount * (circleResolution + 1));
    for (std::size_t i = 0; i < std::size_t{circleCount}; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { Scalar{RandomRange(0.0f, 12.8f)}, Scalar{RandomRange(0.0f, 7.2f)} };
        body.velocity = Vec2f{ Scalar{RandomRange(-1.0f, 1.0f)}, Scalar{RandomRange(-1.0f, 1.0f)} }*maxSpeed;
        body.inverseMass = Scalar{ 1.0f };

        const SDL_FColor color{
            static_cast<float>(RandomRange(100u,255u))/1.0f,
            static_cast<float>(RandomRange(100u,255u))/1.0f,
            static_cast<float>(RandomRange(100u,255u))/1.0f,
            1.0f
        };

        for (std::size_t j = 0; j < std::size_t{circleResolution} + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            SDL_Vertex vertex{};
            vertex.color = color;
            
            if(j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(Scalar{ 2.0f } * Pi<Scalar>() * Scalar(j - 1) / Scalar(circleResolution));
            }
            vertex.position.x = float{pos.x};
            vertex.position.y = float{pos.y};
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
    world_.Step(Scalar{ dt });
    for (std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({static_cast<int>(i)});
        if ((body.velocity.x > Scalar{0.0f}&& body.position.x > Scalar{12.8f}) || (body.velocity.x < Scalar{0.0f}&& body.position.x < Scalar{0.0f}))
        {
            body.velocity.x = -body.velocity.x;
        }

        if ((body.velocity.y > Scalar{0.0f}&& body.position.y > Scalar{7.2f}) || 
            (body.velocity.y < Scalar{0.0f}&& body.position.y < Scalar{0.0f}))
        {
            body.velocity.y = -body.velocity.y;
        }


        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            auto& vertex = vertices_[i * (circleResolution + 1) + j];

            if (j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(Scalar{ 2.0f } * Pi<Scalar>() * Scalar(static_cast<int>(j) - 1) / Scalar{circleResolution});
            }
            vertex.position.x = float{pos.x};
            vertex.position.y = float{pos.y};
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
