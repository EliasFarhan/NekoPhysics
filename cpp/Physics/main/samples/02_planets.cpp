#include "02_planets.h"
#include "math/fixed_lut.h"

#include <SDL_log.h>

#include "random_utils.h"

namespace neko
{

constexpr static auto G = Scalar{ 10.0f };
constexpr static auto blackHoleMass = Scalar{ 10.0f };
constexpr static std::size_t circleCount = 1'000;
constexpr static std::size_t circleResolution = 10;
constexpr static auto circleRadius = Scalar{ 10.0f };
constexpr static auto pixelPerMeter = Scalar{ 100.0f };
constexpr static auto innerRadius =  1.0f ;
constexpr static auto outerRadius =  6.0f ;
constexpr static auto speedDisturbanceFactor = Scalar{ 2.0f };
constexpr static Vec2f worldCenter = { Scalar{12.8f / 2.0f}, Scalar{7.2f / 2.0f} };

void PlanetSample::Begin()
{
    indices_.reserve(circleCount * (circleResolution * 3));
    vertices_.reserve(circleCount * (circleResolution + 1));
    for (std::size_t i = 0; i < circleCount; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.mass = Scalar{ 1.0f };
        body.position = worldCenter + (Vec2f::up() * Scalar { RandomRange(innerRadius, outerRadius) }).Rotate(Scalar{ RandomRange(0.0f, 2.0f * Pi<float>()) });
        const auto delta = body.position - worldCenter;
        const auto force = G * body.mass * blackHoleMass / delta.SquareLength();
        const auto speed = Sqrt(force / body.mass * delta.Length());
        body.velocity = speed * delta.Perpendicular().Normalized();
        body.velocity += Scalar{RandomRange(-1.0f, 1.0f)} *speedDisturbanceFactor* body.velocity.Perpendicular().Normalized();
        

        const SDL_Color color{
            static_cast<Uint8>(RandomRange(100u,255u)),
            static_cast<Uint8>(RandomRange(100u,255u)),
            static_cast<Uint8>(RandomRange(100u,255u)),
            255u
        };

        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            SDL_Vertex vertex{};
            vertex.color = color;

            if (j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(Scalar{ 2.0f } * Pi<Scalar>() * Scalar(j - 1) / Scalar(circleResolution));
            }
            vertex.position.x = float{pos.x};
            vertex.position.y = float{pos.y};
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

void PlanetSample::Update(float dt)
{
    for(std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({ static_cast<int>(i) });
        const auto delta = body.position - worldCenter;
        const auto force = G * body.mass * blackHoleMass / delta.SquareLength();
        body.force += force * (-delta).Normalized();
    }
    world_.Step(Scalar{ dt });
    for (std::size_t i = 0; i < circleCount; i++)
    {
        const auto& body = world_.body({ static_cast<int>(i) });

        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            auto& vertex = vertices_[i * (circleResolution + 1) + j];

            if (j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(Scalar{ 2 } * Pi<Scalar>() * Scalar(j - 1) / Scalar(circleResolution));
            }
            vertex.position.x = float{pos.x};
            vertex.position.y = float{pos.y};
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

void PlanetSample::Draw(SDL_Renderer* renderer)
{
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    if (SDL_RenderGeometry(renderer, nullptr,
        vertices_.data(), static_cast<int>(vertices_.size()),
        indices_.data(), static_cast<int>(indices_.size())))
    {
        SDL_Log("%s\n", SDL_GetError());
    }
}

void PlanetSample::End()
{
    indices_.clear();
    vertices_.clear();
    world_.Clear();
}
} // namespace neko
