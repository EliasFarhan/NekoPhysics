#include "02_planets.h"

#include <numbers>
#include <SDL_log.h>

#include "random_utils.h"

namespace neko
{

constexpr static float G = 10.0f;
constexpr static float blackHoleMass = 10.0f;
constexpr static std::size_t planetCount = 1'000;
constexpr static std::size_t circleResolution = 10;
constexpr static float circleRadius = 10.0f;
constexpr static float pixelPerMeter = 100.0f;
constexpr static auto pi = std::numbers::pi_v<float>;
constexpr static float innerRadius = 1.0f;
constexpr static float outerRadius = 6.0f;
constexpr static float speedDisturbanceFactor = 2.0f;
constexpr static Vec2f worldCenter = { 12.8f / 2.0f, 7.2f / 2.0f };

void PlanetSample::Begin()
{
    indices_.reserve(planetCount * (circleResolution * 3));
    vertices_.reserve(planetCount * (circleResolution + 1));
    for (std::size_t i = 0; i < planetCount; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.mass = 1.0f;
        body.position = worldCenter+(Vec2f::up()*RandomRange(innerRadius, outerRadius)).Rotate(RandomRange(0.0f, 2.0f*pi));
        const auto delta = body.position - worldCenter;
        const auto force = G * body.mass * blackHoleMass / delta.SquareLength();
        const auto speed = std::sqrt(force / body.mass * delta.Length());
        body.velocity = speed * delta.Perpendicular().Normalized();
        body.velocity += RandomRange(-1.0f, 1.0f) * speedDisturbanceFactor * body.velocity.Perpendicular().Normalized();
        

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
                pos += (Vec2f::up() * circleRadius).Rotate(2.0f * pi * static_cast<float>(j - 1) / static_cast<float>(circleResolution));
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

void PlanetSample::Update(float dt)
{
    for(std::size_t i = 0; i < planetCount; i++)
    {
        auto& body = world_.body({ i });
        const auto delta = body.position - worldCenter;
        const auto force = G * body.mass * blackHoleMass / delta.SquareLength();
        body.force += force * (-delta).Normalized();
    }
    world_.Step(dt);
    for (std::size_t i = 0; i < planetCount; i++)
    {
        const auto& body = world_.body({ i });

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
