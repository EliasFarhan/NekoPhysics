#include "02_planets.h"
#include "math/fixed_trig.h"

#include "random_utils.h"

namespace neko
{

constexpr static auto G = Scalar{ 10.0f };
constexpr static auto blackHoleMass = Scalar{ 10.0f };
constexpr static auto circleRadius = Scalar{ 0.1f };
constexpr static auto innerRadius =  1.0f ;
constexpr static auto outerRadius =  6.0f ;
constexpr static auto speedDisturbanceFactor = Scalar{ 2.0f };
constexpr static Vec2f worldCenter = { Scalar{12.8f / 2.0f}, Scalar{7.2f / 2.0f} };

void PlanetSample::Begin([[maybe_unused]] const SampleContext& context)
{
    colors_.reserve(circleCount);
    for (std::size_t i = 0; i < circleCount; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.inverseMass = Scalar{ 1.0f };
        body.position = worldCenter + (Vec2f::up() * Scalar { RandomRange(innerRadius, outerRadius) }).Rotate(Scalar{ RandomRange(0.0f, 2.0f * Pi<float>()) });
        const auto delta = body.position - worldCenter;
        const auto force = G * blackHoleMass / (delta.SquareLength() * body.inverseMass);
        const auto speed = Sqrt(force * body.inverseMass * delta.Length());
        body.velocity = speed * delta.Perpendicular().Normalized();
        body.velocity += Scalar{RandomRange(-1.0f, 1.0f)} *speedDisturbanceFactor* body.velocity.Perpendicular().Normalized();
        colors_.push_back(static_cast<ColorId>(RandomRange(int{ sample_color::FIRST_BRIGHT }, int{ sample_color::LAST_BRIGHT })));
    }
}

void PlanetSample::FixedUpdate()
{
    for(std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({ static_cast<Index>(i) });
        const auto delta = body.position - worldCenter;
        const auto force = G * blackHoleMass / (delta.SquareLength() * body.inverseMass);
        body.force += force * (-delta).Normalized();
    }
    world_.Step();
}

void PlanetSample::Draw(DrawList& drawList) const
{
    for (std::size_t i = 0; i < colors_.size(); i++)
    {
        const auto& body = world_.body({ static_cast<Index>(i) });
        drawList.circles.push_back({ body.position, circleRadius, colors_[i] });
    }
}

void PlanetSample::End()
{
    colors_.clear();
    world_.Clear();
}
} // namespace neko
