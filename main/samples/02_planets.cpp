#include "02_planets.h"
#include "sample_rng.h"

namespace neko
{

constexpr static auto G = Scalar{ 10 };
constexpr static auto blackHoleMass = Scalar{ 10 };
constexpr static auto circleRadius = Ratio(1, 10);
constexpr static auto innerRadius = Scalar{ 1 };
constexpr static auto outerRadius = Scalar{ 6 };
constexpr static auto speedDisturbanceFactor = Scalar{ 2 };
constexpr static std::uint32_t seed = 0x02C0FFEEu;

void PlanetSample::Begin(const SampleContext& context)
{
    center_ = context.worldBounds.GetCenter();
    SampleRng rng{ seed };
    colors_.reserve(circleCount);
    for (Index i = 0; i < circleCount; ++i)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.inverseMass = Scalar{ 1 };
        const auto distance = rng.Range(innerRadius, outerRadius);
        const auto angle = rng.Range(Scalar{ 0 }, Scalar{ 2 } * Pi<Scalar>());
        body.position = center_ + (Vec2f::up() * distance).Rotate(angle);
        const auto delta = body.position - center_;
        const auto force = G * blackHoleMass / (delta.SquareLength() * body.inverseMass);
        const auto speed = Sqrt(force * body.inverseMass * delta.Length());
        body.velocity = speed * delta.Perpendicular().Normalized();
        body.velocity += rng.Range(Scalar{ -1 }, Scalar{ 1 }) * speedDisturbanceFactor * body.velocity.Perpendicular().Normalized();
        colors_.push_back(rng.RandomColor());
    }
}

void PlanetSample::FixedUpdate()
{
    for (Index i = 0; i < colors_.size(); ++i)
    {
        auto& body = world_.body({ i });
        const auto delta = body.position - center_;
        const auto force = G * blackHoleMass / (delta.SquareLength() * body.inverseMass);
        body.force += force * (-delta).Normalized();
    }
    world_.Step();
}

void PlanetSample::Draw(DrawList& drawList) const
{
    for (Index i = 0; i < colors_.size(); ++i)
    {
        drawList.circles.push_back({ world_.body({ i }).position, circleRadius, colors_[i] });
    }
}

void PlanetSample::End()
{
    colors_.clear();
    world_.Clear();
}
} // namespace neko
