#include "main/samples/02_planets.h"
#include "main/samples/sample_rng.h"

namespace neko
{

constexpr static auto kG = Scalar{ 10 };
constexpr static auto kBlackHoleMass = Scalar{ 10 };
constexpr static auto kCircleRadius = Ratio(1, 10);
constexpr static auto kInnerRadius = Scalar{ 1 };
constexpr static auto kOuterRadius = Scalar{ 6 };
constexpr static auto kSpeedDisturbanceFactor = Scalar{ 2 };
constexpr static std::uint32_t kSeed = 0x02C0FFEEu;

void PlanetSample::Begin(const SampleContext& context)
{
    center_ = context.world_bounds.GetCenter();
    SampleRng rng{ kSeed };
    colors_.reserve(kCircleCount);
    for (Index i = 0; i < kCircleCount; ++i)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.inverse_mass = Scalar{ 1 };
        const auto distance = rng.Range(kInnerRadius, kOuterRadius);
        const auto angle = rng.Range(Scalar{ 0 }, Scalar{ 2 } * Pi<Scalar>());
        body.position = center_ + (Vec2f::Up() * distance).Rotate(angle);
        const auto delta = body.position - center_;
        const auto force = kG * kBlackHoleMass / (delta.SquareLength() * body.inverse_mass);
        const auto speed = Sqrt(force * body.inverse_mass * delta.Length());
        body.velocity = speed * delta.Perpendicular().Normalized();
        body.velocity += rng.Range(Scalar{ -1 }, Scalar{ 1 }) * kSpeedDisturbanceFactor * body.velocity.Perpendicular().Normalized();
        colors_.push_back(rng.RandomColor());
    }
}

void PlanetSample::FixedUpdate()
{
    for (Index i = 0; i < colors_.size(); ++i)
    {
        auto& body = world_.body(BodyIndex{ i });
        const auto delta = body.position - center_;
        const auto force = kG * kBlackHoleMass / (delta.SquareLength() * body.inverse_mass);
        body.force += force * (-delta).Normalized();
    }
    world_.Step();
}

void PlanetSample::Draw(DrawList& draw_list) const
{
    for (Index i = 0; i < colors_.size(); ++i)
    {
        draw_list.circles.push_back({ world_.body(BodyIndex{ i }).position, kCircleRadius, colors_[i] });
    }
}

void PlanetSample::End()
{
    colors_.clear();
    world_.Clear();
}
} // namespace neko
