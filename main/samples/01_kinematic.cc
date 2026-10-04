#include "main/samples/01_kinematic.h"
#include "main/samples/sample_rng.h"

namespace neko
{

constexpr static auto maxSpeed = Scalar{ 10 };
constexpr static auto circleRadius = Ratio(1, 10);
constexpr static std::uint32_t seed = 0x01C0FFEEu;

void KinematicSample::Begin(const SampleContext& context)
{
    bounds_ = context.worldBounds;
    SampleRng rng{ seed };
    colors_.reserve(circleCount);
    for (Index i = 0; i < circleCount; ++i)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { rng.Range(bounds_.minBound.x, bounds_.maxBound.x),
            rng.Range(bounds_.minBound.y, bounds_.maxBound.y) };
        body.velocity = Vec2f{ rng.Range(Scalar{ -1 }, Scalar{ 1 }), rng.Range(Scalar{ -1 }, Scalar{ 1 }) } * maxSpeed;
        body.inverseMass = Scalar{ 1 };
        colors_.push_back(rng.RandomColor());
    }
}

void KinematicSample::FixedUpdate()
{
    world_.Step();
    for (Index i = 0; i < colors_.size(); ++i)
    {
        BounceInside(world_.body({ i }), bounds_);
    }
}

void KinematicSample::End()
{
    colors_.clear();
    world_.Clear();
}

void KinematicSample::Draw(DrawList& drawList) const
{
    for (Index i = 0; i < colors_.size(); ++i)
    {
        drawList.circles.push_back({ world_.body({ i }).position, circleRadius, colors_[i] });
    }
}
}
