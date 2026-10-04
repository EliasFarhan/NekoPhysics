#include "main/samples/01_kinematic.h"
#include "main/samples/sample_rng.h"

namespace neko
{

constexpr static auto kMaxSpeed = Scalar{ 10 };
constexpr static auto kCircleRadius = Ratio(1, 10);
constexpr static std::uint32_t kSeed = 0x01C0FFEEu;

void KinematicSample::Begin(const SampleContext& context)
{
    bounds_ = context.world_bounds;
    SampleRng rng{ kSeed };
    colors_.reserve(kCircleCount);
    for (Index i = 0; i < kCircleCount; ++i)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { rng.Range(bounds_.min_bound().x(), bounds_.max_bound().x()),
            rng.Range(bounds_.min_bound().y(), bounds_.max_bound().y()) };
        body.velocity = Vec2f{ rng.Range(Scalar{ -1 }, Scalar{ 1 }), rng.Range(Scalar{ -1 }, Scalar{ 1 }) } * kMaxSpeed;
        body.inverse_mass = Scalar{ 1 };
        colors_.push_back(rng.RandomColor());
    }
}

void KinematicSample::FixedUpdate()
{
    world_.Step();
    for (Index i = 0; i < colors_.size(); ++i)
    {
        BounceInside(world_.body(BodyIndex{ i }), bounds_);
    }
}

void KinematicSample::End()
{
    colors_.clear();
    world_.Clear();
}

void KinematicSample::Draw(DrawList& draw_list) const
{
    for (Index i = 0; i < colors_.size(); ++i)
    {
        draw_list.circles.push_back({ world_.body(BodyIndex{ i }).position, kCircleRadius, colors_[i] });
    }
}
}
