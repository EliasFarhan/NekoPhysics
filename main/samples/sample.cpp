#include "sample.h"

#include "01_kinematic.h"
#include "02_planets.h"
#include "03_triggers.h"
#include "04_ground.h"
#include "05_collisions.h"
#include "06_move_into.h"

#include "core/profile.h"
#include "physics/physics.h"

namespace neko
{

void BounceInside(Body& body, const Aabbf& bounds)
{
    if ((body.velocity.x > Scalar{ 0 } && body.position.x > bounds.maxBound.x) ||
        (body.velocity.x < Scalar{ 0 } && body.position.x < bounds.minBound.x))
    {
        body.velocity.x = -body.velocity.x;
    }
    if ((body.velocity.y > Scalar{ 0 } && body.position.y > bounds.maxBound.y) ||
        (body.velocity.y < Scalar{ 0 } && body.position.y < bounds.minBound.y))
    {
        body.velocity.y = -body.velocity.y;
    }
}

SampleRegistry::SampleRegistry()
{
    samples_.reserve(6);
    samples_.push_back({ "01 Kinematic", std::make_unique<KinematicSample>() });
    samples_.push_back({ "02 Planets", std::make_unique<PlanetSample>() });
    samples_.push_back({ "03 Triggers", std::make_unique<TriggersSample>() });
    samples_.push_back({ "04 Ground", std::make_unique<GroundSample>() });
    samples_.push_back({ "05 Collisions", std::make_unique<CollisionsSample>() });
    samples_.push_back({ "06 Move Into", std::make_unique<MoveIntoSample>() });
}

SampleRegistry::~SampleRegistry()
{
    Select(NONE, context_);
}

void SampleRegistry::Select(std::size_t index, const SampleContext& context)
{
    if (currentIndex_ != NONE)
    {
        samples_[currentIndex_].sample->End();
    }
    currentIndex_ = index < samples_.size() ? index : NONE;
    context_ = context;
    if (currentIndex_ != NONE)
    {
        samples_[currentIndex_].sample->Begin(context_);
    }
}

void SampleRegistry::FixedUpdate()
{
    NEKO_PROFILE_FUNCTION();
    if (currentIndex_ != NONE)
    {
        samples_[currentIndex_].sample->FixedUpdate();
    }
}

void SampleRegistry::Draw(DrawList& drawList) const
{
    NEKO_PROFILE_FUNCTION();
    drawList.Clear();
    drawList.camera = context_.worldBounds;
    if (currentIndex_ != NONE)
    {
        samples_[currentIndex_].sample->Draw(drawList);
    }
}

} // namespace neko
