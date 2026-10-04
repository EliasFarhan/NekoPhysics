#include "main/samples/sample.h"

#include "main/samples/01_kinematic.h"
#include "main/samples/02_planets.h"
#include "main/samples/03_triggers.h"
#include "main/samples/04_ground.h"
#include "main/samples/05_collisions.h"
#include "main/samples/06_move_into.h"

#include "neko/core/profile.h"
#include "neko/physics/physics.h"

namespace neko
{

void BounceInside(Body& body, const Aabbf& bounds)
{
    if ((body.velocity.x() > Scalar{ 0 } && body.position.x() > bounds.max_bound().x()) ||
        (body.velocity.x() < Scalar{ 0 } && body.position.x() < bounds.min_bound().x()))
    {
        body.velocity.set_x(-body.velocity.x());
    }
    if ((body.velocity.y() > Scalar{ 0 } && body.position.y() > bounds.max_bound().y()) ||
        (body.velocity.y() < Scalar{ 0 } && body.position.y() < bounds.min_bound().y()))
    {
        body.velocity.set_y(-body.velocity.y());
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
    Select(kNone, context_);
}

void SampleRegistry::Select(std::size_t index, const SampleContext& context)
{
    if (current_index_ != kNone)
    {
        samples_[current_index_].sample->End();
    }
    current_index_ = index < samples_.size() ? index : kNone;
    context_ = context;
    if (current_index_ != kNone)
    {
        samples_[current_index_].sample->Begin(context_);
    }
}

void SampleRegistry::FixedUpdate()
{
    NEKO_PROFILE_FUNCTION();
    if (current_index_ != kNone)
    {
        samples_[current_index_].sample->FixedUpdate();
    }
}

void SampleRegistry::Draw(DrawList& draw_list) const
{
    NEKO_PROFILE_FUNCTION();
    draw_list.Clear();
    draw_list.camera = context_.world_bounds;
    if (current_index_ != kNone)
    {
        samples_[current_index_].sample->Draw(draw_list);
    }
}

}  // namespace neko
