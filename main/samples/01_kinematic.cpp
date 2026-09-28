#include "01_kinematic.h"
#include "random_utils.h"

namespace neko
{

constexpr static auto maxSpeed = Scalar{ 10.0f };
constexpr static auto circleRadius = Scalar{ 0.1f };

void KinematicSample::Begin([[maybe_unused]] const SampleContext& context)
{
    colors_.reserve(circleCount);
    for (std::size_t i = 0; i < std::size_t{circleCount}; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { Scalar{RandomRange(0.0f, 12.8f)}, Scalar{RandomRange(0.0f, 7.2f)} };
        body.velocity = Vec2f{ Scalar{RandomRange(-1.0f, 1.0f)}, Scalar{RandomRange(-1.0f, 1.0f)} }*maxSpeed;
        body.inverseMass = Scalar{ 1.0f };
        colors_.push_back(static_cast<ColorId>(RandomRange(int{ sample_color::FIRST_BRIGHT }, int{ sample_color::LAST_BRIGHT })));
    }
}

void KinematicSample::FixedUpdate()
{
    world_.Step();
    for (std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({ static_cast<Index>(i) });
        if ((body.velocity.x > Scalar{0.0f}&& body.position.x > Scalar{12.8f}) || (body.velocity.x < Scalar{0.0f}&& body.position.x < Scalar{0.0f}))
        {
            body.velocity.x = -body.velocity.x;
        }

        if ((body.velocity.y > Scalar{0.0f}&& body.position.y > Scalar{7.2f}) ||
            (body.velocity.y < Scalar{0.0f}&& body.position.y < Scalar{0.0f}))
        {
            body.velocity.y = -body.velocity.y;
        }
    }
}

void KinematicSample::End()
{
    colors_.clear();
    world_.Clear();
}

void KinematicSample::Draw(DrawList& drawList) const
{
    for (std::size_t i = 0; i < colors_.size(); i++)
    {
        const auto& body = world_.body({ static_cast<Index>(i) });
        drawList.circles.push_back({ body.position, circleRadius, colors_[i] });
    }
}
}
