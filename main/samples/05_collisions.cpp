#include <SDL3/SDL_log.h>
#include "05_collisions.h"
#include "random_utils.h"

namespace neko
{

constexpr static std::size_t circleCount = 50;
constexpr static std::size_t circleResolution = 30;
constexpr static Scalar maxSpeed = Scalar{ 4 };
constexpr static Scalar maxCircleRadius = Scalar{ 0.2f };
constexpr static Scalar minCircleRadius = Scalar{ 0.1f };
constexpr static Scalar pixelPerMeter = Scalar{ 100.0f };
constexpr static SDL_FColor triggerColor{ 0,255,0,255 };
constexpr static SDL_FColor untriggerColor{ 255,0,0,255 };


void CollisionsSample::Begin()
{
    world_.SetContactListener(this);
    world_.SetBSH(&quadTree_);
    indices_.reserve(circleCount * (circleResolution * 3));
    vertices_.reserve(circleCount * (circleResolution + 1));
    bodies_.resize(circleCount);
    for (std::size_t i = 0; i < circleCount; i++)
    {
        const auto index = world_.AddBody();
        Body& body = world_.body(index);
        body.position = { Scalar{RandomRange(0.0f, 12.8f)}, Scalar{RandomRange(0.0f, 7.2f)} };
        body.velocity = Vec2f{ Scalar{RandomRange(-1.0f, 1.0f)}, Scalar{RandomRange(-1.0f, 1.0f)} } *maxSpeed;
        body.inverseMass = Scalar{ 1 };

        auto& triggerBody = bodies_[i];


        const auto circleRadius = Scalar{ RandomRange(float{minCircleRadius}, float{maxCircleRadius}) };
        triggerBody.circleRadius = circleRadius;
        triggerBody.index = world_.AddCircleCollider(index);
        auto& triggerCollider = world_.collider({ triggerBody.index });
        triggerCollider.isTrigger = false;
        auto& circleCollider = world_.circle(triggerCollider.shapeIndex);
        circleCollider.radius = triggerBody.circleRadius;

        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            SDL_Vertex vertex{};
            vertex.color = untriggerColor;

            if (j != 0)
            {
                pos += (Vec2f::up() * circleRadius).Rotate(Scalar{ 2 } * Pi<Scalar>() * Scalar(j - 1) / Scalar (circleResolution)) * pixelPerMeter;
            }
            vertex.position.x = float{pos.x};
            vertex.position.y = float{ pos.y };
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

void CollisionsSample::Update([[maybe_unused]] float dt)
{
    for (std::size_t i = 0; i < circleCount; i++)
    {
        auto& body = world_.body({ static_cast<int>(i) });
        if ((body.velocity.x > Scalar{0.0f}&& body.position.x > Scalar{12.8f}) || (body.velocity.x < Scalar{0.0f}&& body.position.x < Scalar{0.0f}))
        {
            body.velocity.x = -body.velocity.x;
        }

        if ((body.velocity.y > Scalar{0.0f} && body.position.y > Scalar{7.2f}) || (body.velocity.y < Scalar{0.0f}&& body.position.y < Scalar{0.0f}))
        {
            body.velocity.y = -body.velocity.y;
        }

        const auto& triggerBody = bodies_[i];

        for (std::size_t j = 0; j < circleResolution + 1; j++)
        {
            auto pos = body.position * pixelPerMeter;
            auto& vertex = vertices_[i * (circleResolution + 1) + j];

            if (j != 0)
            {
                pos += (Vec2f::up() * triggerBody.circleRadius).Rotate(Scalar{ 2 } * Pi<Scalar>() * Scalar(j - 1) / Scalar(circleResolution)) * pixelPerMeter;
            }
            vertex.position.x = float{pos.x};
            vertex.position.y = float{ pos.y };
            vertex.color = triggerBody.count == 0 ? untriggerColor : triggerColor;
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

void CollisionsSample::Draw(SDL_Renderer *renderer)
{
    if (SDL_RenderGeometry(renderer, nullptr,
                           vertices_.data(), static_cast<int>(vertices_.size()),
                           indices_.data(), static_cast<int>(indices_.size())))
    {
        SDL_Log("%s\n", SDL_GetError());
    }
}

void CollisionsSample::End()
{
    world_.Clear();
    bodies_.clear();
    vertices_.clear();
    indices_.clear();
    quadTree_.Clear();
}


void CollisionsSample::FixedUpdate()
{
    world_.Step(fixedDeltaTime);
}

void CollisionsSample::OnCollisionEnter(const ColliderPair &p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count++;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count++;
}

void CollisionsSample::OnCollisionExit(const ColliderPair &p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count--;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count--;
}

void CollisionsSample::OnTriggerEnter([[maybe_unused]]const ColliderPair &p)
{

}

void CollisionsSample::OnTriggerExit([[maybe_unused]] const ColliderPair &p)
{

}
}