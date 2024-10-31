#include "03_triggers.h"

#include <SDL_log.h>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

#include "random_utils.h"

namespace neko
{


constexpr static std::size_t circleCount = 1'000;
constexpr static std::size_t circleResolution = 30;
constexpr static Scalar maxSpeed = Scalar{ 4 };
constexpr static Scalar maxCircleRadius = Scalar{ 0.02f };
constexpr static Scalar minCircleRadius = Scalar{ 0.01f };
constexpr static Scalar pixelPerMeter = Scalar{ 100.0f };
constexpr static SDL_Color triggerColor{ 0,255,0,255 };
constexpr static SDL_Color untriggerColor{ 255,0,0,255 };

void TriggersSample::Begin()
{

#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
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
        const auto& triggerCollider = world_.collider({ triggerBody.index });
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

void TriggersSample::Update([[maybe_unused]]float dt)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
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

void TriggersSample::Draw(SDL_Renderer* renderer)
{

#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    if (SDL_RenderGeometry(renderer, nullptr,
        vertices_.data(), static_cast<int>(vertices_.size()),
        indices_.data(), static_cast<int>(indices_.size())))
    {
        SDL_Log("%s\n", SDL_GetError());
    }
    std::function<void(const QuadNode*)> drawQuad = [renderer, &drawQuad](const QuadNode* node)
    {
        const auto& aabb = node->aabb;
        const auto center = aabb.GetCenter();
        const auto halfSize = aabb.GetHalfSize();
        std::array<Vec2f, 4> pos
        {
            center+Vec2f(-halfSize.x, halfSize.y),
            center+Vec2f(-halfSize.x, -halfSize.y),
            center+Vec2f(halfSize.x, -halfSize.y),
            center+Vec2f(halfSize.x, halfSize.y),
        };
        for(int i = 0; i < 4; i++)
        {
            const auto pos1 = pos[i]*pixelPerMeter;
            const auto pos2 = pos[(i+1)%4]*pixelPerMeter;
            if(SDL_RenderDrawLine(renderer, (int)pos1.x, (int)pos1.y, (int)pos2.x, (int)pos2.y))
            {
                SDL_Log("%s\n", SDL_GetError());
            }
        }
        if(node->nodes[0] != nullptr)
        {
            for(const auto childNode : node->nodes)
            {
                drawQuad(childNode);
            }
        }
    };
    SDL_SetRenderDrawColor(renderer, 255,255,255,255);
    drawQuad(&quadTree_.GetRootNode());
}

void TriggersSample::End()
{
    world_.Clear();
    bodies_.clear();
    vertices_.clear();
    indices_.clear();
    quadTree_.Clear();
}

void TriggersSample::OnTriggerEnter(const ColliderPair& p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count++;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count++;
}

void TriggersSample::OnTriggerExit(const ColliderPair& p)
{
    const auto& c1 = world_.collider(p.c1);
    bodies_[c1.bodyIndex.index].count--;
    const auto& c2 = world_.collider(p.c2);
    bodies_[c2.bodyIndex.index].count--;
}

void TriggersSample::FixedUpdate()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    world_.Step(fixedDeltaTime);
    
}

void TriggersSample::OnCollisionEnter([[maybe_unused]] const ColliderPair& p)
{
}

void TriggersSample::OnCollisionExit([[maybe_unused]] const ColliderPair& p)
{
}
}
