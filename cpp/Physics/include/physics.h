#pragma once


#include "vec2.h"

#include "shape.h"

#include "physics_type.h"

#include <vector>
#include <unordered_set>
#include <string>

#include "contact_listener.h"
#include "quadtree.h"

namespace neko
{


enum class BodyType
{
    DYNAMIC,
    KINEMATIC,
    STATIC,
    NONE
};

struct Body
{
    Vec2f position{};
    Vec2f velocity{};
    Vec2f force{};
    Scalar mass{};
    BodyType type = BodyType::DYNAMIC;
};

struct Collider
{
    Vec2f offset{};
    BodyIndex bodyIndex{};
    ColliderIndex colliderIndex{};
    ShapeIndex shapeIndex{};
    ColliderType type = ColliderType::NONE;
};

struct CircleCollider
{
    Scalar radius = -1.0f;
};

struct AabbCollider
{
    Vec2f halfSize{ -1.0f, -1.0f };
};

class PhysicsWorld
{
public:
    PhysicsWorld() = default;
    explicit PhysicsWorld(Vec2f gravity);
    BodyIndex AddBody();
    void RemoveBody(BodyIndex index);
    void Step(Scalar dt);
    void Clear();
    void ResolveBroadphase();
    void ResolveTriggers();

    [[nodiscard]] Body& body(BodyIndex index) { return bodies_[index.index]; }
    [[nodiscard]] const Body& body(BodyIndex index) const { return bodies_[index.index]; }

    ColliderIndex AddCircleCollider(BodyIndex body);
    ColliderIndex AddAabbCollider(BodyIndex body);

    Collider& collider(ColliderIndex colliderIndex) { return colliders_[colliderIndex.index]; }
    const Collider& collider(ColliderIndex colliderIndex) const { return colliders_[colliderIndex.index]; }

    AabbCollider& aabb(ShapeIndex shapeIndex) { return aabbs_[shapeIndex.index]; }
    const AabbCollider& aabb(ShapeIndex shapeIndex) const { return aabbs_[shapeIndex.index]; }
    CircleCollider& circle(ShapeIndex shapeIndex) { return circles_[shapeIndex.index]; }
    const CircleCollider& circle(ShapeIndex shapeIndex) const { return circles_[shapeIndex.index]; }
    
    void RemoveAabbCollider(ColliderIndex index);
    void RemoveCircleCollider(ColliderIndex index);

    void SetContactListener(ContactListener* contactListener) { contactListener_ = contactListener; }
private:
    std::vector<Body> bodies_;
    std::vector<AabbCollider> aabbs_;
    std::vector<CircleCollider> circles_;
    std::vector<Collider> colliders_;
    std::unordered_set<TriggerPair, TriggerHash> triggerPairs_;
    QuadTree quadTree_;
    static constexpr Vec2f defaultGravity{0.0f, -9.81f};
    Vec2f gravity_;
    ContactListener* contactListener_ = nullptr;

};

} // namespace neko
