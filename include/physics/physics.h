#pragma once


#include "math/vec2.h"
#include "math/shape.h"

#include "physics_type.h"
#include "core/allocator.h"
#include "contact_listener.h"
#include "bsh/quadtree.h"

#include <vector>
#include <unordered_set>
#include <string>

#include "contact_solver.h"


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
    Scalar inverseMass{1};
    BodyType type = BodyType::DYNAMIC;
	bool isActive = true;
};

struct Collider
{
	void* userData = nullptr;
    Vec2f offset{};
    BodyIndex bodyIndex{};
    ColliderIndex colliderIndex{};
    ShapeIndex shapeIndex{};
    Scalar restitution{ 1 };
    ShapeType type = ShapeType::NONE;
    bool isTrigger = true;
};

struct CircleCollider
{
    Scalar radius{ -1 };
};

struct AabbCollider
{
    Vec2f halfSize{ Scalar {-1}, Scalar {-1} };
};

struct PlaneCollider
{
    Vec2f normal{ Vec2f::zero() };
};

class PhysicsWorld
{
public:
    PhysicsWorld() = default;
    explicit PhysicsWorld(Vec2f gravity);
    BodyIndex AddBody();
    void RemoveBody(BodyIndex index);
    bool DetectContact(Body& body1, Collider& collider1, Body& body2, Collider& collider2, Contact* contact);
    void Step(Scalar dt);
    void Clear();
    void ResolveBroadphase();
    void ResolveNarrowphase(Scalar dt);

    [[nodiscard]] Body& body(BodyIndex index) { return bodies_[index.index]; }
    [[nodiscard]] const Body& body(BodyIndex index) const { return bodies_[index.index]; }

    ColliderIndex AddCircleCollider(BodyIndex body);
    ColliderIndex AddAabbCollider(BodyIndex body);
    ColliderIndex AddPlaneCollider(BodyIndex body);

    Collider& collider(ColliderIndex colliderIndex) { return colliders_[colliderIndex.index]; }
    const Collider& collider(ColliderIndex colliderIndex) const { return colliders_[colliderIndex.index]; }

    AabbCollider& aabb(ShapeIndex shapeIndex) { return aabbs_[shapeIndex.index]; }
    const AabbCollider& aabb(ShapeIndex shapeIndex) const { return aabbs_[shapeIndex.index]; }
    CircleCollider& circle(ShapeIndex shapeIndex) { return circles_[shapeIndex.index]; }
    const CircleCollider& circle(ShapeIndex shapeIndex) const { return circles_[shapeIndex.index]; }
    
    void RemoveAabbCollider(ColliderIndex index);
    void RemoveCircleCollider(ColliderIndex index);
    void RemovePlaneCollider(ColliderIndex index);

    void SetBSH(BoundingSurfaceHierarchy* boundingSurfaceHierarchy) { bsh_ = boundingSurfaceHierarchy; }
    void SetContactListener(ContactListener* contactListener) { contactListener_ = contactListener; }
private:
    HeapAllocator heapAllocator_;
    ArrayList<Body> bodies_{{heapAllocator_}};
    ArrayList<AabbCollider> aabbs_{{heapAllocator_}};
    ArrayList<CircleCollider> circles_{{heapAllocator_}};
    ArrayList<PlaneCollider> planes_{{heapAllocator_}};
    ArrayList<Collider> colliders_{{heapAllocator_}};
    std::unordered_set<ColliderPair, ColliderHash, std::equal_to<>, StandardAllocator<ColliderPair>>
        manifold_{manifoldBaseSize, StandardAllocator<ColliderPair>{heapAllocator_}};

    static constexpr Vec2f defaultGravity{Scalar{0.0f}, Scalar{ -9.81f }};
    static constexpr auto manifoldBaseSize = 1000;

    ContactListener* contactListener_ = nullptr;
    BoundingSurfaceHierarchy* bsh_;
    Vec2f gravity_;

};

} // namespace neko
