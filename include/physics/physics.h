#pragma once


#include "math/vec2.h"
#include "math/shape.h"

#include "physics_type.h"
#include "contact_solver.h"
#include "core/allocator.h"
#include "contact_listener.h"
#include "bsh/quadtree.h"
#include <ankerl/unordered_dense.h>

#include <vector>
#include <string>
#include <numeric>
#include <bit>
#include <type_traits>

#include "container/index_based_container.h"


namespace neko
{


enum class BodyType : std::uint8_t
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
    [[nodiscard]] bool IsInvalid() const noexcept { return inverseMass < 0;}
    static Body GenerateInvalidValue() {return {.inverseMass = -1};}

};
using BodyIndex = Index<Body>;

constexpr auto INVALID_BODY_INDEX = BodyIndex{ -1 };

[[nodiscard]] std::uint32_t GenerateChecksum(const Body& body);

struct Collider;
using ColliderIndex = Index<Collider>;
constexpr auto INVALID_COLLIDER_INDEX = ColliderIndex{ -1 };
struct Collider
{
	const void* userData = nullptr;
    Vec2f offset{};
    BodyIndex bodyIndex = INVALID_BODY_INDEX;
    ColliderIndex colliderIndex = INVALID_COLLIDER_INDEX;
    ShapeIndex shapeIndex{};
    Scalar restitution{ 1 };
    ShapeType type = ShapeType::NONE;
    bool isTrigger = true;

    [[nodiscard]] bool IsInvalid() const noexcept { return restitution < 0; }
    static Collider GenerateInvalidValue() {return {.restitution = -1};}
};
[[nodiscard]] std::uint32_t GenerateChecksum(const Collider& collider);

struct CircleCollider
{
    Scalar radius{ 0 };
};

[[nodiscard]] std::uint32_t GenerateChecksum(const CircleCollider& collider);

struct AabbCollider
{
    Vec2f halfSize{ Scalar {0}, Scalar {0} };
    [[nodiscard]] bool IsInvalid() const noexcept { return halfSize.x < 0; }
    static AabbCollider GenerateInvalidValue() {return {.halfSize = {Scalar {-1}, Scalar {-1}}};}
};

[[nodiscard]] std::uint32_t GenerateChecksum(const AabbCollider& collider);

struct PlaneCollider
{
    Vec2f normal{ Vec2f::zero() };
};

[[nodiscard]] std::uint32_t GenerateChecksum(const PlaneCollider& collider);

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

    [[nodiscard]] Body& body(BodyIndex index) { return bodyManager_.at(index); }
    [[nodiscard]] const Body& body(BodyIndex index) const { return bodyManager_.at(index); }

	ColliderIndex AddCircleCollider(BodyIndex body);
	ColliderIndex AddAabbCollider(BodyIndex body);
	ColliderIndex AddPlaneCollider(BodyIndex body);

	[[nodiscard]] Collider& collider(ColliderIndex colliderIndex) { return colliders_.at(colliderIndex); }
	[[nodiscard]] const Collider& collider(ColliderIndex colliderIndex) const { return colliders_.at(colliderIndex); }

	[[nodiscard]] AabbCollider& aabb(ShapeIndex shapeIndex) { return aabbs_[shapeIndex.index]; }
	[[nodiscard]] const AabbCollider& aabb(ShapeIndex shapeIndex) const { return aabbs_[shapeIndex.index]; }
	[[nodiscard]] CircleCollider& circle(ShapeIndex shapeIndex) { return circles_[shapeIndex.index]; }
    [[nodiscard]] const CircleCollider& circle(ShapeIndex shapeIndex) const { return circles_[shapeIndex.index]; }
    
    void RemoveAabbCollider(ColliderIndex index);
    void RemoveCircleCollider(ColliderIndex index);
    void RemovePlaneCollider(ColliderIndex index);

    void SetBSH(BoundingSurfaceHierarchy* boundingSurfaceHierarchy) { bsh_ = boundingSurfaceHierarchy; }
    void SetContactListener(ContactListener* contactListener) { contactListener_ = contactListener; }

	[[nodiscard]] Vec2f gravity() const {return gravity_; }

	void CopyFrom(const PhysicsWorld& physicsWorld);
private:
    HeapAllocator heapAllocator_;

    template<typename T>
    using allocator_type = StandardAllocator<std::pair<T, typename Index<T>::generation_index_type>>;
    template<typename T>
    using container_type = IndexBasedContainer<T, allocator_type<T>>;

    container_type<Body> bodyManager_{allocator_type<Body>{heapAllocator_}};
    container_type<AabbCollider> aabbs_{allocator_type<AabbCollider>{heapAllocator_}};
    container_type<CircleCollider> circles_{allocator_type<CircleCollider>{heapAllocator_}};
    container_type<PlaneCollider> planes_{allocator_type<PlaneCollider>{heapAllocator_}};
    container_type<Collider> colliders_{allocator_type<Collider>{heapAllocator_}};
    ankerl::unordered_dense::map<ColliderPair, std::optional<Contact>, ColliderHash, std::equal_to<>, StandardAllocator<std::pair<ColliderPair, std::optional<Contact>>>>
        manifold_{manifoldBaseSize, StandardAllocator<std::pair<ColliderPair, std::optional<Contact>>>{heapAllocator_}};

    static constexpr Vec2f defaultGravity{Scalar{0.0f}, Scalar{ -9.81f }};
    static constexpr auto manifoldBaseSize = 1000;

    ContactListener* contactListener_ = nullptr;
    BoundingSurfaceHierarchy* bsh_;
    Vec2f gravity_;

};

} // namespace neko
