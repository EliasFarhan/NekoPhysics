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
	bool isActive = true;
};

[[nodiscard]] constexpr std::uint32_t GenerateChecksum(const Body& body)
{
	std::uint32_t result = 0;
	result += std::bit_cast<std::uint32_t>(body.position.x);
	result += std::bit_cast<std::uint32_t>(body.position.y);
	result += std::bit_cast<std::uint32_t>(body.velocity.x);
	result += std::bit_cast<std::uint32_t>(body.velocity.y);
	result += std::bit_cast<std::uint32_t>(body.force.x);
	result += std::bit_cast<std::uint32_t>(body.force.y);
	result += std::bit_cast<std::uint32_t>(body.inverseMass);
	result += static_cast<std::uint32_t>(body.type);
	result += static_cast<std::uint32_t>(body.isActive);
	return result;
}

struct Collider
{
	const void* userData = nullptr;
    Vec2f offset{};
    BodyIndex bodyIndex{};
    ColliderIndex colliderIndex{};
    ShapeIndex shapeIndex{};
    Scalar restitution{ 1 };
    ShapeType type = ShapeType::NONE;
    bool isTrigger = true;
};
[[nodiscard]] constexpr std::uint32_t GenerateChecksum(const Collider& collider)
{
	std::uint32_t result = 0;
	result += std::bit_cast<std::uint32_t>(collider.offset.x);
	result += std::bit_cast<std::uint32_t>(collider.offset.y);
	result += std::bit_cast<std::uint32_t>(collider.bodyIndex);
	result += std::bit_cast<std::uint32_t>(collider.colliderIndex);
	result += std::bit_cast<std::uint32_t>(collider.shapeIndex);
	result += std::bit_cast<std::uint32_t>(collider.restitution);
	result += static_cast<std::uint32_t>(collider.type);
	result += static_cast<std::uint32_t>(collider.isTrigger);
	return result;
}

struct CircleCollider
{
    Scalar radius{ -1 };
};

[[nodiscard]] constexpr std::uint32_t GenerateChecksum(const CircleCollider& collider)
{
	std::uint32_t result = 0;
	result += std::bit_cast<std::uint32_t>(collider.radius);
	return result;
}

struct AabbCollider
{
    Vec2f halfSize{ Scalar {-1}, Scalar {-1} };
};

[[nodiscard]] constexpr std::uint32_t GenerateChecksum(const AabbCollider& collider)
{
	std::uint32_t result = 0;
	result += std::bit_cast<std::uint32_t>(collider.halfSize.x);
	result += std::bit_cast<std::uint32_t>(collider.halfSize.y);
	return result;
}

struct PlaneCollider
{
    Vec2f normal{ Vec2f::zero() };
};

[[nodiscard]] constexpr std::uint32_t GenerateChecksum(const PlaneCollider& collider)
{
	std::uint32_t result = 0;
	result += std::bit_cast<std::uint32_t>(collider.normal.x);
	result += std::bit_cast<std::uint32_t>(collider.normal.y);
	return result;
}

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

	[[nodiscard]] Collider& collider(ColliderIndex colliderIndex) { return colliders_[colliderIndex.index]; }
	[[nodiscard]] const Collider& collider(ColliderIndex colliderIndex) const { return colliders_[colliderIndex.index]; }

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
    ArrayList<Body> bodies_{StandardAllocator<Body>{heapAllocator_}};
    ArrayList<AabbCollider> aabbs_{StandardAllocator<AabbCollider>{heapAllocator_}};
    ArrayList<CircleCollider> circles_{StandardAllocator<CircleCollider>{heapAllocator_}};
    ArrayList<PlaneCollider> planes_{StandardAllocator<PlaneCollider>{heapAllocator_}};
    ArrayList<Collider> colliders_{StandardAllocator<Collider>{heapAllocator_}};
    ankerl::unordered_dense::map<ColliderPair, std::optional<Contact>, ColliderHash, std::equal_to<>, StandardAllocator<std::pair<ColliderPair, std::optional<Contact>>>>
        manifold_{manifoldBaseSize, StandardAllocator<std::pair<ColliderPair, std::optional<Contact>>>{heapAllocator_}};

    static constexpr Vec2f defaultGravity{Scalar{0.0f}, Scalar{ -9.81f }};
    static constexpr auto manifoldBaseSize = 1000;

    ContactListener* contactListener_ = nullptr;
    BoundingSurfaceHierarchy* bsh_;
    Vec2f gravity_;

};

} // namespace neko
