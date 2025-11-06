#pragma once



#include "math/shape.h"

#include "physics_type.h"
#include "contact_solver.h"
#include "core/allocator.h"
#include "contact_listener.h"
#include "bsh/quadtree.h"
#include <ankerl/unordered_dense.h>

#include <vector>
#include <variant>
#include <string>
#include <numeric>
#include <bit>
#include <span>
#include <type_traits>

#include "container/index_based_container.h"


namespace neko
{


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

	auto& bodies(){return bodyManager_;}
    [[nodiscard]] Body& body(BodyIndex index) { return bodyManager_.at(index); }
    [[nodiscard]] const Body& body(BodyIndex index) const { return bodyManager_.at(index); }

	ColliderIndex AddCircleCollider(BodyIndex body);
	ColliderIndex AddAabbCollider(BodyIndex body);
	ColliderIndex AddPlaneCollider(BodyIndex body);

	[[nodiscard]] Collider& collider(ColliderIndex colliderIndex) { return colliderManager_.at(colliderIndex); }
	[[nodiscard]] const Collider& collider(ColliderIndex colliderIndex) const { return colliderManager_.at(colliderIndex); }

	[[nodiscard]] AabbCollider& aabb(ShapeIndex shapeIndex) { return std::get<static_cast<int>(ShapeType::AABB)>(shapes_[shapeIndex]); }
	[[nodiscard]] const AabbCollider& aabb(ShapeIndex shapeIndex) const { return std::get<static_cast<int>(ShapeType::AABB)>(shapes_[shapeIndex]); }
	[[nodiscard]] CircleCollider& circle(ShapeIndex shapeIndex) { return std::get<static_cast<int>(ShapeType::CIRCLE)>(shapes_[shapeIndex]); }
    [[nodiscard]] const CircleCollider& circle(ShapeIndex shapeIndex) const { return std::get<static_cast<int>(ShapeType::CIRCLE)>(shapes_[shapeIndex]); }
    
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
    container_type<Shape> shapes_{allocator_type<Shape>{heapAllocator_}};
    container_type<Collider> colliderManager_{allocator_type<Collider>{heapAllocator_}};
    ankerl::unordered_dense::map<ColliderPair, std::optional<Contact>, ColliderHash, std::equal_to<>, StandardAllocator<std::pair<ColliderPair, std::optional<Contact>>>>
        manifold_{manifoldBaseSize, StandardAllocator<std::pair<ColliderPair, std::optional<Contact>>>{heapAllocator_}};

    static constexpr Vec2f defaultGravity{Scalar{0.0f}, Scalar{ -9.81f }};
    static constexpr auto manifoldBaseSize = 1000;

    ContactListener* contactListener_ = nullptr;
    BoundingSurfaceHierarchy* bsh_;
    Vec2f gravity_;

};

} // namespace neko
