#pragma once


#include "math/vec2.h"
#include "math/shape.h"

#include "physics_type.h"
#include "contact_solver.h"
#include "core/allocator.h"
#include "contact_listener.h"
#include "bsh/bsh.h"
#include "bsh/sweep_and_prune.h"

#include <cstddef>
#include <cstdint>


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

[[nodiscard]] std::uint32_t GenerateChecksum(const Body& body);

struct Collider
{
	const void* userData = nullptr;
    Vec2f offset{};
    Scalar restitution{ 1 };
    BodyIndex bodyIndex{};
    ColliderIndex colliderIndex{};
    ShapeIndex shapeIndex{};
    ShapeType type = ShapeType::NONE;
    bool isTrigger = true;
};
[[nodiscard]] std::uint32_t GenerateChecksum(const Collider& collider);

/**
 * @brief A negative radius marks a free slot
 */
struct CircleCollider
{
    Scalar radius{ -1 };
};

[[nodiscard]] std::uint32_t GenerateChecksum(const CircleCollider& collider);

/**
 * @brief A negative half size marks a free slot
 */
struct AabbCollider
{
    Vec2f halfSize{ Scalar {-1}, Scalar {-1} };
};

[[nodiscard]] std::uint32_t GenerateChecksum(const AabbCollider& collider);

/**
 * @brief Pair of colliders that touched at the end of the last step, used to send enter and exit events
 */
struct TouchingPair
{
    ColliderPair pair{};
    bool isTrigger = false;
};

struct PhysicsWorldConfig
{
    Vec2f gravity{ Scalar{0.0f}, Scalar{-9.81f} };
    /**
     * @brief Duration of one Step. Fixed, so that every peer and every rollback resimulation uses the same value
     */
    Scalar dt{ 1.0f / 60.0f };
    /**
     * @brief Capacities are allocated once, at construction: Step and CopyFrom never allocate
     */
    std::size_t maxBodies = 256;
    std::size_t maxColliders = 256;
    /**
     * @brief Maximum number of touching pairs tracked for enter/exit events
     */
    std::size_t maxTouchingPairs = 1024;
    /**
     * @brief Maximum number of pairs whose AABBs overlap, for the default broadphase
     */
    std::size_t maxBroadphasePairs = 2048;
    /**
     * @brief Must outlive the world. nullptr means DefaultAllocator()
     */
    Allocator* allocator = nullptr;
};

class PhysicsWorld
{
public:
    explicit PhysicsWorld(const PhysicsWorldConfig& config = {});
    /**
     * @brief Copies would allocate: rollback snapshots use CopyFrom into an existing world instead
     */
    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;
    PhysicsWorld(PhysicsWorld&&) = default;
    PhysicsWorld& operator=(PhysicsWorld&&) = default;
    ~PhysicsWorld() = default;

    /**
     * @brief Returns INVALID_BODY_INDEX when maxBodies are in use
     */
    BodyIndex AddBody();
    /**
     * @brief Removes the body and all its colliders
     */
    void RemoveBody(BodyIndex index);
    bool DetectContact(Body& body1, Collider& collider1, Body& body2, Collider& collider2, Contact* contact);
    /**
     * @brief Advances the world by config.dt
     */
    void Step();
    void Clear();
    void ResolveBroadphase();
    void ResolveNarrowphase();
    /**
     * @brief Sorts newTouchingPairs_ by pair (c1 then c2), the same total order as
     * std::sort(..., [](a, b){ return a.pair < b.pair; }), but in O(n + maxColliders): a counting sort on c1
     * (colliders are indices into a small, fixed-size table) followed by an insertion sort of each c1 bucket, which
     * is short (a body only touches a handful of others).
     */
    void SortNewTouchingPairs();

    [[nodiscard]] Body& body(BodyIndex index) { return bodies_[index.index]; }
    [[nodiscard]] const Body& body(BodyIndex index) const { return bodies_[index.index]; }

    /**
     * @brief Return INVALID_COLLIDER_INDEX when maxColliders are in use
     */
	ColliderIndex AddCircleCollider(BodyIndex body);
	ColliderIndex AddAabbCollider(BodyIndex body);
    void RemoveCollider(ColliderIndex index);

	[[nodiscard]] Collider& collider(ColliderIndex colliderIndex) { return colliders_[colliderIndex.index]; }
	[[nodiscard]] const Collider& collider(ColliderIndex colliderIndex) const { return colliders_[colliderIndex.index]; }

	[[nodiscard]] AabbCollider& aabb(ShapeIndex shapeIndex) { return aabbs_[shapeIndex.index]; }
	[[nodiscard]] const AabbCollider& aabb(ShapeIndex shapeIndex) const { return aabbs_[shapeIndex.index]; }
	[[nodiscard]] CircleCollider& circle(ShapeIndex shapeIndex) { return circles_[shapeIndex.index]; }
    [[nodiscard]] const CircleCollider& circle(ShapeIndex shapeIndex) const { return circles_[shapeIndex.index]; }

    /**
     * @brief Replaces the default broadphase (sort and sweep); nullptr restores it
     */
    void SetBSH(BoundingSurfaceHierarchy* boundingSurfaceHierarchy) { bsh_ = boundingSurfaceHierarchy; }
    void SetContactListener(ContactListener* contactListener) { contactListener_ = contactListener; }

	[[nodiscard]] Vec2f gravity() const { return config_.gravity; }
	[[nodiscard]] Scalar dt() const { return config_.dt; }
    [[nodiscard]] const ArrayList<TouchingPair>& touchingPairs() const { return touchingPairs_; }
    /**
     * @brief Number of pairs whose AABBs overlap, found by the broadphase during the last ResolveBroadphase
     */
    [[nodiscard]] std::size_t broadphasePairCount() const { return Broadphase().GetPossiblePairs().size(); }

    /**
     * @brief Copies the simulation state (bodies, colliders, shapes, touching pairs), not the listener nor the BSH.
     * Doesn't allocate when both worlds have the same config.
     */
	void CopyFrom(const PhysicsWorld& physicsWorld);
    /**
     * @brief Hash of the whole simulation state, to detect desyncs between peers
     */
    [[nodiscard]] std::uint32_t GenerateChecksum() const;
private:
    void NotifyContactChanges() const;
    [[nodiscard]] BoundingSurfaceHierarchy& Broadphase() { return bsh_ != nullptr ? *bsh_ : defaultBroadphase_; }
    [[nodiscard]] const BoundingSurfaceHierarchy& Broadphase() const
    {
        return bsh_ != nullptr ? static_cast<const BoundingSurfaceHierarchy&>(*bsh_) : defaultBroadphase_;
    }
    [[nodiscard]] Aabbf GetAabb(const Collider& collider) const;

    PhysicsWorldConfig config_;
    SweepAndPrune defaultBroadphase_;
    ArrayList<Body> bodies_;
    ArrayList<AabbCollider> aabbs_;
    ArrayList<CircleCollider> circles_;
    ArrayList<Collider> colliders_;
    ArrayList<TouchingPair> touchingPairs_;
    ArrayList<TouchingPair> newTouchingPairs_;
    /**
     * @brief Scratch buffer for ResolveBroadphase: each active collider's Aabbf, computed once per Step and reused
     * for both the world box pass and the insert pass (GetAabb isn't free: a body lookup plus a shape switch)
     */
    ArrayList<Aabbf> colliderAabbCache_;
    /**
     * @brief Scratch for SortNewTouchingPairs: bucketCount_[k] then bucketStart_[k] hold, then temporarily the write
     * cursor for, colliders whose ColliderPair::c1.index == k; sorted_ is the counting sort's output buffer.
     */
    ArrayList<std::uint32_t> touchingPairBucketCount_;
    ArrayList<std::uint32_t> touchingPairBucketStart_;
    ArrayList<TouchingPair> sortedTouchingPairs_;

    ContactListener* contactListener_ = nullptr;
    BoundingSurfaceHierarchy* bsh_ = nullptr;
};

} // namespace neko
