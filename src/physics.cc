#include "neko/physics/physics.h"

#include <algorithm>
#include <bit>
#include <cassert>
#include <limits>
#include <optional>
#include <type_traits>

#include "neko/core/platform.h"
#include "neko/core/step_profile.h"
#include "neko/math/vec2.h"
#include "neko/physics/contact_solver.h"

#include "neko/core/profile.h"



namespace neko
{
namespace
{
/**
 * @brief FNV-1a over the bytes of each 32-bit word: order-dependent, so equal or swapped fields don't cancel out
 */
class Checksum
{
public:
    void Add(std::uint32_t word) noexcept
    {
        for (int shift = 0; shift < 32; shift += 8)
        {
            hash_ ^= (word >> shift) & 0xFFu;
            hash_ *= prime;
        }
    }
    void Add(Scalar value) noexcept
    {
        static_assert(sizeof(Scalar) == sizeof(std::uint32_t));
        Add(std::bit_cast<std::uint32_t>(value));
    }
    void Add(Vec2f value) noexcept
    {
        Add(value.x);
        Add(value.y);
    }
    void Add(bool value) noexcept { Add(static_cast<std::uint32_t>(value)); }
    template<typename E>
    requires std::is_enum_v<E>
    void Add(E value) noexcept { Add(static_cast<std::uint32_t>(value)); }
    template<typename I>
    requires std::is_same_v<decltype(I::index), Index>
    void Add(I value) noexcept { Add(static_cast<std::uint32_t>(value.index)); }

    [[nodiscard]] std::uint32_t Get() const noexcept { return hash_; }
private:
    static constexpr std::uint32_t offsetBasis = 2166136261u;
    static constexpr std::uint32_t prime = 16777619u;
    std::uint32_t hash_ = offsetBasis;
};

/**
 * @brief Reuses the first free slot, or appends one while under capacity
 * @return the slot index, or std::nullopt when the list is full
 */
template<typename T, typename IsFree>
std::optional<Index> AcquireSlot(ArrayList<T>& list, std::size_t capacity, IsFree isFree)
{
    const auto it = std::find_if(list.begin(), list.end(), isFree);
    if (it != list.end())
    {
        *it = T{};
        return static_cast<Index>(it - list.begin());
    }
    if (list.size() >= capacity)
    {
        return std::nullopt;
    }
    list.emplace_back();
    return static_cast<Index>(list.size() - 1);
}

template<typename T>
ArrayList<T> MakeList(Allocator& allocator, std::size_t capacity)
{
    ArrayList<T> list{ StandardAllocator<T>{allocator} };
    list.reserve(capacity);
    return list;
}

Allocator& ConfigAllocator(const PhysicsWorldConfig& config)
{
    return config.allocator != nullptr ? *config.allocator : DefaultAllocator();
}

constexpr bool IsFreeBody(const Body& body) { return body.type == BodyType::NONE; }
constexpr bool IsFreeCollider(const Collider& collider) { return collider.type == ShapeType::NONE; }
constexpr bool IsFreeCircle(const CircleCollider& circle) { return circle.radius < Scalar{0}; }
constexpr bool IsFreeAabb(const AabbCollider& aabb) { return aabb.halfSize.x < Scalar{0}; }
} // namespace

std::uint32_t GenerateChecksum(const Body& body)
{
    Checksum checksum;
    checksum.Add(body.position);
    checksum.Add(body.velocity);
    checksum.Add(body.force);
    checksum.Add(body.inverseMass);
    checksum.Add(body.type);
    checksum.Add(body.isActive);
    return checksum.Get();
}

std::uint32_t GenerateChecksum(const Collider& collider)
{
    Checksum checksum;
    checksum.Add(collider.offset);
    checksum.Add(collider.restitution);
    checksum.Add(collider.bodyIndex);
    checksum.Add(collider.colliderIndex);
    checksum.Add(collider.shapeIndex);
    checksum.Add(collider.type);
    checksum.Add(collider.isTrigger);
    return checksum.Get();
}

std::uint32_t GenerateChecksum(const CircleCollider& collider)
{
    Checksum checksum;
    checksum.Add(collider.radius);
    return checksum.Get();
}

std::uint32_t GenerateChecksum(const AabbCollider& collider)
{
    Checksum checksum;
    checksum.Add(collider.halfSize);
    return checksum.Get();
}

PhysicsWorld::PhysicsWorld(const PhysicsWorldConfig& config) :
    config_(config),
    defaultBroadphase_(config.maxColliders, config.maxBroadphasePairs, ConfigAllocator(config)),
    bodies_(MakeList<Body>(ConfigAllocator(config), config.maxBodies)),
    aabbs_(MakeList<AabbCollider>(ConfigAllocator(config), config.maxColliders)),
    circles_(MakeList<CircleCollider>(ConfigAllocator(config), config.maxColliders)),
    colliders_(MakeList<Collider>(ConfigAllocator(config), config.maxColliders)),
    touchingPairs_(MakeList<TouchingPair>(ConfigAllocator(config), config.maxTouchingPairs)),
    newTouchingPairs_(MakeList<TouchingPair>(ConfigAllocator(config), config.maxTouchingPairs)),
    colliderAabbCache_(MakeList<Aabbf>(ConfigAllocator(config), config.maxColliders)),
    touchingPairBucketCount_(MakeList<std::uint32_t>(ConfigAllocator(config), config.maxColliders)),
    touchingPairBucketStart_(MakeList<std::uint32_t>(ConfigAllocator(config), config.maxColliders + 1)),
    sortedTouchingPairs_(MakeList<TouchingPair>(ConfigAllocator(config), config.maxTouchingPairs))
{
    assert(config.maxBodies <= std::numeric_limits<Index>::max() && config.maxColliders <= std::numeric_limits<Index>::max());
    // Fixed size from here on: SortNewTouchingPairs indexes these two directly instead of push_back-ing every step
    touchingPairBucketCount_.resize(config.maxColliders);
    touchingPairBucketStart_.resize(config.maxColliders + 1);
}

BodyIndex PhysicsWorld::AddBody()
{
    const auto slot = AcquireSlot(bodies_, config_.maxBodies, IsFreeBody);
    return slot.has_value() ? BodyIndex{ *slot } : INVALID_BODY_INDEX;
}

void PhysicsWorld::RemoveBody(BodyIndex index)
{
    bodies_[index.index].type = BodyType::NONE;
    for (const auto& collider : colliders_)
    {
        if (collider.bodyIndex == index && collider.type != ShapeType::NONE)
        {
            RemoveCollider(collider.colliderIndex);
        }
    }
}

NEKO_HOT bool PhysicsWorld::DetectContact(
    Body& body1, 
    Collider& collider1,
    Body& body2, 
    Collider& collider2,
    Contact* contact)
{
    bool doesIntersect = false;
    contact->bodies[0].body = &body1;
    contact->bodies[1].body = &body2;
    contact->bodies[0].collider = &collider1;
    contact->bodies[1].collider = &collider2;

	contact->restitution = Max(collider1.restitution, collider2.restitution);

    const bool collision = !(collider1.isTrigger || collider2.isTrigger);

    switch (collider1.type)
    {
    case ShapeType::AABB:
        switch (collider2.type)
        {
        case ShapeType::AABB:
        {
            const auto center1 = body1.position + collider1.offset;
            const auto center2 = body2.position + collider2.offset;
            const auto half1 = aabbs_[collider1.shapeIndex.index].halfSize;
            const auto half2 = aabbs_[collider2.shapeIndex.index].halfSize;
            doesIntersect = Intersect(Aabbf::FromCenter(center1, half1), Aabbf::FromCenter(center2, half2));
			if(collision && doesIntersect)
			{
				const auto delta = center1 - center2;
				contact->contactPoint = center1 - delta * Scalar{0.5f};

				const auto penetrationX = half1.x+half2.x-Abs(delta.x);
				const auto penetrationY = half1.y+half2.y-Abs(delta.y);
				if(penetrationX < penetrationY)
				{
					contact->contactNormal = delta.x > neko::Scalar{0} ? Vec2f::right() : Vec2f::left();
					contact->penetration = penetrationX;
				}
				else
				{
					contact->contactNormal = delta.y > neko::Scalar{0} ? Vec2f::up() : Vec2f::down();
					contact->penetration = penetrationY;
				}
			}
            break;
        }
        case ShapeType::CIRCLE:
        {
            const auto center = body1.position + collider1.offset;
            const auto halfSize = aabbs_[collider1.shapeIndex.index].halfSize;
			const Aabbf aabb1 = Aabbf::FromCenter(center, halfSize);
			const Circlef circle = Circlef{ body2.position + collider2.offset, circles_[collider2.shapeIndex.index].radius };
			doesIntersect = Intersect(circle, aabb1);
			if(!collision || !doesIntersect)
			{
				break;
			}
			// The normal points from the circle (body 2) to the box (body 1)
			const auto closestPoint = ClosestPoint(aabb1, circle.position);
			const auto boxToCircle = circle.position - closestPoint;
			if(boxToCircle != Vec2f::zero())
			{
				const auto distance = boxToCircle.Length();
				// One reciprocal and two multiplies instead of Normalized()'s two divisions
				const auto invDistance = Scalar{1} / distance;
				contact->contactNormal = -(boxToCircle * invDistance);
				contact->contactPoint = closestPoint;
				contact->penetration = circle.radius - distance;
				break;
			}
			// The circle center is inside the box: push it out through the nearest face
			const auto delta = circle.position - center;
			const auto distanceX = halfSize.x - Abs(delta.x);
			const auto distanceY = halfSize.y - Abs(delta.y);
			if(distanceX < distanceY)
			{
				const auto outward = delta.x < Scalar{0} ? Vec2f::left() : Vec2f::right();
				contact->contactNormal = -outward;
				contact->contactPoint = { center.x + outward.x * halfSize.x, circle.position.y };
				contact->penetration = circle.radius + distanceX;
			}
			else
			{
				const auto outward = delta.y < Scalar{0} ? Vec2f::down() : Vec2f::up();
				contact->contactNormal = -outward;
				contact->contactPoint = { circle.position.x, center.y + outward.y * halfSize.y };
				contact->penetration = circle.radius + distanceY;
			}
            break;
        }
        default: break;
        }
        break;
    case ShapeType::CIRCLE:
        switch (collider2.type)
        {
        case ShapeType::AABB:
        {
			doesIntersect = DetectContact(body2, collider2, body1, collider1, contact);
            break;
        }
        case ShapeType::CIRCLE:
        {
            const Circlef c1 = { body1.position + collider1.offset, circles_[collider1.shapeIndex.index].radius };
            const Circlef c2 = { body2.position + collider2.offset, circles_[collider2.shapeIndex.index].radius };

            doesIntersect = Intersect(c1, c2);
            if(collision && doesIntersect)
            {
                const auto midline = c1.position - c2.position;
                const auto distance = midline.Length();
                // Same position: any direction separates them, pick a deterministic one. Otherwise one
                // reciprocal and two multiplies instead of dividing both components by distance.
                contact->contactNormal =
                    distance == Scalar{0} ? Vec2f::up() : midline * (Scalar{1} / distance);
                contact->contactPoint = c1.position - midline * Scalar{0.5f};
                contact->penetration = c1.radius + c2.radius - distance;
            }
            break;
        }
        default: break;
        }
        break;
    default: break;
    }
    return doesIntersect;
}

NEKO_HOT void PhysicsWorld::ResolveNarrowphase()
{
    NEKO_PROFILE_FUNCTION();
    newTouchingPairs_.clear();
    for(const auto& pair : Broadphase().GetPossiblePairs())
    {
        auto& collider1 = colliders_[pair.c1.index];
        auto& collider2 = colliders_[pair.c2.index];

        auto& body1 = bodies_[collider1.bodyIndex.index];
        auto& body2 = bodies_[collider2.bodyIndex.index];
		if(!body1.isActive || !body2.isActive)
		{
			continue;
		}
		if(body1.type == BodyType::STATIC && body2.type == BodyType::STATIC)
		{
			continue;
		}
		if(collider1.bodyIndex == collider2.bodyIndex)
		{
			continue;
		}
        const bool isTrigger = collider1.isTrigger || collider2.isTrigger;
        // Contacts only live during this step, so their pointers never outlive the containers
        Contact contact{};
        const auto tDetectStart = StepProfileNow();
        const bool doesIntersect = DetectContact(body1, collider1, body2, collider2, &contact);
        StepProfileMark(StepPhase::DetectContact, tDetectStart);
        if(!doesIntersect)
        {
            continue;
        }
        if(!isTrigger)
        {
            const auto tResolveStart = StepProfileNow();
            contact.Resolve();
            StepProfileMark(StepPhase::Resolve, tResolveStart);
        }
        if(newTouchingPairs_.size() < config_.maxTouchingPairs)
        {
            newTouchingPairs_.push_back({ pair, isTrigger });
        }
        else
        {
            assert(false && "PhysicsWorldConfig::maxTouchingPairs reached");
        }
    }
    const auto tSortStart = StepProfileNow();
    SortNewTouchingPairs();
    NotifyContactChanges();
    StepProfileMark(StepPhase::SortAndNotify, tSortStart);
    std::swap(touchingPairs_, newTouchingPairs_);
}

NEKO_HOT void PhysicsWorld::SortNewTouchingPairs()
{
    std::fill(touchingPairBucketCount_.begin(), touchingPairBucketCount_.end(), std::uint32_t{ 0 });
    for (const auto& touchingPair : newTouchingPairs_)
    {
        ++touchingPairBucketCount_[touchingPair.pair.c1.index];
    }

    // Exclusive prefix sum: bucketStart[k] is where collider k's pairs begin in the sorted output
    std::uint32_t runningTotal = 0;
    for (std::size_t k = 0; k < touchingPairBucketCount_.size(); ++k)
    {
        touchingPairBucketStart_[k] = runningTotal;
        runningTotal += touchingPairBucketCount_[k];
    }
    touchingPairBucketStart_.back() = runningTotal;

    sortedTouchingPairs_.resize(newTouchingPairs_.size());
    // Reuse the counts as write cursors, seeded from the bucket starts just computed
    std::copy(touchingPairBucketStart_.begin(), touchingPairBucketStart_.end() - 1, touchingPairBucketCount_.begin());
    for (const auto& touchingPair : newTouchingPairs_)
    {
        sortedTouchingPairs_[touchingPairBucketCount_[touchingPair.pair.c1.index]++] = touchingPair;
    }

    // Each bucket only holds one collider's contacts (typically a handful): insertion sort by c2 finishes the order
    for (std::size_t k = 0; k + 1 < touchingPairBucketStart_.size(); ++k)
    {
        if (touchingPairBucketStart_[k] == touchingPairBucketStart_[k + 1])
        {
            continue;
        }
        const auto bucketBegin = sortedTouchingPairs_.begin() + static_cast<std::ptrdiff_t>(touchingPairBucketStart_[k]);
        const auto bucketEnd = sortedTouchingPairs_.begin() + static_cast<std::ptrdiff_t>(touchingPairBucketStart_[k + 1]);
        for (auto it = bucketBegin + 1; it < bucketEnd; ++it)
        {
            const TouchingPair value = *it;
            auto insertAt = it;
            while (insertAt > bucketBegin && value.pair.c2 < (insertAt - 1)->pair.c2)
            {
                *insertAt = *(insertAt - 1);
                --insertAt;
            }
            *insertAt = value;
        }
    }
    std::swap(newTouchingPairs_, sortedTouchingPairs_);
}

NEKO_HOT void PhysicsWorld::NotifyContactChanges() const
{
    if(contactListener_ == nullptr)
    {
        return;
    }
    const auto enter = [this](const TouchingPair& touchingPair)
    {
        if(touchingPair.isTrigger)
        {
            contactListener_->OnTriggerEnter(touchingPair.pair);
        }
        else
        {
            contactListener_->OnCollisionEnter(touchingPair.pair);
        }
    };
    const auto exit = [this](const TouchingPair& touchingPair)
    {
        if(touchingPair.isTrigger)
        {
            contactListener_->OnTriggerExit(touchingPair.pair);
        }
        else
        {
            contactListener_->OnCollisionExit(touchingPair.pair);
        }
    };
    // Both lists are sorted: merging them gives the events in a deterministic order
    auto oldIt = touchingPairs_.begin();
    auto newIt = newTouchingPairs_.begin();
    while(oldIt != touchingPairs_.end() || newIt != newTouchingPairs_.end())
    {
        if(newIt == newTouchingPairs_.end() || (oldIt != touchingPairs_.end() && oldIt->pair < newIt->pair))
        {
            exit(*oldIt);
            ++oldIt;
        }
        else if(oldIt == touchingPairs_.end() || newIt->pair < oldIt->pair)
        {
            enter(*newIt);
            ++newIt;
        }
        else
        {
            // Still touching: a trigger that became solid (or the reverse) ends one kind of contact and starts the other
            if(oldIt->isTrigger != newIt->isTrigger)
            {
                exit(*oldIt);
                enter(*newIt);
            }
            ++oldIt;
            ++newIt;
        }
    }
}

ColliderIndex PhysicsWorld::AddCircleCollider(BodyIndex body)
{
    const auto colliderSlot = AcquireSlot(colliders_, config_.maxColliders, IsFreeCollider);
    const auto shapeSlot = colliderSlot.has_value() ? AcquireSlot(circles_, config_.maxColliders, IsFreeCircle) : std::nullopt;
    if(!colliderSlot.has_value() || !shapeSlot.has_value())
    {
        return INVALID_COLLIDER_INDEX;
    }
    circles_[*shapeSlot].radius = Scalar{ 0 };
    auto& collider = colliders_[*colliderSlot];
    collider.bodyIndex = body;
    collider.colliderIndex = { *colliderSlot };
    collider.shapeIndex = { *shapeSlot };
    collider.type = ShapeType::CIRCLE;
    return collider.colliderIndex;
}

ColliderIndex PhysicsWorld::AddAabbCollider(BodyIndex body)
{
    const auto colliderSlot = AcquireSlot(colliders_, config_.maxColliders, IsFreeCollider);
    const auto shapeSlot = colliderSlot.has_value() ? AcquireSlot(aabbs_, config_.maxColliders, IsFreeAabb) : std::nullopt;
    if(!colliderSlot.has_value() || !shapeSlot.has_value())
    {
        return INVALID_COLLIDER_INDEX;
    }
    aabbs_[*shapeSlot].halfSize = Vec2f::zero();
    auto& collider = colliders_[*colliderSlot];
    collider.bodyIndex = body;
    collider.colliderIndex = { *colliderSlot };
    collider.shapeIndex = { *shapeSlot };
    collider.type = ShapeType::AABB;
    return collider.colliderIndex;
}

void PhysicsWorld::RemoveCollider(ColliderIndex index)
{
    auto& collider = colliders_[index.index];
    switch (collider.type)
    {
    case ShapeType::AABB:
        aabbs_[collider.shapeIndex.index] = AabbCollider{};
        break;
    case ShapeType::CIRCLE:
        circles_[collider.shapeIndex.index] = CircleCollider{};
        break;
    default:
        break;
    }
    collider = Collider{};
}

NEKO_HOT void PhysicsWorld::Step()
{
    NEKO_PROFILE_FUNCTION();
    const auto dt = config_.dt;

    const auto tIntegrationStart = StepProfileNow();
    for (auto& body : bodies_)
    {
		if(!body.isActive)
		{
			continue;
		}
        switch (body.type)
        {
        case BodyType::DYNAMIC:
        {
            // Gravity is an acceleration: no division by the inverse mass
            body.velocity += (config_.gravity + body.force * body.inverseMass) * dt;

            body.position += body.velocity * dt;
            body.force = Vec2f::zero();
            break;
        }
        case BodyType::KINEMATIC:
        {
            body.position += body.velocity * dt;
            break;
        }
        default: break;
        }

    }
    StepProfileMark(StepPhase::Integration, tIntegrationStart);

    ResolveBroadphase();
    ResolveNarrowphase();
}

void PhysicsWorld::Clear()
{
    bodies_.clear();
    aabbs_.clear();
    circles_.clear();
    colliders_.clear();
    touchingPairs_.clear();
    newTouchingPairs_.clear();
}

NEKO_HOT Aabbf PhysicsWorld::GetAabb(const Collider& collider) const
{
    const auto center = bodies_[collider.bodyIndex.index].position + collider.offset;
    if (collider.type == ShapeType::AABB)
    {
        return Aabbf::FromCenter(center, aabbs_[collider.shapeIndex.index].halfSize);
    }
    return Circlef{ center, circles_[collider.shapeIndex.index].radius }.GetAabb();
}

NEKO_HOT void PhysicsWorld::ResolveBroadphase()
{
    NEKO_PROFILE_FUNCTION();
    const auto tBroadphaseStart = StepProfileNow();
    auto& broadphase = Broadphase();
    broadphase.Clear();
    colliderAabbCache_.clear();

    const auto isInserted = [this](const Collider& collider)
    {
        return collider.type != ShapeType::NONE && bodies_[collider.bodyIndex.index].isActive;
    };
    // World box from the collider extents, starting from the first one (no infinite bounds to overflow). Each
    // Aabbf is computed once here and cached, since a BSH may need SetWorldAabb called before any Insert.
    bool hasCollider = false;
    Aabbf worldBox{};
    for (const auto& collider : colliders_)
    {
        if (!isInserted(collider))
        {
            continue;
        }
        const auto aabb = GetAabb(collider);
        colliderAabbCache_.push_back(aabb);
        if (!hasCollider)
        {
            worldBox = aabb;
            hasCollider = true;
            continue;
        }
        worldBox.minBound = { Min(worldBox.minBound.x, aabb.minBound.x), Min(worldBox.minBound.y, aabb.minBound.y) };
        worldBox.maxBound = { Max(worldBox.maxBound.x, aabb.maxBound.x), Max(worldBox.maxBound.y, aabb.maxBound.y) };
    }
    if (hasCollider)
    {
        broadphase.SetWorldAabb(worldBox);
        std::size_t cacheIndex = 0;
        for (const auto& collider : colliders_)
        {
            if (isInserted(collider))
            {
                broadphase.Insert({ colliderAabbCache_[cacheIndex], collider.colliderIndex });
                ++cacheIndex;
            }
        }
    }
    broadphase.CalculatePairs();
    StepProfileMark(StepPhase::Broadphase, tBroadphaseStart);
}

void PhysicsWorld::CopyFrom(const PhysicsWorld& physicsWorld)
{
    // Assignment reuses the reserved capacity: no allocation when both worlds share a config
	bodies_ = physicsWorld.bodies_;
	aabbs_ = physicsWorld.aabbs_;
	circles_ = physicsWorld.circles_;
	colliders_ = physicsWorld.colliders_;
	touchingPairs_ = physicsWorld.touchingPairs_;
}

std::uint32_t PhysicsWorld::GenerateChecksum() const
{
    Checksum checksum;
    for (const auto& body : bodies_)
    {
        checksum.Add(neko::GenerateChecksum(body));
    }
    for (const auto& collider : colliders_)
    {
        checksum.Add(neko::GenerateChecksum(collider));
    }
    for (const auto& aabb : aabbs_)
    {
        checksum.Add(neko::GenerateChecksum(aabb));
    }
    for (const auto& circle : circles_)
    {
        checksum.Add(neko::GenerateChecksum(circle));
    }
    for (const auto& touchingPair : touchingPairs_)
    {
        checksum.Add(touchingPair.pair.c1);
        checksum.Add(touchingPair.pair.c2);
        checksum.Add(touchingPair.isTrigger);
    }
    return checksum.Get();
}
} // namespace neko
