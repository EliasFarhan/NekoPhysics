#include "physics/physics.h"

#include <iostream>

#include "math/vec2.h"
#include "physics/contact_solver.h"

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif



namespace neko
{
std::uint32_t GenerateChecksum(const Body& body)
{
    std::uint32_t result = 0;
    result ^= *(reinterpret_cast<const uint32_t*>(&body.position.x));
    result ^= *(reinterpret_cast<const uint32_t*>(&body.position.y));
    result ^= *(reinterpret_cast<const uint32_t*>(&body.velocity.x));
    result ^= *(reinterpret_cast<const uint32_t*>(&body.velocity.y));
    result ^= *(reinterpret_cast<const uint32_t*>(&body.force.x));
    result ^= *(reinterpret_cast<const uint32_t*>(&body.force.y));
    result ^= *(reinterpret_cast<const uint32_t*>(&body.inverseMass));
    result ^= static_cast<std::uint32_t>(body.type);
    return result;
}

std::uint32_t GenerateChecksum(const Collider& collider)
{
    std::uint32_t result = 0;
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.offset.x);
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.offset.y);
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.restitution);
    result ^= std::bit_cast<std::uint32_t>(collider.bodyIndex.index());
    result ^= std::bit_cast<std::uint32_t>(collider.bodyIndex.generationIndex());
    result ^= std::bit_cast<std::uint32_t>(collider.colliderIndex.index());
    result ^= std::bit_cast<std::uint32_t>(collider.colliderIndex.generationIndex());
    result ^= std::bit_cast<std::uint32_t>(collider.shapeIndex.index());
    result ^= std::bit_cast<std::uint32_t>(collider.shapeIndex.generationIndex());
    result ^= static_cast<std::uint32_t>(collider.type);
    result ^= static_cast<std::uint32_t>(collider.isTrigger);
    return result;
}

std::uint32_t GenerateChecksum(const CircleCollider& collider)
{
    std::uint32_t result = 0;
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.radius);
    return result;
}

std::uint32_t GenerateChecksum(const AabbCollider& collider)
{
    std::uint32_t result = 0;
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.halfSize.x);
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.halfSize.y);
    return result;
}

std::uint32_t GenerateChecksum(const PlaneCollider& collider)
{
    std::uint32_t result = 0;
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.normal.x);
    result ^= *reinterpret_cast<const std::uint32_t*>(&collider.normal.y);
    return result;
}

PhysicsWorld::PhysicsWorld(Vec2f gravity): gravity_(gravity)
{
}

BodyIndex PhysicsWorld::AddBody()
{
    return bodyManager_.add();
}

void PhysicsWorld::RemoveBody(BodyIndex index)
{
    bodyManager_.remove(index);
}

bool PhysicsWorld::DetectContact(
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
            const Aabbf aabb1 = Aabbf::FromCenter(body1.position + collider1.offset, aabbs_[AabbIndex{collider1.shapeIndex.index(), collider1.shapeIndex.generationIndex()}].halfSize);
            const Aabbf aabb2 = Aabbf::FromCenter(body2.position + collider2.offset, aabbs_[AabbIndex{collider2.shapeIndex.index(), collider2.shapeIndex.generationIndex()}].halfSize);
            doesIntersect = Intersect(aabb1, aabb2);
			if(collision)
			{
				const auto delta = aabb1.GetCenter()-aabb2.GetCenter();
				contact->contactPoint = aabb1.GetCenter()+delta*neko::Scalar {0.5f};
				const auto half1 = aabb1.GetHalfSize();
				const auto half2 = aabb2.GetHalfSize();

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
			const Aabbf aabb1 = Aabbf::FromCenter(body1.position + collider1.offset, aabbs_[AabbIndex{collider1.shapeIndex.index(), collider1.shapeIndex.generationIndex()}].halfSize);
			const Circlef circle = Circlef{ body2.position + collider2.offset, circles_[CircleIndex{collider2.shapeIndex.index(), collider2.shapeIndex.generationIndex()}].radius };
			if(collision)
			{

				const auto delta = circle.position - aabb1.GetCenter();
				const auto halfSize = aabb1.GetHalfSize();

				Vec2f closestPoint = Vec2f::Clamp(delta, -halfSize, halfSize);

				Scalar dist = (closestPoint - delta).Length();
				doesIntersect = dist <= circle.radius;
				if (!doesIntersect)
				{
					break;
				}
				const auto closestPointWorld = aabb1.GetCenter() + closestPoint;
				auto circleToRect = (circle.position - closestPointWorld);
				if(circleToRect.Length() < Scalar{0.001f})
				{
					circleToRect = Vec2f::up();
				}
				contact->contactNormal = -circleToRect.Normalized();
				contact->contactPoint = closestPointWorld;
				contact->penetration = circle.radius - dist;

			}
			else
			{
				doesIntersect = Intersect(aabb1, circle);
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
            const Circlef c1 = { body1.position + collider1.offset, circles_[CircleIndex{collider1.shapeIndex.index(), collider1.shapeIndex.generationIndex()}].radius };
            const Circlef c2 = { body2.position + collider2.offset, circles_[CircleIndex{collider2.shapeIndex.index(), collider2.shapeIndex.generationIndex()}].radius };

            if(collision)
            {
                const auto midline = c1.position - c2.position;
                const auto size = midline.Length();
                if(size >= c1.radius+c2.radius)
                {
                    doesIntersect = false;
                    break;
                }
                doesIntersect = true;
                contact->contactNormal = midline / size;
                contact->contactPoint = c1.position + midline * Scalar {0.5f};
                contact->penetration = c1.radius + c2.radius - size;
            }
            else
            {
                doesIntersect = Intersect(c1, c2);
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

void PhysicsWorld::ResolveNarrowphase(Scalar dt)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    if(contactListener_ == nullptr || bsh_ == nullptr)
    {
        return;
    }
    const auto& newPossiblePairs = bsh_->GetPossiblePairs();
    ArrayList<std::pair<ColliderPair, std::optional<Contact>>> newPairs{
        StandardAllocator<std::pair<ColliderPair, std::optional<Contact>>>{heapAllocator_} };
    ArrayList<ColliderPair> removePairs{ StandardAllocator<ColliderPair>{heapAllocator_} };
    for(const auto& newColliderPair : newPossiblePairs)
    {
#ifdef TRACY_ENABLE
        ZoneNamedN(newColliderPairCheck, "Check New Collider Pair", true);
#endif
        auto& collider1 = colliderManager_[newColliderPair.c1];
        auto& collider2 = colliderManager_[newColliderPair.c2];

        auto& body1 = bodyManager_[collider1.bodyIndex];
        auto& body2 = bodyManager_[collider2.bodyIndex];
		if(body1.IsInvalid() || body2.IsInvalid())
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
        const auto it = manifold_.find(newColliderPair);


        if (it != manifold_.end())
        {
			Contact newContact{};
			const bool doesIntersect = DetectContact(body1, collider1, body2, collider2, collider1.isTrigger || collider2.isTrigger? &newContact : &it->second.value());
			if (!doesIntersect)
            {
                if (collider1.isTrigger || collider2.isTrigger)
                {
                    contactListener_->OnTriggerExit(newColliderPair);
                }
                else
                {
                    contactListener_->OnCollisionExit(newColliderPair);
                }
                //manifold_.erase(it);
                removePairs.push_back(it->first);
            }
			else
            {
                if(!collider1.isTrigger && !collider2.isTrigger)
                {
                    it->second->Resolve(dt);
                }
            }
        }
        else
        {
			Contact newContact{};
			const bool doesIntersect = DetectContact(body1, collider1, body2, collider2, &newContact);

			if (doesIntersect)
            {
                if (collider1.isTrigger || collider2.isTrigger)
                {
                    contactListener_->OnTriggerEnter(newColliderPair);
					newPairs.emplace_back(newColliderPair, std::nullopt);
                }
                else
                {
                    newContact.Resolve(dt);
                    contactListener_->OnCollisionEnter(newColliderPair);
					newPairs.emplace_back(newColliderPair, newContact);
                }
                //manifold_.insert(newColliderPair);
            }
        }
    }
    for (auto pair : removePairs)
    {
        manifold_.erase(pair);
    }
    manifold_.insert(newPairs.begin(), newPairs.end());
}

ColliderIndex PhysicsWorld::AddCircleCollider(BodyIndex body)
{
    auto index = colliderManager_.add();
    auto& collider = colliderManager_.at(index);
    collider.bodyIndex = body;
    collider.colliderIndex = index;
    collider.type = ShapeType::CIRCLE;

    auto circleIndex = circles_.add();
    collider.shapeIndex = ShapeIndex{circleIndex};
    return collider.colliderIndex;
}

ColliderIndex PhysicsWorld::AddAabbCollider(BodyIndex body)
{
    auto index = colliderManager_.add();
    auto& collider = colliderManager_.at(index);
    collider.bodyIndex = body;
    collider.colliderIndex = index;
    collider.type = ShapeType::AABB;

    auto aabbIndex = aabbs_.add();

    collider.shapeIndex = ShapeIndex{ aabbIndex };
    return collider.colliderIndex;
}

ColliderIndex PhysicsWorld::AddPlaneCollider(BodyIndex body)
{
    auto index = colliderManager_.add();
    auto& collider = colliderManager_.at(index);
    collider.bodyIndex = body;
    collider.colliderIndex = index;
    collider.type = ShapeType::PLANE;

    auto planeIndex = planes_.add();

    collider.shapeIndex = ShapeIndex{ planeIndex };
    return collider.colliderIndex;
}

void PhysicsWorld::RemoveAabbCollider(ColliderIndex index)
{
    auto& collider = colliderManager_.at(index);
   aabbs_.remove(AabbIndex{collider.shapeIndex.index(), collider.shapeIndex.generationIndex()});

    collider.type = ShapeType::NONE;
}

void PhysicsWorld::RemoveCircleCollider(ColliderIndex index)
{
    auto& collider = colliderManager_.at(index);
    circles_.remove(CircleIndex{collider.shapeIndex.index(), collider.shapeIndex.generationIndex()});
    collider.type = ShapeType::NONE;
}

void PhysicsWorld::RemovePlaneCollider(ColliderIndex index)
{
    auto& collider = colliderManager_.at(index);
    planes_.remove(PlaneIndex{collider.shapeIndex.index(), collider.shapeIndex.generationIndex()});

    collider.type = ShapeType::NONE;
}

void PhysicsWorld::Step(Scalar dt)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif

    for (auto& body : bodyManager_)
    {
		if(body.IsInvalid())
		{
			continue;
		}
        switch (body.type)
        {
        case BodyType::DYNAMIC:
        {
            body.force += gravity_ / body.inverseMass;

            body.velocity += body.force * body.inverseMass * dt;

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

    ResolveBroadphase();
    ResolveNarrowphase(dt);

    
}

void PhysicsWorld::Clear()
{
    bodyManager_.clear();
    aabbs_.clear();
    circles_.clear();
}

void PhysicsWorld::ResolveBroadphase()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    if (contactListener_ == nullptr || bsh_ == nullptr)
    {
        return;
    }
    bsh_->Clear();

    Aabbf worldBox{
        {std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max()},
        {std::numeric_limits<Scalar>::lowest(), std::numeric_limits<Scalar>::lowest()} };
    for(auto& collider: colliderManager_)
    {
        switch (collider.type)
        {
        case ShapeType::AABB:
        case ShapeType::CIRCLE:
        case ShapeType::PLANE:
        {
            const auto center = body(collider.bodyIndex).position + collider.offset;
            if (worldBox.minBound.x > center.x)
            {
                worldBox.minBound.x = center.x;
            }
            if (worldBox.maxBound.x < center.x)
            {
                worldBox.maxBound.x = center.x;
            }
            if (worldBox.minBound.y > center.y)
            {
                worldBox.minBound.y = center.y;
            }
            if (worldBox.maxBound.y < center.y)
            {
                worldBox.maxBound.y = center.y;
            }
            break;
        }
        case ShapeType::NONE: 
            continue;
        default: 
            break;
        }

    }
    bsh_->SetWorldAabb(worldBox);

    for(const auto& collider : colliderManager_)
    {
		if(bodyManager_[collider.bodyIndex].IsInvalid())
		{
			continue;
		}
        switch(collider.type)
        {
        case ShapeType::AABB:
        {
            const auto aabbCollider = Aabbf::FromCenter(
                bodyManager_[collider.bodyIndex].position + collider.offset,
                aabbs_[AabbIndex{collider.shapeIndex.index(), collider.shapeIndex.generationIndex()}].halfSize);
            bsh_->Insert({ aabbCollider, collider.colliderIndex });
            break;
        }
        case ShapeType::CIRCLE:
        {
            const auto circleCollider = Circlef{
                bodyManager_[collider.bodyIndex].position+collider.offset,
                circles_[CircleIndex{collider.shapeIndex.index(), collider.shapeIndex.generationIndex()}].radius};
            bsh_->Insert({ circleCollider.GetAabb(), collider.colliderIndex });
            break;
        }
        default: 
            break;
        }
    }
    bsh_->CalculatePairs();

}

void PhysicsWorld::CopyFrom(const PhysicsWorld& physicsWorld)
{
	bodyManager_ = physicsWorld.bodyManager_;
	aabbs_ = physicsWorld.aabbs_;
	circles_ = physicsWorld.circles_;
	planes_ = physicsWorld.planes_;
	colliderManager_ = physicsWorld.colliderManager_;
	manifold_ = physicsWorld.manifold_;
}
} // namespace neko
