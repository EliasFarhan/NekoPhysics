#include "physics.h"

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif

#include <ranges>

namespace neko
{
PhysicsWorld::PhysicsWorld(Vec2f gravity): gravity_(gravity)
{
}

BodyIndex PhysicsWorld::AddBody()
{
    const auto it = std::ranges::find_if(bodies_, [](const auto& body)
    {
        return body.type == BodyType::NONE;
    });
    if(it != bodies_.end())
    {
        it->type = BodyType::DYNAMIC;
        return { static_cast<int>(std::distance(bodies_.begin(), it)) };
    }
    const BodyIndex index{ static_cast<int>(bodies_.size())};
    bodies_.push_back({});
    return index;
}

void PhysicsWorld::RemoveBody(BodyIndex index)
{
    bodies_[index.index].type = BodyType::NONE;
    //remove all colliders attached to this body
    auto it = std::ranges::find_if(colliders_, [&index](const auto& collider)
        {
            return collider.bodyIndex == index;
        });
    while(it != colliders_.end())
    {
        if(it->type == ColliderType::AABB)
        {
            RemoveAabbCollider(it->colliderIndex);
        }
        else if(it->type == ColliderType::CIRCLE)
        {
            RemoveCircleCollider(it->colliderIndex);
        }
        it = std::ranges::find_if(colliders_, [&index](const auto& collider)
            {
                return collider.bodyIndex == index;
            });
    }
}

void PhysicsWorld::ResolveTriggers()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    if(contactListener_ == nullptr)
    {
        return;
    }
    const auto& pairs = quadTree_.GetPossiblePairs();
    for(const auto& triggerPair : pairs)
    {
        const auto& collider = colliders_[triggerPair.c1.index];
        const auto& otherCollider = colliders_[triggerPair.c2.index];
        auto it = triggerPairs_.find(triggerPair);
        bool doesIntersect = false;
        switch (collider.type)
        {
        case ColliderType::AABB:
            switch (otherCollider.type)
            {
            case ColliderType::AABB:
            {
                const auto& body1 = bodies_[collider.bodyIndex.index];
                const auto& body2 = bodies_[otherCollider.bodyIndex.index];
                const Aabbf aabb1 = Aabbf::FromCenter(body1.position + collider.offset, aabbs_[collider.colliderIndex.index].halfSize);
                const Aabbf aabb2 = Aabbf::FromCenter(body2.position + otherCollider.offset, aabbs_[otherCollider.colliderIndex.index].halfSize);
                doesIntersect = Intersect(aabb1, aabb2);
                break;
            }
            case ColliderType::CIRCLE:
            {
                const auto& body1 = bodies_[collider.bodyIndex.index];
                const auto& body2 = bodies_[otherCollider.bodyIndex.index];
                const Aabbf aabb1 = Aabbf::FromCenter(body1.position + collider.offset, aabbs_[collider.colliderIndex.index].halfSize);
                const Circle circle = Circle{ body2.position + otherCollider.offset, circles_[otherCollider.colliderIndex.index].radius };
                doesIntersect = Intersect(aabb1, circle);
                break;
            }
            default: break;
            }
            break;
        case ColliderType::CIRCLE:
            switch (otherCollider.type)
            {
            case ColliderType::AABB:
            {
                const auto& body1 = bodies_[collider.bodyIndex.index];
                const auto& body2 = bodies_[otherCollider.bodyIndex.index];
                const Circlef c1 = { body1.position + collider.offset, circles_[collider.colliderIndex.index].radius };
                const Aabbf aabb2 = Aabbf::FromCenter( body2.position + otherCollider.offset, aabbs_[otherCollider.colliderIndex.index].halfSize);
                doesIntersect = Intersect(c1, aabb2);
                break;
            }
            case ColliderType::CIRCLE:
            {
                const auto& body1 = bodies_[collider.bodyIndex.index];
                const auto& body2 = bodies_[otherCollider.bodyIndex.index];
                const Circlef c1 = { body1.position + collider.offset, circles_[collider.colliderIndex.index].radius };
                const Circlef c2 = { body2.position + otherCollider.offset, circles_[otherCollider.colliderIndex.index].radius };
                doesIntersect = Intersect(c1, c2);
                break;
            }
            default: break;
            }
            break;
        default: break;
        }

        if (it != triggerPairs_.end())
        {
            if (!doesIntersect)
            {
                contactListener_->OnTriggerExit(triggerPair);
                //End Trigger
                triggerPairs_.erase(it);
            }
        }
        else
        {
            if (doesIntersect)
            {
                //Begin Trigger
                contactListener_->OnTriggerEnter(triggerPair);
                triggerPairs_.insert(triggerPair);
            }
        }
    }

}

ColliderIndex PhysicsWorld::AddCircleCollider(BodyIndex body)
{
    const auto it = std::ranges::find_if(colliders_, [](const Collider& collider)
        {
            return collider.type == ColliderType::NONE;
        });
    int index = -1;
    if (it != colliders_.end())
    {
        it->type = ColliderType::CIRCLE;
        index = static_cast<int>(std::distance(colliders_.begin(), it));
    }
    else
    {
        index = static_cast<int>(colliders_.size());
        colliders_.push_back({});
    }
    auto& collider = colliders_[index];
    collider.bodyIndex = body;
    collider.colliderIndex.index = index;
    collider.type = ColliderType::CIRCLE;

    const auto circleIt = std::ranges::find_if(circles_, [](const auto& circle)
    {
        return circle.radius < 0.0f;
    });
    int shapeIndex;
    if(circleIt != circles_.end())
    {
        circleIt->radius = 0.0f;
        shapeIndex = static_cast<int>(std::distance(circles_.begin(), circleIt));
    }
    else
    {
        shapeIndex = static_cast<int>(circles_.size());
        circles_.push_back({});
    }
    collider.shapeIndex = { shapeIndex };
    return collider.colliderIndex;
}

ColliderIndex PhysicsWorld::AddAabbCollider(BodyIndex body)
{
    const auto it = std::ranges::find_if(colliders_, [](const Collider& collider)
        {
            return collider.type == ColliderType::NONE;
        });
    int index = -1;
    if (it != colliders_.end())
    {
        it->type = ColliderType::AABB;
        index = static_cast<int>(std::distance(colliders_.begin(), it));
    }
    else
    {
        index = static_cast<int>(colliders_.size());
        colliders_.push_back({});
    }
    auto& collider = colliders_[index];
    collider.bodyIndex = body;
    collider.colliderIndex.index = index;
    collider.type = ColliderType::AABB;

    const auto aabbIt = std::ranges::find_if(aabbs_, [](const auto& aabb)
        {
            return aabb.halfSize.x < -1.0f;
        });
    int shapeIndex;
    if (aabbIt != aabbs_.end())
    {
        aabbIt->halfSize.x = 0.0f;
        shapeIndex = static_cast<int>(std::distance(aabbs_.begin(), aabbIt));
    }
    else
    {
        shapeIndex = static_cast<int>(aabbs_.size());
        aabbs_.push_back({});
    }
    collider.shapeIndex = { shapeIndex };
    return collider.colliderIndex;
}

void PhysicsWorld::RemoveAabbCollider(ColliderIndex index)
{
    auto& collider = colliders_[index.index];
    auto& [halfSize] = aabbs_[collider.shapeIndex.index];
    halfSize = {-1,-1};

    collider.type = ColliderType::NONE;
}

void PhysicsWorld::RemoveCircleCollider(ColliderIndex index)
{
    auto& collider = colliders_[index.index];
    auto& circle = circles_[collider.shapeIndex.index];
    circle.radius = -1.0f;

    collider.type = ColliderType::NONE;
}

void PhysicsWorld::Step(Scalar dt)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    ResolveBroadphase();
    ResolveTriggers();

    for (auto& body : bodies_)
    {
        switch (body.type)
        {
        case BodyType::DYNAMIC:
        {
            body.force += body.mass * gravity_;

            body.velocity += body.force / body.mass * dt;

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
}

void PhysicsWorld::Clear()
{
    bodies_.clear();
    aabbs_.clear();
    circles_.clear();
}

void PhysicsWorld::ResolveBroadphase()
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif
    if (contactListener_ == nullptr)
    {
        return;
    }
    quadTree_.Clear();

    Aabbf worldBox{
        {std::numeric_limits<Scalar>::max(), std::numeric_limits<Scalar>::max()},
        {std::numeric_limits<Scalar>::lowest(), std::numeric_limits<Scalar>::lowest()} };
    for(auto& collider: colliders_)
    {
        switch (collider.type)
        {
        case ColliderType::AABB:
        case ColliderType::CIRCLE:
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
        case ColliderType::NONE: 
            continue;
        default: 
            break;
        }

    }
    quadTree_.SetWorldAabb(worldBox);

    for(auto& collider : colliders_)
    {
        switch(collider.type)
        {
        case ColliderType::AABB:
        {
            const auto aabbCollider = Aabbf::FromCenter(
                bodies_[collider.bodyIndex.index].position + collider.offset, 
                aabbs_[collider.shapeIndex.index].halfSize);
            quadTree_.Insert({ aabbCollider, collider.colliderIndex });
            break;
        }
        case ColliderType::CIRCLE:
        {
            const auto circleCollider = Circlef{
                bodies_[collider.bodyIndex.index].position+collider.offset,
                circles_[collider.shapeIndex.index].radius};
            quadTree_.Insert({ circleCollider.GetAabb(), collider.colliderIndex });
            break;
        }
        default: 
            break;
        }
    }
    quadTree_.CalculatePairs();

}
} // namespace neko
