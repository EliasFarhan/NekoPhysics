#include "physics.h"

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
        if(it->colliderIndex.type == ColliderType::AABB)
        {
            RemoveAabbCollider(it->colliderIndex);
        }
        else if(it->colliderIndex.type == ColliderType::CIRCLE)
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
    if(contactListener_ == nullptr)
    {
        return;
    }
    for(std::size_t i = 0; i < colliders_.size(); i++)
    {
        const auto& collider = colliders_[i];
        if(collider.colliderIndex.type == ColliderType::NONE)
            continue;
        for(std::size_t j = i+1; j < colliders_.size(); j++)
        {
            const auto& otherCollider = colliders_[j];
            if(otherCollider.colliderIndex.type == ColliderType::NONE)
                continue;
            
            const TriggerPair triggerPair{
                .b1 = collider.bodyIndex,
                .c1 = collider.colliderIndex,
                .b2 = otherCollider.bodyIndex,
                .c2 = otherCollider.colliderIndex
            };
            auto it = triggerPairs_.find(triggerPair);
            bool doesIntersect = false;
            switch (collider.colliderIndex.type)
            {
            case ColliderType::AABB:
                switch (otherCollider.colliderIndex.type)
                {
                case ColliderType::AABB: 
                {
                    const auto& body1 = bodies_[collider.bodyIndex.index];
                    const auto& body2 = bodies_[otherCollider.bodyIndex.index];
                    const Aabbf aabb1 = Aabbf::FromCenter(body1.position + collider.offset, aabbs_[collider.colliderIndex.index].GetHalfSize());
                    const Aabbf aabb2 = Aabbf::FromCenter(body2.position + otherCollider.offset, aabbs_[otherCollider.colliderIndex.index].GetHalfSize());
                    doesIntersect = Aabbf::Intersect(aabb1, aabb2);
                    break;
                }
                case ColliderType::CIRCLE: break;
                default: break;
                }
                break;
            case ColliderType::CIRCLE: 
                switch (otherCollider.colliderIndex.type)
                {
                case ColliderType::AABB: break;
                case ColliderType::CIRCLE:
                {
                    const auto& body1 = bodies_[collider.bodyIndex.index];
                    const auto& body2 = bodies_[otherCollider.bodyIndex.index];
                    const Circlef c1 = { body1.position + collider.offset, circles_[collider.colliderIndex.index].radius };
                    const Circlef c2 = { body2.position + otherCollider.offset, circles_[otherCollider.colliderIndex.index].radius };
                    doesIntersect = Circlef::Intersect(c1, c2);
                    break;
                }
                default: break;
                }
                break;
            default: break;
            }

            if(it != triggerPairs_.end())
            {
                if(!doesIntersect)
                {
                    contactListener_->OnTriggerExit(triggerPair);
                    //End Trigger
                    triggerPairs_.erase(it);
                }
            }
            else
            {
                if(doesIntersect)
                {
                    //Begin Trigger
                    contactListener_->OnTriggerEnter(triggerPair);
                    triggerPairs_.insert(triggerPair);
                }
            }

        }
    }

}

ColliderIndex PhysicsWorld::AddCircleCollider(BodyIndex body)
{
    const auto it = std::ranges::find_if(colliders_, [](const Collider& collider)
        {
            return collider.colliderIndex.type == ColliderType::NONE;
        });
    int index = -1;
    if (it != colliders_.end())
    {
        it->colliderIndex.type = ColliderType::CIRCLE;
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
    collider.colliderIndex.type = ColliderType::CIRCLE;

    const auto circleIt = std::ranges::find_if(circles_, [](const Circlef& circle)
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
    collider.colliderIndex.shapeIndex = { shapeIndex };
    return collider.colliderIndex;
}

ColliderIndex PhysicsWorld::AddAabbCollider(BodyIndex body)
{
    const auto it = std::ranges::find_if(colliders_, [](const Collider& collider)
        {
            return collider.colliderIndex.type == ColliderType::NONE;
        });
    int index = -1;
    if (it != colliders_.end())
    {
        it->colliderIndex.type = ColliderType::AABB;
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
    collider.colliderIndex.type = ColliderType::AABB;

    const auto aabbIt = std::ranges::find_if(aabbs_, [](const Aabbf& aabb)
        {
            return aabb.maxBound.x < aabb.minBound.x;
        });
    int shapeIndex;
    if (aabbIt != aabbs_.end())
    {
        aabbIt->maxBound.x = aabbIt->minBound.x;
        shapeIndex = static_cast<int>(std::distance(aabbs_.begin(), aabbIt));
    }
    else
    {
        shapeIndex = static_cast<int>(aabbs_.size());
        aabbs_.push_back({});
    }
    collider.colliderIndex.shapeIndex = { shapeIndex };
    return collider.colliderIndex;
}

void PhysicsWorld::RemoveAabbCollider(ColliderIndex index)
{
    auto& [minBound, maxBound] = aabbs_[index.shapeIndex.index];
    maxBound.x = minBound.x;

    colliders_[index.index].colliderIndex.type = ColliderType::NONE;
}

void PhysicsWorld::RemoveCircleCollider(ColliderIndex index)
{
    auto& circle = circles_[index.shapeIndex.index];
    circle.radius = -1.0f;

    colliders_[index.index].colliderIndex.type = ColliderType::NONE;
}

void PhysicsWorld::Step(Scalar dt)
{

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
} // namespace neko
