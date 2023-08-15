#include "physics/physics.h"

#include <iostream>

#include "math/vec2.h"
#include "physics/contact_solver.h"

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#endif



namespace neko
{
PhysicsWorld::PhysicsWorld(Vec2f gravity): gravity_(gravity)
{
}

BodyIndex PhysicsWorld::AddBody()
{
    const auto it = std::find_if(bodies_.begin(), bodies_.end(), [](const auto& body)
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
    auto it = std::find_if(colliders_.begin(), colliders_.end(), [&index](const auto& collider)
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
        it = std::find_if(colliders_.begin(), colliders_.end(), 
            [&index](const auto& collider)
            {
                return collider.bodyIndex == index;
            });
    }
}

bool PhysicsWorld::DetectCollision(
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

    const bool collision = !(collider1.isTrigger || collider2.isTrigger);

    switch (collider1.type)
    {
    case ColliderType::AABB:
        switch (collider2.type)
        {
        case ColliderType::AABB:
        {
            const Aabbf aabb1 = Aabbf::FromCenter(body1.position + collider1.offset, aabbs_[collider1.shapeIndex.index].halfSize);
            const Aabbf aabb2 = Aabbf::FromCenter(body2.position + collider2.offset, aabbs_[collider2.shapeIndex.index].halfSize);
            doesIntersect = Intersect(aabb1, aabb2);
            break;
        }
        case ColliderType::CIRCLE:
        {
            const Aabbf aabb1 = Aabbf::FromCenter(body1.position + collider1.offset, aabbs_[collider1.shapeIndex.index].halfSize);
            const Circlef circle = Circlef{ body2.position + collider2.offset, circles_[collider2.shapeIndex.index].radius };
            if(collision)
            {

                const auto delta = circle.position - aabb1.GetCenter();
                const auto halfSize = aabb1.GetHalfSize();
                Vec2f closestPoint{};
                Scalar dist{};

                dist = delta.x;
                if (delta.x > halfSize.x) dist = halfSize.x;
                if (delta.x < -halfSize.x) dist = -halfSize.x;
                closestPoint.x = dist;

                dist = delta.y;
                if (delta.y > halfSize.y) dist = halfSize.y;
                if (delta.y < -halfSize.y) dist = -halfSize.y;
                closestPoint.y = dist;

                dist = (closestPoint - delta).Length();
                doesIntersect = dist <= circle.radius;
                if (!doesIntersect)
                {
                    break;
                }
                const auto closestPointWorld = aabb1.GetCenter() + closestPoint;
                contact->contactNormal = (circle.position - closestPointWorld).Normalized();
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
    case ColliderType::CIRCLE:
        switch (collider2.type)
        {
        case ColliderType::AABB:
        {
            doesIntersect = DetectCollision(body2, collider2, body1, collider1, contact);
            break;
        }
        case ColliderType::CIRCLE:
        {
            const Circlef c1 = { body1.position + collider1.offset, circles_[collider1.shapeIndex.index].radius };
            const Circlef c2 = { body2.position + collider2.offset, circles_[collider2.shapeIndex.index].radius };

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
    for(const auto& newColliderPair : newPossiblePairs)
    {
        auto& collider1 = colliders_[newColliderPair.c1.index];
        auto& collider2 = colliders_[newColliderPair.c2.index];

        auto& body1 = bodies_[collider1.bodyIndex.index];
        auto& body2 = bodies_[collider2.bodyIndex.index];
        auto it = manifold_.find(newColliderPair);
        Contact contact;
        const bool doesIntersect = DetectCollision(body1, collider1, body2, collider2, &contact);
        

        if (it != manifold_.end())
        {
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
                manifold_.erase(it);
            }
            else
            {
                if(!collider1.isTrigger && !collider2.isTrigger)
                {
                    contact.Resolve(dt);

                }
            }
        }
        else
        {
            if (doesIntersect)
            {
                if (collider1.isTrigger || collider2.isTrigger)
                {
                    contactListener_->OnTriggerEnter(newColliderPair);
                }
                else
                {
                    contact.Resolve(dt);
                    contactListener_->OnCollisionEnter(newColliderPair);
                }
                manifold_.insert(newColliderPair);
            }
        }
    }

}

ColliderIndex PhysicsWorld::AddCircleCollider(BodyIndex body)
{
    const auto it = std::find_if(colliders_.begin(), colliders_.end(), [](const Collider& collider)
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

    const auto circleIt = std::find_if(circles_.begin(), circles_.end(), [](const auto& circle)
    {
        return circle.radius < Scalar{0};
    });
    int shapeIndex;
    if(circleIt != circles_.end())
    {
        circleIt->radius = Scalar{ 0 };
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
    const auto it = std::find_if(colliders_.begin(), colliders_.end(), [](const Collider& collider)
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

    const auto aabbIt = std::find_if(aabbs_.begin(), aabbs_.end(), [](const auto& aabb)
        {
            return aabb.halfSize.x < Scalar{-1};
        });
    int shapeIndex;
    if (aabbIt != aabbs_.end())
    {
        aabbIt->halfSize.x = Scalar{ 0 };
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

ColliderIndex PhysicsWorld::AddPlaneCollider(BodyIndex body)
{
    const auto it = std::find_if(colliders_.begin(), colliders_.end(), [](const Collider& collider)
        {
            return collider.type == ColliderType::NONE;
        });
    int index = -1;
    if (it != colliders_.end())
    {
        it->type = ColliderType::PLANE;
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
    collider.type = ColliderType::PLANE;

    const auto planeIt = std::find_if(planes_.begin(), planes_.end(), [](const auto& plane)
        {
            return plane.normal == Vec2f::zero();
        });
    int shapeIndex;
    if (planeIt != planes_.end())
    {
        planeIt->normal = Vec2f::zero();
        shapeIndex = static_cast<int>(std::distance(planes_.begin(), planeIt));
    }
    else
    {
        shapeIndex = static_cast<int>(planes_.size());
        planes_.push_back({});
    }
    collider.shapeIndex = { shapeIndex };
    return collider.colliderIndex;
}

void PhysicsWorld::RemoveAabbCollider(ColliderIndex index)
{
    auto& collider = colliders_[index.index];
    auto& [halfSize] = aabbs_[collider.shapeIndex.index];
    halfSize = Vec2f{ Scalar{-1},Scalar{-1} };

    collider.type = ColliderType::NONE;
}

void PhysicsWorld::RemoveCircleCollider(ColliderIndex index)
{
    auto& collider = colliders_[index.index];
    auto& circle = circles_[collider.shapeIndex.index];
    circle.radius = Scalar{ -1 };

    collider.type = ColliderType::NONE;
}

void PhysicsWorld::RemovePlaneCollider(ColliderIndex index)
{
    auto& collider = colliders_[index.index];
    auto& plane = planes_[collider.shapeIndex.index];
    plane.normal = Vec2f::zero();

    collider.type = ColliderType::NONE;
}

void PhysicsWorld::Step(Scalar dt)
{
#ifdef TRACY_ENABLE
    ZoneScoped;
#endif

    for (auto& body : bodies_)
    {
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
    bodies_.clear();
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
    for(auto& collider: colliders_)
    {
        switch (collider.type)
        {
        case ColliderType::AABB:
        case ColliderType::CIRCLE:
        case ColliderType::PLANE:
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
    bsh_->SetWorldAabb(worldBox);

    for(const auto& collider : colliders_)
    {
        switch(collider.type)
        {
        case ColliderType::AABB:
        {
            const auto aabbCollider = Aabbf::FromCenter(
                bodies_[collider.bodyIndex.index].position + collider.offset, 
                aabbs_[collider.shapeIndex.index].halfSize);
            bsh_->Insert({ aabbCollider, collider.colliderIndex });
            break;
        }
        case ColliderType::CIRCLE:
        {
            const auto circleCollider = Circlef{
                bodies_[collider.bodyIndex.index].position+collider.offset,
                circles_[collider.shapeIndex.index].radius};
            bsh_->Insert({ circleCollider.GetAabb(), collider.colliderIndex });
            break;
        }
        default: 
            break;
        }
    }
    bsh_->CalculatePairs();

}
} // namespace neko
