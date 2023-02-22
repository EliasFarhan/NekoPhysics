#pragma once


#include "math/shape.h"
#include "physics/physics_type.h"

namespace neko
{
    

struct ColliderAabb
{
    Aabbf aabb;
    ColliderIndex colliderIndex = INVALID_COLLIDER_INDEX;
};

class BoundingSurfaceHierarchy
{
public:
    virtual ~BoundingSurfaceHierarchy() = default;
    virtual void Insert(const ColliderAabb& colliderAabb) = 0;
    virtual void CalculatePairs() = 0;
    virtual void Clear() = 0;
    virtual void SetWorldAabb(const Aabbf& worldAabb) = 0;
    [[nodiscard]] virtual const std::vector<TriggerPair>& GetPossiblePairs() const = 0;
};

} // namespace neko