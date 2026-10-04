#ifndef NEKO_BSH_BSH_H_
#define NEKO_BSH_BSH_H_


#include "neko/math/shape.h"
#include "neko/physics/physics_type.h"
#include "neko/core/allocator.h"

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
    /**
     * @brief Insert a ColliderAabb in the BSH. Needs to be called after Clear and SetWorldAabb
     */
    virtual void Insert(const ColliderAabb& colliderAabb) = 0;
    virtual void CalculatePairs() = 0;
    virtual void Clear() = 0;
    virtual void SetWorldAabb(const Aabbf& worldAabb) = 0;
    [[nodiscard]] virtual const ArrayList<ColliderPair>& GetPossiblePairs() const = 0;
};

} // namespace neko

#endif  // NEKO_BSH_BSH_H_
