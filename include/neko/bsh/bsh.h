#ifndef NEKO_BSH_BSH_H_
#define NEKO_BSH_BSH_H_

#include "neko/core/allocator.h"
#include "neko/math/shape.h"
#include "neko/physics/physics_type.h"

namespace neko {

/// A collider's bounding box tagged with the collider it belongs to.
///
/// This is what the broadphase works on: it never sees the real shapes, only
/// these boxes, and reports pairs of @ref ColliderIndex.
struct ColliderAabb {
  Aabbf aabb;  ///< World-space box enclosing the collider's shape.
  /// @ref Collider owning the box, reported back in @ref ColliderPair.
  ColliderIndex collider_index = kInvalidColliderIndex;
};

/// Interface of a broadphase: finds the pairs of colliders that may touch.
///
/// Testing every collider against every other one costs n * (n - 1) / 2
/// exact tests, which is far too slow on the DS past a few dozen bodies.
/// Collision detection is therefore split in two phases:
/// - the broadphase (this interface) cheaply discards pairs that are far
///   apart by looking only at axis-aligned bounding boxes, and outputs the
///   "possible pairs" whose boxes overlap;
/// - the narrowphase (in @ref PhysicsWorld) runs the exact shape test
///   (circle-circle, circle-box...) on those pairs only.
///
/// The name comes from bounding volume hierarchies (BVH): spatial structures
/// that group objects by region so that whole groups can be skipped at once.
/// In 2D the bounding volumes are surfaces, hence "bounding surface
/// hierarchy". Two implementations are provided: @ref SweepAndPrune (the
/// default) and @ref QuadTree.
///
/// Every step, @ref PhysicsWorld calls, in this order: @ref Clear,
/// @ref SetWorldAabb, @ref Insert for each active collider,
/// @ref CalculatePairs, then reads @ref GetPossiblePairs. To use another
/// implementation, derive from this class and pass it to
/// @ref PhysicsWorld::SetBSH.
///
/// @code
/// neko::PhysicsWorld world;
/// neko::QuadTree quad_tree;      // must outlive its use by the world
/// world.SetBSH(&quad_tree);      // nullptr goes back to SweepAndPrune
/// world.Step();
/// @endcode
///
/// @note For rollback netcode the output must be deterministic: the same
/// inserts in the same order must give the same pairs in the same order,
/// since contacts are resolved in that order. Implementations must also not
/// allocate during a step (capacities are fixed at construction).
class BoundingSurfaceHierarchy {
 public:
  /// Virtual destructor, so implementations can be deleted through the base.
  virtual ~BoundingSurfaceHierarchy() = default;
  /// Adds a collider's box to the structure.
  ///
  /// @param collider_aabb Box and index of the collider.
  /// @pre @ref Clear then @ref SetWorldAabb were called since the last
  /// @ref CalculatePairs.
  virtual void Insert(const ColliderAabb& collider_aabb) = 0;
  /// Computes the possible pairs from the inserted boxes.
  ///
  /// The result is read with @ref GetPossiblePairs.
  virtual void CalculatePairs() = 0;
  /// Removes all the inserted boxes and computed pairs.
  ///
  /// Keeps the capacity: clearing never frees memory.
  virtual void Clear() = 0;
  /// Sets the box enclosing every collider of this step.
  ///
  /// Spatial subdivisions like @ref QuadTree need it to know which region to
  /// split; implementations that don't (like @ref SweepAndPrune) ignore it.
  ///
  /// @param world_aabb Box containing all the boxes about to be inserted.
  virtual void SetWorldAabb(const Aabbf& world_aabb) = 0;
  /// Returns the pairs found by the last @ref CalculatePairs.
  ///
  /// Each pair appears once, with @ref ColliderPair ordering its two
  /// indices. A pair only means the boxes overlap: the narrowphase still has
  /// to test the real shapes.
  ///
  /// @return The pairs, valid until the next @ref Clear.
  [[nodiscard]] virtual const ArrayList<ColliderPair>& GetPossiblePairs()
      const = 0;
};

}  // namespace neko

#endif  // NEKO_BSH_BSH_H_
