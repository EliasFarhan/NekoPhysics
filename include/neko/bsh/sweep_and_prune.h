#ifndef NEKO_BSH_SWEEP_AND_PRUNE_H_
#define NEKO_BSH_SWEEP_AND_PRUNE_H_

#include <cstddef>

#include "neko/bsh/bsh.h"
#include "neko/core/allocator.h"

namespace neko {

/// Sort-and-sweep broadphase on the x axis, the default broadphase of
/// @ref PhysicsWorld.
///
/// The boxes are sorted by their left edge (`min_bound().x()`). Sweeping
/// that list from left to right, each box only needs to be compared with
/// the boxes that follow it until one starts past its right edge: everything
/// further right cannot overlap it either. The remaining candidates are then
/// checked on the y axis. With n boxes this costs a sort plus roughly
/// n * k comparisons, where k is the number of boxes overlapping on x,
/// instead of n * n.
///
/// Deterministic (ties are broken by collider index, so the order is total
/// and every `std::sort` gives the same result), allocation-free after
/// construction and stateless between steps, so it needs no snapshot for
/// rollback. Pairs are exactly the colliders whose AABBs intersect.
///
/// @see @ref BoundingSurfaceHierarchy for the order of the calls.
class SweepAndPrune final : public BoundingSurfaceHierarchy {
 public:
  /// Builds the broadphase and allocates all its memory.
  ///
  /// @ref PhysicsWorld builds its own from
  /// @ref PhysicsWorldConfig::max_colliders and
  /// @ref PhysicsWorldConfig::max_broadphase_pairs.
  ///
  /// @param max_colliders Maximum number of colliders inserted per step.
  /// @param max_pairs Maximum number of possible pairs per step.
  /// @param allocator @ref Allocator for the arrays; must outlive the object.
  SweepAndPrune(std::size_t max_colliders, std::size_t max_pairs,
                Allocator& allocator = DefaultAllocator());

  /// Appends a collider's box to the list to sort.
  ///
  /// @param collider_aabb Box and index of the collider.
  /// @pre Fewer than `max_colliders` colliders were inserted since
  /// @ref Clear; past that the collider is ignored and counted in
  /// @ref dropped_collider_count.
  void Insert(const ColliderAabb& collider_aabb) override;
  /// Sorts the boxes on x and sweeps them to find the overlapping pairs.
  ///
  /// Pairs beyond `max_pairs` are dropped and counted in
  /// @ref dropped_pair_count.
  void CalculatePairs() override;
  /// Removes all boxes and pairs, keeping the capacity.
  void Clear() override;
  /// Does nothing: sweep and prune needs no world bounds.
  void SetWorldAabb([[maybe_unused]] const Aabbf& world_aabb) override {}
  /// Returns the pairs found by the last @ref CalculatePairs.
  ///
  /// @return The pairs, valid until the next @ref Clear.
  [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs()
      const override {
    return possible_pairs_;
  }
  /// Returns how many colliders @ref Insert ignored since the last
  /// @ref Clear.
  ///
  /// @return The number of ignored colliders.
  [[nodiscard]] std::size_t dropped_collider_count() const override {
    return dropped_colliders_;
  }
  /// Returns how many pairs @ref CalculatePairs dropped since the last
  /// @ref Clear.
  ///
  /// @return The number of dropped pairs.
  [[nodiscard]] std::size_t dropped_pair_count() const override {
    return dropped_pairs_;
  }

 private:
  std::size_t max_colliders_;
  std::size_t max_pairs_;
  ArrayList<ColliderAabb> colliders_;
  ArrayList<ColliderPair> possible_pairs_;
  std::size_t dropped_colliders_ = 0;
  std::size_t dropped_pairs_ = 0;
};

}  // namespace neko

#endif  // NEKO_BSH_SWEEP_AND_PRUNE_H_
