#ifndef NEKO_BSH_QUADTREE_H_
#define NEKO_BSH_QUADTREE_H_

#include <cstddef>
#include <limits>

#include "neko/bsh/bsh.h"
#include "neko/core/allocator.h"

namespace neko {

/// One node of a quadtree (@ref QuadTree), a square region of the world and the
/// colliders stored in it.
///
/// Nodes refer to each other with indices into the tree's node array rather
/// than pointers, so a tree can be copied safely. Read them with
/// @ref QuadTree::GetNode, for example to draw the subdivision for debugging.
struct QuadNode {
  /// Index value meaning "no node" or "no entry".
  static constexpr Index kNone = std::numeric_limits<Index>::max();

  Aabbf aabb{};  ///< Region of the world covered by this node.
  /// Index of the first of the four children, or @ref kNone for a leaf.
  ///
  /// The children are contiguous: `first_child` to `first_child + 3` are
  /// the bottom-left, bottom-right, top-left and top-right quadrants.
  Index first_child = kNone;
  /// Head of the linked list of colliders stored in this node, or
  /// @ref kNone when it is empty (internal to the tree).
  Index first_entry = kNone;
  Index entry_count = 0;  ///< Number of colliders stored in this node.

  /// Returns true when the node has no children.
  [[nodiscard]] constexpr bool IsLeaf() const noexcept {
    return first_child == kNone;
  }
};

/// Quadtree broadphase: recursively splits the world into four quadrants.
///
/// The root node covers the world box given by @ref SetWorldAabb. When a
/// leaf already holds `max_size` colliders and another one arrives, it
/// splits into four equal quadrants and its colliders move down, until
/// `max_depth` is reached. A collider that overlaps several quadrants stays
/// in the parent node. Colliders are then only tested against the ones in
/// the same node and in the nodes below it, so two colliders in different
/// quadrants are never compared.
///
/// It shines when objects are spread over a 2D area; when they are lined up
/// along one axis or the world is small, the default @ref SweepAndPrune is
/// usually as fast and simpler. The tree is rebuilt from scratch every step,
/// so it keeps no state between steps and needs no rollback snapshot.
///
/// Nodes and colliders live in arrays sized at construction (a full tree of
/// `1 + 4 + ... + 4^max_depth` nodes) and link to each other with indices:
/// no allocation after construction, and copies are safe.
///
/// @code
/// neko::QuadTree quad_tree{/*max_colliders=*/64, /*max_pairs=*/512};
/// physics_world.SetBSH(&quad_tree);
/// @endcode
///
/// @see @ref BoundingSurfaceHierarchy for the order of the calls.
class QuadTree final : public BoundingSurfaceHierarchy {
 public:
  /// Default maximum depth of the tree.
  static constexpr std::size_t kMaxDepth = 3;
  /// Default number of colliders a leaf holds before splitting.
  static constexpr std::size_t kMaxSize = 4;
  /// Index of the root node, for @ref GetNode.
  static constexpr Index kRoot = 0;

  /// Builds the tree and allocates all its memory.
  ///
  /// @param max_colliders Maximum number of colliders inserted per step.
  /// @param max_pairs Maximum number of possible pairs per step.
  /// @param max_depth Depth of the deepest nodes (the root is at depth 0).
  /// The node array grows as 4^max_depth, so keep it small.
  /// @param max_size Number of colliders a leaf holds before it splits.
  /// @param allocator @ref Allocator for the arrays; must outlive the tree.
  /// @pre The node count and @p max_colliders fit in @ref Index (below
  /// 65535).
  explicit QuadTree(std::size_t max_colliders = 256,
                    std::size_t max_pairs = 2048,
                    std::size_t max_depth = kMaxDepth,
                    std::size_t max_size = kMaxSize,
                    Allocator& allocator = DefaultAllocator());

  /// Inserts a collider, splitting leaves that are full.
  ///
  /// @param collider_aabb Box and index of the collider.
  /// @pre Fewer than `max_colliders` colliders were inserted since
  /// @ref Clear (asserts in debug, ignores the collider in release).
  void Insert(const ColliderAabb& collider_aabb) override;
  /// Collects the pairs whose boxes overlap, walking the tree from the root.
  ///
  /// Pairs beyond `max_pairs` are dropped (asserts in debug).
  void CalculatePairs() override;
  /// Removes all colliders and pairs and collapses the tree to its root.
  ///
  /// The root keeps its world box.
  void Clear() override;
  /// Sets the region covered by the root node.
  ///
  /// @param world_aabb Box containing every collider of the step.
  void SetWorldAabb(const Aabbf& world_aabb) override;
  /// Returns the pairs found by the last @ref CalculatePairs.
  ///
  /// @return The pairs, valid until the next @ref Clear.
  [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs()
      const override {
    return possible_pairs_;
  }

  /// Returns a node of the tree, to inspect or draw the subdivision.
  ///
  /// Start from @ref kRoot and follow @ref QuadNode::first_child until
  /// @ref QuadNode::IsLeaf.
  ///
  /// @param index Node index.
  /// @pre @p index is @ref kRoot or comes from a node's `first_child` (+0 to
  /// +3).
  [[nodiscard]] const QuadNode& GetNode(Index index) const {
    return nodes_[index];
  }

 private:
  struct Entry {
    ColliderAabb collider_aabb;
    Index next = QuadNode::kNone;
  };

  void InsertEntry(Index entry, Index node, std::size_t depth);
  void PushEntry(Index entry, Index node);
  void Split(Index node, std::size_t depth);
  void AddPairs(Index node);
  void AddPairsWithSubtree(Index entry, Index node);
  void AddPair(Index entry1, Index entry2);

  std::size_t max_colliders_;
  std::size_t max_pairs_;
  std::size_t max_depth_;
  std::size_t max_size_;
  ArrayList<QuadNode> nodes_;
  ArrayList<Entry> entries_;
  ArrayList<ColliderPair> possible_pairs_;
  std::size_t used_nodes_ = 1;
};

}  // namespace neko

#endif  // NEKO_BSH_QUADTREE_H_
