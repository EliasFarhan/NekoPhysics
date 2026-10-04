#ifndef NEKO_BSH_QUADTREE_H_
#define NEKO_BSH_QUADTREE_H_

#include <cstddef>
#include <limits>

#include "neko/bsh/bsh.h"
#include "neko/core/allocator.h"

namespace neko {

struct QuadNode {
  static constexpr Index kNone = std::numeric_limits<Index>::max();

  Aabbf aabb{};
  // The four children are contiguous, starting at first_child
  Index first_child = kNone;
  // Head of the linked list of colliders stored in this node
  Index first_entry = kNone;
  Index entry_count = 0;

  [[nodiscard]] constexpr bool IsLeaf() const noexcept {
    return first_child == kNone;
  }
};

// Quadtree broadphase. Nodes and colliders live in arrays sized at construction
// and link to each other with indices: no allocation after construction, and
// copies are safe.
class QuadTree final : public BoundingSurfaceHierarchy {
 public:
  static constexpr std::size_t kMaxDepth = 3;
  static constexpr std::size_t kMaxSize = 4;
  static constexpr Index kRoot = 0;

  // `max_depth` is the depth of the deepest nodes (the root is at depth 0).
  // `max_size` is the number of colliders above which a leaf splits.
  explicit QuadTree(std::size_t max_colliders = 256,
                    std::size_t max_pairs = 2048,
                    std::size_t max_depth = kMaxDepth,
                    std::size_t max_size = kMaxSize,
                    Allocator& allocator = DefaultAllocator());

  void Insert(const ColliderAabb& collider_aabb) override;
  void CalculatePairs() override;
  void Clear() override;
  void SetWorldAabb(const Aabbf& world_aabb) override;
  [[nodiscard]] const ArrayList<ColliderPair>& GetPossiblePairs()
      const override {
    return possible_pairs_;
  }

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
