#include "neko/bsh/quadtree.h"

#include <cassert>

#include "neko/core/profile.h"

namespace neko {
namespace {
// Number of nodes of a full tree: 1 + 4 + ... + 4^max_depth
constexpr std::size_t FullTreeNodeCount(std::size_t max_depth) {
  std::size_t count = 0;
  std::size_t level_count = 1;
  for (std::size_t depth = 0; depth <= max_depth; ++depth) {
    count += level_count;
    level_count *= 4;
  }
  return count;
}
}  // namespace

QuadTree::QuadTree(std::size_t max_colliders, std::size_t max_pairs,
                   std::size_t max_depth, std::size_t max_size,
                   Allocator& allocator)
    : max_colliders_(max_colliders),
      max_pairs_(max_pairs),
      max_depth_(max_depth),
      max_size_(max_size),
      nodes_(FullTreeNodeCount(max_depth), QuadNode{},
             StandardAllocator<QuadNode>{allocator}),
      entries_(StandardAllocator<Entry>{allocator}),
      possible_pairs_(StandardAllocator<ColliderPair>{allocator}) {
  assert(nodes_.size() < QuadNode::kNone && max_colliders < QuadNode::kNone);
  entries_.reserve(max_colliders_);
  possible_pairs_.reserve(max_pairs_);
}

void QuadTree::Insert(const ColliderAabb& collider_aabb) {
  NEKO_PROFILE_FUNCTION();
  if (entries_.size() >= max_colliders_) {
    ++dropped_colliders_;
    return;
  }
  entries_.push_back({collider_aabb, QuadNode::kNone});
  InsertEntry(static_cast<Index>(entries_.size() - 1), kRoot, 0);
}

void QuadTree::CalculatePairs() {
  NEKO_PROFILE_FUNCTION();
  AddPairs(kRoot);
}

void QuadTree::Clear() {
  NEKO_PROFILE_FUNCTION();
  entries_.clear();
  possible_pairs_.clear();
  dropped_colliders_ = 0;
  dropped_pairs_ = 0;
  used_nodes_ = 1;
  nodes_[kRoot] = QuadNode{nodes_[kRoot].aabb};
}

void QuadTree::SetWorldAabb(const Aabbf& world_aabb) {
  nodes_[kRoot].aabb = world_aabb;
}

void QuadTree::InsertEntry(Index entry, Index node, std::size_t depth) {
  const auto& quad_node = nodes_[node];
  if (quad_node.IsLeaf()) {
    if (depth < max_depth_ && quad_node.entry_count >= max_size_ &&
        used_nodes_ + 4 <= nodes_.size()) {
      Split(node, depth);
      InsertEntry(entry, node, depth);
    } else {
      PushEntry(entry, node);
    }
    return;
  }
  // Go down only when a single child overlaps the collider, otherwise it stays
  // in this node
  const auto& aabb = entries_[entry].collider_aabb.aabb;
  Index target = QuadNode::kNone;
  int overlap_count = 0;
  for (Index child = quad_node.first_child; child < quad_node.first_child + 4;
       ++child) {
    if (Intersect(aabb, nodes_[child].aabb)) {
      ++overlap_count;
      target = child;
    }
  }
  if (overlap_count == 1) {
    InsertEntry(entry, target, depth + 1);
  } else {
    PushEntry(entry, node);
  }
}

void QuadTree::PushEntry(Index entry, Index node) {
  auto& quad_node = nodes_[node];
  entries_[entry].next = quad_node.first_entry;
  quad_node.first_entry = entry;
  ++quad_node.entry_count;
}

void QuadTree::Split(Index node, std::size_t depth) {
  const std::size_t first_child = used_nodes_;
  used_nodes_ += 4;
  // Exact quadrants: no gap between children because of rounding
  const auto min_bound = nodes_[node].aabb.min_bound();
  const auto max_bound = nodes_[node].aabb.max_bound();
  const auto center = nodes_[node].aabb.GetCenter();
  nodes_[first_child] = QuadNode{{min_bound, center}};
  nodes_[first_child + 1] =
      QuadNode{{{center.x(), min_bound.y()}, {max_bound.x(), center.y()}}};
  nodes_[first_child + 2] =
      QuadNode{{{min_bound.x(), center.y()}, {center.x(), max_bound.y()}}};
  nodes_[first_child + 3] = QuadNode{{center, max_bound}};

  auto entry = nodes_[node].first_entry;
  nodes_[node].first_child = static_cast<Index>(first_child);
  nodes_[node].first_entry = QuadNode::kNone;
  nodes_[node].entry_count = 0;
  while (entry != QuadNode::kNone) {
    const auto next = entries_[entry].next;
    InsertEntry(entry, node, depth);
    entry = next;
  }
}

void QuadTree::AddPairs(Index node) {
  const auto& quad_node = nodes_[node];
  for (auto entry = quad_node.first_entry; entry != QuadNode::kNone;
       entry = entries_[entry].next) {
    for (auto other = entries_[entry].next; other != QuadNode::kNone;
         other = entries_[other].next) {
      AddPair(entry, other);
    }
    if (!quad_node.IsLeaf()) {
      for (Index child = quad_node.first_child;
           child < quad_node.first_child + 4; ++child) {
        AddPairsWithSubtree(entry, child);
      }
    }
  }
  if (!quad_node.IsLeaf()) {
    for (Index child = quad_node.first_child; child < quad_node.first_child + 4;
         ++child) {
      AddPairs(child);
    }
  }
}

void QuadTree::AddPairsWithSubtree(Index entry, Index node) {
  const auto& quad_node = nodes_[node];
  for (auto other = quad_node.first_entry; other != QuadNode::kNone;
       other = entries_[other].next) {
    AddPair(entry, other);
  }
  if (!quad_node.IsLeaf()) {
    for (Index child = quad_node.first_child; child < quad_node.first_child + 4;
         ++child) {
      AddPairsWithSubtree(entry, child);
    }
  }
}

void QuadTree::AddPair(Index entry1, Index entry2) {
  const auto& collider_aabb1 = entries_[entry1].collider_aabb;
  const auto& collider_aabb2 = entries_[entry2].collider_aabb;
  if (!Intersect(collider_aabb1.aabb, collider_aabb2.aabb)) {
    return;
  }
  if (possible_pairs_.size() < max_pairs_) {
    possible_pairs_.emplace_back(collider_aabb1.collider_index,
                                 collider_aabb2.collider_index);
  } else {
    ++dropped_pairs_;
  }
}

}  // namespace neko
