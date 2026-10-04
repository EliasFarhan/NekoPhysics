#ifndef NEKO_PHYSICS_PHYSICS_H_
#define NEKO_PHYSICS_PHYSICS_H_

#include <cstddef>
#include <cstdint>

#include "neko/bsh/bsh.h"
#include "neko/bsh/sweep_and_prune.h"
#include "neko/core/allocator.h"
#include "neko/math/shape.h"
#include "neko/math/vec2.h"
#include "neko/physics/contact_listener.h"
#include "neko/physics/contact_solver.h"
#include "neko/physics/physics_type.h"

namespace neko {

enum class BodyType : std::uint8_t { kDynamic, kKinematic, kStatic, kNone };

struct Body {
  Vec2f position{};
  Vec2f velocity{};
  Vec2f force{};
  Scalar inverse_mass{1};
  BodyType type = BodyType::kDynamic;
  bool is_active = true;
};

[[nodiscard]] std::uint32_t GenerateChecksum(const Body& body);

struct Collider {
  const void* user_data = nullptr;
  Vec2f offset{};
  Scalar restitution{1};
  BodyIndex body_index{};
  ColliderIndex collider_index{};
  ShapeIndex shape_index{};
  ShapeType type = ShapeType::kNone;
  bool is_trigger = true;
};
[[nodiscard]] std::uint32_t GenerateChecksum(const Collider& collider);

// A negative radius marks a free slot
struct CircleCollider {
  Scalar radius{-1};
};

[[nodiscard]] std::uint32_t GenerateChecksum(const CircleCollider& collider);

// A negative half size marks a free slot
struct AabbCollider {
  Vec2f half_size{Scalar{-1}, Scalar{-1}};
};

[[nodiscard]] std::uint32_t GenerateChecksum(const AabbCollider& collider);

// Pair of colliders that touched at the end of the last step, used to send
// enter and exit events
struct TouchingPair {
  ColliderPair pair{};
  bool is_trigger = false;
};

struct PhysicsWorldConfig {
  Vec2f gravity{Scalar{0.0f}, Scalar{-9.81f}};
  // Duration of one Step. Fixed, so that every peer and every rollback
  // resimulation uses the same value
  Scalar dt{1.0f / 60.0f};
  // Capacities are allocated once, at construction: Step and CopyFrom never
  // allocate
  std::size_t max_bodies = 256;
  std::size_t max_colliders = 256;
  // Maximum number of touching pairs tracked for enter/exit events
  std::size_t max_touching_pairs = 1024;
  // Maximum number of pairs whose AABBs overlap, for the default broadphase
  std::size_t max_broadphase_pairs = 2048;
  // Must outlive the world. nullptr means DefaultAllocator()
  Allocator* allocator = nullptr;
};

class PhysicsWorld {
 public:
  explicit PhysicsWorld(const PhysicsWorldConfig& config = {});
  // Copies would allocate: rollback snapshots use CopyFrom into an existing
  // world instead
  PhysicsWorld(const PhysicsWorld&) = delete;
  PhysicsWorld& operator=(const PhysicsWorld&) = delete;
  PhysicsWorld(PhysicsWorld&&) = default;
  PhysicsWorld& operator=(PhysicsWorld&&) = default;
  ~PhysicsWorld() = default;

  // Returns kInvalidBodyIndex when max_bodies are in use
  BodyIndex AddBody();
  // Removes the body and all its colliders
  void RemoveBody(BodyIndex index);
  bool DetectContact(Body& body1, Collider& collider1, Body& body2,
                     Collider& collider2, Contact* contact);
  // Advances the world by config.dt
  void Step();
  void Clear();
  void ResolveBroadphase();
  void ResolveNarrowphase();
  // Sorts new_touching_pairs_ by pair (c1 then c2), the same total order as
  // std::sort(..., [](a, b){ return a.pair < b.pair; }), but in O(n +
  // max_colliders): a counting sort on c1 (colliders are indices into a small,
  // fixed-size table) followed by an insertion sort of each c1 bucket, which is
  // short (a body only touches a handful of others).
  void SortNewTouchingPairs();

  [[nodiscard]] Body& body(BodyIndex index) { return bodies_[index.index()]; }
  [[nodiscard]] const Body& body(BodyIndex index) const {
    return bodies_[index.index()];
  }

  // Return kInvalidColliderIndex when max_colliders are in use
  ColliderIndex AddCircleCollider(BodyIndex body);
  ColliderIndex AddAabbCollider(BodyIndex body);
  void RemoveCollider(ColliderIndex index);

  [[nodiscard]] Collider& collider(ColliderIndex collider_index) {
    return colliders_[collider_index.index()];
  }
  [[nodiscard]] const Collider& collider(ColliderIndex collider_index) const {
    return colliders_[collider_index.index()];
  }

  [[nodiscard]] AabbCollider& aabb(ShapeIndex shape_index) {
    return aabbs_[shape_index.index()];
  }
  [[nodiscard]] const AabbCollider& aabb(ShapeIndex shape_index) const {
    return aabbs_[shape_index.index()];
  }
  [[nodiscard]] CircleCollider& circle(ShapeIndex shape_index) {
    return circles_[shape_index.index()];
  }
  [[nodiscard]] const CircleCollider& circle(ShapeIndex shape_index) const {
    return circles_[shape_index.index()];
  }

  // Replaces the default broadphase (sort and sweep); nullptr restores it
  void SetBSH(BoundingSurfaceHierarchy* bounding_surface_hierarchy) {
    bsh_ = bounding_surface_hierarchy;
  }
  void SetContactListener(ContactListener* contact_listener) {
    contact_listener_ = contact_listener;
  }

  [[nodiscard]] Vec2f gravity() const { return config_.gravity; }
  [[nodiscard]] Scalar dt() const { return config_.dt; }
  [[nodiscard]] const ArrayList<TouchingPair>& touching_pairs() const {
    return touching_pairs_;
  }
  // Number of pairs whose AABBs overlap, found by the broadphase during the
  // last ResolveBroadphase
  [[nodiscard]] std::size_t BroadphasePairCount() const {
    return Broadphase().GetPossiblePairs().size();
  }

  // Copies the simulation state (bodies, colliders, shapes, touching pairs),
  // not the listener nor the BSH. Doesn't allocate when both worlds have the
  // same config.
  void CopyFrom(const PhysicsWorld& physics_world);
  // Hash of the whole simulation state, to detect desyncs between peers
  [[nodiscard]] std::uint32_t GenerateChecksum() const;

 private:
  void NotifyContactChanges() const;
  [[nodiscard]] BoundingSurfaceHierarchy& Broadphase() {
    return bsh_ != nullptr ? *bsh_ : default_broadphase_;
  }
  [[nodiscard]] const BoundingSurfaceHierarchy& Broadphase() const {
    return bsh_ != nullptr ? static_cast<const BoundingSurfaceHierarchy&>(*bsh_)
                           : default_broadphase_;
  }
  [[nodiscard]] Aabbf GetAabb(const Collider& collider) const;

  PhysicsWorldConfig config_;
  SweepAndPrune default_broadphase_;
  ArrayList<Body> bodies_;
  ArrayList<AabbCollider> aabbs_;
  ArrayList<CircleCollider> circles_;
  ArrayList<Collider> colliders_;
  ArrayList<TouchingPair> touching_pairs_;
  ArrayList<TouchingPair> new_touching_pairs_;
  // Scratch buffer for ResolveBroadphase: each active collider's Aabbf,
  // computed once per Step and reused for both the world box pass and the
  // insert pass (GetAabb isn't free: a body lookup plus a shape switch)
  ArrayList<Aabbf> collider_aabb_cache_;
  // Scratch for SortNewTouchingPairs: bucketCount_[k] then bucketStart_[k]
  // hold, then temporarily the write cursor for, colliders whose
  // ColliderPair::c1.index == k; sorted_ is the counting sort's output buffer.
  ArrayList<std::uint32_t> touching_pair_bucket_count_;
  ArrayList<std::uint32_t> touching_pair_bucket_start_;
  ArrayList<TouchingPair> sorted_touching_pairs_;

  ContactListener* contact_listener_ = nullptr;
  BoundingSurfaceHierarchy* bsh_ = nullptr;
};

}  // namespace neko

#endif  // NEKO_PHYSICS_PHYSICS_H_
