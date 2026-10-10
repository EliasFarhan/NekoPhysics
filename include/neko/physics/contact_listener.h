#ifndef NEKO_PHYSICS_CONTACT_LISTENER_H_
#define NEKO_PHYSICS_CONTACT_LISTENER_H_
#include "neko/physics/physics_type.h"

namespace neko {

/// Interface that receives the contact events of a @ref PhysicsWorld.
///
/// Implement it in your game code and register it with
/// @ref PhysicsWorld::SetContactListener. The world only reports *changes*:
/// an "enter" event in the step two colliders start touching and an "exit"
/// event in the step they stop touching, not one event per step while they
/// touch (read @ref PhysicsWorld::touching_pairs for the current state).
///
/// - Trigger events (`OnTrigger*`) fire when at least one of the two
///   colliders has @ref Collider::is_trigger set. Triggers detect overlaps
///   (pickups, zones, sensors) but never push bodies apart.
/// - Collision events (`OnCollision*`) fire for two solid colliders, which the
///   world also separates and bounces.
///
/// If a touching pair switches between trigger and solid, it gets the exit
/// event of the old kind and the enter event of the new kind in the same step.
///
/// Events are sent at the end of @ref PhysicsWorld::Step, after all contacts
/// are resolved, sorted by @ref ColliderPair, so every peer of a networked
/// game sees them in the same order.
///
/// @note The listener is not part of the simulation state:
/// @ref PhysicsWorld::CopyFrom does not copy it, and restoring a rollback
/// snapshot does not replay events.
/// @warning Events are sent from inside @ref PhysicsWorld::Step: record them
/// and act on them after @ref PhysicsWorld::Step returns rather than adding or
/// removing bodies and colliders in the callback.
class ContactListener {
 public:
  /// Destroys the listener.
  virtual ~ContactListener() = default;
  /// Called when a pair involving a trigger starts overlapping.
  ///
  /// @param p The two colliders, at least one of them a trigger.
  virtual void OnTriggerEnter(const ColliderPair& p) = 0;
  /// Called when a pair involving a trigger stops overlapping.
  ///
  /// @param p The two colliders, at least one of them a trigger.
  virtual void OnTriggerExit(const ColliderPair& p) = 0;
  /// Called when two solid colliders start touching.
  ///
  /// @param p The two colliders, neither of them a trigger.
  virtual void OnCollisionEnter(const ColliderPair& p) = 0;
  /// Called when two solid colliders stop touching.
  ///
  /// @param p The two colliders, neither of them a trigger.
  virtual void OnCollisionExit(const ColliderPair& p) = 0;
};

}  // namespace neko

#endif  // NEKO_PHYSICS_CONTACT_LISTENER_H_
