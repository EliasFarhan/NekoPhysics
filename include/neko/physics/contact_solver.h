#ifndef NEKO_PHYSICS_CONTACT_SOLVER_H_
#define NEKO_PHYSICS_CONTACT_SOLVER_H_
#include <array>

#include "neko/math/vec2.h"

namespace neko {
struct Collider;
struct Body;

struct CollidingBody {
  Body* body = nullptr;
  Collider* collider = nullptr;
};

struct Contact {
  std::array<CollidingBody, 2> bodies{};
  Vec2f contact_normal{};
  Vec2f contact_point{};
  Scalar restitution{1};
  Scalar penetration{};
  void Resolve();
  [[nodiscard]] Scalar CalculateSeparateVelocity() const;
};
}  // namespace neko

#endif  // NEKO_PHYSICS_CONTACT_SOLVER_H_
