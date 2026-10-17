#include "neko/physics/physics.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <limits>
#include <optional>
#include <type_traits>
#include <vector>

#include "neko/core/platform.h"
#include "neko/core/profile.h"
#include "neko/core/step_profile.h"
#include "neko/math/vec2.h"
#include "neko/physics/contact_solver.h"

namespace neko {
namespace {
// FNV-1a over the bytes of each 32-bit word: order-dependent, so equal or
// swapped fields don't cancel out
class Checksum {
 public:
  void Add(std::uint32_t word) noexcept {
    for (int shift = 0; shift < 32; shift += 8) {
      hash_ ^= (word >> shift) & 0xFFu;
      hash_ *= kPrime;
    }
  }
  void Add(Scalar value) noexcept {
    static_assert(sizeof(Scalar) == sizeof(std::uint32_t));
    Add(std::bit_cast<std::uint32_t>(value));
  }
  void Add(Vec2f value) noexcept {
    Add(value.x());
    Add(value.y());
  }
  void Add(Rotation value) noexcept {
    Add(value.angle().value());
    Add(value.sin());
    Add(value.cos());
  }
  void Add(bool value) noexcept { Add(static_cast<std::uint32_t>(value)); }
  template <typename E>
    requires std::is_enum_v<E>
  void Add(E value) noexcept {
    Add(static_cast<std::uint32_t>(value));
  }
  template <typename I>
    requires std::is_same_v<decltype(std::declval<I>().index()), Index>
  void Add(I value) noexcept {
    Add(static_cast<std::uint32_t>(value.index()));
  }

  [[nodiscard]] std::uint32_t Get() const noexcept { return hash_; }

 private:
  static constexpr std::uint32_t kOffsetBasis = 2166136261u;
  static constexpr std::uint32_t kPrime = 16777619u;
  std::uint32_t hash_ = kOffsetBasis;
};

// Reuses the first free slot, or appends one while under capacity
// Returns the slot index, or std::nullopt when the list is full
template <typename T, typename IsFree>
std::optional<Index> AcquireSlot(ArrayList<T>& list, std::size_t capacity,
                                 IsFree is_free) {
  const auto it = std::find_if(list.begin(), list.end(), is_free);
  if (it != list.end()) {
    *it = T{};
    return static_cast<Index>(it - list.begin());
  }
  if (list.size() >= capacity) {
    return std::nullopt;
  }
  list.emplace_back();
  return static_cast<Index>(list.size() - 1);
}

template <typename T>
ArrayList<T> MakeList(Allocator& allocator, std::size_t capacity) {
  ArrayList<T> list{StandardAllocator<T>{allocator}};
  list.reserve(capacity);
  return list;
}

Allocator& ConfigAllocator(const PhysicsWorldConfig& config) {
  return config.allocator != nullptr ? *config.allocator : DefaultAllocator();
}

constexpr bool IsFreeBody(const Body& body) {
  return body.type == BodyType::kNone;
}
constexpr bool IsFreeCollider(const Collider& collider) {
  return collider.type == ShapeType::kNone;
}
constexpr bool IsFreeCircle(const CircleCollider& circle) {
  return circle.radius < Scalar{0};
}
constexpr bool IsFreeAabb(const AabbCollider& aabb) {
  return aabb.half_size.x() < Scalar{0};
}
constexpr bool IsFreeOrientedBox(const OrientedBoxCollider& box) {
  return box.half_size.x() < Scalar{0};
}
constexpr bool IsFreeJoint(const DistanceJoint& joint) {
  return joint.type == DistanceJointType::kNone;
}

// A box in the world for the narrowphase: center, unit axes (its local x and y
// in world space) and half size along them. An AabbCollider is the same with
// the world axes.
struct BoxShape {
  Vec2f center;
  Vec2f axis_x;
  Vec2f axis_y;
  Vec2f half_size;
};

// Half the length of the box's shadow on a unit axis
Scalar ProjectedRadius(const BoxShape& box, Vec2f axis) {
  return box.half_size.x() * Abs(Vec2f::Dot(axis, box.axis_x)) +
         box.half_size.y() * Abs(Vec2f::Dot(axis, box.axis_y));
}

// Overlap of the shadows of a and b on a unit axis (negative when separated),
// and the axis turned to point from b toward a, the contact normal convention
struct AxisOverlap {
  Scalar overlap;
  Vec2f normal;
  // Which axis of its box: false for x, true for y
  bool is_y_axis;
};

AxisOverlap OverlapAlong(const BoxShape& a, const BoxShape& b, Vec2f axis,
                         bool is_y_axis) {
  const Scalar distance = Vec2f::Dot(a.center - b.center, axis);
  return {ProjectedRadius(a, axis) + ProjectedRadius(b, axis) - Abs(distance),
          distance < Scalar{0} ? -axis : axis, is_y_axis};
}

// Keeps the part of the segment from v1 to v2 where Dot(side, p) <= offset.
// Returns false when nothing is left.
bool ClipSegment(Vec2f& v1, Vec2f& v2, Vec2f side, Scalar offset) {
  const Scalar distance1 = Vec2f::Dot(side, v1) - offset;
  const Scalar distance2 = Vec2f::Dot(side, v2) - offset;
  if (distance1 > Scalar{0} && distance2 > Scalar{0}) {
    return false;
  }
  if (distance1 > Scalar{0}) {
    v1 = v1 + (v2 - v1) * (distance1 / (distance1 - distance2));
  } else if (distance2 > Scalar{0}) {
    v2 = v2 + (v1 - v2) * (distance2 / (distance2 - distance1));
  }
  return true;
}

// The contact manifold of two overlapping boxes, given the reference box (the
// owner of the separating axis), the axis as found by SAT and the outward
// normal of the reference face (pointing toward the incident box):
// - the incident face is the face of the other box most opposed to it;
// - its two corners are clipped to the width of the reference face;
// - the clipped points below the reference face are the manifold, each with
//   its depth below that face.
//
// Each point gets a feature id from the reference face (which box, which axis,
// which side), the incident face (axis, side) and the incident corner, so the
// same way of touching gives the same id next step.
void ClipBoxes(const BoxShape& reference, const BoxShape& incident,
               const AxisOverlap& axis, Vec2f reference_normal,
               std::uint8_t reference_is_b, Contact* contact) {
  const Scalar reference_half =
      axis.is_y_axis ? reference.half_size.y() : reference.half_size.x();
  const Vec2f tangent = axis.is_y_axis ? reference.axis_x : reference.axis_y;
  const Scalar tangent_half =
      axis.is_y_axis ? reference.half_size.x() : reference.half_size.y();
  const Vec2f face_center =
      reference.center + reference_normal * reference_half;

  // Incident face: the incident box's face whose normal points most against
  // the reference normal (x wins a tie, for determinism)
  const Scalar along_x = Vec2f::Dot(incident.axis_x, reference_normal);
  const Scalar along_y = Vec2f::Dot(incident.axis_y, reference_normal);
  const bool incident_on_x = Abs(along_x) >= Abs(along_y);
  const Vec2f axis_n = incident_on_x ? incident.axis_x : incident.axis_y;
  const Scalar along = incident_on_x ? along_x : along_y;
  const Vec2f incident_normal = along > Scalar{0} ? -axis_n : axis_n;
  const Vec2f edge = incident_on_x ? incident.axis_y : incident.axis_x;
  const Scalar normal_half =
      incident_on_x ? incident.half_size.x() : incident.half_size.y();
  const Scalar edge_half =
      incident_on_x ? incident.half_size.y() : incident.half_size.x();
  const Vec2f incident_center = incident.center + incident_normal * normal_half;
  Vec2f v1 = incident_center - edge * edge_half;
  Vec2f v2 = incident_center + edge * edge_half;

  // Side planes of the reference face
  const Scalar center_along_tangent = Vec2f::Dot(tangent, reference.center);
  if (!ClipSegment(v1, v2, tangent, center_along_tangent + tangent_half) ||
      !ClipSegment(v1, v2, -tangent, tangent_half - center_along_tangent)) {
    return;
  }
  const Vec2f reference_axis =
      axis.is_y_axis ? reference.axis_y : reference.axis_x;
  const auto face_id = static_cast<std::uint8_t>(
      reference_is_b | (axis.is_y_axis ? 2u : 0u) |
      (Vec2f::Dot(reference_normal, reference_axis) > Scalar{0} ? 4u : 0u) |
      (incident_on_x ? 0u : 8u) | (along > Scalar{0} ? 0u : 16u));
  const std::array<Vec2f, 2> clipped = {v1, v2};
  for (std::uint8_t corner = 0; corner < 2; ++corner) {
    const Scalar separation =
        Vec2f::Dot(reference_normal, clipped[corner] - face_center);
    if (separation <= Scalar{0}) {
      ContactPoint& point = contact->points[contact->point_count];
      point = {clipped[corner], -separation};
      point.id = static_cast<std::uint8_t>(face_id | (corner << 5));
      ++contact->point_count;
    }
  }
}

// The box's farthest point in a direction: a corner, or the middle of an edge
// when the direction is exactly perpendicular to it
Vec2f Support(const BoxShape& box, Vec2f direction) {
  Vec2f point = box.center;
  const Scalar along_x = Vec2f::Dot(direction, box.axis_x);
  const Scalar along_y = Vec2f::Dot(direction, box.axis_y);
  if (along_x > Scalar{0}) {
    point += box.axis_x * box.half_size.x();
  } else if (along_x < Scalar{0}) {
    point -= box.axis_x * box.half_size.x();
  }
  if (along_y > Scalar{0}) {
    point += box.axis_y * box.half_size.y();
  } else if (along_y < Scalar{0}) {
    point -= box.axis_y * box.half_size.y();
  }
  return point;
}

// Separating axis test of two boxes. The face normals of both boxes are the
// only axes that can separate them: if one of them separates the shadows, the
// boxes don't overlap (touching counts as overlapping, like the other
// shapes). Otherwise the axis of smallest overlap is the contact normal and
// the overlap is the penetration. The order is fixed (a's x and y axes, then
// b's) and a later axis replaces an earlier one only when strictly smaller, so
// the choice is deterministic. Fills contact when it isn't null.
bool CollideBoxes(const BoxShape& a, const BoxShape& b, Contact* contact) {
  AxisOverlap best_a = OverlapAlong(a, b, a.axis_x, false);
  if (best_a.overlap < Scalar{0}) {
    return false;
  }
  const AxisOverlap a_y = OverlapAlong(a, b, a.axis_y, true);
  if (a_y.overlap < Scalar{0}) {
    return false;
  }
  if (a_y.overlap < best_a.overlap) {
    best_a = a_y;
  }
  AxisOverlap best_b = OverlapAlong(a, b, b.axis_x, false);
  if (best_b.overlap < Scalar{0}) {
    return false;
  }
  const AxisOverlap b_y = OverlapAlong(a, b, b.axis_y, true);
  if (b_y.overlap < Scalar{0}) {
    return false;
  }
  if (b_y.overlap < best_b.overlap) {
    best_b = b_y;
  }
  if (contact == nullptr) {
    return true;
  }
  // b's face wins only when clearly smaller (under 15/16 of a's): when two
  // overlaps are nearly equal, rounding would otherwise flip the normal from
  // one step to the next
  if (best_b.overlap * Scalar{16} < best_a.overlap * Scalar{15}) {
    // Reference face on b: a's corner deepest toward b
    contact->contact_normal = best_b.normal;
    contact->contact_point = Support(a, -best_b.normal);
    contact->penetration = best_b.overlap;
    // b's face looks toward a: along the normal
    ClipBoxes(b, a, best_b, best_b.normal, 1, contact);
  } else {
    // Reference face on a: b's corner deepest toward a
    contact->contact_normal = best_a.normal;
    contact->contact_point = Support(b, best_a.normal);
    contact->penetration = best_a.overlap;
    // a's face looks toward b: against the normal
    ClipBoxes(a, b, best_a, -best_a.normal, 0, contact);
  }
  return true;
}

// A box (body 1) against a circle (body 2), computed in the box's frame like
// the AabbCollider case: the closest point of the box to the circle center,
// or the nearest face when the center is inside. The normal points from the
// circle toward the box. Fills contact when it isn't null.
bool CollideBoxCircle(const BoxShape& box, Vec2f circle_center, Scalar radius,
                      Contact* contact) {
  const Vec2f delta = circle_center - box.center;
  const Vec2f local{Vec2f::Dot(delta, box.axis_x),
                    Vec2f::Dot(delta, box.axis_y)};
  const Vec2f clamped{Clamp(local.x(), -box.half_size.x(), box.half_size.x()),
                      Clamp(local.y(), -box.half_size.y(), box.half_size.y())};
  const Vec2f box_to_circle = local - clamped;
  if (box_to_circle != Vec2f::Zero()) {
    if (CompareLength(box_to_circle, radius) > 0) {
      return false;
    }
    if (contact != nullptr) {
      const Scalar distance = box_to_circle.Length();
      const Vec2f local_normal = box_to_circle * (Scalar{1} / distance);
      contact->contact_normal =
          -(box.axis_x * local_normal.x() + box.axis_y * local_normal.y());
      contact->contact_point =
          box.center + box.axis_x * clamped.x() + box.axis_y * clamped.y();
      contact->penetration = radius - distance;
    }
    return true;
  }
  if (contact == nullptr) {
    return true;
  }
  // The circle center is inside the box: push it out through the nearest
  // face
  const Scalar distance_x = box.half_size.x() - Abs(local.x());
  const Scalar distance_y = box.half_size.y() - Abs(local.y());
  if (distance_x < distance_y) {
    const Vec2f outward = local.x() < Scalar{0} ? -box.axis_x : box.axis_x;
    contact->contact_normal = -outward;
    contact->contact_point =
        box.center + outward * box.half_size.x() + box.axis_y * local.y();
    contact->penetration = radius + distance_x;
  } else {
    const Vec2f outward = local.y() < Scalar{0} ? -box.axis_y : box.axis_y;
    contact->contact_normal = -outward;
    contact->contact_point =
        box.center + box.axis_x * local.x() + outward * box.half_size.y();
    contact->penetration = radius + distance_y;
  }
  return true;
}

// Center of the collider's shape in the world: the offset turns with the body.
// At angle 0 (sin 0, cos 1) the rotated offset is the offset bit for bit, and
// a zero offset (most colliders) skips the rotation: the same result without
// the four multiplications, which the narrowphase pays for every pair.
Vec2f WorldCenter(const Body& body, const Collider& collider) {
  if (collider.offset == Vec2f::Zero()) {
    return body.position;
  }
  return body.position + body.rotation.Rotate(collider.offset);
}

// value / N for a small positive integer N, with the same bits as
// value / Scalar{N}: for Fixed16 both truncate raw * 2^16 / (N * 2^16), which
// is the raw value divided by N, a shift or a multiplication by a constant
// instead of the 64-bit divider
template <int N>
Scalar DivideBy(Scalar value) {
#ifdef NEKO_SCALAR_FIXED16
  return Scalar::FromUnderlyingValue(value.underlying_value() / N);
#else
  return value / Scalar{N};
#endif
}

// Share of a collider's inertia per kilogram when the body's mass is spread
// over its colliders by area. A lone collider gets the whole mass, without a
// division.
Scalar AreaWeighted(Scalar unit_inertia, Scalar area, Scalar total_area) {
  if (area == Scalar{0}) {
    return Scalar{0};
  }
  if (area == total_area) {
    return unit_inertia;
  }
  return unit_inertia * (area / total_area);
}

// inverse_mass / (inertia per kilogram), 0 (no spin) without solid shapes
Scalar InverseInertia(Scalar inverse_mass, Scalar unit_inertia) {
  return unit_inertia > Scalar{0} ? inverse_mass / unit_inertia : Scalar{0};
}

// Turns the body by angular_velocity * dt. A body that doesn't turn keeps its
// rotation without recomputing the sine and cosine.
void Turn(Body& body, Scalar dt) {
  if (body.angular_velocity == Scalar{0}) {
    return;
  }
  const Scalar turn = body.angular_velocity * dt;
  if (turn != Scalar{0}) {
    body.rotation.set_angle(body.rotation.angle() + Radian{turn});
  }
}
}  // namespace

std::uint32_t GenerateChecksum(const Body& body) {
  Checksum checksum;
  checksum.Add(body.position);
  checksum.Add(body.velocity);
  checksum.Add(body.force);
  checksum.Add(body.inverse_mass);
  checksum.Add(body.rotation);
  checksum.Add(body.angular_velocity);
  checksum.Add(body.torque);
  checksum.Add(body.inverse_inertia);
  checksum.Add(body.inertia_mode);
  checksum.Add(body.type);
  checksum.Add(body.is_active);
  return checksum.Get();
}

std::uint32_t GenerateChecksum(const Collider& collider) {
  Checksum checksum;
  checksum.Add(collider.offset);
  checksum.Add(collider.restitution);
  checksum.Add(collider.friction);
  checksum.Add(collider.body_index);
  checksum.Add(collider.collider_index);
  checksum.Add(collider.shape_index);
  checksum.Add(collider.type);
  checksum.Add(collider.is_trigger);
  return checksum.Get();
}

std::uint32_t GenerateChecksum(const DistanceJoint& joint) {
  Checksum checksum;
  checksum.Add(joint.body_a);
  checksum.Add(joint.body_b);
  checksum.Add(joint.local_anchor_a);
  checksum.Add(joint.local_anchor_b);
  checksum.Add(joint.length);
  checksum.Add(joint.stiffness);
  checksum.Add(joint.damping);
  checksum.Add(joint.impulse);
  checksum.Add(joint.type);
  checksum.Add(joint.collide_connected);
  return checksum.Get();
}

std::uint32_t GenerateChecksum(const CircleCollider& collider) {
  Checksum checksum;
  checksum.Add(collider.radius);
  return checksum.Get();
}

std::uint32_t GenerateChecksum(const AabbCollider& collider) {
  Checksum checksum;
  checksum.Add(collider.half_size);
  return checksum.Get();
}

std::uint32_t GenerateChecksum(const OrientedBoxCollider& collider) {
  Checksum checksum;
  checksum.Add(collider.half_size);
  return checksum.Get();
}

PhysicsWorld::PhysicsWorld(const PhysicsWorldConfig& config)
    : config_(config),
      default_broadphase_(config.max_colliders, config.max_broadphase_pairs,
                          ConfigAllocator(config)),
      bodies_(MakeList<Body>(ConfigAllocator(config), config.max_bodies)),
      aabbs_(MakeList<AabbCollider>(ConfigAllocator(config),
                                    config.max_colliders)),
      circles_(MakeList<CircleCollider>(ConfigAllocator(config),
                                        config.max_colliders)),
      oriented_boxes_(MakeList<OrientedBoxCollider>(ConfigAllocator(config),
                                                    config.max_colliders)),
      colliders_(
          MakeList<Collider>(ConfigAllocator(config), config.max_colliders)),
      touching_pairs_(MakeList<TouchingPair>(ConfigAllocator(config),
                                             config.max_touching_pairs)),
      new_touching_pairs_(MakeList<TouchingPair>(ConfigAllocator(config),
                                                 config.max_touching_pairs)),
      collider_aabb_cache_(
          MakeList<Aabbf>(ConfigAllocator(config), config.max_colliders)),
      body_area_(MakeList<Scalar>(ConfigAllocator(config), config.max_bodies)),
      collider_mass_cache_(MakeList<ColliderMass>(ConfigAllocator(config),
                                                  config.max_colliders)),
      body_unit_inertia_(
          MakeList<Scalar>(ConfigAllocator(config), config.max_bodies)),
      touching_pair_bucket_count_(MakeList<std::uint32_t>(
          ConfigAllocator(config), config.max_colliders)),
      touching_pair_bucket_start_(MakeList<std::uint32_t>(
          ConfigAllocator(config), config.max_colliders + 1)),
      sorted_touching_pairs_(MakeList<TouchingPair>(ConfigAllocator(config),
                                                    config.max_touching_pairs)),
      contacts_(MakeList<Contact>(ConfigAllocator(config),
                                  config.max_touching_pairs)),
      contact_scratch_(MakeList<internal::ContactPointScratch>(
          ConfigAllocator(config), 2 * config.max_touching_pairs)),
      contact_pairs_(MakeList<ColliderPair>(ConfigAllocator(config),
                                            config.max_touching_pairs)),
      contact_cache_(MakeList<CachedContact>(ConfigAllocator(config),
                                             config.max_touching_pairs)),
      new_contact_cache_(MakeList<CachedContact>(ConfigAllocator(config),
                                                 config.max_touching_pairs)),
      joints_(
          MakeList<DistanceJoint>(ConfigAllocator(config), config.max_joints)),
      joint_scratch_(MakeList<internal::JointScratch>(ConfigAllocator(config),
                                                      config.max_joints)),
      body_joint_filter_(
          MakeList<std::uint8_t>(ConfigAllocator(config), config.max_bodies)) {
  assert(config.max_bodies <= std::numeric_limits<Index>::max() &&
         config.max_colliders <= std::numeric_limits<Index>::max() &&
         config.max_joints <= std::numeric_limits<Index>::max());
  // Fixed size from here on: SortNewTouchingPairs indexes these two directly
  // instead of push_back-ing every step
  touching_pair_bucket_count_.resize(config.max_colliders);
  touching_pair_bucket_start_.resize(config.max_colliders + 1);
  body_area_.resize(config.max_bodies);
  contact_scratch_.resize(2 * config.max_touching_pairs);
  body_unit_inertia_.resize(config.max_bodies);
  joint_scratch_.resize(config.max_joints);
  body_joint_filter_.resize(config.max_bodies);
}

BodyIndex PhysicsWorld::AddBody() {
  const auto slot = AcquireSlot(bodies_, config_.max_bodies, IsFreeBody);
  return slot.has_value() ? BodyIndex{*slot} : kInvalidBodyIndex;
}

void PhysicsWorld::RemoveBody(BodyIndex index) {
  bodies_[index.index()].type = BodyType::kNone;
  for (auto& joint : joints_) {
    if (joint.type != DistanceJointType::kNone &&
        (joint.body_a == index || joint.body_b == index)) {
      joint = DistanceJoint{};
    }
  }
  for (const auto& collider : colliders_) {
    if (collider.body_index == index && collider.type != ShapeType::kNone) {
      RemoveCollider(collider.collider_index);
    }
  }
}

NEKO_HOT bool PhysicsWorld::DetectContact(Body& body1, Collider& collider1,
                                          Body& body2, Collider& collider2,
                                          Contact* contact) {
  bool does_intersect = false;
  contact->bodies[0].body = &body1;
  contact->bodies[1].body = &body2;
  contact->bodies[0].collider = &collider1;
  contact->bodies[1].collider = &collider2;

  contact->restitution = Max(collider1.restitution, collider2.restitution);
  contact->friction = Sqrt(collider1.friction * collider2.friction);
  contact->point_count = 0;

  const bool collision = !(collider1.is_trigger || collider2.is_trigger);
  // The contact details are only needed for solid pairs
  Contact* const details = collision ? contact : nullptr;
  const auto axis_aligned_box = [this](const Body& body,
                                       const Collider& collider) {
    return BoxShape{WorldCenter(body, collider), Vec2f::Right(), Vec2f::Up(),
                    aabbs_[collider.shape_index.index()].half_size};
  };
  const auto oriented_box = [this](const Body& body, const Collider& collider) {
    return BoxShape{WorldCenter(body, collider), body.rotation.x_axis(),
                    body.rotation.y_axis(),
                    oriented_boxes_[collider.shape_index.index()].half_size};
  };

  switch (collider1.type) {
    case ShapeType::kOrientedBox:
      switch (collider2.type) {
        case ShapeType::kOrientedBox:
          does_intersect =
              CollideBoxes(oriented_box(body1, collider1),
                           oriented_box(body2, collider2), details);
          break;
        case ShapeType::kAabb:
          does_intersect =
              CollideBoxes(oriented_box(body1, collider1),
                           axis_aligned_box(body2, collider2), details);
          break;
        case ShapeType::kCircle:
          does_intersect = CollideBoxCircle(
              oriented_box(body1, collider1), WorldCenter(body2, collider2),
              circles_[collider2.shape_index.index()].radius, details);
          break;
        default:
          break;
      }
      break;
    case ShapeType::kAabb:
      switch (collider2.type) {
        case ShapeType::kOrientedBox:
          does_intersect =
              CollideBoxes(axis_aligned_box(body1, collider1),
                           oriented_box(body2, collider2), details);
          break;
        case ShapeType::kAabb: {
          const auto center1 = WorldCenter(body1, collider1);
          const auto center2 = WorldCenter(body2, collider2);
          const auto half1 = aabbs_[collider1.shape_index.index()].half_size;
          const auto half2 = aabbs_[collider2.shape_index.index()].half_size;
          does_intersect = Intersect(Aabbf::FromCenter(center1, half1),
                                     Aabbf::FromCenter(center2, half2));
          if (collision && does_intersect) {
            const auto delta = center1 - center2;
            contact->contact_point = center1 - delta * Scalar{0.5f};

            const auto penetration_x = half1.x() + half2.x() - Abs(delta.x());
            const auto penetration_y = half1.y() + half2.y() - Abs(delta.y());
            if (penetration_x < penetration_y) {
              contact->contact_normal =
                  delta.x() > neko::Scalar{0} ? Vec2f::Right() : Vec2f::Left();
              contact->penetration = penetration_x;
            } else {
              contact->contact_normal =
                  delta.y() > neko::Scalar{0} ? Vec2f::Up() : Vec2f::Down();
              contact->penetration = penetration_y;
            }
            // Manifold: the ends of the overlap along the face, on body 2's
            // face toward body 1 (like the incident face of oriented boxes),
            // all at the same depth
            const bool on_x = penetration_x < penetration_y;
            const auto face =
                center2 + Vec2f{contact->contact_normal.x() * half2.x(),
                                contact->contact_normal.y() * half2.y()};
            const auto low =
                on_x ? Max(center1.y() - half1.y(), center2.y() - half2.y())
                     : Max(center1.x() - half1.x(), center2.x() - half2.x());
            const auto high =
                on_x ? Min(center1.y() + half1.y(), center2.y() + half2.y())
                     : Min(center1.x() + half1.x(), center2.x() + half2.x());
            contact->points[0] = {
                on_x ? Vec2f{face.x(), low} : Vec2f{low, face.y()},
                contact->penetration};
            contact->points[1] = {
                on_x ? Vec2f{face.x(), high} : Vec2f{high, face.y()},
                contact->penetration};
            contact->point_count = low == high ? 1 : 2;
            // Ids: which axis, which side, which end of the overlap
            const auto side = static_cast<std::uint8_t>(
                (on_x ? 2u : 0u) |
                (contact->contact_normal.x() + contact->contact_normal.y() >
                         Scalar{0}
                     ? 4u
                     : 0u));
            contact->points[0].id = side;
            contact->points[1].id = static_cast<std::uint8_t>(side | 1u);
          }
          break;
        }
        case ShapeType::kCircle: {
          const auto center = WorldCenter(body1, collider1);
          const auto half_size =
              aabbs_[collider1.shape_index.index()].half_size;
          const Aabbf aabb1 = Aabbf::FromCenter(center, half_size);
          const Circlef circle =
              Circlef{WorldCenter(body2, collider2),
                      circles_[collider2.shape_index.index()].radius};
          does_intersect = Intersect(circle, aabb1);
          if (!collision || !does_intersect) {
            break;
          }
          // The normal points from the circle (body 2) to the box (body 1)
          const auto closest_point = ClosestPoint(aabb1, circle.position());
          const auto box_to_circle = circle.position() - closest_point;
          if (box_to_circle != Vec2f::Zero()) {
            const auto distance = box_to_circle.Length();
            // One reciprocal and two multiplies instead of Normalized()'s two
            // divisions
            const auto inv_distance = Scalar{1} / distance;
            contact->contact_normal = -(box_to_circle * inv_distance);
            contact->contact_point = closest_point;
            contact->penetration = circle.radius() - distance;
            break;
          }
          // The circle center is inside the box: push it out through the
          // nearest face
          const auto delta = circle.position() - center;
          const auto distance_x = half_size.x() - Abs(delta.x());
          const auto distance_y = half_size.y() - Abs(delta.y());
          if (distance_x < distance_y) {
            const auto outward =
                delta.x() < Scalar{0} ? Vec2f::Left() : Vec2f::Right();
            contact->contact_normal = -outward;
            contact->contact_point = {center.x() + outward.x() * half_size.x(),
                                      circle.position().y()};
            contact->penetration = circle.radius() + distance_x;
          } else {
            const auto outward =
                delta.y() < Scalar{0} ? Vec2f::Down() : Vec2f::Up();
            contact->contact_normal = -outward;
            contact->contact_point = {circle.position().x(),
                                      center.y() + outward.y() * half_size.y()};
            contact->penetration = circle.radius() + distance_y;
          }
          break;
        }
        default:
          break;
      }
      break;
    case ShapeType::kCircle:
      switch (collider2.type) {
        case ShapeType::kAabb:
        case ShapeType::kOrientedBox: {
          // The box is body 1 of the contact, so the normal points from the
          // circle toward the box
          does_intersect =
              DetectContact(body2, collider2, body1, collider1, contact);
          break;
        }
        case ShapeType::kCircle: {
          const Circlef c1 = {WorldCenter(body1, collider1),
                              circles_[collider1.shape_index.index()].radius};
          const Circlef c2 = {WorldCenter(body2, collider2),
                              circles_[collider2.shape_index.index()].radius};

          does_intersect = Intersect(c1, c2);
          if (collision && does_intersect) {
            const auto midline = c1.position() - c2.position();
            const auto distance = midline.Length();
            // Same position: any direction separates them, pick a deterministic
            // one. Otherwise one reciprocal and two multiplies instead of
            // dividing both components by distance.
            contact->contact_normal = distance == Scalar{0}
                                          ? Vec2f::Up()
                                          : midline * (Scalar{1} / distance);
            contact->contact_point = c1.position() - midline * Scalar{0.5f};
            contact->penetration = c1.radius() + c2.radius() - distance;
          }
          break;
        }
        default:
          break;
      }
      break;
    default:
      break;
  }
  // Circles, and boxes whose clipping kept no point (only when rounding
  // leaves the corners a hair above the face): one point, the deepest one
  if (collision && does_intersect && contact->point_count == 0) {
    contact->points[0] = {contact->contact_point, contact->penetration};
    contact->point_count = 1;
  }
  return does_intersect;
}

NEKO_HOT void PhysicsWorld::ResolveNarrowphase() {
  NEKO_PROFILE_FUNCTION();
  new_touching_pairs_.clear();
  contacts_.clear();
  contact_pairs_.clear();
  const bool has_joint_filter = UpdateJointFilter();
  for (const auto& pair : Broadphase().GetPossiblePairs()) {
    auto& collider1 = colliders_[pair.c1().index()];
    auto& collider2 = colliders_[pair.c2().index()];

    auto& body1 = bodies_[collider1.body_index.index()];
    auto& body2 = bodies_[collider2.body_index.index()];
    if (!body1.is_active || !body2.is_active) {
      continue;
    }
    if (body1.type == BodyType::kStatic && body2.type == BodyType::kStatic) {
      continue;
    }
    if (collider1.body_index == collider2.body_index) {
      continue;
    }
    const bool is_trigger = collider1.is_trigger || collider2.is_trigger;
    if (has_joint_filter && !is_trigger &&
        AreJointFiltered(collider1.body_index, collider2.body_index)) {
      continue;
    }
    // Contacts only live during this step, so their pointers never outlive the
    // containers
    Contact contact{};
    const auto t_detect_start = StepProfileNow();
    const bool does_intersect =
        DetectContact(body1, collider1, body2, collider2, &contact);
    StepProfileMark(StepPhase::kDetectContact, t_detect_start);
    if (!does_intersect) {
      continue;
    }
    if (!is_trigger) {
      if (contacts_.size() < config_.max_touching_pairs) {
        contacts_.push_back(contact);
        contact_pairs_.push_back(pair);
      } else {
        assert(false && "PhysicsWorldConfig::max_touching_pairs reached");
      }
    }
    if (new_touching_pairs_.size() < config_.max_touching_pairs) {
      new_touching_pairs_.push_back({pair, is_trigger});
    } else {
      assert(false && "PhysicsWorldConfig::max_touching_pairs reached");
    }
  }
  // All the step's contacts together: each pass sees the others' pushes
  const auto t_resolve_start = StepProfileNow();
  if (config_.warm_starting) {
    WarmStartContacts();
  }
  SolveConstraints(contacts_, {contact_scratch_.data(), 2 * contacts_.size()},
                   joints_, bodies_, joint_scratch_,
                   {.dt = config_.dt,
                    .velocity_iterations = config_.velocity_iterations,
                    .baumgarte = config_.baumgarte,
                    .penetration_slop = config_.penetration_slop,
                    .restitution_threshold = config_.restitution_threshold,
                    .warm_starting = config_.warm_starting,
                    .joint_hertz = config_.joint_hertz,
                    .joint_damping_ratio = config_.joint_damping_ratio});
  if (config_.warm_starting) {
    CacheContactImpulses();
  }
  StepProfileMark(StepPhase::kResolve, t_resolve_start);

  const auto t_sort_start = StepProfileNow();
  SortNewTouchingPairs();
  NotifyContactChanges();
  StepProfileMark(StepPhase::kSortAndNotify, t_sort_start);
  std::swap(touching_pairs_, new_touching_pairs_);
}

NEKO_HOT void PhysicsWorld::SortNewTouchingPairs() {
  std::fill(touching_pair_bucket_count_.begin(),
            touching_pair_bucket_count_.end(), std::uint32_t{0});
  for (const auto& touching_pair : new_touching_pairs_) {
    ++touching_pair_bucket_count_[touching_pair.pair.c1().index()];
  }

  // Exclusive prefix sum: bucketStart[k] is where collider k's pairs begin in
  // the sorted output
  std::uint32_t running_total = 0;
  for (std::size_t k = 0; k < touching_pair_bucket_count_.size(); ++k) {
    touching_pair_bucket_start_[k] = running_total;
    running_total += touching_pair_bucket_count_[k];
  }
  touching_pair_bucket_start_.back() = running_total;

  sorted_touching_pairs_.resize(new_touching_pairs_.size());
  // Reuse the counts as write cursors, seeded from the bucket starts just
  // computed
  std::copy(touching_pair_bucket_start_.begin(),
            touching_pair_bucket_start_.end() - 1,
            touching_pair_bucket_count_.begin());
  for (const auto& touching_pair : new_touching_pairs_) {
    sorted_touching_pairs_
        [touching_pair_bucket_count_[touching_pair.pair.c1().index()]++] =
            touching_pair;
  }

  // Each bucket only holds one collider's contacts (typically a handful):
  // insertion sort by c2 finishes the order
  for (std::size_t k = 0; k + 1 < touching_pair_bucket_start_.size(); ++k) {
    if (touching_pair_bucket_start_[k] == touching_pair_bucket_start_[k + 1]) {
      continue;
    }
    const auto bucket_begin =
        sorted_touching_pairs_.begin() +
        static_cast<std::ptrdiff_t>(touching_pair_bucket_start_[k]);
    const auto bucket_end =
        sorted_touching_pairs_.begin() +
        static_cast<std::ptrdiff_t>(touching_pair_bucket_start_[k + 1]);
    for (auto it = bucket_begin + 1; it < bucket_end; ++it) {
      const TouchingPair value = *it;
      auto insert_at = it;
      while (insert_at > bucket_begin &&
             value.pair.c2() < (insert_at - 1)->pair.c2()) {
        *insert_at = *(insert_at - 1);
        --insert_at;
      }
      *insert_at = value;
    }
  }
  std::swap(new_touching_pairs_, sorted_touching_pairs_);
}

NEKO_HOT void PhysicsWorld::NotifyContactChanges() const {
  if (contact_listener_ == nullptr) {
    return;
  }
  const auto enter = [this](const TouchingPair& touching_pair) {
    if (touching_pair.is_trigger) {
      contact_listener_->OnTriggerEnter(touching_pair.pair);
    } else {
      contact_listener_->OnCollisionEnter(touching_pair.pair);
    }
  };
  const auto exit = [this](const TouchingPair& touching_pair) {
    if (touching_pair.is_trigger) {
      contact_listener_->OnTriggerExit(touching_pair.pair);
    } else {
      contact_listener_->OnCollisionExit(touching_pair.pair);
    }
  };
  // Both lists are sorted: merging them gives the events in a deterministic
  // order
  auto old_it = touching_pairs_.begin();
  auto new_it = new_touching_pairs_.begin();
  while (old_it != touching_pairs_.end() ||
         new_it != new_touching_pairs_.end()) {
    if (new_it == new_touching_pairs_.end() ||
        (old_it != touching_pairs_.end() && old_it->pair < new_it->pair)) {
      exit(*old_it);
      ++old_it;
    } else if (old_it == touching_pairs_.end() || new_it->pair < old_it->pair) {
      enter(*new_it);
      ++new_it;
    } else {
      // Still touching: a trigger that became solid (or the reverse) ends one
      // kind of contact and starts the other
      if (old_it->is_trigger != new_it->is_trigger) {
        exit(*old_it);
        enter(*new_it);
      }
      ++old_it;
      ++new_it;
    }
  }
}

ColliderIndex PhysicsWorld::AddCircleCollider(BodyIndex body) {
  const auto collider_slot =
      AcquireSlot(colliders_, config_.max_colliders, IsFreeCollider);
  const auto shape_slot =
      collider_slot.has_value()
          ? AcquireSlot(circles_, config_.max_colliders, IsFreeCircle)
          : std::nullopt;
  if (!collider_slot.has_value() || !shape_slot.has_value()) {
    return kInvalidColliderIndex;
  }
  circles_[*shape_slot].radius = Scalar{0};
  auto& collider = colliders_[*collider_slot];
  collider.body_index = body;
  collider.collider_index = ColliderIndex{*collider_slot};
  collider.shape_index = ShapeIndex{*shape_slot};
  collider.type = ShapeType::kCircle;
  return collider.collider_index;
}

ColliderIndex PhysicsWorld::AddAabbCollider(BodyIndex body) {
  const auto collider_slot =
      AcquireSlot(colliders_, config_.max_colliders, IsFreeCollider);
  const auto shape_slot =
      collider_slot.has_value()
          ? AcquireSlot(aabbs_, config_.max_colliders, IsFreeAabb)
          : std::nullopt;
  if (!collider_slot.has_value() || !shape_slot.has_value()) {
    return kInvalidColliderIndex;
  }
  aabbs_[*shape_slot].half_size = Vec2f::Zero();
  auto& collider = colliders_[*collider_slot];
  collider.body_index = body;
  collider.collider_index = ColliderIndex{*collider_slot};
  collider.shape_index = ShapeIndex{*shape_slot};
  collider.type = ShapeType::kAabb;
  return collider.collider_index;
}

ColliderIndex PhysicsWorld::AddOrientedBoxCollider(BodyIndex body) {
  const auto collider_slot =
      AcquireSlot(colliders_, config_.max_colliders, IsFreeCollider);
  const auto shape_slot =
      collider_slot.has_value()
          ? AcquireSlot(oriented_boxes_, config_.max_colliders,
                        IsFreeOrientedBox)
          : std::nullopt;
  if (!collider_slot.has_value() || !shape_slot.has_value()) {
    return kInvalidColliderIndex;
  }
  oriented_boxes_[*shape_slot].half_size = Vec2f::Zero();
  auto& collider = colliders_[*collider_slot];
  collider.body_index = body;
  collider.collider_index = ColliderIndex{*collider_slot};
  collider.shape_index = ShapeIndex{*shape_slot};
  collider.type = ShapeType::kOrientedBox;
  return collider.collider_index;
}

NEKO_HOT void PhysicsWorld::WarmStartContacts() {
  // The cache is sorted by pair: one binary search per contact, then the
  // points matched by feature id
  for (std::size_t c = 0; c < contacts_.size(); ++c) {
    const auto cached = std::lower_bound(
        contact_cache_.begin(), contact_cache_.end(), contact_pairs_[c],
        [](const CachedContact& entry, const ColliderPair& pair) {
          return entry.pair < pair;
        });
    if (cached == contact_cache_.end() || cached->pair != contact_pairs_[c]) {
      continue;
    }
    Contact& contact = contacts_[c];
    for (std::uint8_t i = 0; i < contact.point_count; ++i) {
      for (std::uint8_t k = 0; k < cached->point_count; ++k) {
        if (cached->points[k].id == contact.points[i].id) {
          contact.points[i].normal_impulse = cached->points[k].normal_impulse;
          contact.points[i].tangent_impulse = cached->points[k].tangent_impulse;
          break;
        }
      }
    }
  }
}

NEKO_HOT void PhysicsWorld::CacheContactImpulses() {
  // Sorted by pair like SortNewTouchingPairs, in O(n + max_colliders): a
  // counting sort on c1 writes each entry straight into its bucket, then an
  // insertion sort orders each (short) bucket by c2. Pairs are unique, so this
  // is the same order as a comparison sort, without its ~1000 cycles per
  // entry on the DS. The bucket arrays are SortNewTouchingPairs' scratch,
  // free until it runs after this.
  std::fill(touching_pair_bucket_count_.begin(),
            touching_pair_bucket_count_.end(), std::uint32_t{0});
  for (const auto& pair : contact_pairs_) {
    ++touching_pair_bucket_count_[pair.c1().index()];
  }
  std::uint32_t running_total = 0;
  for (std::size_t k = 0; k < touching_pair_bucket_count_.size(); ++k) {
    touching_pair_bucket_start_[k] = running_total;
    running_total += touching_pair_bucket_count_[k];
  }
  touching_pair_bucket_start_.back() = running_total;
  // Write cursors, seeded from the bucket starts
  std::copy(touching_pair_bucket_start_.begin(),
            touching_pair_bucket_start_.end() - 1,
            touching_pair_bucket_count_.begin());
  new_contact_cache_.resize(contacts_.size());
  for (std::size_t c = 0; c < contacts_.size(); ++c) {
    const Contact& contact = contacts_[c];
    CachedContact& entry = new_contact_cache_
        [touching_pair_bucket_count_[contact_pairs_[c].c1().index()]++];
    entry.pair = contact_pairs_[c];
    entry.point_count = contact.point_count;
    for (std::uint8_t i = 0; i < contact.point_count; ++i) {
      entry.points[i] = {contact.points[i].id, contact.points[i].normal_impulse,
                         contact.points[i].tangent_impulse};
    }
  }
  for (std::size_t k = 0; k + 1 < touching_pair_bucket_start_.size(); ++k) {
    if (touching_pair_bucket_start_[k] == touching_pair_bucket_start_[k + 1]) {
      continue;
    }
    const auto bucket_begin =
        new_contact_cache_.begin() +
        static_cast<std::ptrdiff_t>(touching_pair_bucket_start_[k]);
    const auto bucket_end =
        new_contact_cache_.begin() +
        static_cast<std::ptrdiff_t>(touching_pair_bucket_start_[k + 1]);
    for (auto it = bucket_begin + 1; it < bucket_end; ++it) {
      const CachedContact value = *it;
      auto insert_at = it;
      while (insert_at > bucket_begin && value.pair < (insert_at - 1)->pair) {
        *insert_at = *(insert_at - 1);
        --insert_at;
      }
      *insert_at = value;
    }
  }
  std::swap(contact_cache_, new_contact_cache_);
}

void PhysicsWorld::RemoveCollider(ColliderIndex index) {
  // Forget its cached impulses: a later collider may reuse the index
  std::erase_if(contact_cache_, [index](const CachedContact& entry) {
    return entry.pair.c1() == index || entry.pair.c2() == index;
  });
  auto& collider = colliders_[index.index()];
  switch (collider.type) {
    case ShapeType::kAabb:
      aabbs_[collider.shape_index.index()] = AabbCollider{};
      break;
    case ShapeType::kCircle:
      circles_[collider.shape_index.index()] = CircleCollider{};
      break;
    case ShapeType::kOrientedBox:
      oriented_boxes_[collider.shape_index.index()] = OrientedBoxCollider{};
      break;
    default:
      break;
  }
  collider = Collider{};
}

JointIndex PhysicsWorld::AddDistanceJoint(BodyIndex body_a, BodyIndex body_b) {
  const auto slot = AcquireSlot(joints_, config_.max_joints, IsFreeJoint);
  if (!slot.has_value()) {
    return kInvalidJointIndex;
  }
  DistanceJoint& joint = joints_[*slot];
  joint.body_a = body_a;
  joint.body_b = body_b;
  joint.type = DistanceJointType::kRigid;
  joint.length =
      (bodies_[body_b.index()].position - bodies_[body_a.index()].position)
          .Length();
  return JointIndex{*slot};
}

void PhysicsWorld::RemoveJoint(JointIndex index) {
  joints_[index.index()] = DistanceJoint{};
}

std::array<Vec2f, 2> PhysicsWorld::JointAnchors(JointIndex index) const {
  const DistanceJoint& joint = joints_[index.index()];
  const Body& body_a = bodies_[joint.body_a.index()];
  const Body& body_b = bodies_[joint.body_b.index()];
  return {body_a.position + body_a.rotation.Rotate(joint.local_anchor_a),
          body_b.position + body_b.rotation.Rotate(joint.local_anchor_b)};
}

Scalar PhysicsWorld::JointDistance(JointIndex index) const {
  const auto anchors = JointAnchors(index);
  return (anchors[1] - anchors[0]).Length();
}

void PhysicsWorld::ApplySpringForces() {
  for (const DistanceJoint& joint : joints_) {
    if (joint.type != DistanceJointType::kSpring) {
      continue;
    }
    Body& body_a = bodies_[joint.body_a.index()];
    Body& body_b = bodies_[joint.body_b.index()];
    if (!body_a.is_active || !body_b.is_active) {
      continue;
    }
    const Vec2f r_a = body_a.rotation.Rotate(joint.local_anchor_a);
    const Vec2f r_b = body_b.rotation.Rotate(joint.local_anchor_b);
    const Vec2f delta = (body_b.position + r_b) - (body_a.position + r_a);
    const Scalar distance = delta.Length();
    // No direction to pull along when the anchors meet: no force
    if (distance == Scalar{0}) {
      continue;
    }
    const Vec2f axis{delta.x() / distance, delta.y() / distance};
    // Separation speed of the anchors, v + w x r of each
    const Vec2f velocity_a =
        body_a.velocity + r_a.Perpendicular2() * body_a.angular_velocity;
    const Vec2f velocity_b =
        body_b.velocity + r_b.Perpendicular2() * body_b.angular_velocity;
    const Scalar separation = Vec2f::Dot(velocity_b - velocity_a, axis);
    // Hooke's law plus damping: positive pulls the anchors together
    const Vec2f pull = axis * (joint.stiffness * (distance - joint.length) +
                               joint.damping * separation);
    // Static and kinematic bodies ignore forces: leave theirs untouched
    if (body_a.type == BodyType::kDynamic) {
      body_a.force += pull;
      body_a.torque += Vec2f::Cross(r_a, pull);
    }
    if (body_b.type == BodyType::kDynamic) {
      body_b.force -= pull;
      body_b.torque -= Vec2f::Cross(r_b, pull);
    }
  }
}

bool PhysicsWorld::UpdateJointFilter() {
  bool has_filter = false;
  for (const DistanceJoint& joint : joints_) {
    if (joint.type != DistanceJointType::kNone && !joint.collide_connected) {
      if (!has_filter) {
        std::fill(body_joint_filter_.begin(), body_joint_filter_.end(),
                  std::uint8_t{0});
        has_filter = true;
      }
      body_joint_filter_[joint.body_a.index()] = 1;
      body_joint_filter_[joint.body_b.index()] = 1;
    }
  }
  return has_filter;
}

bool PhysicsWorld::AreJointFiltered(BodyIndex body1, BodyIndex body2) const {
  // Most pairs have a body without joints: one lookup each, no scan
  if (body_joint_filter_[body1.index()] == 0 ||
      body_joint_filter_[body2.index()] == 0) {
    return false;
  }
  for (const DistanceJoint& joint : joints_) {
    if (joint.type != DistanceJointType::kNone && !joint.collide_connected &&
        ((joint.body_a == body1 && joint.body_b == body2) ||
         (joint.body_a == body2 && joint.body_b == body1))) {
      return true;
    }
  }
  return false;
}

void PhysicsWorld::ApplyForceAtPoint(BodyIndex index, Vec2f force,
                                     Vec2f point) {
  Body& body = bodies_[index.index()];
  body.force += force;
  body.torque += Vec2f::Cross(point - body.position, force);
}

void PhysicsWorld::ApplyImpulseAtPoint(BodyIndex index, Vec2f impulse,
                                       Vec2f point) {
  Body& body = bodies_[index.index()];
  if (body.type != BodyType::kDynamic) {
    return;
  }
  switch (body.inertia_mode) {
    case InertiaMode::kFromColliders:
      body.inverse_inertia = ComputeInverseInertia(index);
      break;
    case InertiaMode::kFixedRotation:
      body.inverse_inertia = Scalar{0};
      break;
    default:
      break;
  }
  body.velocity += impulse * body.inverse_mass;
  body.angular_velocity +=
      Vec2f::Cross(point - body.position, impulse) * body.inverse_inertia;
}

NEKO_HOT PhysicsWorld::ColliderMass PhysicsWorld::GetColliderMass(
    const Collider& collider) const {
  if (collider.is_trigger) {
    return {};
  }
  // Per kilogram, before the mass: the squares are about the shape's size,
  // so they overflow only past about 100 m
  const Vec2f offset = collider.offset;
  const Scalar offset_square =
      offset.x() * offset.x() + offset.y() * offset.y();
  switch (collider.type) {
    case ShapeType::kCircle: {
      const Scalar radius = circles_[collider.shape_index.index()].radius;
      const Scalar radius_square = radius * radius;
      // Disk: m r^2 / 2
      return {Pi<Scalar>() * radius_square,
              DivideBy<2>(radius_square) + offset_square};
    }
    case ShapeType::kAabb:
    case ShapeType::kOrientedBox: {
      const Vec2f half_size =
          collider.type == ShapeType::kAabb
              ? aabbs_[collider.shape_index.index()].half_size
              : oriented_boxes_[collider.shape_index.index()].half_size;
      // Box: m (w^2 + h^2) / 12 = m (half_w^2 + half_h^2) / 3
      return {Scalar{4} * half_size.x() * half_size.y(),
              DivideBy<3>(half_size.x() * half_size.x() +
                          half_size.y() * half_size.y()) +
                  offset_square};
    }
    default:
      return {};
  }
}

Scalar PhysicsWorld::ComputeInverseInertia(BodyIndex index) const {
  // Same sums, in the same collider order, as UpdateInertia: same bits
  Scalar total_area{};
  for (const auto& collider : colliders_) {
    if (collider.type != ShapeType::kNone && collider.body_index == index) {
      total_area += GetColliderMass(collider).area;
    }
  }
  Scalar unit_inertia{};
  for (const auto& collider : colliders_) {
    if (collider.type != ShapeType::kNone && collider.body_index == index) {
      const auto mass = GetColliderMass(collider);
      unit_inertia += AreaWeighted(mass.unit_inertia, mass.area, total_area);
    }
  }
  return InverseInertia(bodies_[index.index()].inverse_mass, unit_inertia);
}

Vec2f PhysicsWorld::ColliderCenter(ColliderIndex index) const {
  const Collider& collider = colliders_[index.index()];
  return WorldCenter(bodies_[collider.body_index.index()], collider);
}

NEKO_HOT void PhysicsWorld::UpdateInertia() {
  const auto is_computed = [](const Body& body) {
    return body.is_active && body.type == BodyType::kDynamic &&
           body.inertia_mode == InertiaMode::kFromColliders;
  };
  const auto body_count = static_cast<std::ptrdiff_t>(bodies_.size());
  std::fill_n(body_area_.begin(), body_count, Scalar{0});
  std::fill_n(body_unit_inertia_.begin(), body_count, Scalar{0});
  // Two passes over the colliders instead of one scan per body: the total
  // area of each body first, then each collider's share of the inertia. Each
  // collider's mass is computed once, in the first pass, and kept for the
  // second (zero for colliders that don't count).
  collider_mass_cache_.resize(colliders_.size());
  for (std::size_t i = 0; i < colliders_.size(); ++i) {
    const Collider& collider = colliders_[i];
    const bool counts = collider.type != ShapeType::kNone &&
                        is_computed(bodies_[collider.body_index.index()]);
    collider_mass_cache_[i] =
        counts ? GetColliderMass(collider) : ColliderMass{};
    if (counts) {
      body_area_[collider.body_index.index()] += collider_mass_cache_[i].area;
    }
  }
  for (std::size_t i = 0; i < colliders_.size(); ++i) {
    const ColliderMass& mass = collider_mass_cache_[i];
    if (mass.area != Scalar{0}) {
      const auto body_index = colliders_[i].body_index.index();
      body_unit_inertia_[body_index] +=
          AreaWeighted(mass.unit_inertia, mass.area, body_area_[body_index]);
    }
  }
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    Body& body = bodies_[i];
    if (is_computed(body)) {
      body.inverse_inertia =
          InverseInertia(body.inverse_mass, body_unit_inertia_[i]);
    } else if (body.inertia_mode == InertiaMode::kFixedRotation) {
      body.inverse_inertia = Scalar{0};
    }
  }
}

NEKO_HOT void PhysicsWorld::Step() {
  NEKO_PROFILE_FUNCTION();
  const auto dt = config_.dt;

  const auto t_inertia_start = StepProfileNow();
  UpdateInertia();
  StepProfileMark(StepPhase::kInertia, t_inertia_start);

  // Semi-implicit Euler around the contacts: velocities first (forces and
  // gravity), then the contact solver corrects them, then the positions move
  // with the corrected velocities. A resting body ends the step at rest
  // instead of sinking by one step of gravity and being pushed back.
  auto t_integration_start = StepProfileNow();
  ApplySpringForces();
  for (auto& body : bodies_) {
    if (!body.is_active || body.type != BodyType::kDynamic) {
      continue;
    }
    // Gravity is an acceleration: no division by the inverse mass
    body.velocity += (config_.gravity + body.force * body.inverse_mass) * dt;
    // The same for the spin: torque / inertia is the angular acceleration
    if (body.torque != Scalar{0}) {
      body.angular_velocity += body.torque * body.inverse_inertia * dt;
    }
    body.force = Vec2f::Zero();
    body.torque = Scalar{0};
  }
  StepProfileMark(StepPhase::kIntegration, t_integration_start);

  ResolveBroadphase();
  ResolveNarrowphase();

  t_integration_start = StepProfileNow();
  for (auto& body : bodies_) {
    if (!body.is_active || (body.type != BodyType::kDynamic &&
                            body.type != BodyType::kKinematic)) {
      continue;
    }
    body.position += body.velocity * dt;
    Turn(body, dt);
  }
  StepProfileMark(StepPhase::kIntegration, t_integration_start);
}

void PhysicsWorld::Clear() {
  bodies_.clear();
  aabbs_.clear();
  oriented_boxes_.clear();
  circles_.clear();
  colliders_.clear();
  touching_pairs_.clear();
  new_touching_pairs_.clear();
  contact_cache_.clear();
  joints_.clear();
}

NEKO_HOT Aabbf PhysicsWorld::GetAabb(const Collider& collider) const {
  const Body& body = bodies_[collider.body_index.index()];
  const auto center = WorldCenter(body, collider);
  if (collider.type == ShapeType::kAabb) {
    return Aabbf::FromCenter(center,
                             aabbs_[collider.shape_index.index()].half_size);
  }
  if (collider.type == ShapeType::kOrientedBox) {
    // The tight box around the turned box: each half extent adds the shadows
    // of both turned half sides on that world axis
    const Vec2f half_size =
        oriented_boxes_[collider.shape_index.index()].half_size;
    const Scalar cos = Abs(body.rotation.cos());
    const Scalar sin = Abs(body.rotation.sin());
    return Aabbf::FromCenter(center,
                             {cos * half_size.x() + sin * half_size.y(),
                              sin * half_size.x() + cos * half_size.y()});
  }
  return Circlef{center, circles_[collider.shape_index.index()].radius}
      .GetAabb();
}

NEKO_HOT void PhysicsWorld::ResolveBroadphase() {
  NEKO_PROFILE_FUNCTION();
  const auto t_broadphase_start = StepProfileNow();
  auto& broadphase = Broadphase();
  broadphase.Clear();
  collider_aabb_cache_.clear();

  const auto is_inserted = [this](const Collider& collider) {
    return collider.type != ShapeType::kNone &&
           bodies_[collider.body_index.index()].is_active;
  };
  // World box from the collider extents, starting from the first one (no
  // infinite bounds to overflow). Each Aabbf is computed once here and cached,
  // since a BSH may need SetWorldAabb called before any Insert.
  bool has_collider = false;
  Aabbf world_box{};
  for (const auto& collider : colliders_) {
    if (!is_inserted(collider)) {
      continue;
    }
    const auto aabb = GetAabb(collider);
    collider_aabb_cache_.push_back(aabb);
    if (!has_collider) {
      world_box = aabb;
      has_collider = true;
      continue;
    }
    world_box.set_min_bound(
        {Min(world_box.min_bound().x(), aabb.min_bound().x()),
         Min(world_box.min_bound().y(), aabb.min_bound().y())});
    world_box.set_max_bound(
        {Max(world_box.max_bound().x(), aabb.max_bound().x()),
         Max(world_box.max_bound().y(), aabb.max_bound().y())});
  }
  if (has_collider) {
    broadphase.SetWorldAabb(world_box);
    std::size_t cache_index = 0;
    for (const auto& collider : colliders_) {
      if (is_inserted(collider)) {
        broadphase.Insert(
            {collider_aabb_cache_[cache_index], collider.collider_index});
        ++cache_index;
      }
    }
  }
  broadphase.CalculatePairs();
  StepProfileMark(StepPhase::kBroadphase, t_broadphase_start);
}

void PhysicsWorld::CopyFrom(const PhysicsWorld& physics_world) {
  // Assignment reuses the reserved capacity: no allocation when both worlds
  // share a config
  bodies_ = physics_world.bodies_;
  aabbs_ = physics_world.aabbs_;
  oriented_boxes_ = physics_world.oriented_boxes_;
  circles_ = physics_world.circles_;
  colliders_ = physics_world.colliders_;
  touching_pairs_ = physics_world.touching_pairs_;
  // Warm starting reads it next step: part of the simulation state
  contact_cache_ = physics_world.contact_cache_;
  joints_ = physics_world.joints_;
}

std::uint32_t PhysicsWorld::GenerateChecksum() const {
  Checksum checksum;
  for (const auto& body : bodies_) {
    checksum.Add(neko::GenerateChecksum(body));
  }
  for (const auto& collider : colliders_) {
    checksum.Add(neko::GenerateChecksum(collider));
  }
  for (const auto& aabb : aabbs_) {
    checksum.Add(neko::GenerateChecksum(aabb));
  }
  for (const auto& circle : circles_) {
    checksum.Add(neko::GenerateChecksum(circle));
  }
  // Empty in worlds without oriented boxes, which keep their old checksums
  for (const auto& oriented_box : oriented_boxes_) {
    checksum.Add(neko::GenerateChecksum(oriented_box));
  }
  for (const auto& touching_pair : touching_pairs_) {
    checksum.Add(touching_pair.pair.c1());
    checksum.Add(touching_pair.pair.c2());
    checksum.Add(touching_pair.is_trigger);
  }
  for (const auto& entry : contact_cache_) {
    checksum.Add(entry.pair.c1());
    checksum.Add(entry.pair.c2());
    checksum.Add(static_cast<std::uint32_t>(entry.point_count));
    for (std::uint8_t i = 0; i < entry.point_count; ++i) {
      checksum.Add(static_cast<std::uint32_t>(entry.points[i].id));
      checksum.Add(entry.points[i].normal_impulse);
      checksum.Add(entry.points[i].tangent_impulse);
    }
  }
  // Empty in worlds without joints, which keep their old checksums
  for (const auto& joint : joints_) {
    checksum.Add(neko::GenerateChecksum(joint));
  }
  return checksum.Get();
}
}  // namespace neko
