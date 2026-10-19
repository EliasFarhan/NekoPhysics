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
constexpr bool IsFreeRevoluteJoint(const RevoluteJoint& joint) {
  return !joint.is_used;
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
  using Traits = ScalarTraits<Scalar>;
  if constexpr (Traits::kIsFixedPoint) {
    return Traits::FromBits(Traits::ToBits(value) / N);
  } else {
    return value / Scalar{N};
  }
}

// Velocity of the body's material point at a world point: v + w x r
Vec2f PointVelocity(const Body& body, Vec2f point) {
  return body.velocity +
         (point - body.position).Perpendicular2() * body.angular_velocity;
}

// The one-way rule of a one-way collider of body against the other shape
// (of the body other) at the moment the pair starts touching: true when the
// pair may be solid. Solid needs the normal toward_other, from the one-way
// collider toward the other shape, strictly within half the arc of the world
// one-way normal (by more than the scalar's rounding tolerance, so a side hit
// on a half plane passes with both scalars), and the other shape not moving
// along that normal relative to the one-way collider at the contact point.
// Shared by the narrowphase and the sweeps of bullets.
bool OneWayRuleAllowsSolid(const Body& body, const Collider& one_way,
                           const Body& other, Vec2f toward_other,
                           Vec2f contact_point) {
  const Vec2f normal = body.rotation.Rotate(one_way.one_way_normal);
  const Scalar threshold =
      Cos(Radian{DivideBy<2>(one_way.one_way_arc.value())}) +
      ScalarTraits<Scalar>::Tolerance();
  if (!(Vec2f::Dot(toward_other, normal) > threshold)) {
    return false;
  }
  const Vec2f relative =
      PointVelocity(other, contact_point) - PointVelocity(body, contact_point);
  return Vec2f::Dot(relative, normal) <= Scalar{0};
}

// The one-way rule of the collider on one side of a contact (0 for the
// contact's body 1, 1 for its body 2); always true for a collider that isn't
// one-way
bool OneWayAllowsSolid(const Contact& contact, std::size_t side) {
  const Collider& one_way = *contact.bodies[side].collider;
  if (!one_way.is_one_way) {
    return true;
  }
  // The contact normal points from body 2 toward body 1
  return OneWayRuleAllowsSolid(
      *contact.bodies[side].body, one_way, *contact.bodies[1 - side].body,
      side == 0 ? -contact.contact_normal : contact.contact_normal,
      contact.contact_point);
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

// num / den for a ray parameter, den != 0. Only fractions between 0 and 1
// matter, so a quotient beyond 2 is returned as +-2: dividing by a tiny
// component of the ray direction would overflow Fixed16 otherwise.
Scalar RayFraction(Scalar num, Scalar den) {
  if (Abs(num) >= Scalar{2} * Abs(den)) {
    return (num < Scalar{0}) != (den < Scalar{0}) ? Scalar{-2} : Scalar{2};
  }
  return num / den;
}

// Where the segment start + t delta, t from 0 to 1, enters the box centered
// at the origin with that half size (all in the box's frame). Slab method:
// along each axis the segment is between the two sides for t in an interval;
// it is in the box on the overlap of both intervals, which begins at the
// latest entry. The entry axis gives the normal (x wins a tie). A start
// inside the box (boundary included) hits at t = 0 with a zero normal.
struct LocalHit {
  Scalar fraction;
  Vec2f normal;
};
std::optional<LocalHit> RaycastLocalBox(Vec2f start, Vec2f delta,
                                        Vec2f half_size) {
  if (Abs(start.x()) <= half_size.x() && Abs(start.y()) <= half_size.y()) {
    return LocalHit{Scalar{0}, Vec2f::Zero()};
  }
  Scalar enter{-2};
  Scalar exit{2};
  Vec2f normal{};
  const std::array<Vec2f, 2> axes = {Vec2f::Right(), Vec2f::Up()};
  for (std::size_t axis = 0; axis < 2; ++axis) {
    const Scalar origin = axis == 0 ? start.x() : start.y();
    const Scalar direction = axis == 0 ? delta.x() : delta.y();
    const Scalar half = axis == 0 ? half_size.x() : half_size.y();
    if (direction == Scalar{0}) {
      // Parallel to these sides: inside the slab for every t, or never
      if (Abs(origin) > half) {
        return std::nullopt;
      }
      continue;
    }
    const Scalar to_low = RayFraction(-half - origin, direction);
    const Scalar to_high = RayFraction(half - origin, direction);
    const bool forward = direction > Scalar{0};
    const Scalar axis_enter = forward ? to_low : to_high;
    const Scalar axis_exit = forward ? to_high : to_low;
    if (axis_enter > enter) {
      enter = axis_enter;
      normal = forward ? -axes[axis] : axes[axis];
    }
    exit = Min(exit, axis_exit);
  }
  if (enter > exit || enter < Scalar{0} || enter > Scalar{1}) {
    return std::nullopt;
  }
  return LocalHit{enter, normal};
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

// The sweeps of bullets (Body::is_bullet). A collider's shape at one place:
// a circle (is_circle, its center in box.center) or a box.
struct SweptShape {
  BoxShape box;
  Scalar radius;
  bool is_circle;
};

// The world box around a swept shape
Aabbf Bounds(const SweptShape& shape) {
  if (shape.is_circle) {
    return Aabbf::FromCenter(shape.box.center, {shape.radius, shape.radius});
  }
  const BoxShape& box = shape.box;
  return Aabbf::FromCenter(box.center,
                           {box.half_size.x() * Abs(box.axis_x.x()) +
                                box.half_size.y() * Abs(box.axis_y.x()),
                            box.half_size.x() * Abs(box.axis_x.y()) +
                                box.half_size.y() * Abs(box.axis_y.y())});
}

// The box covering box at its place and moved by offset
Aabbf Swept(const Aabbf& box, Vec2f offset) {
  const Vec2f low = box.min_bound();
  const Vec2f high = box.max_bound();
  return {
      {Min(low.x(), low.x() + offset.x()), Min(low.y(), low.y() + offset.y())},
      {Max(high.x(), high.x() + offset.x()),
       Max(high.y(), high.y() + offset.y())}};
}

// Where a shape moving by delta first touches a still target: the fraction
// of delta, from 0 to 1, and the normal there, from the target toward the
// moving shape. A fraction of 0 (zero normal) means they overlap at the
// start; touching counts, like contacts.
struct SweepHit {
  Scalar fraction;
  Vec2f normal;
};

// The point start + t delta against a circle: the closest approach of the
// line to the center, then back by sqrt(radius^2 - distance^2), as in
// Raycast
std::optional<SweepHit> SweepPointCircle(Vec2f start, Vec2f delta, Vec2f center,
                                         Scalar radius) {
  const Vec2f to_start = start - center;
  if (CompareLength(to_start, radius) <= 0) {
    return SweepHit{Scalar{0}, Vec2f::Zero()};
  }
  const Scalar length = delta.Length();
  if (length == Scalar{0}) {
    return std::nullopt;
  }
  const Vec2f direction{delta.x() / length, delta.y() / length};
  const Scalar along = -Vec2f::Dot(to_start, direction);
  const Scalar off_line = Abs(Vec2f::Cross(to_start, direction));
  if (off_line > radius) {
    return std::nullopt;
  }
  const Scalar distance =
      along - Sqrt((radius - off_line) * (radius + off_line));
  if (distance < Scalar{0} || distance > length) {
    return std::nullopt;
  }
  const Vec2f outward = start + direction * distance - center;
  return SweepHit{Min(distance / length, Scalar{1}),
                  radius > Scalar{0}
                      ? Vec2f{outward.x() / radius, outward.y() / radius}
                      : -direction};
}

// A moving circle against a box, in the box's frame: the circle's center
// against the box grown by the radius with rounded corners (their Minkowski
// sum). The slab test of the grown box finds the entry; an entry in a corner
// square (beyond the box on both axes) is replaced by the center against the
// circle of that corner, and missing it misses the shape.
std::optional<SweepHit> SweepCircleBox(Vec2f start, Vec2f delta, Scalar radius,
                                       const BoxShape& box) {
  if (CollideBoxCircle(box, start, radius, nullptr)) {
    return SweepHit{Scalar{0}, Vec2f::Zero()};
  }
  const Vec2f to_start = start - box.center;
  const Vec2f local_start{Vec2f::Dot(to_start, box.axis_x),
                          Vec2f::Dot(to_start, box.axis_y)};
  const Vec2f local_delta{Vec2f::Dot(delta, box.axis_x),
                          Vec2f::Dot(delta, box.axis_y)};
  const Vec2f half = box.half_size;
  const auto hit =
      RaycastLocalBox(local_start, local_delta, half + Vec2f{radius, radius});
  if (!hit.has_value()) {
    return std::nullopt;
  }
  SweepHit local{hit->fraction, hit->normal};
  const Vec2f point = local_start + local_delta * hit->fraction;
  if (Abs(point.x()) > half.x() && Abs(point.y()) > half.y()) {
    const Vec2f corner{point.x() < Scalar{0} ? -half.x() : half.x(),
                       point.y() < Scalar{0} ? -half.y() : half.y()};
    const auto corner_hit =
        SweepPointCircle(local_start, local_delta, corner, radius);
    if (!corner_hit.has_value()) {
      return std::nullopt;
    }
    local = *corner_hit;
  }
  return SweepHit{local.fraction, box.axis_x * local.normal.x() +
                                      box.axis_y * local.normal.y()};
}

// A moving box against a box: the separating axis test over time. On each
// face normal of the two boxes the shadows overlap during an interval of the
// fraction; the boxes touch on the overlap of the four intervals, which
// begins at the latest entry (the earliest axis wins a tie). That axis is the
// normal.
std::optional<SweepHit> SweepBoxBox(const BoxShape& moving, Vec2f delta,
                                    const BoxShape& target) {
  if (CollideBoxes(moving, target, nullptr)) {
    return SweepHit{Scalar{0}, Vec2f::Zero()};
  }
  Scalar enter{-2};
  Scalar exit{2};
  Vec2f normal{};
  const std::array<Vec2f, 4> axes = {moving.axis_x, moving.axis_y,
                                     target.axis_x, target.axis_y};
  for (const Vec2f axis : axes) {
    // The target's shadow is ahead by separation - t speed: they overlap
    // while that stays within reach
    const Scalar separation = Vec2f::Dot(target.center - moving.center, axis);
    const Scalar reach =
        ProjectedRadius(moving, axis) + ProjectedRadius(target, axis);
    const Scalar speed = Vec2f::Dot(delta, axis);
    if (speed == Scalar{0}) {
      if (Abs(separation) > reach) {
        return std::nullopt;
      }
      continue;
    }
    const Scalar to_near = RayFraction(separation - reach, speed);
    const Scalar to_far = RayFraction(separation + reach, speed);
    const Scalar axis_enter = Min(to_near, to_far);
    if (axis_enter > enter) {
      enter = axis_enter;
      normal = speed > Scalar{0} ? -axis : axis;
    }
    exit = Min(exit, Max(to_near, to_far));
  }
  if (enter > exit || enter < Scalar{0} || enter > Scalar{1}) {
    return std::nullopt;
  }
  return SweepHit{enter, normal};
}

// Any pair of swept shapes. A moving box against a circle is the circle
// moving the other way against the box, with the normal turned around.
std::optional<SweepHit> SweepShapes(const SweptShape& moving, Vec2f delta,
                                    const SweptShape& target) {
  if (moving.is_circle) {
    if (target.is_circle) {
      return SweepPointCircle(moving.box.center, delta, target.box.center,
                              moving.radius + target.radius);
    }
    return SweepCircleBox(moving.box.center, delta, moving.radius, target.box);
  }
  if (target.is_circle) {
    auto hit =
        SweepCircleBox(target.box.center, -delta, target.radius, moving.box);
    if (hit.has_value()) {
      hit->normal = -hit->normal;
    }
    return hit;
  }
  return SweepBoxBox(moving.box, delta, target.box);
}

// The core of a bullet's shape: shrunk on every side by its smallest half
// size, so a circle's core is its center and a box's is the segment along
// its middle (a point for a square). A bullet sliding on a floor rests a
// little into it; as long as its middle stays above the floor's top, its
// core stays clear of the next tile at a seam, whatever the size of the
// bullet and however deep the solver left it.
SweptShape CoreOf(const SweptShape& shape) {
  SweptShape core = shape;
  if (shape.is_circle) {
    core.radius = Scalar{0};
  } else {
    const Scalar extent = Min(shape.box.half_size.x(), shape.box.half_size.y());
    core.box.half_size = shape.box.half_size - Vec2f{extent, extent};
  }
  return core;
}

// Where a shape moving from moving_start by delta touches the target at
// hit: its center there, back along the normal by its reach
Vec2f TouchPoint(const SweptShape& moving_start, Vec2f delta,
                 const SweepHit& hit) {
  const Vec2f center = moving_start.box.center + delta * hit.fraction;
  const Scalar reach = moving_start.is_circle
                           ? moving_start.radius
                           : ProjectedRadius(moving_start.box, hit.normal);
  return center - hit.normal * reach;
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
  // Only when scaled: worlds that don't scale gravity keep their checksums
  if (body.gravity_scale != Scalar{1}) {
    checksum.Add(body.gravity_scale);
  }
  // Only when set, for the same reason
  if (body.is_bullet) {
    checksum.Add(body.is_bullet);
  }
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
  // Only when filtering: worlds that don't filter keep their checksums
  if (collider.category != Collider{}.category ||
      collider.mask != Collider{}.mask) {
    checksum.Add(static_cast<std::uint32_t>(collider.category));
    checksum.Add(static_cast<std::uint32_t>(collider.mask));
  }
  // Only when used, for the same reason
  if (collider.is_one_way) {
    checksum.Add(collider.is_one_way);
    checksum.Add(collider.one_way_normal);
    checksum.Add(collider.one_way_arc.value());
  }
  if (!collider.is_enabled) {
    checksum.Add(collider.is_enabled);
  }
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

std::uint32_t GenerateChecksum(const RevoluteJoint& joint) {
  Checksum checksum;
  checksum.Add(joint.body_a);
  checksum.Add(joint.body_b);
  checksum.Add(joint.local_anchor_a);
  checksum.Add(joint.local_anchor_b);
  checksum.Add(joint.reference_angle.value());
  checksum.Add(joint.lower_angle.value());
  checksum.Add(joint.upper_angle.value());
  checksum.Add(joint.impulse);
  checksum.Add(joint.lower_impulse);
  checksum.Add(joint.upper_impulse);
  checksum.Add(joint.enable_limit);
  checksum.Add(joint.collide_connected);
  checksum.Add(joint.is_used);
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
      passing_pairs_(MakeList<ColliderPair>(ConfigAllocator(config),
                                            config.max_passing_pairs)),
      new_passing_pairs_(MakeList<ColliderPair>(ConfigAllocator(config),
                                                config.max_passing_pairs)),
      kinematic_targets_(MakeList<KinematicTarget>(ConfigAllocator(config),
                                                   config.max_bodies)),
      bullet_trigger_hits_(MakeList<BulletTriggerHit>(
          ConfigAllocator(config), config.max_bullet_trigger_hits)),
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
      revolute_joints_(MakeList<RevoluteJoint>(ConfigAllocator(config),
                                               config.max_revolute_joints)),
      revolute_scratch_(MakeList<internal::RevoluteJointScratch>(
          ConfigAllocator(config), config.max_revolute_joints)),
      body_joint_filter_(
          MakeList<std::uint8_t>(ConfigAllocator(config), config.max_bodies)) {
  assert(config.max_bodies <= std::numeric_limits<Index>::max() &&
         config.max_colliders <= std::numeric_limits<Index>::max() &&
         config.max_joints <= std::numeric_limits<Index>::max() &&
         config.max_revolute_joints <= std::numeric_limits<Index>::max());
  // Fixed size from here on: SortNewTouchingPairs indexes these two directly
  // instead of push_back-ing every step
  touching_pair_bucket_count_.resize(config.max_colliders);
  touching_pair_bucket_start_.resize(config.max_colliders + 1);
  body_area_.resize(config.max_bodies);
  contact_scratch_.resize(2 * config.max_touching_pairs);
  body_unit_inertia_.resize(config.max_bodies);
  joint_scratch_.resize(config.max_joints);
  revolute_scratch_.resize(config.max_revolute_joints);
  body_joint_filter_.resize(config.max_bodies);
}

BodyIndex PhysicsWorld::AddBody() {
  const auto slot = AcquireSlot(bodies_, config_.max_bodies, IsFreeBody);
  return slot.has_value() ? BodyIndex{*slot} : kInvalidBodyIndex;
}

void PhysicsWorld::RemoveBody(BodyIndex index) {
  bodies_[index.index()].type = BodyType::kNone;
  // A later body may reuse the index
  std::erase_if(kinematic_targets_, [index](const KinematicTarget& target) {
    return target.body == index;
  });
  for (auto& joint : joints_) {
    if (joint.type != DistanceJointType::kNone &&
        (joint.body_a == index || joint.body_b == index)) {
      joint = DistanceJoint{};
    }
  }
  for (auto& joint : revolute_joints_) {
    if (joint.is_used && (joint.body_a == index || joint.body_b == index)) {
      joint = RevoluteJoint{};
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
  new_passing_pairs_.clear();
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
    if (!ShouldCollide(collider1, collider2)) {
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
    // A passing pair of a one-way collider: no contact, no events
    if (!is_trigger && (collider1.is_one_way || collider2.is_one_way) &&
        !IsOneWaySolid(pair, contact)) {
      continue;
    }
    // Past a capacity the first pairs, in broadphase order, are kept
    if (!is_trigger) {
      if (contacts_.size() < config_.max_touching_pairs) {
        contacts_.push_back(contact);
        contact_pairs_.push_back(pair);
      } else {
        ++step_overflow_.contacts;
      }
    }
    if (new_touching_pairs_.size() < config_.max_touching_pairs) {
      new_touching_pairs_.push_back({pair, is_trigger});
    } else {
      ++step_overflow_.touching_pairs;
    }
  }
  // Sorted like the touching pairs, for the binary searches of the next step
  // (insertion sort: the table is short and the pairs are unique)
  for (auto it = new_passing_pairs_.begin(); it != new_passing_pairs_.end();
       ++it) {
    const ColliderPair value = *it;
    auto insert_at = it;
    while (insert_at != new_passing_pairs_.begin() &&
           value < *(insert_at - 1)) {
      *insert_at = *(insert_at - 1);
      --insert_at;
    }
    *insert_at = value;
  }
  std::swap(passing_pairs_, new_passing_pairs_);
  // All the step's contacts together: each pass sees the others' pushes
  const auto t_resolve_start = StepProfileNow();
  if (config_.warm_starting) {
    WarmStartContacts();
  }
  internal::SolveConstraints(
      contacts_, {contact_scratch_.data(), 2 * contacts_.size()}, joints_,
      revolute_joints_, bodies_, joint_scratch_, revolute_scratch_,
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

bool PhysicsWorld::IsOneWaySolid(ColliderPair pair, const Contact& contact) {
  // Passing stays passing until the shapes stop overlapping, solid stays
  // solid while they touch: only a pair that starts touching is decided
  bool is_solid = false;
  if (!IsPassing(pair)) {
    const auto touching = std::lower_bound(
        touching_pairs_.begin(), touching_pairs_.end(), pair,
        [](const TouchingPair& entry, const ColliderPair& value) {
          return entry.pair < value;
        });
    const bool was_solid = touching != touching_pairs_.end() &&
                           touching->pair == pair && !touching->is_trigger;
    is_solid = was_solid ||
               (OneWayAllowsSolid(contact, 0) && OneWayAllowsSolid(contact, 1));
  }
  if (!is_solid) {
    if (new_passing_pairs_.size() < config_.max_passing_pairs) {
      new_passing_pairs_.push_back(pair);
    } else {
      ++step_overflow_.passing_pairs;
    }
  }
  return is_solid;
}

bool PhysicsWorld::IsPassing(ColliderPair pair) const {
  return std::binary_search(passing_pairs_.begin(), passing_pairs_.end(), pair);
}

std::size_t PhysicsWorld::DropThrough(BodyIndex index) {
  std::size_t count = 0;
  for (const TouchingPair& touching : touching_pairs_) {
    if (touching.is_trigger) {
      continue;
    }
    const Collider& collider1 = colliders_[touching.pair.c1().index()];
    const Collider& collider2 = colliders_[touching.pair.c2().index()];
    if ((collider1.body_index != index && collider2.body_index != index) ||
        (!collider1.is_one_way && !collider2.is_one_way)) {
      continue;
    }
    const auto it = std::lower_bound(passing_pairs_.begin(),
                                     passing_pairs_.end(), touching.pair);
    if (it != passing_pairs_.end() && *it == touching.pair) {
      continue;
    }
    if (passing_pairs_.size() >= config_.max_passing_pairs) {
      ++pending_overflow_.passing_pairs;
      continue;
    }
    // Within the reserved capacity: no allocation
    passing_pairs_.insert(it, touching.pair);
    ++count;
  }
  return count;
}

void PhysicsWorld::SetKinematicTarget(BodyIndex index, Vec2f position,
                                      Radian angle) {
  Body& body = bodies_[index.index()];
  if (body.type != BodyType::kKinematic) {
    return;
  }
  // The velocities that cover the distance in one step; the short way around
  // for the angle
  const Scalar dt = config_.dt;
  body.velocity = (position - body.position) / dt;
  body.angular_velocity = WrapAngle(angle - body.rotation.angle()).value() / dt;
  // Sorted by body, one entry each: within the capacity of max_bodies, so no
  // allocation
  const auto it = std::lower_bound(
      kinematic_targets_.begin(), kinematic_targets_.end(), index,
      [](const KinematicTarget& target, BodyIndex value) {
        return target.body.index() < value.index();
      });
  if (it != kinematic_targets_.end() && it->body == index) {
    *it = {index, position, angle};
  } else {
    kinematic_targets_.insert(it, {index, position, angle});
  }
}

void PhysicsWorld::Teleport(BodyIndex index, Vec2f position, Radian angle) {
  Body& body = bodies_[index.index()];
  body.position = position;
  body.rotation.set_angle(angle);
  // What the world remembers of the old place: no warm start from it, no
  // passing state, no target computed from it
  const auto has_body = [this, index](ColliderPair pair) {
    return colliders_[pair.c1().index()].body_index == index ||
           colliders_[pair.c2().index()].body_index == index;
  };
  std::erase_if(contact_cache_, [&has_body](const CachedContact& entry) {
    return has_body(entry.pair);
  });
  std::erase_if(passing_pairs_, has_body);
  std::erase_if(kinematic_targets_, [index](const KinematicTarget& target) {
    return target.body == index;
  });
  for (DistanceJoint& joint : joints_) {
    if (joint.type != DistanceJointType::kNone &&
        (joint.body_a == index || joint.body_b == index)) {
      joint.impulse = Scalar{0};
    }
  }
  for (RevoluteJoint& joint : revolute_joints_) {
    if (joint.is_used && (joint.body_a == index || joint.body_b == index)) {
      joint.impulse = Vec2f::Zero();
      joint.lower_impulse = Scalar{0};
      joint.upper_impulse = Scalar{0};
    }
  }
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
  // Forget its cached impulses and passing pairs: a later collider may reuse
  // the index
  std::erase_if(contact_cache_, [index](const CachedContact& entry) {
    return entry.pair.c1() == index || entry.pair.c2() == index;
  });
  std::erase_if(passing_pairs_, [index](const ColliderPair& pair) {
    return pair.Contains(index);
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
    ++pending_overflow_.joints;
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

void PhysicsWorld::SetJointBodies(JointIndex index, BodyIndex body_a,
                                  BodyIndex body_b) {
  DistanceJoint& joint = joints_[index.index()];
  joint.body_a = body_a;
  joint.body_b = body_b;
  // Last step's impulse belonged to the old bodies
  joint.impulse = Scalar{0};
}

RevoluteJointIndex PhysicsWorld::AddRevoluteJoint(BodyIndex body_a,
                                                  BodyIndex body_b,
                                                  Vec2f world_anchor) {
  const auto slot = AcquireSlot(revolute_joints_, config_.max_revolute_joints,
                                IsFreeRevoluteJoint);
  if (!slot.has_value()) {
    ++pending_overflow_.revolute_joints;
    return kInvalidRevoluteJointIndex;
  }
  const Body& a = bodies_[body_a.index()];
  const Body& b = bodies_[body_b.index()];
  RevoluteJoint& joint = revolute_joints_[*slot];
  joint.body_a = body_a;
  joint.body_b = body_b;
  joint.local_anchor_a = a.rotation.InverseRotate(world_anchor - a.position);
  joint.local_anchor_b = b.rotation.InverseRotate(world_anchor - b.position);
  joint.reference_angle = WrapAngle(b.rotation.angle() - a.rotation.angle());
  joint.is_used = true;
  return RevoluteJointIndex{*slot};
}

void PhysicsWorld::RemoveRevoluteJoint(RevoluteJointIndex index) {
  revolute_joints_[index.index()] = RevoluteJoint{};
}

std::array<Vec2f, 2> PhysicsWorld::RevoluteJointAnchors(
    RevoluteJointIndex index) const {
  const RevoluteJoint& joint = revolute_joints_[index.index()];
  const Body& body_a = bodies_[joint.body_a.index()];
  const Body& body_b = bodies_[joint.body_b.index()];
  return {body_a.position + body_a.rotation.Rotate(joint.local_anchor_a),
          body_b.position + body_b.rotation.Rotate(joint.local_anchor_b)};
}

Radian PhysicsWorld::RevoluteJointAngle(RevoluteJointIndex index) const {
  const RevoluteJoint& joint = revolute_joints_[index.index()];
  return WrapAngle(bodies_[joint.body_b.index()].rotation.angle() -
                   bodies_[joint.body_a.index()].rotation.angle() -
                   joint.reference_angle);
}

void PhysicsWorld::SetRevoluteJointBodies(RevoluteJointIndex index,
                                          BodyIndex body_a, BodyIndex body_b) {
  RevoluteJoint& joint = revolute_joints_[index.index()];
  joint.body_a = body_a;
  joint.body_b = body_b;
  // Last step's impulses belonged to the old bodies
  joint.impulse = Vec2f::Zero();
  joint.lower_impulse = Scalar{0};
  joint.upper_impulse = Scalar{0};
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
  for (const RevoluteJoint& joint : revolute_joints_) {
    if (joint.is_used && !joint.collide_connected) {
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
  for (const RevoluteJoint& joint : revolute_joints_) {
    if (joint.is_used && !joint.collide_connected &&
        ((joint.body_a == body1 && joint.body_b == body2) ||
         (joint.body_a == body2 && joint.body_b == body1))) {
      return true;
    }
  }
  return false;
}

void PhysicsWorld::ApplyForce(BodyIndex index, Vec2f force) {
  bodies_[index.index()].force += force;
}

void PhysicsWorld::ApplyImpulse(BodyIndex index, Vec2f impulse) {
  Body& body = bodies_[index.index()];
  if (body.type != BodyType::kDynamic) {
    return;
  }
  body.velocity += impulse * body.inverse_mass;
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
  if (collider.is_trigger || !collider.is_enabled) {
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
  // This step's counters start with what the calls since the last one
  // dropped
  step_overflow_ = pending_overflow_;
  pending_overflow_ = {};

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
    // Gravity is an acceleration: no division by the inverse mass. A scale of
    // 1 (most bodies) skips the multiplication.
    const Vec2f gravity = body.gravity_scale == Scalar{1}
                              ? config_.gravity
                              : config_.gravity * body.gravity_scale;
    body.velocity += (gravity + body.force * body.inverse_mass) * dt;
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
  // Kinematic targets: exactly on the target, whatever velocity * dt rounded
  // to
  for (const KinematicTarget& target : kinematic_targets_) {
    Body& body = bodies_[target.body.index()];
    if (!body.is_active || body.type != BodyType::kKinematic) {
      continue;
    }
    body.position = target.position;
    if (body.rotation.angle().value() != WrapAngle(target.angle).value()) {
      body.rotation.set_angle(target.angle);
    }
  }
  kinematic_targets_.clear();
  // Continuous collision, counted with the integration
  SolveBullets();
  StepProfileMark(StepPhase::kIntegration, t_integration_start);
}

void PhysicsWorld::SolveBullets() {
  const auto is_bullet = [](const Body& body) {
    return body.is_bullet && body.is_active && body.type == BodyType::kDynamic;
  };
  if (std::none_of(bodies_.begin(), bodies_.end(), is_bullet)) {
    return;
  }
  const Scalar dt = config_.dt;
  const bool has_joint_filter = UpdateJointFilter();
  // Where a bullet stops: half the slop into the shape, so the next step
  // finds the contact (touching counts) without pushing it out
  const Scalar depth =
      Max(DivideBy<2>(config_.penetration_slop), Scalar{1} / Scalar{1024});
  // A collider's shape with its body at position, turned by rotation
  const auto swept_shape = [this](const Collider& collider, Vec2f position,
                                  const Rotation& rotation) {
    const Vec2f center = collider.offset == Vec2f::Zero()
                             ? position
                             : position + rotation.Rotate(collider.offset);
    switch (collider.type) {
      case ShapeType::kCircle:
        return SweptShape{{center, Vec2f::Right(), Vec2f::Up(), Vec2f::Zero()},
                          circles_[collider.shape_index.index()].radius,
                          true};
      case ShapeType::kAabb:
        return SweptShape{{center, Vec2f::Right(), Vec2f::Up(),
                           aabbs_[collider.shape_index.index()].half_size},
                          Scalar{0},
                          false};
      default:
        return SweptShape{
            {center, rotation.x_axis(), rotation.y_axis(),
             oriented_boxes_[collider.shape_index.index()].half_size},
            Scalar{0},
            false};
    }
  };
  // How far a body moved in this step's integration
  const auto displacement_of = [dt](const Body& body) {
    return body.is_active && (body.type == BodyType::kDynamic ||
                              body.type == BodyType::kKinematic)
               ? body.velocity * dt
               : Vec2f::Zero();
  };
  const auto is_live = [this](const Collider& collider) {
    return collider.type != ShapeType::kNone && collider.is_enabled &&
           bodies_[collider.body_index.index()].is_active;
  };

  for (std::size_t b = 0; b < bodies_.size(); ++b) {
    Body& bullet = bodies_[b];
    if (!is_bullet(bullet)) {
      continue;
    }
    const BodyIndex bullet_index{static_cast<Index>(b)};
    // Steps shorter than a quarter of the smallest size overlap anything on
    // the way (Box2D's rule)
    std::optional<Scalar> min_extent;
    for (const Collider& collider : colliders_) {
      if (collider.body_index != bullet_index || !is_live(collider)) {
        continue;
      }
      const SweptShape shape = swept_shape(collider, Vec2f::Zero(), Rotation{});
      const Scalar extent = shape.is_circle ? shape.radius
                                            : Min(shape.box.half_size.x(),
                                                  shape.box.half_size.y());
      min_extent = min_extent.has_value() ? Min(*min_extent, extent) : extent;
    }
    const Vec2f displacement = displacement_of(bullet);
    if (!min_extent.has_value() ||
        CompareLength(displacement, DivideBy<2>(*min_extent)) <= 0) {
      continue;
    }
    const Vec2f end = bullet.position;
    const Vec2f start = end - displacement;
    // Calls visit(moving, other, moving_start, delta, target) for every pair
    // of a collider of the bullet and another collider that passes accept and
    // whose swept bounds meet. The sweep is relative: the other shape
    // (target) stays at its end place and the bullet's moves from
    // moving_start by delta, the difference of the two displacements.
    const auto for_each_candidate = [&](auto accept, auto visit) {
      for (Collider& moving : colliders_) {
        if (moving.body_index != bullet_index || !is_live(moving)) {
          continue;
        }
        const SweptShape moving_end = swept_shape(moving, end, bullet.rotation);
        const Aabbf moving_end_box = Bounds(moving_end);
        for (Collider& other : colliders_) {
          if (other.body_index == bullet_index || !is_live(other) ||
              !ShouldCollide(moving, other) || !accept(moving, other)) {
            continue;
          }
          const Body& other_body = bodies_[other.body_index.index()];
          const Vec2f delta = displacement - displacement_of(other_body);
          const SweptShape target =
              swept_shape(other, other_body.position, other_body.rotation);
          if (!Intersect(Swept(moving_end_box, -delta), Bounds(target))) {
            continue;
          }
          SweptShape moving_start = moving_end;
          moving_start.box.center = moving_end.box.center - delta;
          visit(moving, other, moving_start, delta, target);
        }
      }
    };

    // Solid pairs: the earliest time of impact against static and kinematic
    // bodies (equal times: the lower collider index, then the lower collider
    // of the bullet)
    struct Impact {
      SweepHit hit;
      ColliderIndex other;
      ColliderIndex moving;
    };
    std::optional<Impact> impact;
    for_each_candidate(
        [this, has_joint_filter](const Collider& moving,
                                 const Collider& other) {
          const BodyType type = bodies_[other.body_index.index()].type;
          return !moving.is_trigger && !other.is_trigger &&
                 (type == BodyType::kStatic || type == BodyType::kKinematic) &&
                 !(has_joint_filter &&
                   AreJointFiltered(moving.body_index, other.body_index)) &&
                 !IsPassing({moving.collider_index, other.collider_index});
        },
        [&](const Collider& moving, const Collider& other,
            const SweptShape& moving_start, Vec2f delta,
            const SweptShape& target) {
          // Only an obstacle the core of the shape (its middle) meets
          // counts: a bullet sliding on a floor rests a little into it and
          // grazes the next tile's corner at a seam, which must not stop it
          const SweptShape core = CoreOf(moving_start);
          const auto core_hit = SweepShapes(core, delta, target);
          if (!core_hit.has_value() || core_hit->fraction == Scalar{0}) {
            return;
          }
          // The impact is where the whole shape touches (it overlaps
          // already at the start: it stays there). The core lies inside the
          // shape, so a core hit with no whole-shape hit can only come from
          // rounding: the bullet is held at its start for this step
          const auto touch = SweepShapes(moving_start, delta, target);
          const SweepHit hit = touch.has_value() && touch->fraction > Scalar{0}
                                   ? *touch
                                   : SweepHit{Scalar{0}, core_hit->normal};
          if (impact.has_value() &&
              (hit.fraction > impact->hit.fraction ||
               (hit.fraction == impact->hit.fraction &&
                (other.collider_index > impact->other ||
                 (other.collider_index == impact->other &&
                  moving.collider_index > impact->moving))))) {
            return;
          }
          // A one-way collider blocks only from its solid side, judged at
          // the point where the shapes touch
          const Body& other_body = bodies_[other.body_index.index()];
          const Vec2f point = TouchPoint(moving_start, delta, hit);
          if ((other.is_one_way &&
               !OneWayRuleAllowsSolid(other_body, other, bullet, hit.normal,
                                      point)) ||
              (moving.is_one_way &&
               !OneWayRuleAllowsSolid(bullet, moving, other_body, -hit.normal,
                                      point))) {
            return;
          }
          impact = Impact{hit, other.collider_index, moving.collider_index};
        });
    Scalar end_fraction{1};
    if (impact.has_value()) {
      // Back to the impact, the rest of the step is dropped; the velocity
      // stays for the next step's contact
      end_fraction = impact->hit.fraction;
      bullet.position = start + displacement * end_fraction;
      if (end_fraction > Scalar{0}) {
        bullet.position -= impact->hit.normal * depth;
      }
    }

    // Trigger pairs: every crossing up to the impact, not already touching
    const auto by_pair = [](const TouchingPair& pair1,
                            const TouchingPair& pair2) {
      return pair1.pair < pair2.pair;
    };
    bullet_trigger_hits_.clear();
    for_each_candidate(
        [](const Collider& moving, const Collider& other) {
          return moving.is_trigger || other.is_trigger;
        },
        [&](const Collider& moving, const Collider& other,
            const SweptShape& moving_start, Vec2f delta,
            const SweptShape& target) {
          const auto touch = SweepShapes(moving_start, delta, target);
          if (!touch.has_value() || touch->fraction == Scalar{0}) {
            return;
          }
          const SweepHit& hit = *touch;
          if (hit.fraction > end_fraction ||
              std::binary_search(
                  touching_pairs_.begin(), touching_pairs_.end(),
                  TouchingPair{{moving.collider_index, other.collider_index}},
                  by_pair)) {
            return;
          }
          if (bullet_trigger_hits_.size() >= config_.max_bullet_trigger_hits) {
            ++step_overflow_.bullet_trigger_hits;
            return;
          }
          bullet_trigger_hits_.push_back(
              {hit.fraction, moving.collider_index, other.collider_index});
        });
    // In the order of the crossings (insertion sort: few entries)
    const auto is_before = [](const BulletTriggerHit& first,
                              const BulletTriggerHit& second) {
      if (first.fraction != second.fraction) {
        return first.fraction < second.fraction;
      }
      if (first.other != second.other) {
        return first.other < second.other;
      }
      return first.bullet < second.bullet;
    };
    for (auto it = bullet_trigger_hits_.begin();
         it != bullet_trigger_hits_.end(); ++it) {
      const BulletTriggerHit value = *it;
      auto insert_at = it;
      while (insert_at != bullet_trigger_hits_.begin() &&
             is_before(value, *(insert_at - 1))) {
        *insert_at = *(insert_at - 1);
        --insert_at;
      }
      *insert_at = value;
    }
    for (const BulletTriggerHit& crossing : bullet_trigger_hits_) {
      const TouchingPair touching{{crossing.bullet, crossing.other}, true};
      const auto it = std::lower_bound(
          touching_pairs_.begin(), touching_pairs_.end(), touching, by_pair);
      if (touching_pairs_.size() >= config_.max_touching_pairs) {
        ++step_overflow_.touching_pairs;
        continue;
      }
      // Listed until the next step, which sends the exit if the shapes no
      // longer overlap; within the reserved capacity, no allocation
      touching_pairs_.insert(it, touching);
      if (contact_listener_ != nullptr) {
        contact_listener_->OnTriggerEnter(touching.pair);
      }
    }
  }
}

void PhysicsWorld::Clear() {
  bodies_.clear();
  aabbs_.clear();
  oriented_boxes_.clear();
  circles_.clear();
  colliders_.clear();
  touching_pairs_.clear();
  new_touching_pairs_.clear();
  passing_pairs_.clear();
  kinematic_targets_.clear();
  contact_cache_.clear();
  joints_.clear();
  revolute_joints_.clear();
  step_overflow_ = {};
  pending_overflow_ = {};
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

bool PhysicsWorld::IsQueried(const Collider& collider,
                             QueryFilter filter) const {
  return collider.type != ShapeType::kNone && collider.is_enabled &&
         bodies_[collider.body_index.index()].is_active &&
         (collider.category & filter.mask) != 0 &&
         (filter.include_triggers || !collider.is_trigger);
}

std::size_t PhysicsWorld::OverlapCircle(Vec2f center, Scalar radius,
                                        std::span<ColliderIndex> hits,
                                        QueryFilter filter) const {
  const Circlef query{center, radius};
  std::size_t count = 0;
  for (const Collider& collider : colliders_) {
    if (!IsQueried(collider, filter)) {
      continue;
    }
    const Body& body = bodies_[collider.body_index.index()];
    const Vec2f shape_center = WorldCenter(body, collider);
    bool overlaps = false;
    switch (collider.type) {
      case ShapeType::kCircle:
        overlaps = Intersect(
            query, Circlef{shape_center,
                           circles_[collider.shape_index.index()].radius});
        break;
      case ShapeType::kAabb:
        overlaps = Intersect(
            query,
            Aabbf::FromCenter(shape_center,
                              aabbs_[collider.shape_index.index()].half_size));
        break;
      case ShapeType::kOrientedBox:
        overlaps = CollideBoxCircle(
            BoxShape{shape_center, body.rotation.x_axis(),
                     body.rotation.y_axis(),
                     oriented_boxes_[collider.shape_index.index()].half_size},
            center, radius, nullptr);
        break;
      default:
        break;
    }
    if (overlaps) {
      if (count < hits.size()) {
        hits[count] = collider.collider_index;
      }
      ++count;
    }
  }
  return count;
}

std::size_t PhysicsWorld::OverlapAabb(const Aabbf& box,
                                      std::span<ColliderIndex> hits,
                                      QueryFilter filter) const {
  const BoxShape query{box.GetCenter(), Vec2f::Right(), Vec2f::Up(),
                       box.GetHalfSize()};
  std::size_t count = 0;
  for (const Collider& collider : colliders_) {
    if (!IsQueried(collider, filter)) {
      continue;
    }
    const Body& body = bodies_[collider.body_index.index()];
    const Vec2f shape_center = WorldCenter(body, collider);
    bool overlaps = false;
    switch (collider.type) {
      case ShapeType::kCircle:
        overlaps = Intersect(
            box, Circlef{shape_center,
                         circles_[collider.shape_index.index()].radius});
        break;
      case ShapeType::kAabb:
        overlaps = Intersect(
            box,
            Aabbf::FromCenter(shape_center,
                              aabbs_[collider.shape_index.index()].half_size));
        break;
      case ShapeType::kOrientedBox:
        overlaps = CollideBoxes(
            query,
            BoxShape{shape_center, body.rotation.x_axis(),
                     body.rotation.y_axis(),
                     oriented_boxes_[collider.shape_index.index()].half_size},
            nullptr);
        break;
      default:
        break;
    }
    if (overlaps) {
      if (count < hits.size()) {
        hits[count] = collider.collider_index;
      }
      ++count;
    }
  }
  return count;
}

std::optional<RaycastHit> PhysicsWorld::RaycastCollider(
    const Collider& collider, Vec2f start, Vec2f delta) const {
  const Body& body = bodies_[collider.body_index.index()];
  const Vec2f center = WorldCenter(body, collider);
  RaycastHit hit{.collider = collider.collider_index,
                 .body = collider.body_index};
  switch (collider.type) {
    case ShapeType::kCircle: {
      const Scalar radius = circles_[collider.shape_index.index()].radius;
      const Vec2f to_start = start - center;
      if (CompareLength(to_start, radius) <= 0) {
        hit.point = start;
        return hit;
      }
      const Scalar length = delta.Length();
      if (length == Scalar{0}) {
        return std::nullopt;
      }
      // Unit direction, component by component like the joints: a tiny
      // segment doesn't overflow a reciprocal
      const Vec2f direction{delta.x() / length, delta.y() / length};
      // Distance along the segment to the closest approach of the center, and
      // the distance of the center from the line there. Using the distance to
      // the line instead of |to_start|^2 - along^2 keeps the squares small.
      const Scalar along = -Vec2f::Dot(to_start, direction);
      const Scalar off_line = Abs(Vec2f::Cross(to_start, direction));
      if (off_line > radius) {
        return std::nullopt;
      }
      const Scalar distance =
          along - Sqrt((radius - off_line) * (radius + off_line));
      if (distance < Scalar{0} || distance > length) {
        return std::nullopt;
      }
      hit.fraction = distance / length;
      hit.point = start + direction * distance;
      const Vec2f outward = hit.point - center;
      hit.normal = radius > Scalar{0}
                       ? Vec2f{outward.x() / radius, outward.y() / radius}
                       : -direction;
      return hit;
    }
    case ShapeType::kAabb:
    case ShapeType::kOrientedBox: {
      // An AabbCollider is an oriented box along the world axes
      const bool is_aabb = collider.type == ShapeType::kAabb;
      const Vec2f axis_x = is_aabb ? Vec2f::Right() : body.rotation.x_axis();
      const Vec2f axis_y = is_aabb ? Vec2f::Up() : body.rotation.y_axis();
      const Vec2f half_size =
          is_aabb ? aabbs_[collider.shape_index.index()].half_size
                  : oriented_boxes_[collider.shape_index.index()].half_size;
      const Vec2f to_start = start - center;
      const auto local = RaycastLocalBox(
          {Vec2f::Dot(to_start, axis_x), Vec2f::Dot(to_start, axis_y)},
          {Vec2f::Dot(delta, axis_x), Vec2f::Dot(delta, axis_y)}, half_size);
      if (!local.has_value()) {
        return std::nullopt;
      }
      hit.fraction = local->fraction;
      hit.point = start + delta * local->fraction;
      hit.normal = axis_x * local->normal.x() + axis_y * local->normal.y();
      return hit;
    }
    default:
      return std::nullopt;
  }
}

std::size_t PhysicsWorld::Raycast(Vec2f start, Vec2f end,
                                  std::span<RaycastHit> hits,
                                  QueryFilter filter) const {
  const Vec2f delta = end - start;
  // The segment's box: shapes whose box misses it can't be hit
  const Aabbf segment_box{
      Vec2f{Min(start.x(), end.x()), Min(start.y(), end.y())},
      Vec2f{Max(start.x(), end.x()), Max(start.y(), end.y())}};
  std::size_t count = 0;
  for (const Collider& collider : colliders_) {
    if (!IsQueried(collider, filter) ||
        !Intersect(segment_box, GetAabb(collider))) {
      continue;
    }
    const auto hit = RaycastCollider(collider, start, delta);
    if (!hit.has_value()) {
      continue;
    }
    // Keep the nearest hits.size() in order. Colliders come in index order
    // and only a strictly smaller fraction moves a hit up, so equal
    // fractions stay in index order.
    const std::size_t kept = std::min(count, hits.size());
    if (kept < hits.size() ||
        (kept > 0 && hit->fraction < hits[kept - 1].fraction)) {
      std::size_t slot = kept < hits.size() ? kept : kept - 1;
      while (slot > 0 && hit->fraction < hits[slot - 1].fraction) {
        hits[slot] = hits[slot - 1];
        --slot;
      }
      hits[slot] = *hit;
    }
    ++count;
  }
  return count;
}

std::optional<RaycastHit> PhysicsWorld::RaycastClosest(
    Vec2f start, Vec2f end, QueryFilter filter) const {
  std::array<RaycastHit, 1> hit{};
  if (Raycast(start, end, hit, filter) == 0) {
    return std::nullopt;
  }
  return hit[0];
}

NEKO_HOT void PhysicsWorld::ResolveBroadphase() {
  NEKO_PROFILE_FUNCTION();
  const auto t_broadphase_start = StepProfileNow();
  auto& broadphase = Broadphase();
  broadphase.Clear();
  collider_aabb_cache_.clear();

  const auto is_inserted = [this](const Collider& collider) {
    return collider.type != ShapeType::kNone && collider.is_enabled &&
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
  step_overflow_.broadphase_colliders +=
      static_cast<std::uint32_t>(broadphase.dropped_collider_count());
  step_overflow_.broadphase_pairs +=
      static_cast<std::uint32_t>(broadphase.dropped_pair_count());
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
  revolute_joints_ = physics_world.revolute_joints_;
  // Which pairs pass through one-way colliders: part of the simulation state
  passing_pairs_ = physics_world.passing_pairs_;
  // Targets set before the snapshot: the next step must reach them
  kinematic_targets_ = physics_world.kinematic_targets_;
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
  // Empty in worlds without one-way colliders, which keep their old checksums
  for (const auto& pair : passing_pairs_) {
    checksum.Add(pair.c1());
    checksum.Add(pair.c2());
  }
  // Empty between steps unless SetKinematicTarget was called
  for (const auto& target : kinematic_targets_) {
    checksum.Add(target.body);
    checksum.Add(target.position);
    checksum.Add(target.angle.value());
  }
  // Empty in worlds without revolute joints, which keep their old checksums
  for (const auto& joint : revolute_joints_) {
    checksum.Add(neko::GenerateChecksum(joint));
  }
  return checksum.Get();
}
}  // namespace neko
