#include "tutorials/10_explosion/solutions/reach_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kCrateHalf = Ratio(1, 4);
constexpr Scalar kBarrelRadius = Ratio(1, 4);
constexpr Vec2f kBomb{Ratio(32, 5), Ratio(5, 2)};
// The bisection: radii from 0 (not found) to 8 m (found), down to 1 mm
constexpr Scalar kLargestRadius{8};
constexpr Scalar kTolerance = Ratio(1, 1000);
constexpr int kCircleSegments = 32;

struct TargetSpec {
  Scalar x;
  bool is_barrel;
};
// The chapter's targets: crate, barrel, shielded crate, crate behind the
// wall, far crate
constexpr std::array<TargetSpec, ReachSample::kTargetCount> kTargetSpecs{{
    {Ratio(27, 5), false},
    {Ratio(38, 5), true},
    {Ratio(22, 5), false},
    {Ratio(47, 5), false},
    {Ratio(12, 5), false},
}};

void DrawCircleOutline(Vec2f center, Scalar radius, ColorId color,
                       DrawList& draw_list) {
  Vec2f previous = center + Vec2f{radius, Scalar{0}};
  for (int i = 1; i <= kCircleSegments; ++i) {
    const Radian angle{Scalar{2} * Pi<Scalar>() * static_cast<Scalar>(i) /
                       static_cast<Scalar>(kCircleSegments)};
    const Vec2f next = center + Vec2f{radius * Cos(angle), radius * Sin(angle)};
    draw_list.lines.push_back({previous, next, color});
    previous = next;
  }
}
}  // namespace

void ReachSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  {
    const BodyIndex ground = world_.AddBody();
    world_.body(ground).type = BodyType::kStatic;
    world_.body(ground).inverse_mass = Scalar{0};
    world_.body(ground).position = ground_.GetCenter();
    const ColliderIndex collider_index = world_.AddAabbCollider(ground);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = ground_.GetHalfSize();
  }
  for (std::size_t i = 0; i < kTargetCount; ++i) {
    const TargetSpec& spec = kTargetSpecs[i];
    bodies_[i] = world_.AddBody();
    Body& body = world_.body(bodies_[i]);
    body.inverse_mass = Scalar{1};
    body.inertia_mode = InertiaMode::kFixedRotation;
    body.position = {spec.x, kGroundTop + kCrateHalf};
    colliders_[i] = spec.is_barrel ? world_.AddCircleCollider(bodies_[i])
                                   : world_.AddAabbCollider(bodies_[i]);
    Collider& collider = world_.collider(colliders_[i]);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.category = kCrate;
    if (spec.is_barrel) {
      world_.circle(collider.shape_index).radius = kBarrelRadius;
    } else {
      world_.aabb(collider.shape_index).half_size = {kCrateHalf, kCrateHalf};
    }
  }

  // Bisection on the radius: low never finds the target, high always does
  for (std::size_t i = 0; i < kTargetCount; ++i) {
    Scalar low{0};
    Scalar high = kLargestRadius;
    queries_[i] = 0;
    while (high - low > kTolerance) {
      const Scalar middle = low + (high - low) / Scalar{2};
      ++queries_[i];
      if (Finds(i, middle)) {
        high = middle;
      } else {
        low = middle;
      }
    }
    reach_[i] = high;
  }

  const std::size_t chosen =
      static_cast<std::size_t>(parameters_[kTarget].value);
  readouts_[0].value = ToDecimal(reach_[chosen], readouts_[0].scale);
  readouts_[1].value = ToDecimal(predicted_reach(chosen), readouts_[1].scale);
  readouts_[2].value = ToDecimal(center_distance(chosen), readouts_[2].scale);
  readouts_[3].value = queries_[chosen];
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

bool ReachSample::Finds(std::size_t i, Scalar radius) const {
  std::array<ColliderIndex, kTargetCount> hits{};
  const std::size_t count =
      world_.OverlapCircle(kBomb, radius, hits, {.mask = kCrate});
  const std::size_t kept = count < hits.size() ? count : hits.size();
  for (std::size_t h = 0; h < kept; ++h) {
    if (hits[h] == colliders_[i]) {
      return true;
    }
  }
  return false;
}

Vec2f ReachSample::closest_point(std::size_t i) const {
  const Vec2f center = world_.body(bodies_[i]).position;
  if (kTargetSpecs[i].is_barrel) {
    // Along the line to the center, one radius short of it
    const Vec2f away = center - kBomb;
    const Scalar distance = away.Length();
    return center -
           Vec2f{away.x() / distance, away.y() / distance} * kBarrelRadius;
  }
  // The bomb clamped to the box
  return ClosestPoint(Aabbf::FromCenter(center, {kCrateHalf, kCrateHalf}),
                      kBomb);
}

Scalar ReachSample::predicted_reach(std::size_t i) const {
  return (closest_point(i) - kBomb).Length();
}

Scalar ReachSample::center_distance(std::size_t i) const {
  return (world_.body(bodies_[i]).position - kBomb).Length();
}

void ReachSample::FixedUpdate() { world_.Step(); }

void ReachSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  const std::size_t chosen =
      static_cast<std::size_t>(parameters_[kTarget].value);
  for (std::size_t i = 0; i < kTargetCount; ++i) {
    const Vec2f center = world_.body(bodies_[i]).position;
    const ColorId color =
        i == chosen ? sample_color::kOrange : sample_color::kDarkGrey;
    if (kTargetSpecs[i].is_barrel) {
      draw_list.circles.push_back({center, kBarrelRadius, color});
    } else {
      draw_list.boxes.push_back({center, {kCrateHalf, kCrateHalf}, color});
    }
  }
  draw_list.circles.push_back({kBomb, Ratio(3, 20), sample_color::kRed});
  DrawCircleOutline(kBomb, reach_[chosen], sample_color::kRed, draw_list);
  draw_list.lines.push_back(
      {kBomb, world_.body(bodies_[chosen]).position, sample_color::kGrey});
  draw_list.lines.push_back(
      {kBomb, closest_point(chosen), sample_color::kGreen});
}

void ReachSample::End() {
  world_.Clear();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
