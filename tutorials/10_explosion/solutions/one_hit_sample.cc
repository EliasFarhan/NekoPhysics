#include "tutorials/10_explosion/solutions/one_hit_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kCrateHalf = Ratio(1, 4);
constexpr Vec2f kBomb{Ratio(32, 5), Ratio(5, 2)};
constexpr Scalar kBlastImpulse{8};
// The lid: a thin box on top of the crate, part of the same body
constexpr Vec2f kLidOffset{Scalar{0}, Ratio(3, 10)};
constexpr Vec2f kLidHalf{Ratio(1, 4), Ratio(1, 20)};
constexpr int kCircleSegments = 32;
}  // namespace

BodyIndex OneHitSample::AddCrate(Scalar x) {
  const BodyIndex index = world_.AddBody();
  Body& body = world_.body(index);
  body.inverse_mass = Scalar{1};
  body.inertia_mode = InertiaMode::kFixedRotation;
  body.position = {x, kGroundTop + kCrateHalf};
  const ColliderIndex collider_index = world_.AddAabbCollider(index);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  collider.category = kCrate;
  world_.aabb(collider.shape_index).half_size = {kCrateHalf, kCrateHalf};
  return index;
}

void OneHitSample::Begin(const SampleContext& context) {
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
    collider.friction = Scalar{0};
    world_.aabb(collider.shape_index).half_size = ground_.GetHalfSize();
  }
  // Two meters on either side of the bomb
  plain_ = AddCrate(kBomb.x() - Scalar{2});
  lidded_ = AddCrate(kBomb.x() + Scalar{2});
  const ColliderIndex lid_index = world_.AddAabbCollider(lidded_);
  Collider& lid = world_.collider(lid_index);
  lid.is_trigger = false;
  lid.restitution = Scalar{0};
  lid.category = kCrate;
  lid.offset = kLidOffset;
  world_.aabb(lid.shape_index).half_size = kLidHalf;
}

void OneHitSample::Explode() {
  const Scalar radius = parameters_[kRadius].ToScalar();
  const bool one_per_body = parameters_[kOnePerBody].value != 0;
  std::array<ColliderIndex, 4> hits{};
  const std::size_t count =
      world_.OverlapCircle(kBomb, radius, hits, {.mask = kCrate});
  const std::size_t kept = count < hits.size() ? count : hits.size();
  colliders_found_ = static_cast<int>(count);
  // The bodies already pushed: a game would use its entity ids
  std::array<BodyIndex, 4> pushed{};
  std::size_t pushed_count = 0;
  for (std::size_t h = 0; h < kept; ++h) {
    const BodyIndex body_index = world_.collider(hits[h]).body_index;
    bool seen = false;
    for (std::size_t p = 0; p < pushed_count; ++p) {
      seen = seen || pushed[p] == body_index;
    }
    if (seen && one_per_body) {
      continue;
    }
    if (!seen) {
      pushed[pushed_count++] = body_index;
    }
    const Vec2f center = world_.body(body_index).position;
    const Vec2f away = center - kBomb;
    const Scalar distance = away.Length();
    if (distance == Scalar{0} || distance >= radius) {
      continue;
    }
    const Vec2f direction{away.x() / distance, away.y() / distance};
    world_.ApplyImpulse(
        body_index,
        direction * (kBlastImpulse * (Scalar{1} - distance / radius)));
    ++impulses_;
  }
  bodies_pushed_ = static_cast<int>(pushed_count);
  exploded_ = true;
  readouts_[0].value = colliders_found_;
  readouts_[1].value = bodies_pushed_;
  readouts_[2].value = impulses_;
  for (std::size_t i = 0; i < 3; ++i) {
    readouts_[i].is_valid = true;
  }
}

void OneHitSample::FixedUpdate() {
  ++step_;
  if (step_ == kBlastStep) {
    Explode();
  }
  world_.Step();
  if (exploded_) {
    readouts_[3].value =
        ToDecimal(Abs(plain_velocity().x()), readouts_[3].scale);
    readouts_[4].value =
        ToDecimal(Abs(lidded_velocity().x()), readouts_[4].scale);
    readouts_[3].is_valid = true;
    readouts_[4].is_valid = true;
  }
}

void OneHitSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  for (const BodyIndex crate : {plain_, lidded_}) {
    draw_list.boxes.push_back({world_.body(crate).position,
                               {kCrateHalf, kCrateHalf},
                               sample_color::kOrange});
  }
  draw_list.boxes.push_back({world_.body(lidded_).position + kLidOffset,
                             kLidHalf, sample_color::kYellow});
  const Scalar radius = parameters_[kRadius].ToScalar();
  Vec2f previous = kBomb + Vec2f{radius, Scalar{0}};
  for (int i = 1; i <= kCircleSegments; ++i) {
    const Radian angle{Scalar{2} * Pi<Scalar>() * static_cast<Scalar>(i) /
                       static_cast<Scalar>(kCircleSegments)};
    const Vec2f next = kBomb + Vec2f{radius * Cos(angle), radius * Sin(angle)};
    draw_list.lines.push_back({previous, next, sample_color::kRed});
    previous = next;
  }
  if (!exploded_) {
    draw_list.circles.push_back({kBomb, Ratio(3, 20), sample_color::kRed});
  }
}

void OneHitSample::End() {
  world_.Clear();
  step_ = 0;
  colliders_found_ = 0;
  bodies_pushed_ = 0;
  impulses_ = 0;
  exploded_ = false;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
