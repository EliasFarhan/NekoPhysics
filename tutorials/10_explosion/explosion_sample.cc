#include "tutorials/10_explosion/explosion_sample.h"

#include "neko/math/angle.h"
#include "neko/math/const.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kCrateHalf = Ratio(1, 4);
constexpr Scalar kBarrelRadius = Ratio(1, 4);
// The bomb goes off in the air, so the sight lines go down to the targets
// at different slopes and don't line up behind each other
constexpr Vec2f kBomb{Ratio(32, 5), Ratio(5, 2)};
// Impulse at the bomb, in N s, falling off to 0 at the radius
constexpr Scalar kBlastImpulse{8};
// The wall between the bomb and the crate behind it
constexpr Vec2f kWallCenter{Ratio(43, 5), Ratio(13, 10)};
constexpr Vec2f kWallHalf{Ratio(1, 10), Ratio(4, 5)};
// The laser's obstacles: a plank turned 15 degrees and a round post at the
// laser's height
constexpr Vec2f kLaserOrigin{Ratio(2, 5), Ratio(32, 5)};
constexpr Scalar kLaserLength{12};
constexpr Vec2f kPlankCenter{Ratio(32, 5), Scalar{4}};
constexpr Vec2f kPlankHalf{Ratio(3, 2), Ratio(3, 20)};
constexpr Scalar kPlankDegrees{15};
constexpr Vec2f kPostCenter{Ratio(19, 2), Ratio(32, 5)};
constexpr Scalar kPostRadius = Ratio(2, 5);
constexpr Scalar kNormalLength = Ratio(1, 2);
constexpr int kCircleSegments = 32;

// Positions and categories of the targets, in targets() order
struct TargetSpec {
  Scalar x;
  bool is_barrel;
  std::uint16_t category;
};
constexpr std::array<TargetSpec, ExplosionSample::kTargetCount> kTargetSpecs{{
    {Ratio(27, 5), false, ExplosionSample::kCrate},
    {Ratio(38, 5), true, ExplosionSample::kCrate},
    {Ratio(22, 5), false, ExplosionSample::kShielded},
    {Ratio(47, 5), false, ExplosionSample::kCrate},
    {Ratio(12, 5), false, ExplosionSample::kCrate},
}};

// Every solid category blocks the view except the ground
constexpr QueryFilter kSightFilter{.mask = ExplosionSample::kWall |
                                           ExplosionSample::kCrate |
                                           ExplosionSample::kShielded};

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

ColliderIndex ExplosionSample::AddStaticBox(Vec2f center, Vec2f half_size,
                                            std::uint16_t category) {
  const BodyIndex body = world_.AddBody();
  world_.body(body).type = BodyType::kStatic;
  world_.body(body).inverse_mass = Scalar{0};
  world_.body(body).position = center;
  const ColliderIndex collider_index = world_.AddAabbCollider(body);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  collider.restitution = Scalar{0};
  collider.category = category;
  world_.aabb(collider.shape_index).half_size = half_size;
  return collider_index;
}

void ExplosionSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  const ColliderIndex ground =
      AddStaticBox(ground_.GetCenter(), ground_.GetHalfSize(), kGround);
  // Ice: the blast also pushes the targets down into the ground, and
  // friction would eat most of the push along it
  world_.collider(ground).friction = Scalar{0};
  if (parameters_[kWallOn].value != 0) {
    wall_ = AddStaticBox(kWallCenter, kWallHalf, kWall);
  }

  plank_ = world_.AddBody();
  {
    Body& plank = world_.body(plank_);
    plank.type = BodyType::kStatic;
    plank.inverse_mass = Scalar{0};
    plank.position = kPlankCenter;
    plank.rotation.set_angle(Radian{Degree{kPlankDegrees}});
    const ColliderIndex collider_index = world_.AddOrientedBoxCollider(plank_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.category = kWall;
    world_.oriented_box(collider.shape_index).half_size = kPlankHalf;
  }
  {
    const BodyIndex post = world_.AddBody();
    world_.body(post).type = BodyType::kStatic;
    world_.body(post).inverse_mass = Scalar{0};
    world_.body(post).position = kPostCenter;
    const ColliderIndex collider_index = world_.AddCircleCollider(post);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.category = kWall;
    world_.circle(collider.shape_index).radius = kPostRadius;
  }

  for (std::size_t i = 0; i < kTargetCount; ++i) {
    const TargetSpec& spec = kTargetSpecs[i];
    Target& target = targets_[i];
    target = Target{};
    target.body = world_.AddBody();
    Body& body = world_.body(target.body);
    body.inverse_mass = Scalar{1};
    body.inertia_mode = InertiaMode::kFixedRotation;
    // Standing on the ground, touching it
    body.position = {spec.x, kGroundTop + kCrateHalf};
    target.collider = spec.is_barrel ? world_.AddCircleCollider(target.body)
                                     : world_.AddAabbCollider(target.body);
    Collider& collider = world_.collider(target.collider);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.category = spec.category;
    if (spec.is_barrel) {
      world_.circle(collider.shape_index).radius = kBarrelRadius;
    } else {
      world_.aabb(collider.shape_index).half_size = {kCrateHalf, kCrateHalf};
    }
    target.sight_end = body.position;
  }
  bomb_ = kBomb;
  laser_origin_ = kLaserOrigin;
}

void ExplosionSample::Explode() {
  const Scalar radius = parameters_[kRadius].ToScalar();
  // Who is in the blast: the categories it affects, from the parameter
  const std::uint16_t blast_mask = static_cast<std::uint16_t>(
      kCrate | (parameters_[kHitShielded].value != 0 ? kShielded : 0));
  std::array<ColliderIndex, 16> hits{};
  const std::size_t count =
      world_.OverlapCircle(bomb_, radius, hits, {.mask = blast_mask});
  const std::size_t kept = count < hits.size() ? count : hits.size();
  for (std::size_t h = 0; h < kept; ++h) {
    for (Target& target : targets_) {
      if (target.collider != hits[h]) {
        continue;
      }
      target.in_radius = true;
      ++in_radius_count_;
      const Vec2f center = world_.body(target.body).position;
      // Line of sight: the first collider between the bomb and the center
      const auto sight = world_.RaycastClosest(bomb_, center, kSightFilter);
      target.visible = sight.has_value() && sight->collider == target.collider;
      target.sight_end =
          sight.has_value() && !target.visible ? sight->point : center;
      if (!target.visible) {
        continue;
      }
      ++visible_count_;
      const Vec2f away = center - bomb_;
      const Scalar distance = away.Length();
      if (distance == Scalar{0} || distance >= radius) {
        continue;
      }
      const Vec2f direction{away.x() / distance, away.y() / distance};
      const Scalar impulse = kBlastImpulse * (Scalar{1} - distance / radius);
      world_.ApplyImpulseAtPoint(target.body, direction * impulse, center);
    }
  }
  exploded_ = true;
  readouts_[0].value = in_radius_count_;
  readouts_[1].value = visible_count_;
  readouts_[2].value = in_radius_count_ - visible_count_;
  for (std::size_t i = 0; i < 3; ++i) {
    readouts_[i].is_valid = true;
  }
}

void ExplosionSample::FixedUpdate() {
  ++step_;
  if (step_ == kBlastStep) {
    Explode();
  }
  world_.Step();
  laser_hit_ = world_.RaycastClosest(laser_start(), laser_end());
  for (std::size_t i = 3; i < 6; ++i) {
    readouts_[i].is_valid = laser_hit_.has_value();
  }
  if (laser_hit_.has_value()) {
    readouts_[3].value = ToDecimal(laser_hit_->fraction, readouts_[3].scale);
    readouts_[4].value = ToDecimal(laser_hit_->normal.x(), readouts_[4].scale);
    readouts_[5].value = ToDecimal(laser_hit_->normal.y(), readouts_[5].scale);
  }
}

void ExplosionSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  if (wall_ != kInvalidColliderIndex) {
    draw_list.boxes.push_back(
        {kWallCenter, kWallHalf, sample_color::kDarkGrey});
  }
  DrawBoxOutline(kPlankCenter, kPlankHalf, world_.body(plank_).rotation,
                 sample_color::kGrey, draw_list);
  draw_list.circles.push_back({kPostCenter, kPostRadius, sample_color::kGrey});
  for (std::size_t i = 0; i < kTargetCount; ++i) {
    const Body& body = world_.body(targets_[i].body);
    const ColorId color = kTargetSpecs[i].category == kShielded
                              ? sample_color::kCyan
                              : sample_color::kOrange;
    if (kTargetSpecs[i].is_barrel) {
      draw_list.circles.push_back(
          {body.position, kBarrelRadius, sample_color::kYellow});
    } else {
      draw_list.boxes.push_back(
          {body.position, {kCrateHalf, kCrateHalf}, color});
    }
  }

  const Scalar radius = parameters_[kRadius].ToScalar();
  DrawCircleOutline(bomb_, radius, sample_color::kRed, draw_list);
  if (!exploded_) {
    draw_list.circles.push_back({bomb_, Ratio(3, 20), sample_color::kRed});
  }
  for (const Target& target : targets_) {
    if (target.in_radius) {
      draw_list.lines.push_back(
          {bomb_, target.sight_end,
           target.visible ? sample_color::kGreen : sample_color::kRed});
    }
  }

  if (laser_hit_.has_value()) {
    draw_list.lines.push_back(
        {laser_start(), laser_hit_->point, sample_color::kWhite});
    draw_list.lines.push_back(
        {laser_hit_->point,
         laser_hit_->point + laser_hit_->normal * kNormalLength,
         sample_color::kYellow});
  } else {
    draw_list.lines.push_back(
        {laser_start(), laser_end(), sample_color::kWhite});
  }
}

void ExplosionSample::End() {
  world_.Clear();
  wall_ = kInvalidColliderIndex;
  plank_ = kInvalidBodyIndex;
  laser_hit_.reset();
  step_ = 0;
  in_radius_count_ = 0;
  visible_count_ = 0;
  exploded_ = false;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Vec2f ExplosionSample::laser_start() const { return laser_origin_; }

Vec2f ExplosionSample::laser_end() const {
  const Radian angle{
      Degree{static_cast<Scalar>(parameters_[kLaserAngle].value)}};
  return laser_origin_ +
         Vec2f{kLaserLength * Cos(angle), -kLaserLength * Sin(angle)};
}

}  // namespace neko::tutorial
