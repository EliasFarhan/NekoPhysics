#include "tutorials/14_moving_platform/solutions/wrap_events_sample.h"

#include "tutorials/13_one_way/one_way_sample.h"
#include "tutorials/14_moving_platform/moving_platform_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kScreenHalfWidth = Ratio(27, 5);
constexpr Vec2f kPlatformHalfSize{Ratio(3, 5), Ratio(3, 20)};
constexpr Vec2f kCrateHalfSize{Ratio(1, 5), Ratio(1, 5)};
constexpr Vec2f kPostHalfSize{Ratio(1, 20), Ratio(3, 10)};
constexpr Vec2f kWallHalfSize{Ratio(1, 10), Scalar{3}};
constexpr Scalar kLow = Ratio(5, 2);
constexpr Scalar kHigh{5};
}  // namespace

void WrapEventsSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  left_post_ = bounds_center.x() - kScreenHalfWidth;
  right_post_ = bounds_center.x() + kScreenHalfWidth;
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
  // A wall at the right end of the ground: a crate left behind by the
  // platform stops there
  const Scalar right = context.world_bounds.max_bound().x();
  wall_ = Aabbf::FromCenter(
      {right - kWallHalfSize.x(), kGroundTop + kWallHalfSize.y()},
      kWallHalfSize);
  {
    const BodyIndex wall = world_.AddBody();
    world_.body(wall).type = BodyType::kStatic;
    world_.body(wall).inverse_mass = Scalar{0};
    world_.body(wall).position = wall_.GetCenter();
    const ColliderIndex collider_index = world_.AddAabbCollider(wall);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = kWallHalfSize;
  }
  const Vec2f start{left_post_ + Scalar{1}, rng_.Range(kLow, kHigh)};
  platform_ = AddKinematicPlatform(world_, start, kPlatformHalfSize);
  crate_ = AddCrate(
      world_,
      {start.x(), start.y() + kPlatformHalfSize.y() + kCrateHalfSize.y()},
      kCrateHalfSize, 1);
  listener_.watched = {platform_, crate_};
  listener_.step = &step_;
  world_.SetContactListener(&listener_);
}

void WrapEventsSample::Wrap() {
  const BodyIndex platform = world_.collider(platform_).body_index;
  const Vec2f from = world_.body(platform).position;
  if (from.x() - kPlatformHalfSize.x() <= right_post_) {
    return;
  }
  const Vec2f to{left_post_ - kPlatformHalfSize.x(), rng_.Range(kLow, kHigh)};
  if (parameters_[kMode].value == 1 &&
      IsTouching(world_, {platform_, crate_})) {
    const BodyIndex crate = world_.collider(crate_).body_index;
    const Body& body = world_.body(crate);
    world_.Teleport(crate, body.position + (to - from), body.rotation.angle());
  }
  world_.Teleport(platform, to, Radian{});
  ++wraps_;
  if (first_wrap_step_ < 0) {
    first_wrap_step_ = step_;
  }
}

void WrapEventsSample::FixedUpdate() {
  Wrap();
  const BodyIndex platform = world_.collider(platform_).body_index;
  world_.SetKinematicTarget(
      platform,
      world_.body(platform).position +
          Vec2f{parameters_[kSpeed].ToScalar() * world_.dt(), Scalar{0}},
      Radian{});
  world_.Step();
  ++step_;
  readouts_[0].value = wraps_;
  readouts_[1].value = listener_.enters;
  readouts_[2].value = listener_.exits;
  readouts_[3].value = is_crate_on_platform() ? 1 : 0;
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

bool WrapEventsSample::is_crate_on_platform() const {
  return IsTouching(world_, {platform_, crate_});
}

void WrapEventsSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.boxes.push_back(
      {wall_.GetCenter(), wall_.GetHalfSize(), sample_color::kGrey});
  for (const Scalar x : {left_post_, right_post_}) {
    draw_list.boxes.push_back({{x, kGroundTop + kPostHalfSize.y()},
                               kPostHalfSize,
                               sample_color::kGrey});
  }
  draw_list.boxes.push_back({world_.ColliderCenter(platform_),
                             kPlatformHalfSize, sample_color::kLime});
  draw_list.boxes.push_back(
      {world_.ColliderCenter(crate_), kCrateHalfSize, sample_color::kBlue});
}

void WrapEventsSample::End() {
  world_.SetContactListener(nullptr);
  world_.Clear();
  rng_ = SampleRng{14};
  step_ = 0;
  wraps_ = 0;
  first_wrap_step_ = -1;
  listener_.enters = 0;
  listener_.exits = 0;
  listener_.first_exit = -1;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
