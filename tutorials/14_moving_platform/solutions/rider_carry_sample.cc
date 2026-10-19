#include "tutorials/14_moving_platform/solutions/rider_carry_sample.h"

#include <algorithm>

#include "neko/math/const.h"
#include "neko/math/func.h"

#include "tutorials/13_one_way/one_way_sample.h"
#include "tutorials/14_moving_platform/moving_platform_sample.h"

namespace neko::tutorial {
namespace {
constexpr int kStepsPerSecond = 60;
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Vec2f kPlatformHalfSize{Ratio(4, 5), Ratio(3, 20)};
constexpr Vec2f kPlayerHalfSize{Ratio(19, 100), Ratio(3, 10)};
constexpr Scalar kAmplitude = Ratio(6, 5);
constexpr Scalar kPlatformY = Ratio(5, 2);
// The three patrols' centers from the left of the world, 4.3 m apart
constexpr std::array<Scalar, RiderCarrySample::kRiderCount> kCenterX = {
    Ratio(21, 10), Ratio(64, 10), Ratio(107, 10)};
constexpr Scalar kFriction = Ratio(1, 2);
constexpr std::size_t kCarry = 1;
constexpr std::size_t kFrictionRider = 2;
}  // namespace

void RiderCarrySample::Begin(const SampleContext& context) {
  const Scalar left = context.world_bounds.min_bound().x();
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  period_steps_ = parameters_[kPeriod].value * kStepsPerSecond;
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
  for (std::size_t i = 0; i < kRiderCount; ++i) {
    centers_[i] = {left + kCenterX[i], kPlatformY};
    const Vec2f start = PatrolPoint(centers_[i], kAmplitude, 0, period_steps_);
    platforms_[i] = AddKinematicPlatform(world_, start, kPlatformHalfSize);
    world_.collider(platforms_[i]).friction = kFriction;
    riders_[i] = AddCrate(
        world_,
        {start.x(), kPlatformY + kPlatformHalfSize.y() + kPlayerHalfSize.y()},
        kPlayerHalfSize, static_cast<std::uint32_t>(i + 1));
    world_.collider(riders_[i]).friction =
        i == kFrictionRider ? kFriction : Scalar{0};
  }
  const Scalar w = Scalar{2} * Pi<Scalar>() / parameters_[kPeriod].ToScalar();
  peak_acceleration_ = w * w * kAmplitude;
  readouts_[3].value = ToDecimal(peak_acceleration_, readouts_[3].scale);
  // The friction of the pair is sqrt(0.5 * 0.5)
  readouts_[4].value =
      ToDecimal(kFriction * -world_.gravity().y(), readouts_[4].scale);
  readouts_[3].is_valid = true;
  readouts_[4].is_valid = true;
}

void RiderCarrySample::FixedUpdate() {
  for (std::size_t i = 0; i < kRiderCount; ++i) {
    const BodyIndex platform = world_.collider(platforms_[i]).body_index;
    world_.SetKinematicTarget(
        platform,
        PatrolPoint(centers_[i], kAmplitude, step_ + 1, period_steps_),
        Radian{});
    // The game-side carry, after the target: the platform's velocity is the
    // one of the coming step
    if (i == kCarry && IsTouching(world_, {platforms_[i], riders_[i]})) {
      Body& rider = world_.body(world_.collider(riders_[i]).body_index);
      rider.velocity.set_x(world_.body(platform).velocity.x());
    }
  }
  world_.Step();
  ++step_;
  for (std::size_t i = 0; i < kRiderCount; ++i) {
    max_offset_[i] = std::max(max_offset_[i], Abs(offset(i)));
    readouts_[i].value =
        ToDecimal(offset(i) * Scalar{1000}, readouts_[i].scale);
    readouts_[i].is_valid = true;
  }
}

Scalar RiderCarrySample::offset(std::size_t rider) const {
  return world_.ColliderCenter(riders_[rider]).x() -
         world_.ColliderCenter(platforms_[rider]).x();
}

bool RiderCarrySample::is_on_platform(std::size_t rider) const {
  // Not below the platform's top
  return world_.ColliderCenter(riders_[rider]).y() >
         kPlatformY + kPlatformHalfSize.y();
}

void RiderCarrySample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  for (std::size_t i = 0; i < kRiderCount; ++i) {
    draw_list.boxes.push_back({world_.ColliderCenter(platforms_[i]),
                               kPlatformHalfSize, sample_color::kTeal});
    draw_list.boxes.push_back(
        {world_.ColliderCenter(riders_[i]), kPlayerHalfSize,
         is_on_platform(i) ? sample_color::kOrange : sample_color::kRed});
  }
}

void RiderCarrySample::End() {
  world_.Clear();
  step_ = 0;
  max_offset_ = {};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
