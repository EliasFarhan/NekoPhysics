#include "tutorials/13_one_way/solutions/stacked_one_way_sample.h"

#include "main/samples/body_draw.h"
#include "tutorials/13_one_way/one_way_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kPlatformTop = Ratio(5, 2);
constexpr Vec2f kPlatformHalfSize{Ratio(3, 2), Ratio(7, 40)};
constexpr Vec2f kCrateHalfSize{Ratio(1, 4), Ratio(1, 4)};
}  // namespace

void StackedOneWaySample::Begin(const SampleContext& context) {
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
  platform_ = AddOneWayPlatform(
      world_, {bounds_center.x(), kPlatformTop - kPlatformHalfSize.y()},
      kPlatformHalfSize, Radian{Pi<Scalar>()});
  // A stack, each crate touching the one below
  for (std::size_t i = 0; i < kCrateCount; ++i) {
    const Scalar y =
        kPlatformTop +
        kCrateHalfSize.y() * static_cast<Scalar>(2 * static_cast<int>(i) + 1);
    crates_[i] = AddCrate(world_, {bounds_center.x(), y}, kCrateHalfSize,
                          static_cast<std::uint32_t>(i + 1));
  }
}

void StackedOneWaySample::FixedUpdate() {
  const int mode = parameters_[kMode].value;
  const bool drops_now =
      step_ == kDropStep ||
      (mode == 2 && step_ > kDropStep && step_ < kDropStep + kRepeatSteps);
  if (drops_now) {
    for (std::size_t i = 0; i < kCrateCount; ++i) {
      if (mode == 0 && i > 0) {
        break;
      }
      passing_made_ += static_cast<int>(
          world_.DropThrough(world_.collider(crates_[i]).body_index));
    }
  }
  world_.Step();
  ++step_;
  readouts_[0].value = crates_below();
  readouts_[1].value = passing_made_;
  readouts_[0].is_valid = true;
  readouts_[1].is_valid = true;
}

int StackedOneWaySample::crates_below() const {
  const Scalar bottom = kPlatformTop - Scalar{2} * kPlatformHalfSize.y();
  int count = 0;
  for (const ColliderIndex crate : crates_) {
    if (world_.ColliderCenter(crate).y() + kCrateHalfSize.y() < bottom) {
      ++count;
    }
  }
  return count;
}

void StackedOneWaySample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.boxes.push_back({world_.ColliderCenter(platform_),
                             kPlatformHalfSize, sample_color::kTeal});
  DrawOneWayNormal(world_, platform_, draw_list);
  for (const ColliderIndex crate : crates_) {
    const Vec2f center = world_.ColliderCenter(crate);
    draw_list.boxes.push_back({center, kCrateHalfSize, sample_color::kBlue});
    if (world_.IsPassing({platform_, crate})) {
      DrawBoxOutline(center, kCrateHalfSize, Rotation{}, sample_color::kMagenta,
                     draw_list);
    }
  }
}

void StackedOneWaySample::End() {
  world_.Clear();
  step_ = 0;
  passing_made_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
