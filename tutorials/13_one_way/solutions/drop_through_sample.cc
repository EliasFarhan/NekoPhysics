#include "tutorials/13_one_way/solutions/drop_through_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"
#include "tutorials/13_one_way/one_way_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kPlatformTop{3};
constexpr Scalar kPlatformHalfWidth = Ratio(3, 2);
constexpr Scalar kPlayerHalfWidth = Ratio(19, 100);
// The longest fall the prediction looks at, in steps
constexpr int kMaxPredictedSteps = 120;
}  // namespace

void DropThroughSample::Begin(const SampleContext& context) {
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
  platform_half_size_ = {kPlatformHalfWidth,
                         parameters_[kThickness].ToScalar() / Scalar{2}};
  player_half_size_ = {kPlayerHalfWidth,
                       parameters_[kPlayerHeight].ToScalar() / Scalar{2}};
  platform_ = AddOneWayPlatform(
      world_, {bounds_center.x(), kPlatformTop - platform_half_size_.y()},
      platform_half_size_, Radian{Pi<Scalar>()});
  // Standing on the platform, touching it
  player_ = AddCrate(world_,
                     {bounds_center.x(), kPlatformTop + player_half_size_.y()},
                     player_half_size_, 1);
  listener_.exits = 0;
  world_.SetContactListener(&listener_);
}

void DropThroughSample::FixedUpdate() {
  if (step_ == kDropStep) {
    const Body& player = world_.body(world_.collider(player_).body_index);
    const Scalar bottom = kPlatformTop - Scalar{2} * platform_half_size_.y();
    // The steps that start with the shapes overlapping (touching counts),
    // from the engine's recurrence
    Scalar y = player.position.y();
    Scalar velocity = player.velocity.y();
    const Scalar dt = world_.dt();
    predicted_steps_ = 0;
    for (int n = 0; n < kMaxPredictedSteps; ++n) {
      if (y + player_half_size_.y() < bottom) {
        break;
      }
      ++predicted_steps_;
      velocity += world_.gravity().y() * dt;
      y += velocity * dt;
    }
    const Scalar fall = player.position.y() + player_half_size_.y() - bottom;
    continuous_steps_ = Sqrt(Scalar{2} * fall / -world_.gravity().y()) / dt;
    drop_result_ = world_.DropThrough(world_.collider(player_).body_index);
    readouts_[1].value = predicted_steps_;
    readouts_[2].value = ToDecimal(continuous_steps_, readouts_[2].scale);
    readouts_[1].is_valid = true;
    readouts_[2].is_valid = true;
  }
  world_.Step();
  ++step_;
  if (world_.IsPassing({platform_, player_})) {
    ++passing_steps_;
  }
  readouts_[0].value = passing_steps_;
  readouts_[3].value = listener_.exits;
  readouts_[0].is_valid = step_ > kDropStep;
  readouts_[3].is_valid = true;
}

void DropThroughSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.boxes.push_back({world_.ColliderCenter(platform_),
                             platform_half_size_, sample_color::kTeal});
  DrawOneWayNormal(world_, platform_, draw_list);
  const Vec2f center = world_.ColliderCenter(player_);
  draw_list.boxes.push_back({center, player_half_size_, sample_color::kOrange});
  if (world_.IsPassing({platform_, player_})) {
    DrawBoxOutline(center, player_half_size_, Rotation{},
                   sample_color::kMagenta, draw_list);
  }
}

void DropThroughSample::End() {
  world_.SetContactListener(nullptr);
  world_.Clear();
  step_ = 0;
  passing_steps_ = 0;
  predicted_steps_ = 0;
  continuous_steps_ = Scalar{0};
  drop_result_ = 0;
  listener_.exits = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
