#include "tutorials/13_one_way/solutions/ghost_timer_sample.h"

#include "tutorials/13_one_way/one_way_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kGhostTop{3};
constexpr Vec2f kGhostHalfSize{Ratio(3, 5), Ratio(7, 40)};
constexpr Vec2f kCrateHalfSize{Ratio(1, 5), Ratio(1, 5)};
constexpr Scalar kFirstColumn = Ratio(3, 2);
constexpr Scalar kColumnSpacing = Ratio(19, 10);
// The longest fall the prediction looks at, in steps
constexpr int kMaxPredictedSteps = 60;
}  // namespace

void GhostTimerSample::Begin(const SampleContext& context) {
  const Vec2f origin = context.world_bounds.min_bound();
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
  for (std::size_t i = 0; i < kGhostCount; ++i) {
    const Scalar x = origin.x() + kFirstColumn +
                     kColumnSpacing * static_cast<Scalar>(static_cast<int>(i));
    ghosts_[i] = AddOneWayPlatform(world_, {x, kGhostTop - kGhostHalfSize.y()},
                                   kGhostHalfSize, Radian{Pi<Scalar>()});
    // Resting on it, touching
    crates_[i] = AddCrate(world_, {x, kGhostTop + kCrateHalfSize.y()},
                          kCrateHalfSize, static_cast<std::uint32_t>(i + 1));
  }
}

void GhostTimerSample::FixedUpdate() {
  if (step_ == kOffStep) {
    const Body& crate = world_.body(world_.collider(crates_[0]).body_index);
    const Scalar ghost_y = world_.ColliderCenter(ghosts_[0]).y();
    rest_offset_ = crate.position.y() - ghost_y;
    // The engine's recurrence from the crate's state, without the ghost
    Scalar y = crate.position.y();
    Scalar velocity = crate.velocity.y();
    const Scalar dt = world_.dt();
    for (int n = 1; n <= kMaxPredictedSteps; ++n) {
      velocity += world_.gravity().y() * dt;
      y += velocity * dt;
      if (y < ghost_y) {
        predicted_min_off_ = n;
        break;
      }
    }
    readouts_[0].value = ToDecimal(rest_offset_, 1000);
    readouts_[1].value = predicted_min_off_;
    readouts_[0].is_valid = true;
    readouts_[1].is_valid = true;
    for (const ColliderIndex ghost : ghosts_) {
      world_.collider(ghost).is_enabled = false;
    }
  }
  for (std::size_t i = 0; i < kGhostCount; ++i) {
    if (step_ == kOffStep + off_steps(i)) {
      world_.collider(ghosts_[i]).is_enabled = true;
    }
  }
  world_.Step();
  ++step_;
  if (step_ == kMeasuredSteps) {
    for (std::size_t i = 0; i < kGhostCount; ++i) {
      const Scalar crate_y = world_.ColliderCenter(crates_[i]).y();
      outcomes_[i] = crate_y > world_.ColliderCenter(ghosts_[i]).y() ? 1 : -1;
    }
    readouts_[2].value = measured_min_off();
    readouts_[2].is_valid = true;
  }
}

int GhostTimerSample::measured_min_off() const {
  for (std::size_t i = 0; i < kGhostCount; ++i) {
    if (outcomes_[i] == -1) {
      return off_steps(i);
    }
  }
  return -1;
}

void GhostTimerSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  for (std::size_t i = 0; i < kGhostCount; ++i) {
    const bool is_on = world_.collider(ghosts_[i]).is_enabled;
    draw_list.boxes.push_back(
        {world_.ColliderCenter(ghosts_[i]), kGhostHalfSize,
         is_on ? sample_color::kTeal : sample_color::kDarkGrey});
    if (is_on) {
      DrawOneWayNormal(world_, ghosts_[i], draw_list);
    }
    const ColorId color = outcomes_[i] == 1    ? sample_color::kGreen
                          : outcomes_[i] == -1 ? sample_color::kRed
                                               : sample_color::kBlue;
    draw_list.boxes.push_back(
        {world_.ColliderCenter(crates_[i]), kCrateHalfSize, color});
  }
  DrawPassingPairs(world_, draw_list);
}

void GhostTimerSample::End() {
  world_.Clear();
  step_ = 0;
  outcomes_ = {};
  rest_offset_ = Scalar{0};
  predicted_min_off_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
