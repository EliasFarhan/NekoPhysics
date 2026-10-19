#include "tutorials/15_fast_projectile/solutions/speed_threshold_sample.h"

#include "tutorials/15_fast_projectile/fast_projectile_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kWallX{8};
constexpr Scalar kRunUp{2};
constexpr Scalar kWallHalfHeight = Ratio(3, 10);
constexpr Scalar kFirstLaneY = Ratio(3, 5);
constexpr Scalar kLaneSpacing = Ratio(4, 5);
}  // namespace

Scalar SpeedThresholdSample::safe_speed() const {
  return (kShotRadius + thickness() / Scalar{2}) / world_.dt();
}

Scalar SpeedThresholdSample::size_speed() const {
  return (Scalar{2} * kShotRadius + thickness()) / world_.dt();
}

void SpeedThresholdSample::Begin(const SampleContext& context) {
  const Vec2f origin = context.world_bounds.min_bound();
  wall_x_ = origin.x() + kWallX;
  for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
    lane_y_[lane] = origin.y() + kFirstLaneY +
                    kLaneSpacing * static_cast<Scalar>(static_cast<int>(lane));
    walls_[lane] = AddWall(world_, {wall_x_, lane_y_[lane]},
                           {thickness() / Scalar{2}, kWallHalfHeight});
    shots_[lane] = AddShot(world_, {}, false, false);
  }
  StartRound();
  UpdateReadouts();
}

void SpeedThresholdSample::StartRound() {
  const Scalar speed = static_cast<Scalar>(speed_);
  const Scalar step_length = speed * world_.dt();
  for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
    // Each lane an eighth of a step farther back
    const Scalar back = step_length *
                        static_cast<Scalar>(static_cast<int>(lane)) /
                        static_cast<Scalar>(static_cast<int>(kLaneCount));
    Fire(world_, shots_[lane], {wall_x_ - kRunUp - back, lane_y_[lane]},
         {speed, Scalar{0}});
  }
  // Long enough to reach the wall and a step past it: 2 m at v / 60 per
  // step, plus margin
  round_steps_ = (2 * 60 + speed_ - 1) / speed_ + 5;
  round_step_ = 0;
}

void SpeedThresholdSample::EndRound() {
  int misses = 0;
  for (const ColliderIndex shot : shots_) {
    if (world_.ColliderCenter(shot).x() >= wall_x_) {
      ++misses;
    }
  }
  misses_[static_cast<std::size_t>(speed_ - kFirstSpeed)] = misses;
  last_speed_ = speed_;
  if (misses > 0 && !first_miss_speed_.has_value()) {
    first_miss_speed_ = speed_;
  }
}

void SpeedThresholdSample::FixedUpdate() {
  if (is_done()) {
    return;
  }
  world_.Step();
  ++round_step_;
  if (round_step_ >= round_steps_) {
    EndRound();
    UpdateReadouts();
    ++speed_;
    if (!is_done()) {
      StartRound();
    }
  }
}

void SpeedThresholdSample::UpdateReadouts() {
  readouts_[0].value = last_speed_;
  readouts_[0].is_valid = last_speed_ > 0;
  readouts_[1].value = last_speed_ > 0 ? misses_at(last_speed_) : 0;
  readouts_[1].is_valid = last_speed_ > 0;
  readouts_[2].value = first_miss_speed_.value_or(0);
  readouts_[2].is_valid = first_miss_speed_.has_value();
  readouts_[3].value = ToDecimal(safe_speed(), readouts_[3].scale);
  readouts_[3].is_valid = true;
  readouts_[4].value = ToDecimal(size_speed(), readouts_[4].scale);
  readouts_[4].is_valid = true;
}

void SpeedThresholdSample::Draw(DrawList& draw_list) const {
  for (const ColliderIndex wall : walls_) {
    draw_list.boxes.push_back(
        {world_.ColliderCenter(wall),
         world_.aabb(world_.collider(wall).shape_index).half_size,
         sample_color::kGrey});
  }
  for (const ColliderIndex shot : shots_) {
    draw_list.circles.push_back(
        {world_.ColliderCenter(shot), kShotRadius, sample_color::kOrange});
  }
}

void SpeedThresholdSample::End() {
  world_.Clear();
  misses_ = {};
  first_miss_speed_.reset();
  speed_ = kFirstSpeed;
  last_speed_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
