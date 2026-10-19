#include "tutorials/15_fast_projectile/solutions/trigger_hits_sample.h"

#include <algorithm>

#include "tutorials/15_fast_projectile/fast_projectile_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kMuzzleX = Ratio(3, 5);
// The coins from the left, and the order they are added in (so that their
// collider indices don't follow the path)
constexpr std::array<Scalar, TriggerHitsSample::kCoinCount> kCoinX{
    Scalar{3}, Ratio(33, 10), Ratio(69, 20), Ratio(41, 10), Ratio(21, 5)};
constexpr std::array<std::size_t, TriggerHitsSample::kCoinCount> kAddOrder{
    2, 4, 0, 3, 1};
constexpr Vec2f kCoinHalfSize{Ratio(1, 40), Ratio(2, 5)};
constexpr std::array<Scalar, 2> kRowHeights{Ratio(24, 5), Ratio(12, 5)};
}  // namespace

void TriggerHitsSample::CoinListener::OnTriggerEnter(const ColliderPair& pair) {
  for (std::size_t row = 0; row < 2; ++row) {
    if (!pair.Contains(sample->shots_[row])) {
      continue;
    }
    for (std::size_t k = 0; k < kCoinCount; ++k) {
      if (!pair.Contains(sample->coins_by_x_[row][k])) {
        continue;
      }
      sample->collected_[row][k] = true;
      ++sample->coins_[row];
      if (row == 0) {
        // In path order when each coin is farther along than the last one
        if (k < sample->next_in_order_) {
          sample->in_order_ = false;
        }
        sample->next_in_order_ = k + 1;
        ++sample->coins_this_step_;
      }
    }
  }
}

void TriggerHitsSample::CoinListener::OnTriggerExit(const ColliderPair& pair) {
  if (pair.Contains(sample->shots_[0])) {
    ++exits;
  }
}

void TriggerHitsSample::Begin(const SampleContext& context) {
  const Vec2f origin = context.world_bounds.min_bound();
  for (std::size_t row = 0; row < 2; ++row) {
    const Scalar y = origin.y() + kRowHeights[row];
    for (const std::size_t k : kAddOrder) {
      const BodyIndex body = world_.AddBody();
      world_.body(body).type = BodyType::kStatic;
      world_.body(body).inverse_mass = Scalar{0};
      world_.body(body).position = {origin.x() + kCoinX[k], y};
      coins_by_x_[row][k] = world_.AddAabbCollider(body);
      Collider& coin = world_.collider(coins_by_x_[row][k]);
      world_.aabb(coin.shape_index).half_size = kCoinHalfSize;
    }
    shots_[row] = AddShot(world_, {}, row == 0, true);
    Fire(world_, shots_[row], {origin.x() + kMuzzleX, y},
         {parameters_[kSpeed].ToScalar(), Scalar{0}});
  }
  listener_.sample = this;
  listener_.exits = 0;
  world_.SetContactListener(&listener_);
  UpdateReadouts();
}

void TriggerHitsSample::FixedUpdate() {
  coins_this_step_ = 0;
  world_.Step();
  most_in_one_step_ = std::max(most_in_one_step_, coins_this_step_);
  UpdateReadouts();
}

void TriggerHitsSample::UpdateReadouts() {
  readouts_[0].value = coins_[0];
  readouts_[1].value = coins_[1];
  readouts_[2].value = in_order_ ? 1 : 0;
  readouts_[3].value = most_in_one_step_;
  for (auto& readout : readouts_) {
    readout.is_valid = true;
  }
}

void TriggerHitsSample::Draw(DrawList& draw_list) const {
  constexpr std::array<ColorId, 2> shot_colors{sample_color::kSky,
                                               sample_color::kCyan};
  for (std::size_t row = 0; row < 2; ++row) {
    for (std::size_t k = 0; k < kCoinCount; ++k) {
      draw_list.boxes.push_back(
          {world_.ColliderCenter(coins_by_x_[row][k]), kCoinHalfSize,
           collected_[row][k] ? sample_color::kGrey : sample_color::kYellow});
    }
    draw_list.circles.push_back(
        {world_.ColliderCenter(shots_[row]), kShotRadius, shot_colors[row]});
  }
}

void TriggerHitsSample::End() {
  world_.SetContactListener(nullptr);
  world_.Clear();
  collected_ = {};
  coins_ = {};
  next_in_order_ = 0;
  in_order_ = true;
  coins_this_step_ = 0;
  most_in_one_step_ = 0;
  listener_.exits = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
