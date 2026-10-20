#include "tutorials/17_same_numbers/solutions/ulp_sweep_sample.h"

namespace neko::tutorial {
namespace {
constexpr std::array<ColorId, UlpSweepSample::kNudgeCount> kColors{
    sample_color::kRed,   sample_color::kOrange, sample_color::kYellow,
    sample_color::kGreen, sample_color::kCyan,   sample_color::kMagenta};
}  // namespace

void UlpSweepSample::Begin(const SampleContext& context) {
  reference_.Build(context.world_bounds, 0);
  for (std::size_t i = 0; i < kNudgeCount; ++i) {
    copies_[i].Build(context.world_bounds, Nudge(i));
    watches_[i].Reset();
    readouts_[i].is_valid = false;
  }
}

void UlpSweepSample::FixedUpdate() {
  reference_.Step();
  for (std::size_t i = 0; i < kNudgeCount; ++i) {
    copies_[i].Step();
    watches_[i].Update(reference_, copies_[i]);
    const auto step = watches_[i].first_visible_step();
    readouts_[i].is_valid = step.has_value();
    readouts_[i].value = step.value_or(0);
  }
}

void UlpSweepSample::Draw(DrawList& draw_list) const {
  reference_.DrawStatic(draw_list);
  reference_.DrawBalls(draw_list, sample_color::kSky, Scalar{1});
  for (std::size_t i = 0; i < kNudgeCount; ++i) {
    if (copies_[i].ball_count() > 0) {
      draw_list.circles.push_back({copies_[i].BallPosition(0),
                                   Board::kBallRadius / Scalar{3}, kColors[i]});
    }
  }
}

void UlpSweepSample::End() {
  reference_.Clear();
  for (Board& board : copies_) {
    board.Clear();
  }
}

std::uint32_t UlpSweepSample::GenerateChecksum() const {
  std::uint32_t checksum = reference_.world().GenerateChecksum();
  for (const Board& board : copies_) {
    checksum = checksum * 31u + board.world().GenerateChecksum();
  }
  return checksum;
}

}  // namespace neko::tutorial
