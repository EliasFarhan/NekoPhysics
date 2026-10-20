#include "tutorials/17_same_numbers/solutions/fused_update_sample.h"

#include <cmath>

#include "neko/math/scalar_traits.h"

namespace neko::tutorial {
namespace {
using Traits = ScalarTraits<Scalar>;

// One coordinate of the position update, as the engine writes it (two
// roundings) and fused (one rounding)
float Separate(float position, float velocity, float dt) {
  return position + velocity * dt;
}
float Fused(float position, float velocity, float dt) {
  return std::fma(velocity, dt, position);
}
}  // namespace

bool FusedUpdateSample::fuses() { return !Traits::kIsFixedPoint; }

void FusedUpdateSample::Begin(const SampleContext& context) {
  board_a_.Build(context.world_bounds, 0);
  board_b_.Build(context.world_bounds, 0);
  watch_.Reset();
  updates_ = 0;
  differing_updates_ = 0;
  for (SampleReadout& readout : readouts_) {
    readout.is_valid = false;
  }
}

void FusedUpdateSample::FixedUpdate() {
  const float dt = Traits::ToFloat(board_a_.world().dt());
  // The balls that exist before the step (a ball dropped by this step has
  // no update to compare yet)
  const int a_balls = board_a_.ball_count();
  const int b_balls = board_b_.ball_count();
  std::array<Vec2f, Board::kBallCount> a_before{};
  for (int ball = 0; ball < a_balls; ++ball) {
    a_before[static_cast<std::size_t>(ball)] = board_a_.BallPosition(ball);
  }
  for (int ball = 0; ball < b_balls; ++ball) {
    before_[static_cast<std::size_t>(ball)] = board_b_.BallPosition(ball);
  }
  board_a_.Step();
  board_b_.Step();

  // Board A's updates computed both ways in float
  for (int ball = 0; ball < a_balls; ++ball) {
    const Vec2f p = a_before[static_cast<std::size_t>(ball)];
    const Vec2f v = board_a_.world().body(board_a_.ball_body(ball)).velocity;
    for (const auto& [position, velocity] :
         {std::pair{p.x(), v.x()}, std::pair{p.y(), v.y()}}) {
      const float x = Traits::ToFloat(position);
      const float u = Traits::ToFloat(velocity);
      ++updates_;
      if (Separate(x, u, dt) != Fused(x, u, dt)) {
        ++differing_updates_;
      }
    }
  }
  // Board B: the machine that contracts the position line
  if (fuses()) {
    for (int ball = 0; ball < b_balls; ++ball) {
      Body& body = board_b_.world().body(board_b_.ball_body(ball));
      const Vec2f p = before_[static_cast<std::size_t>(ball)];
      body.position = {Scalar{Fused(Traits::ToFloat(p.x()),
                                    Traits::ToFloat(body.velocity.x()), dt)},
                       Scalar{Fused(Traits::ToFloat(p.y()),
                                    Traits::ToFloat(body.velocity.y()), dt)}};
    }
    watch_.Update(board_a_, board_b_);
  }

  readouts_[0].value = updates_;
  readouts_[1].value = differing_updates_;
  readouts_[2].value =
      updates_ > 0 ? differing_updates_ * 100 * readouts_[2].scale / updates_
                   : 0;
  for (std::size_t i = 0; i < 3; ++i) {
    readouts_[i].is_valid = true;
  }
  const auto set_step = [](SampleReadout& readout, std::optional<int> step) {
    readout.is_valid = step.has_value();
    readout.value = step.value_or(0);
  };
  set_step(readouts_[3], watch_.first_checksum_step());
  set_step(readouts_[4], watch_.first_visible_step());
}

void FusedUpdateSample::Draw(DrawList& draw_list) const {
  board_a_.DrawStatic(draw_list);
  board_a_.DrawBalls(draw_list, sample_color::kSky, Scalar{1});
  board_b_.DrawBalls(draw_list, sample_color::kOrange, Ratio(3, 5));
}

void FusedUpdateSample::End() {
  board_a_.Clear();
  board_b_.Clear();
}

std::uint32_t FusedUpdateSample::GenerateChecksum() const {
  return board_a_.world().GenerateChecksum() * 31u +
         board_b_.world().GenerateChecksum();
}

}  // namespace neko::tutorial
