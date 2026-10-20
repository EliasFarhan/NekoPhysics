#include "tutorials/17_same_numbers/same_numbers_sample.h"

#include "neko/math/func.h"
#include "neko/math/scalar_traits.h"

#include "main/samples/sample_rng.h"

namespace neko::tutorial {
namespace {
// Height of the drop point and of the rows above the bins' top
constexpr Scalar kDropHeight = Ratio(33, 5);
constexpr Scalar kTopRow = Ratio(28, 5);
constexpr Scalar kBinsTop = Ratio(3, 2);
constexpr Scalar kFloorTop = Ratio(1, 2);
constexpr Scalar kWallHalfWidth = Ratio(1, 20);
constexpr Scalar kRestitution = Ratio(2, 5);

DrawBox AddStaticBox(PhysicsWorld& world, Vec2f center, Vec2f half_size) {
  const BodyIndex body = world.AddBody();
  world.body(body).type = BodyType::kStatic;
  world.body(body).inverse_mass = Scalar{0};
  world.body(body).position = center;
  const ColliderIndex index = world.AddAabbCollider(body);
  Collider& collider = world.collider(index);
  collider.is_trigger = false;
  collider.restitution = kRestitution;
  collider.friction = Scalar{0};
  world.aabb(collider.shape_index).half_size = half_size;
  return {center, half_size, sample_color::kDarkGrey};
}
}  // namespace

Scalar NudgeBits(Scalar value, int units) {
  using Traits = ScalarTraits<Scalar>;
  using Bits = decltype(Traits::ToBits(value));
  const Bits bits = Traits::ToBits(value) + static_cast<Bits>(units);
  return Traits::FromBits(bits);
}

void Board::Build(const Aabbf& world_bounds, int nudge) {
  nudge_ = nudge;
  step_ = 0;
  ball_count_ = 0;
  const Scalar middle = world_bounds.GetCenter().x();
  drop_point_ = {middle, kDropHeight};
  // Ten bins of kSpacing between the walls, centered
  bins_left_ = middle - kSpacing * static_cast<Scalar>(kBinCount) / Scalar{2};
  bins_top_ = kBinsTop;
  const Scalar bins_right =
      bins_left_ + kSpacing * static_cast<Scalar>(kBinCount);
  const Scalar floor_height = kFloorTop - world_bounds.min_bound().y();
  boxes_.clear();
  pegs_.clear();
  boxes_.push_back(AddStaticBox(
      world_, {middle, world_bounds.min_bound().y() + floor_height / Scalar{2}},
      {(bins_right - bins_left_) / Scalar{2} + kWallHalfWidth,
       floor_height / Scalar{2}}));
  // Dividers between the bins, and the two side walls up to the top
  const Scalar divider_half = (kBinsTop - kFloorTop) / Scalar{2};
  for (int i = 1; i < kBinCount; ++i) {
    boxes_.push_back(
        AddStaticBox(world_,
                     {bins_left_ + kSpacing * static_cast<Scalar>(i),
                      kFloorTop + divider_half},
                     {Ratio(1, 50), divider_half}));
  }
  const Scalar wall_half =
      (world_bounds.max_bound().y() - kFloorTop) / Scalar{2};
  for (const Scalar x : {bins_left_, bins_right}) {
    boxes_.push_back(AddStaticBox(world_, {x, kFloorTop + wall_half},
                                  {kWallHalfWidth, wall_half}));
  }
  // Staggered rows: odd rows are shifted by half the spacing
  for (int row = 0; row < kRowCount; ++row) {
    const bool is_odd = row % 2 == 1;
    const int columns = is_odd ? kBinCount : kBinCount - 1;
    const Scalar first_x =
        bins_left_ + (is_odd ? kSpacing / Scalar{2} : kSpacing);
    for (int column = 0; column < columns; ++column) {
      const BodyIndex body = world_.AddBody();
      world_.body(body).type = BodyType::kStatic;
      world_.body(body).inverse_mass = Scalar{0};
      world_.body(body).position = {
          first_x + kSpacing * static_cast<Scalar>(column),
          kTopRow - kSpacing * static_cast<Scalar>(row)};
      const ColliderIndex index = world_.AddCircleCollider(body);
      Collider& collider = world_.collider(index);
      collider.is_trigger = false;
      collider.restitution = kRestitution;
      collider.friction = Scalar{0};
      world_.circle(collider.shape_index).radius = kPegRadius;
      pegs_.push_back(
          {world_.body(body).position, kPegRadius, sample_color::kGrey});
      // Row 0 has a peg every kSpacing from bins_left_ + kSpacing: the
      // fifth is under the middle
      if (row == 0 && column == kBinCount / 2 - 1) {
        top_peg_ = index;
      }
    }
  }
  static_count_ = static_cast<int>(boxes_.size() + pegs_.size());
  // The same offsets on every board: within 6 cm of the middle
  SampleRng rng{17};
  for (Scalar& offset : offsets_) {
    offset = rng.Range(-Ratio(3, 50), Ratio(3, 50));
  }
  first_start_ = {NudgeBits(drop_point_.x() + offsets_[0], nudge_),
                  drop_point_.y()};
}

void Board::DropBall() {
  const BodyIndex body = world_.AddBody();
  const auto ball = static_cast<std::size_t>(ball_count_);
  world_.body(body).position =
      ball == 0 ? first_start_
                : Vec2f{drop_point_.x() + offsets_[ball], drop_point_.y()};
  world_.body(body).inertia_mode = InertiaMode::kFixedRotation;
  const ColliderIndex index = world_.AddCircleCollider(body);
  Collider& collider = world_.collider(index);
  collider.is_trigger = false;
  collider.restitution = kRestitution;
  collider.friction = Scalar{0};
  world_.circle(collider.shape_index).radius = kBallRadius;
  balls_[ball] = body;
  ++ball_count_;
}

void Board::Step() {
  if (ball_count_ < kBallCount && step_ == kDropPeriod * ball_count_) {
    DropBall();
  }
  world_.Step();
  ++step_;
}

void Board::Clear() {
  world_.Clear();
  step_ = 0;
  ball_count_ = 0;
}

Vec2f Board::BallPosition(int ball) const {
  return world_.body(balls_[static_cast<std::size_t>(ball)]).position;
}

std::optional<int> Board::Bin(int ball) const {
  const Vec2f position = BallPosition(ball);
  if (position.y() > bins_top_) {
    return std::nullopt;
  }
  // Bins are counted by comparison: no float-to-int conversion
  int bin = 0;
  Scalar right = bins_left_ + kSpacing;
  while (bin < kBinCount - 1 && position.x() >= right) {
    ++bin;
    right += kSpacing;
  }
  return bin;
}

void Board::DrawStatic(DrawList& draw_list) const {
  draw_list.circles.insert(draw_list.circles.end(), pegs_.begin(), pegs_.end());
  draw_list.boxes.insert(draw_list.boxes.end(), boxes_.begin(), boxes_.end());
}

void Board::DrawBalls(DrawList& draw_list, ColorId ball_color,
                      Scalar ball_radius_scale) const {
  for (int ball = 0; ball < ball_count_; ++ball) {
    draw_list.circles.push_back(
        {BallPosition(ball), kBallRadius * ball_radius_scale, ball_color});
  }
}

void TwinWatch::Reset() { *this = TwinWatch{}; }

void TwinWatch::Update(const Board& a, const Board& b) {
  const int step = a.step() - 1;
  if (!first_checksum_step_.has_value() &&
      a.world().GenerateChecksum() != b.world().GenerateChecksum()) {
    first_checksum_step_ = step;
  }
  bins_differ_ = 0;
  for (int ball = 0; ball < a.ball_count(); ++ball) {
    const Scalar gap = (a.BallPosition(ball) - b.BallPosition(ball)).Length();
    gaps_[static_cast<std::size_t>(ball)] = gap;
    max_gap_ = Max(max_gap_, gap);
    if (!first_visible_step_.has_value() && gap > kVisible) {
      first_visible_step_ = step;
    }
    const auto bin_a = a.Bin(ball);
    const auto bin_b = b.Bin(ball);
    if (bin_a.has_value() && bin_b.has_value() && *bin_a != *bin_b) {
      ++bins_differ_;
    }
  }
}

void SameNumbersSample::Begin(const SampleContext& context) {
  board_a_.Build(context.world_bounds, 0);
  board_b_.Build(context.world_bounds, parameters_[kNudge].value);
  watch_.Reset();
  UpdateReadouts();
}

void SameNumbersSample::FixedUpdate() {
  board_a_.Step();
  board_b_.Step();
  watch_.Update(board_a_, board_b_);
  UpdateReadouts();
}

void SameNumbersSample::UpdateReadouts() {
  const auto set_step = [](SampleReadout& readout, std::optional<int> step) {
    readout.is_valid = step.has_value();
    readout.value = step.value_or(0);
  };
  set_step(readouts_[0], watch_.first_checksum_step());
  set_step(readouts_[1], watch_.first_visible_step());
  readouts_[2].value = ToDecimal(watch_.max_gap(), readouts_[2].scale);
  readouts_[2].is_valid = board_a_.step() > 0;
  readouts_[3].value = watch_.bins_differ();
  readouts_[3].is_valid = board_a_.step() > 0;
  readouts_[4].value = board_a_.ball_count();
  readouts_[4].is_valid = true;
}

void SameNumbersSample::Draw(DrawList& draw_list) const {
  board_a_.DrawStatic(draw_list);
  board_a_.DrawBalls(draw_list, sample_color::kSky, Scalar{1});
  board_b_.DrawBalls(draw_list, sample_color::kOrange, Ratio(3, 5));
  for (int ball = 0; ball < board_a_.ball_count(); ++ball) {
    if (watch_.Gap(ball) > TwinWatch::kVisible) {
      draw_list.lines.push_back({board_a_.BallPosition(ball),
                                 board_b_.BallPosition(ball),
                                 sample_color::kMagenta});
    }
  }
}

void SameNumbersSample::End() {
  board_a_.Clear();
  board_b_.Clear();
}

std::uint32_t SameNumbersSample::GenerateChecksum() const {
  return board_a_.world().GenerateChecksum() * 31u +
         board_b_.world().GenerateChecksum();
}

}  // namespace neko::tutorial
