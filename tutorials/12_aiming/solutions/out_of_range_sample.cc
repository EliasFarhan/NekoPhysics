#include "tutorials/12_aiming/solutions/out_of_range_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kMuzzleX = Ratio(1, 2);
constexpr Scalar kMuzzleHeight = Ratio(1, 2);
constexpr Scalar kBallRadius = Ratio(1, 10);
constexpr Vec2f kTargetHalfSize{Ratio(1, 5), Ratio(1, 10)};
constexpr Scalar kMarkSize = Ratio(1, 5);
constexpr std::array<ColorId, 2> kColors{sample_color::kOrange,
                                         sample_color::kSky};
}  // namespace

void OutOfRangeSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  muzzle_ = {context.world_bounds.min_bound().x() + kMuzzleX,
             kGroundTop + kMuzzleHeight};
  const Scalar distance = parameters_[kDistance].ToScalar();
  target_x_ = muzzle_.x() + distance;
  // Clamped by Asin: 45 degrees when the input passes 1
  angle_ = LowArcAngle(distance, parameters_[kSpeed].ToScalar(), kGravity);
  const Radian eighth_turn{Pi<Scalar>() / Scalar{4}};
  const std::array<Vec2f, 2> velocities{
      Vec2f{Cos(angle_), Sin(angle_)} * parameters_[kSpeed].ToScalar(),
      Vec2f{Cos(eighth_turn), Sin(eighth_turn)} * needed_speed()};
  for (std::size_t shot = 0; shot < balls_.size(); ++shot) {
    balls_[shot] = world_.AddBody();
    Body& ball = world_.body(balls_[shot]);
    ball.inertia_mode = InertiaMode::kFixedRotation;
    ball.position = muzzle_;
    ball.velocity = velocities[shot];
    const ColliderIndex collider = world_.AddCircleCollider(balls_[shot]);
    world_.circle(world_.collider(collider).shape_index).radius = kBallRadius;
    watches_[shot].Start(muzzle_, target_x_);
    paths_[shot][0] = muzzle_;
    path_sizes_[shot] = 1;
  }
  readouts_[0].value = ToDecimal(asin_input(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(max_range(), readouts_[1].scale);
  readouts_[2].value = ToDecimal(Degree{angle_}.value(), readouts_[2].scale);
  readouts_[3].value = ToDecimal(predicted_landing(), readouts_[3].scale);
  readouts_[5].value = ToDecimal(needed_speed(), readouts_[5].scale);
  for (const std::size_t i : {0u, 1u, 2u, 3u, 5u}) {
    readouts_[i].is_valid = true;
  }
}

void OutOfRangeSample::FixedUpdate() {
  world_.Step();
  for (std::size_t shot = 0; shot < balls_.size(); ++shot) {
    if (is_down_[shot]) {
      continue;
    }
    Body& ball = world_.body(balls_[shot]);
    if (path_sizes_[shot] < kMaxPath) {
      paths_[shot][path_sizes_[shot]++] = ball.position;
    }
    const std::optional<Crossing> crossing = watches_[shot].Update(
        ball.position, ball.velocity, target_x_, world_.dt());
    if (!crossing) {
      continue;
    }
    is_down_[shot] = true;
    if (shot == kGiven) {
      landing_ = crossing->projectile_x - muzzle_.x();
      readouts_[4].value = ToDecimal(*landing_, readouts_[4].scale);
      readouts_[4].is_valid = true;
    } else {
      needed_error_ = crossing->projectile_x - crossing->target_x;
      readouts_[6].value = ToDecimal(*needed_error_, readouts_[6].scale);
      readouts_[6].is_valid = true;
    }
    ball.position = {crossing->projectile_x, muzzle_.y()};
    ball.velocity = Vec2f::Zero();
    ball.gravity_scale = Scalar{0};
  }
}

void OutOfRangeSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.lines.push_back({Vec2f{ground_.min_bound().x(), muzzle_.y()},
                             Vec2f{ground_.max_bound().x(), muzzle_.y()},
                             sample_color::kDarkGrey});
  const Vec2f range_end = muzzle_ + Vec2f{max_range(), Scalar{0}};
  draw_list.lines.push_back({range_end - Vec2f{Scalar{0}, kMarkSize},
                             range_end + Vec2f{Scalar{0}, kMarkSize},
                             sample_color::kRed});
  draw_list.boxes.push_back(
      {Vec2f{target_x_, muzzle_.y()}, kTargetHalfSize, sample_color::kGreen});
  for (std::size_t shot = 0; shot < balls_.size(); ++shot) {
    for (std::size_t i = 1; i < path_sizes_[shot]; ++i) {
      draw_list.lines.push_back(
          {paths_[shot][i - 1], paths_[shot][i], kColors[shot]});
    }
    draw_list.circles.push_back(
        {world_.body(balls_[shot]).position, kBallRadius, kColors[shot]});
  }
}

void OutOfRangeSample::End() {
  world_.Clear();
  watches_ = {};
  path_sizes_ = {};
  is_down_ = {};
  landing_.reset();
  needed_error_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar OutOfRangeSample::asin_input() const {
  const Scalar speed = parameters_[kSpeed].ToScalar();
  return kGravity * parameters_[kDistance].ToScalar() / (speed * speed);
}

Scalar OutOfRangeSample::max_range() const {
  const Scalar speed = parameters_[kSpeed].ToScalar();
  return speed * speed / kGravity;
}

Scalar OutOfRangeSample::predicted_landing() const {
  const Scalar speed = parameters_[kSpeed].ToScalar();
  return speed * Cos(angle_) *
         (FlightTime(angle_, speed, kGravity) - world_.dt());
}

Scalar OutOfRangeSample::needed_speed() const {
  return Sqrt(kGravity * parameters_[kDistance].ToScalar());
}

}  // namespace neko::tutorial
