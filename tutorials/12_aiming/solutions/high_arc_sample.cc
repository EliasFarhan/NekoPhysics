#include "tutorials/12_aiming/solutions/high_arc_sample.h"

#include "neko/math/const.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kMuzzleX{1};
constexpr Scalar kMuzzleHeight = Ratio(1, 2);
constexpr Scalar kBallRadius = Ratio(1, 10);
constexpr Vec2f kTargetHalfSize{Ratio(1, 5), Ratio(1, 10)};
constexpr std::array<ColorId, 2> kColors{sample_color::kOrange,
                                         sample_color::kSky};
}  // namespace

void HighArcSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  muzzle_ = {context.world_bounds.min_bound().x() + kMuzzleX,
             kGroundTop + kMuzzleHeight};
  target_x_ = muzzle_.x() + parameters_[kDistance].ToScalar();
  const Scalar speed = parameters_[kSpeed].ToScalar();
  angles_[kLow] =
      LowArcAngle(parameters_[kDistance].ToScalar(), speed, kGravity);
  // The same range: sin(2 (pi / 2 - theta)) = sin(2 theta)
  angles_[kHigh] = Radian{Pi<Scalar>() / Scalar{2}} - angles_[kLow];
  for (std::size_t shot = 0; shot < balls_.size(); ++shot) {
    balls_[shot] = world_.AddBody();
    Body& ball = world_.body(balls_[shot]);
    ball.inertia_mode = InertiaMode::kFixedRotation;
    ball.position = muzzle_;
    ball.velocity = {speed * Cos(angles_[shot]), speed * Sin(angles_[shot])};
    // Triggers: the two balls fly through each other
    const ColliderIndex collider = world_.AddCircleCollider(balls_[shot]);
    world_.circle(world_.collider(collider).shape_index).radius = kBallRadius;
    watches_[shot].Start(muzzle_, target_x_);
    paths_[shot][0] = muzzle_;
    path_sizes_[shot] = 1;
  }
  for (std::size_t shot = 0; shot < balls_.size(); ++shot) {
    readouts_[shot].value =
        ToDecimal(Degree{angles_[shot]}.value(), readouts_[shot].scale);
    readouts_[shot].is_valid = true;
    SampleReadout& predicted = readouts_[4 + 2 * shot];
    predicted.value = ToDecimal(predicted_error(shot), predicted.scale);
    predicted.is_valid = true;
  }
}

void HighArcSample::FixedUpdate() {
  world_.Step();
  for (std::size_t shot = 0; shot < balls_.size(); ++shot) {
    if (errors_[shot]) {
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
    errors_[shot] = crossing->projectile_x - crossing->target_x;
    flight_times_[shot] = crossing->time;
    // Park the ball where it crossed, as the impact mark
    ball.position = {crossing->projectile_x, muzzle_.y()};
    ball.velocity = Vec2f::Zero();
    ball.gravity_scale = Scalar{0};
    SampleReadout& flight = readouts_[2 + shot];
    flight.value = ToDecimal(*flight_times_[shot], flight.scale);
    flight.is_valid = true;
    SampleReadout& measured = readouts_[5 + 2 * shot];
    measured.value = ToDecimal(*errors_[shot], measured.scale);
    measured.is_valid = true;
  }
}

void HighArcSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.lines.push_back({Vec2f{ground_.min_bound().x(), muzzle_.y()},
                             Vec2f{ground_.max_bound().x(), muzzle_.y()},
                             sample_color::kDarkGrey});
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

void HighArcSample::End() {
  world_.Clear();
  watches_ = {};
  path_sizes_ = {};
  errors_ = {};
  flight_times_ = {};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar HighArcSample::predicted_error(std::size_t shot) const {
  return -parameters_[kSpeed].ToScalar() * Cos(angles_[shot]) * world_.dt();
}

}  // namespace neko::tutorial
