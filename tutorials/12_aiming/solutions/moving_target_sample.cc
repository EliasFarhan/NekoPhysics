#include "tutorials/12_aiming/solutions/moving_target_sample.h"

#include "neko/math/const.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kGroundTop = Ratio(1, 2);
constexpr Scalar kMuzzleX{1};
constexpr Scalar kMuzzleHeight{1};
constexpr Scalar kBallRadius = Ratio(1, 10);
constexpr Vec2f kDroneHalfSize{Ratio(1, 5), Ratio(1, 10)};
constexpr std::array<ColorId, MovingTargetSample::kShotCount> kColors{
    sample_color::kRed, sample_color::kOrange, sample_color::kYellow,
    sample_color::kLime};
}  // namespace

void MovingTargetSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  muzzle_ = {context.world_bounds.min_bound().x() + kMuzzleX,
             kGroundTop + kMuzzleHeight};
  const Scalar speed = parameters_[kSpeed].ToScalar();
  const Scalar distance = parameters_[kDistance].ToScalar();
  const Scalar target_speed = parameters_[kTargetSpeed].ToScalar();
  target_ = world_.AddBody();
  Body& target = world_.body(target_);
  target.type = BodyType::kKinematic;
  target.inverse_mass = Scalar{0};
  target.position = muzzle_ + Vec2f{distance, Scalar{0}};
  target.velocity = {target_speed, Scalar{0}};
  const ColliderIndex drone = world_.AddAabbCollider(target_);
  world_.aabb(world_.collider(drone).shape_index).half_size = kDroneHalfSize;
  for (std::size_t shot = 0; shot < kShotCount; ++shot) {
    leads_[shot] = SolveLead(distance, target_speed, speed, kGravity,
                             static_cast<int>(shot));
    balls_[shot] = world_.AddBody();
    Body& ball = world_.body(balls_[shot]);
    ball.inertia_mode = InertiaMode::kFixedRotation;
    ball.position = muzzle_;
    ball.velocity =
        Vec2f{Cos(leads_[shot].angle), Sin(leads_[shot].angle)} * speed;
    // Triggers: the balls fly through each other and through the drone
    const ColliderIndex collider = world_.AddCircleCollider(balls_[shot]);
    world_.circle(world_.collider(collider).shape_index).radius = kBallRadius;
    watches_[shot].Start(muzzle_, world_.body(target_).position.x());
    paths_[shot][0] = muzzle_;
    path_sizes_[shot] = 1;
  }
  const LeadSolution& last = leads_[kShotCount - 1];
  readouts_[4].value = ToDecimal(last.lead_time, readouts_[4].scale);
  readouts_[5].value =
      ToDecimal(FlightTime(last.angle, speed, kGravity), readouts_[5].scale);
  readouts_[6].value =
      ToDecimal(predicted_miss(kShotCount - 1), readouts_[6].scale);
  for (const std::size_t i : {4u, 5u, 6u}) {
    readouts_[i].is_valid = true;
  }
}

void MovingTargetSample::FixedUpdate() {
  world_.Step();
  const Scalar target_x = world_.body(target_).position.x();
  for (std::size_t shot = 0; shot < kShotCount; ++shot) {
    if (misses_[shot]) {
      continue;
    }
    Body& ball = world_.body(balls_[shot]);
    if (path_sizes_[shot] < kMaxPath) {
      paths_[shot][path_sizes_[shot]++] = ball.position;
    }
    const std::optional<Crossing> crossing = watches_[shot].Update(
        ball.position, ball.velocity, target_x, world_.dt());
    if (!crossing) {
      continue;
    }
    misses_[shot] = crossing->projectile_x - crossing->target_x;
    readouts_[shot].value = ToDecimal(*misses_[shot], readouts_[shot].scale);
    readouts_[shot].is_valid = true;
    ball.position = {crossing->projectile_x, muzzle_.y()};
    ball.velocity = Vec2f::Zero();
    ball.gravity_scale = Scalar{0};
  }
}

void MovingTargetSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.lines.push_back({Vec2f{ground_.min_bound().x(), muzzle_.y()},
                             Vec2f{ground_.max_bound().x(), muzzle_.y()},
                             sample_color::kDarkGrey});
  draw_list.boxes.push_back(
      {world_.body(target_).position, kDroneHalfSize, sample_color::kGreen});
  for (std::size_t shot = 0; shot < kShotCount; ++shot) {
    for (std::size_t i = 1; i < path_sizes_[shot]; ++i) {
      draw_list.lines.push_back(
          {paths_[shot][i - 1], paths_[shot][i], kColors[shot]});
    }
    draw_list.circles.push_back(
        {world_.body(balls_[shot]).position, kBallRadius, kColors[shot]});
  }
}

void MovingTargetSample::End() {
  world_.Clear();
  leads_ = {};
  watches_ = {};
  path_sizes_ = {};
  misses_ = {};
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar MovingTargetSample::predicted_miss(std::size_t iterations) const {
  const Scalar speed = parameters_[kSpeed].ToScalar();
  const Radian angle = leads_[iterations].angle;
  const Scalar landing_time = FlightTime(angle, speed, kGravity) - world_.dt();
  return (speed * Cos(angle) - parameters_[kTargetSpeed].ToScalar()) *
             landing_time -
         parameters_[kDistance].ToScalar();
}

}  // namespace neko::tutorial
