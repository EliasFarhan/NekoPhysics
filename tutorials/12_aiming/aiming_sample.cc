#include "tutorials/12_aiming/aiming_sample.h"

#include "neko/math/const.h"
#include "neko/math/func.h"
#include "neko/math/scalar_traits.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kGravity = Ratio(981, 100);
constexpr Scalar kGroundTop = Ratio(1, 2);
// The muzzle, from the left of the world and above the ground top
constexpr Scalar kMuzzleX{1};
constexpr Scalar kMuzzleHeight = Ratio(3, 2);
constexpr Scalar kBallRadius = Ratio(1, 10);
constexpr Vec2f kDroneHalfSize{Ratio(1, 5), Ratio(1, 10)};
constexpr Scalar kBarrelLength = Ratio(3, 5);
constexpr Vec2f kTurretHalfSize{Ratio(1, 4), Ratio(3, 4)};
constexpr Scalar kMarkSize = Ratio(1, 10);
}  // namespace

Radian LowArcAngle(Scalar distance, Scalar speed, Scalar gravity) {
  // Asin clamps an input past 1 (out of range) to 1: 45 degrees
  return Asin(gravity * distance / (speed * speed)) / Scalar{2};
}

Scalar FlightTime(Radian angle, Scalar speed, Scalar gravity) {
  return Scalar{2} * speed * Sin(angle) / gravity;
}

LeadSolution SolveLead(Scalar distance, Scalar target_speed, Scalar speed,
                       Scalar gravity, int iterations) {
  LeadSolution solution;
  for (int i = 0;; ++i) {
    // A target coming closer faster than the shot can't be behind the turret
    solution.aim_distance =
        Max(Scalar{0}, distance + target_speed * solution.lead_time);
    solution.angle = LowArcAngle(solution.aim_distance, speed, gravity);
    if (i == iterations) {
      return solution;
    }
    solution.lead_time = FlightTime(solution.angle, speed, gravity);
  }
}

void CrossingWatch::Start(Vec2f projectile, Scalar target_x) {
  previous_ = projectile;
  previous_target_x_ = target_x;
  height_ = projectile.y();
  steps_ = 0;
  is_watching_ = true;
}

std::optional<Crossing> CrossingWatch::Update(Vec2f projectile, Vec2f velocity,
                                              Scalar target_x, Scalar dt) {
  ++steps_;
  std::optional<Crossing> crossing;
  if (is_watching_ && velocity.y() < Scalar{0} && previous_.y() >= height_ &&
      projectile.y() < height_) {
    // Fraction of the step at which the straight line between the two
    // positions passes the launch height
    const Scalar fraction =
        (previous_.y() - height_) / (previous_.y() - projectile.y());
    crossing = Crossing{
        .time = (static_cast<Scalar>(steps_ - 1) + fraction) * dt,
        .projectile_x =
            previous_.x() + fraction * (projectile.x() - previous_.x()),
        .target_x =
            previous_target_x_ + fraction * (target_x - previous_target_x_),
        .velocity = velocity};
    is_watching_ = false;
  }
  previous_ = projectile;
  previous_target_x_ = target_x;
  return crossing;
}

void AimingSample::Begin(const SampleContext& context) {
  const Vec2f bounds_center = context.world_bounds.GetCenter();
  const Scalar half_width = context.world_bounds.GetHalfSize().x();
  ground_ = Aabbf::FromCenter({bounds_center.x(), kGroundTop / Scalar{2}},
                              {half_width, kGroundTop / Scalar{2}});
  muzzle_ = {context.world_bounds.min_bound().x() + kMuzzleX,
             kGroundTop + kMuzzleHeight};
  target_start_ = muzzle_ + Vec2f{parameters_[kDistance].ToScalar(), Scalar{0}};
  {
    const BodyIndex ground = world_.AddBody();
    world_.body(ground).type = BodyType::kStatic;
    world_.body(ground).inverse_mass = Scalar{0};
    world_.body(ground).position = ground_.GetCenter();
    const ColliderIndex collider_index = world_.AddAabbCollider(ground);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    world_.aabb(collider.shape_index).half_size = ground_.GetHalfSize();
  }
  // The drone flies at its own velocity: kinematic. Its collider and the
  // ball's are triggers, so the ball flies through: only the crossing counts
  target_ = world_.AddBody();
  world_.body(target_).type = BodyType::kKinematic;
  world_.body(target_).inverse_mass = Scalar{0};
  world_.aabb(world_.collider(world_.AddAabbCollider(target_)).shape_index)
      .half_size = kDroneHalfSize;
  ball_ = world_.AddBody();
  world_.body(ball_).inertia_mode = InertiaMode::kFixedRotation;
  world_.circle(world_.collider(world_.AddCircleCollider(ball_)).shape_index)
      .radius = kBallRadius;

  // The same aim every round
  lead_ = SolveLead(parameters_[kDistance].ToScalar(),
                    parameters_[kTargetSpeed].ToScalar(),
                    parameters_[kSpeed].ToScalar(), kGravity, kLeadIterations);
  readouts_[0].value =
      ToDecimal(Degree{lead_.angle}.value(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(lead_.lead_time, readouts_[1].scale);
  readouts_[2].value = ToDecimal(predicted_flight_time(), readouts_[2].scale);
  readouts_[4].value = ToDecimal(predicted_error(), readouts_[4].scale);
  for (const std::size_t i : {0u, 1u, 2u, 4u}) {
    readouts_[i].is_valid = true;
  }
}

void AimingSample::StartRound() {
  Body& target = world_.body(target_);
  target.position = target_start_;
  target.velocity = {parameters_[kTargetSpeed].ToScalar(), Scalar{0}};
  Body& ball = world_.body(ball_);
  const Scalar speed = parameters_[kSpeed].ToScalar();
  ball.position = muzzle_;
  ball.velocity = {speed * Cos(lead_.angle), speed * Sin(lead_.angle)};
  ball.gravity_scale = Scalar{1};
  watch_.Start(ball.position, target.position.x());
  path_[0] = ball.position;
  path_size_ = 1;
  has_crossed_ = false;
}

void AimingSample::FixedUpdate() {
  if (step_in_round_ == 0) {
    StartRound();
  }
  world_.Step();
  ++step_in_round_;

  Body& ball = world_.body(ball_);
  if (!has_crossed_) {
    path_[path_size_++] = ball.position;
    const std::optional<Crossing> crossing =
        watch_.Update(ball.position, ball.velocity,
                      world_.body(target_).position.x(), world_.dt());
    if (crossing) {
      has_crossed_ = true;
      ++rounds_;
      const Scalar error = crossing->projectile_x - crossing->target_x;
      if (rounds_ == 1) {
        first_error_ = error;
      } else if (ScalarTraits<Scalar>::ToBits(error) ==
                 ScalarTraits<Scalar>::ToBits(first_error_)) {
        ++same_rounds_;
      }
      measured_error_ = error;
      measured_flight_time_ = crossing->time;
      impact_angle_ = Atan2(crossing->velocity.y(), crossing->velocity.x());
      // Park the ball where it crossed, as the impact mark
      ball.position = {crossing->projectile_x, muzzle_.y()};
      ball.velocity = Vec2f::Zero();
      ball.gravity_scale = Scalar{0};
      readouts_[3].value =
          ToDecimal(*measured_flight_time_, readouts_[3].scale);
      readouts_[5].value = ToDecimal(*measured_error_, readouts_[5].scale);
      readouts_[6].value =
          ToDecimal(Degree{*impact_angle_}.value(), readouts_[6].scale);
      readouts_[7].value = same_rounds_;
      for (const std::size_t i : {3u, 5u, 6u, 7u}) {
        readouts_[i].is_valid = true;
      }
    }
  }
  if (step_in_round_ == kRoundSteps) {
    step_in_round_ = 0;
  }
}

void AimingSample::Draw(DrawList& draw_list) const {
  draw_list.boxes.push_back(
      {ground_.GetCenter(), ground_.GetHalfSize(), sample_color::kGrey});
  draw_list.boxes.push_back({muzzle_ - Vec2f{Scalar{0}, kTurretHalfSize.y()},
                             kTurretHalfSize, sample_color::kDarkGrey});
  draw_list.lines.push_back({Vec2f{ground_.min_bound().x(), muzzle_.y()},
                             Vec2f{ground_.max_bound().x(), muzzle_.y()},
                             sample_color::kDarkGrey});
  draw_list.lines.push_back(
      {muzzle_,
       muzzle_ + Vec2f{Cos(lead_.angle), Sin(lead_.angle)} * kBarrelLength,
       sample_color::kWhite});
  const Vec2f aim = muzzle_ + Vec2f{lead_.aim_distance, Scalar{0}};
  draw_list.lines.push_back({aim - Vec2f{Scalar{0}, kMarkSize},
                             aim + Vec2f{Scalar{0}, kMarkSize},
                             sample_color::kYellow});
  draw_list.boxes.push_back(
      {world_.body(target_).position, kDroneHalfSize, sample_color::kGreen});
  for (std::size_t i = 1; i < path_size_; ++i) {
    draw_list.lines.push_back({path_[i - 1], path_[i], sample_color::kOrange});
  }
  draw_list.circles.push_back(
      {world_.body(ball_).position, kBallRadius, sample_color::kOrange});
}

void AimingSample::End() {
  world_.Clear();
  lead_ = {};
  watch_ = {};
  path_size_ = 0;
  step_in_round_ = 0;
  rounds_ = 0;
  same_rounds_ = 0;
  has_crossed_ = false;
  first_error_ = Scalar{0};
  measured_flight_time_.reset();
  measured_error_.reset();
  impact_angle_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

Scalar AimingSample::predicted_flight_time() const {
  return FlightTime(lead_.angle, parameters_[kSpeed].ToScalar(), kGravity) -
         world_.dt();
}

Scalar AimingSample::predicted_error() const {
  const Scalar speed_x = parameters_[kSpeed].ToScalar() * Cos(lead_.angle);
  return (speed_x - parameters_[kTargetSpeed].ToScalar()) *
             predicted_flight_time() -
         parameters_[kDistance].ToScalar();
}

}  // namespace neko::tutorial
