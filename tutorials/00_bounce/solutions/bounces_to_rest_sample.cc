#include "tutorials/00_bounce/solutions/bounces_to_rest_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kBallRadius = Ratio(1, 4);
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 4)};
constexpr Scalar kGravity = Ratio(981, 100);
// PhysicsWorldConfig::restitution_threshold, left at its default
constexpr Scalar kThreshold{1};
}  // namespace

void BouncesToRestSample::Begin(const SampleContext& context) {
  const Aabbf& bounds = context.world_bounds;
  const Scalar center_x = bounds.GetCenter().x();
  ground_ = world_.AddBody();
  {
    Body& ground = world_.body(ground_);
    ground.type = BodyType::kStatic;
    ground.inverse_mass = Scalar{0};
    ground.position = {
        center_x, bounds.min_bound().y() + Ratio(1, 4) + kGroundHalfSize.y()};
    ground_top_ = ground.position.y() + kGroundHalfSize.y();
    Collider& collider = world_.collider(world_.AddAabbCollider(ground_));
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
  }
  ball_ = world_.AddBody();
  {
    Body& ball = world_.body(ball_);
    ball.inverse_mass = Scalar{1};
    ball.position = {center_x, ground_top_ + kBallRadius + drop_height()};
    Collider& collider = world_.collider(world_.AddCircleCollider(ball_));
    collider.is_trigger = false;
    collider.restitution = restitution();
    world_.circle(collider.shape_index).radius = kBallRadius;
  }
  step_ = 0;
  bounce_count_ = 0;
  is_rising_ = false;
  last_bounce_.reset();
  apex_count_ = 0;
  readouts_[0].value = predicted_bounces();
  readouts_[2].value = ToDecimal(predicted_last_bounce(), readouts_[2].scale);
  readouts_[0].is_valid = true;
  readouts_[2].is_valid = true;
}

int BouncesToRestSample::predicted_bounces() const {
  // Impacts e^k v0 faster than the threshold
  Scalar speed = Sqrt(Scalar{2} * kGravity * drop_height());
  int count = 0;
  while (speed > kThreshold && count < static_cast<int>(kMaxBounces)) {
    ++count;
    speed = speed * restitution();
  }
  return count;
}

Scalar BouncesToRestSample::predicted_last_bounce() const {
  // The fall, then a flight of 2 e^k v0 / g between impacts k and k + 1
  const Scalar first_impact = Sqrt(Scalar{2} * kGravity * drop_height());
  Scalar time = first_impact / kGravity;
  Scalar rebound = first_impact;
  for (int k = 1; k < predicted_bounces(); ++k) {
    rebound = rebound * restitution();
    time += Scalar{2} * rebound / kGravity;
  }
  return time;
}

std::optional<Scalar> BouncesToRestSample::apex(std::size_t k) const {
  if (k >= apex_count_) {
    return std::nullopt;
  }
  return apexes_[k];
}

std::optional<Scalar> BouncesToRestSample::approach(std::size_t k) const {
  if (k >= static_cast<std::size_t>(bounce_count_) || k >= kMaxBounces) {
    return std::nullopt;
  }
  return approaches_[k];
}

Scalar BouncesToRestSample::BallHeight() const {
  return world_.body(ball_).position.y() - kBallRadius - ground_top_;
}

void BouncesToRestSample::FixedUpdate() {
  const Scalar before = world_.body(ball_).velocity.y();
  world_.Step();
  ++step_;
  const Scalar after = world_.body(ball_).velocity.y();
  // The solver sees the velocity after this step's gravity
  const Scalar approach_speed = -(before + world_.gravity().y() * world_.dt());
  if (after > Scalar{0} && approach_speed > kThreshold &&
      !world_.touching_pairs().empty()) {
    if (static_cast<std::size_t>(bounce_count_) < kMaxBounces) {
      approaches_[static_cast<std::size_t>(bounce_count_)] = approach_speed;
    }
    ++bounce_count_;
    is_rising_ = true;
    last_bounce_ = static_cast<Scalar>(step_) * world_.dt();
    readouts_[1].value = bounce_count_;
    readouts_[3].value = ToDecimal(*last_bounce_, readouts_[3].scale);
    readouts_[1].is_valid = true;
    readouts_[3].is_valid = true;
  } else if (is_rising_ && after <= Scalar{0}) {
    is_rising_ = false;
    if (apex_count_ < kMaxBounces) {
      apexes_[apex_count_] = BallHeight();
      if (apex_count_ < 2) {
        readouts_[4 + apex_count_].value =
            ToDecimal(BallHeight(), readouts_[4 + apex_count_].scale);
        readouts_[4 + apex_count_].is_valid = true;
      }
      ++apex_count_;
    }
  }
}

void BouncesToRestSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(ground_);
  const Body& ball = world_.body(ball_);
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  const Scalar left = ground.position.x() - kGroundHalfSize.x();
  const Scalar right = ground.position.x() + kGroundHalfSize.x();
  const Scalar middle = ground.position.x();
  // Predicted apexes on the left half, measured ones on the right half
  Scalar predicted = drop_height();
  for (int k = 0; k < predicted_bounces(); ++k) {
    predicted = predicted * restitution() * restitution();
    const Scalar y = ground_top_ + predicted;
    draw_list.lines.push_back({{left, y}, {middle, y}, sample_color::kGreen});
  }
  for (std::size_t k = 0; k < apex_count_; ++k) {
    const Scalar y = ground_top_ + apexes_[k];
    draw_list.lines.push_back({{middle, y}, {right, y}, sample_color::kYellow});
  }
  draw_list.circles.push_back(
      {ball.position, kBallRadius, sample_color::kOrange});
  DrawVelocity(ball, sample_color::kSky, draw_list);
}

void BouncesToRestSample::End() {
  world_.Clear();
  step_ = 0;
  bounce_count_ = 0;
  apex_count_ = 0;
  last_bounce_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
