#include "tutorials/00_bounce/bounce_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kBallRadius = Ratio(1, 4);
constexpr Vec2f kGroundHalfSize{Scalar{6}, Ratio(1, 4)};
// Velocity drawn as the distance covered in this time, in seconds
constexpr Scalar kVelocityDrawTime = Ratio(1, 10);
}  // namespace

void BounceSample::Begin(const SampleContext& context) {
  bounds_ = context.world_bounds;
  const Scalar center_x = bounds_.GetCenter().x();

  ground_index_ = world_.AddBody();
  {
    Body& ground = world_.body(ground_index_);
    ground.type = BodyType::kStatic;
    ground.inverse_mass = Scalar{0};
    ground.position = {
        center_x, bounds_.min_bound().y() + Ratio(1, 4) + kGroundHalfSize.y()};
    ground_top_ = ground.position.y() + kGroundHalfSize.y();
    const ColliderIndex collider_index = world_.AddAabbCollider(ground_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = kGroundHalfSize;
  }

  ball_index_ = world_.AddBody();
  {
    Body& ball = world_.body(ball_index_);
    ball.inverse_mass = Scalar{1};
    // The drop height is measured from the ball's bottom, not its center
    ball.position = {center_x, ground_top_ + kBallRadius +
                                   parameters_[kDropHeight].ToScalar()};
    const ColliderIndex collider_index = world_.AddCircleCollider(ball_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = parameters_[kRestitution].ToScalar();
    world_.circle(collider.shape_index).radius = kBallRadius;
  }

  readouts_[0].value = ToDecimal(predicted_apex(), readouts_[0].scale);
  readouts_[0].is_valid = true;
}

void BounceSample::FixedUpdate() {
  world_.Step();
  if (measured_apex_.has_value()) {
    return;
  }
  // Up after the first contact: the bounce. Up then not up: the apex.
  const Scalar vertical_velocity = world_.body(ball_index_).velocity.y();
  if (!has_bounced_) {
    has_bounced_ = vertical_velocity > Scalar{0};
  } else if (vertical_velocity <= Scalar{0}) {
    measured_apex_ = BallHeight();
    readouts_[1].value = ToDecimal(*measured_apex_, readouts_[1].scale);
    readouts_[1].is_valid = true;
  }
}

void BounceSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(ground_index_);
  const Body& ball = world_.body(ball_index_);
  draw_list.boxes.push_back(
      {ground.position, kGroundHalfSize, sample_color::kDarkGrey});
  draw_list.circles.push_back(
      {ball.position, kBallRadius, sample_color::kOrange});

  // Heights above the ground's top, drawn across the ground
  const Scalar left = ground.position.x() - kGroundHalfSize.x();
  const Scalar right = ground.position.x() + kGroundHalfSize.x();
  const auto draw_height = [&](Scalar height, ColorId color) {
    const Scalar y = ground_top_ + height;
    draw_list.lines.push_back({{left, y}, {right, y}, color});
  };
  draw_height(parameters_[kDropHeight].ToScalar(), sample_color::kGrey);
  draw_height(predicted_apex(), sample_color::kGreen);
  if (measured_apex_.has_value()) {
    draw_height(*measured_apex_, sample_color::kYellow);
  }
  draw_list.lines.push_back({ball.position,
                             ball.position + ball.velocity * kVelocityDrawTime,
                             sample_color::kSky});
}

void BounceSample::End() {
  world_.Clear();
  has_bounced_ = false;
  measured_apex_.reset();
  readouts_[1].is_valid = false;
}

Scalar BounceSample::predicted_apex() const {
  const Scalar restitution = parameters_[kRestitution].ToScalar();
  return restitution * restitution * parameters_[kDropHeight].ToScalar();
}

Scalar BounceSample::BallHeight() const {
  return world_.body(ball_index_).position.y() - kBallRadius - ground_top_;
}

}  // namespace neko::tutorial
