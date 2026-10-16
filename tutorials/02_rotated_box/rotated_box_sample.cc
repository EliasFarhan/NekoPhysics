#include "tutorials/02_rotated_box/rotated_box_sample.h"

#include "neko/math/angle.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kBallRadius = Ratio(1, 4);
constexpr Vec2f kRampHalfSize{Scalar{5}, Ratio(1, 4)};
// Directions drawn this long, in meters
constexpr Scalar kArrowLength = Scalar{1};
}  // namespace

void RotatedBoxSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();

  ramp_index_ = world_.AddBody();
  {
    Body& ramp = world_.body(ramp_index_);
    ramp.type = BodyType::kStatic;
    ramp.inverse_mass = Scalar{0};
    ramp.position = center - Vec2f{Scalar{0}, Scalar{1}};
    ramp.rotation.set_angle(
        Radian{Degree{static_cast<Scalar>(parameters_[kRampAngle].value)}});
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(ramp_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.oriented_box(collider.shape_index).half_size = kRampHalfSize;
  }

  ball_index_ = world_.AddBody();
  {
    Body& ball = world_.body(ball_index_);
    ball.inverse_mass = Scalar{1};
    ball.position = world_.body(ramp_index_).position +
                    Vec2f{Scalar{0}, parameters_[kDropHeight].ToScalar()};
    const ColliderIndex collider_index = world_.AddCircleCollider(ball_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = parameters_[kRestitution].ToScalar();
    world_.circle(collider.shape_index).radius = kBallRadius;
  }

  const Vec2f normal = predicted_normal();
  readouts_[0].value = ToDecimal(normal.x(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(normal.y(), readouts_[1].scale);
  readouts_[0].is_valid = true;
  readouts_[1].is_valid = true;
}

void RotatedBoxSample::FixedUpdate() {
  const Vec2f velocity_before = world_.body(ball_index_).velocity;
  world_.Step();
  if (impact_velocity_.has_value() || world_.touching_pairs().empty()) {
    return;
  }
  // First contact: the step added one step of gravity, then the contact
  // impulse. What's left after removing gravity is the impulse, along the
  // contact normal.
  const Body& ball = world_.body(ball_index_);
  impact_velocity_ = velocity_before + world_.gravity() * world_.dt();
  bounce_velocity_ = ball.velocity;
  bounce_position_ = ball.position;
  const Vec2f change = ball.velocity - *impact_velocity_;
  if (change != Vec2f::Zero()) {
    measured_normal_ = change * (Scalar{1} / change.Length());
    readouts_[2].value = ToDecimal(measured_normal_->x(), readouts_[2].scale);
    readouts_[3].value = ToDecimal(measured_normal_->y(), readouts_[3].scale);
    readouts_[2].is_valid = true;
    readouts_[3].is_valid = true;
  }
  if (const auto ratio = rebound_ratio(); ratio.has_value()) {
    readouts_[4].value = ToDecimal(*ratio, readouts_[4].scale);
    readouts_[4].is_valid = true;
  }
}

void RotatedBoxSample::Draw(DrawList& draw_list) const {
  const Body& ramp = world_.body(ramp_index_);
  const Body& ball = world_.body(ball_index_);
  DrawBoxOutline(ramp.position, kRampHalfSize, ramp.rotation,
                 sample_color::kGrey, draw_list);
  // The face normal, from the middle of the top face
  const Vec2f normal = predicted_normal();
  const Vec2f face_center = ramp.position + normal * kRampHalfSize.y();
  draw_list.lines.push_back(
      {face_center, face_center + normal * kArrowLength, sample_color::kWhite});
  draw_list.circles.push_back(
      {ball.position, kBallRadius, sample_color::kOrange});
  DrawVelocity(ball, sample_color::kSky, draw_list);

  const auto predicted = predicted_bounce();
  if (predicted.has_value() && *predicted != Vec2f::Zero() &&
      *bounce_velocity_ != Vec2f::Zero()) {
    const auto draw_direction = [&](Vec2f velocity, ColorId color) {
      draw_list.lines.push_back(
          {bounce_position_,
           bounce_position_ + velocity * (kArrowLength / velocity.Length()),
           color});
    };
    draw_direction(*predicted, sample_color::kGreen);
    draw_direction(*bounce_velocity_, sample_color::kYellow);
  }
}

void RotatedBoxSample::End() {
  world_.Clear();
  impact_velocity_.reset();
  bounce_velocity_.reset();
  measured_normal_.reset();
  for (std::size_t i = 2; i < readouts_.size(); ++i) {
    readouts_[i].is_valid = false;
  }
}

Vec2f RotatedBoxSample::predicted_normal() const {
  return world_.body(ramp_index_).rotation.y_axis();
}

std::optional<Vec2f> RotatedBoxSample::predicted_bounce() const {
  if (!impact_velocity_.has_value()) {
    return std::nullopt;
  }
  const Vec2f normal = predicted_normal();
  const Vec2f impact = *impact_velocity_;
  return impact -
         normal * ((Scalar{1} + restitution()) * Vec2f::Dot(impact, normal));
}

std::optional<Scalar> RotatedBoxSample::rebound_ratio() const {
  if (!impact_velocity_.has_value()) {
    return std::nullopt;
  }
  const Vec2f normal = predicted_normal();
  const Scalar approach = Vec2f::Dot(*impact_velocity_, normal);
  if (approach == Scalar{0}) {
    return std::nullopt;
  }
  return -Vec2f::Dot(*bounce_velocity_, normal) / approach;
}

}  // namespace neko::tutorial
