#include "tutorials/02_rotated_box/solutions/corner_sample.h"

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kBallRadius = Ratio(1, 4);
constexpr Vec2f kRampHalfSize{Scalar{2}, Ratio(1, 4)};
// Directions drawn this long, in meters
constexpr Scalar kArrowLength = Scalar{1};
}  // namespace

void CornerSample::Begin(const SampleContext& context) {
  ramp_index_ = world_.AddBody();
  {
    Body& ramp = world_.body(ramp_index_);
    ramp.type = BodyType::kStatic;
    ramp.inverse_mass = Scalar{0};
    ramp.position =
        context.world_bounds.GetCenter() - Vec2f{Scalar{1}, Ratio(3, 2)};
    ramp.rotation.set_angle(
        Radian{Degree{static_cast<Scalar>(parameters_[kRampAngle].value)}});
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(ramp_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    // Frictionless, as in chapter 02: the impulse stays along the normal
    collider.friction = Scalar{0};
    world_.oriented_box(collider.shape_index).half_size = kRampHalfSize;
  }

  ball_index_ = world_.AddBody();
  {
    Body& ball = world_.body(ball_index_);
    ball.inverse_mass = Scalar{1};
    ball.position = corner() + Vec2f{parameters_[kOffset].ToScalar(),
                                     parameters_[kDropHeight].ToScalar()};
    const ColliderIndex collider_index = world_.AddCircleCollider(ball_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = parameters_[kRestitution].ToScalar();
    world_.circle(collider.shape_index).radius = kBallRadius;
  }

  const Vec2f ideal = ideal_normal();
  readouts_[4].value = ToDecimal(ideal.x(), readouts_[4].scale);
  readouts_[5].value = ToDecimal(ideal.y(), readouts_[5].scale);
  readouts_[4].is_valid = true;
  readouts_[5].is_valid = true;
}

void CornerSample::FixedUpdate() {
  // The world tests contacts at the positions of the start of the step
  const Vec2f position_before = world_.body(ball_index_).position;
  const Vec2f velocity_before = world_.body(ball_index_).velocity;
  world_.Step();
  if (has_bounced_ || world_.touching_pairs().empty()) {
    return;
  }
  has_bounced_ = true;

  // The box-circle test by hand: the center in the ramp's frame, clamped to
  // the box, gives the closest point
  const Body& ramp = world_.body(ramp_index_);
  const Vec2f local =
      ramp.rotation.InverseRotate(position_before - ramp.position);
  const Vec2f clamped{Clamp(local.x(), -kRampHalfSize.x(), kRampHalfSize.x()),
                      Clamp(local.y(), -kRampHalfSize.y(), kRampHalfSize.y())};
  hit_corner_ = Abs(clamped.x()) == kRampHalfSize.x() &&
                Abs(clamped.y()) == kRampHalfSize.y();
  const Vec2f outward = ramp.rotation.Rotate(local - clamped);
  if (outward != Vec2f::Zero()) {
    const Scalar distance = outward.Length();
    predicted_normal_ = outward * (Scalar{1} / distance);
    sink_ = kBallRadius - distance;
    readouts_[0].value = ToDecimal(predicted_normal_->x(), readouts_[0].scale);
    readouts_[1].value = ToDecimal(predicted_normal_->y(), readouts_[1].scale);
    readouts_[6].value = ToDecimal(*sink_, readouts_[6].scale);
    readouts_[0].is_valid = true;
    readouts_[1].is_valid = true;
    readouts_[6].is_valid = true;
  }

  // Measured as in chapter 02: the velocity change minus one step of gravity
  const Vec2f impact = velocity_before + world_.gravity() * world_.dt();
  const Vec2f change = world_.body(ball_index_).velocity - impact;
  if (change != Vec2f::Zero()) {
    measured_normal_ = change * (Scalar{1} / change.Length());
    readouts_[2].value = ToDecimal(measured_normal_->x(), readouts_[2].scale);
    readouts_[3].value = ToDecimal(measured_normal_->y(), readouts_[3].scale);
    readouts_[2].is_valid = true;
    readouts_[3].is_valid = true;
  }
}

void CornerSample::Draw(DrawList& draw_list) const {
  const Body& ramp = world_.body(ramp_index_);
  const Body& ball = world_.body(ball_index_);
  DrawBoxOutline(ramp.position, kRampHalfSize, ramp.rotation,
                 sample_color::kGrey, draw_list);
  const Vec2f normal = face_normal();
  const Vec2f face_center = ramp.position + normal * kRampHalfSize.y();
  draw_list.lines.push_back(
      {face_center, face_center + normal * kArrowLength, sample_color::kWhite});
  draw_list.circles.push_back({corner(), Ratio(1, 20), sample_color::kRed});
  draw_list.circles.push_back(
      {ball.position, kBallRadius, sample_color::kOrange});
  DrawVelocity(ball, sample_color::kSky, draw_list);
  if (predicted_normal_.has_value() && measured_normal_.has_value()) {
    draw_list.lines.push_back({corner(),
                               corner() + *predicted_normal_ * kArrowLength,
                               sample_color::kGreen});
    draw_list.lines.push_back({corner(),
                               corner() + *measured_normal_ * kArrowLength,
                               sample_color::kYellow});
  }
}

void CornerSample::End() {
  world_.Clear();
  has_bounced_ = false;
  hit_corner_ = false;
  predicted_normal_.reset();
  measured_normal_.reset();
  sink_.reset();
  for (std::size_t i = 0; i < readouts_.size(); ++i) {
    if (i != 4 && i != 5) {
      readouts_[i].is_valid = false;
    }
  }
}

Vec2f CornerSample::face_normal() const {
  return world_.body(ramp_index_).rotation.y_axis();
}

Vec2f CornerSample::ideal_normal() const {
  const Scalar offset = parameters_[kOffset].ToScalar();
  return Vec2f{offset, Sqrt(kBallRadius * kBallRadius - offset * offset)} *
         (Scalar{1} / kBallRadius);
}

Vec2f CornerSample::corner() const {
  const Body& ramp = world_.body(ramp_index_);
  return ramp.position + ramp.rotation.Rotate(kRampHalfSize);
}

}  // namespace neko::tutorial
