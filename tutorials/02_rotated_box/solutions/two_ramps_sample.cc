#include "tutorials/02_rotated_box/solutions/two_ramps_sample.h"

#include <algorithm>

#include "neko/math/angle.h"
#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kBallRadius = Ratio(1, 5);
constexpr Vec2f kRampHalfSize{Scalar{2}, Ratio(1, 5)};
// Ramp centers from the middle of the scene: A to the right, B to the left
// and higher, so that every first bounce lands on B's face (angles 15 to 30
// degrees, restitution 0.5 to 1)
constexpr Vec2f kRampAOffset{Ratio(6, 5), Scalar{0}};
constexpr Vec2f kRampBOffset{-Ratio(6, 5), Ratio(1, 2)};
constexpr Scalar kDropHeight = Scalar{2};
// Directions drawn this long, in meters
constexpr Scalar kArrowLength = Scalar{1};
}  // namespace

void TwoRampsSample::Begin(const SampleContext& context) {
  const Vec2f middle =
      context.world_bounds.GetCenter() - Vec2f{Scalar{0}, Ratio(3, 2)};
  const auto angle = static_cast<Scalar>(parameters_[kRampAngle].value);
  const auto add_ramp = [this](Vec2f position, Scalar degrees,
                               BodyIndex& body_index,
                               ColliderIndex& collider_index) {
    body_index = world_.AddBody();
    Body& ramp = world_.body(body_index);
    ramp.type = BodyType::kStatic;
    ramp.inverse_mass = Scalar{0};
    ramp.position = position;
    ramp.rotation.set_angle(Radian{Degree{degrees}});
    collider_index = world_.AddOrientedBoxCollider(body_index);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    collider.friction = Scalar{0};
    world_.oriented_box(collider.shape_index).half_size = kRampHalfSize;
  };
  add_ramp(middle + kRampAOffset, angle, ramp_a_index_, ramp_a_collider_);
  add_ramp(middle + kRampBOffset, -angle, ramp_b_index_, ramp_b_collider_);

  ball_index_ = world_.AddBody();
  Body& ball = world_.body(ball_index_);
  ball.inverse_mass = Scalar{1};
  ball.position =
      world_.body(ramp_a_index_).position + Vec2f{Scalar{0}, kDropHeight};
  const ColliderIndex collider_index = world_.AddCircleCollider(ball_index_);
  Collider& collider = world_.collider(collider_index);
  collider.is_trigger = false;
  collider.restitution = restitution();
  world_.circle(collider.shape_index).radius = kBallRadius;
}

bool TwoRampsSample::Touches(ColliderIndex collider) const {
  return std::ranges::any_of(world_.touching_pairs(),
                             [collider](const TouchingPair& touching) {
                               return touching.pair.Contains(collider);
                             });
}

void TwoRampsSample::PredictFlight(Vec2f start, Vec2f velocity) {
  // n . (start + velocity t + g t^2 / 2 - c) - (h + R) = 0, as
  // quadratic t^2 + linear t + constant = 0
  const Body& ramp_b = world_.body(ramp_b_index_);
  const Vec2f normal = normal_b();
  const Scalar quadratic = Vec2f::Dot(normal, world_.gravity()) / Scalar{2};
  const Scalar linear = Vec2f::Dot(normal, velocity);
  const Scalar constant = Vec2f::Dot(normal, start - ramp_b.position) -
                          kRampHalfSize.y() - kBallRadius;
  const Scalar discriminant =
      linear * linear - Scalar{4} * quadratic * constant;
  if (discriminant < Scalar{0}) {
    return;
  }
  // quadratic < 0 (gravity pulls toward the face): this root is where the
  // height above the face goes down through 0
  const Scalar time = (-linear - Sqrt(discriminant)) / (Scalar{2} * quadratic);
  predicted_flight_ = time;
  predicted_contact_ = start + velocity * time +
                       world_.gravity() * (time * time / Scalar{2}) -
                       normal * kBallRadius;
  readouts_[4].value = ToDecimal(time, readouts_[4].scale);
  readouts_[4].is_valid = true;
}

void TwoRampsSample::FixedUpdate() {
  const Vec2f velocity_before = world_.body(ball_index_).velocity;
  world_.Step();
  const Body& ball = world_.body(ball_index_);
  const Vec2f impact = velocity_before + world_.gravity() * world_.dt();

  if (!measured_bounce_.has_value()) {
    if (!Touches(ramp_a_collider_)) {
      return;
    }
    // First bounce, on A
    const Vec2f normal = normal_a();
    predicted_bounce_ = impact - normal * ((Scalar{1} + restitution()) *
                                           Vec2f::Dot(impact, normal));
    measured_bounce_ = ball.velocity;
    readouts_[0].value = ToDecimal(predicted_bounce_->x(), readouts_[0].scale);
    readouts_[1].value = ToDecimal(predicted_bounce_->y(), readouts_[1].scale);
    readouts_[2].value = ToDecimal(measured_bounce_->x(), readouts_[2].scale);
    readouts_[3].value = ToDecimal(measured_bounce_->y(), readouts_[3].scale);
    for (std::size_t i = 0; i < 4; ++i) {
      readouts_[i].is_valid = true;
    }
    PredictFlight(ball.position, ball.velocity);
    return;
  }
  if (second_normal_.has_value()) {
    return;
  }
  if (!Touches(ramp_b_collider_)) {
    ++steps_since_bounce_;
    return;
  }
  // Second contact, on B: it was found at the start of this step, after
  // steps_since_bounce_ moves since the first bounce
  measured_flight_ = static_cast<Scalar>(steps_since_bounce_) * world_.dt();
  readouts_[5].value = ToDecimal(*measured_flight_, readouts_[5].scale);
  readouts_[5].is_valid = true;
  const Vec2f change = ball.velocity - impact;
  if (change != Vec2f::Zero()) {
    second_normal_ = change * (Scalar{1} / change.Length());
    readouts_[6].value = ToDecimal(second_normal_->x(), readouts_[6].scale);
    readouts_[7].value = ToDecimal(second_normal_->y(), readouts_[7].scale);
    readouts_[6].is_valid = true;
    readouts_[7].is_valid = true;
  }
  const Scalar approach = Vec2f::Dot(impact, normal_b());
  if (approach != Scalar{0}) {
    second_ratio_ = -Vec2f::Dot(ball.velocity, normal_b()) / approach;
    readouts_[8].value = ToDecimal(*second_ratio_, readouts_[8].scale);
    readouts_[8].is_valid = true;
  }
}

void TwoRampsSample::Draw(DrawList& draw_list) const {
  for (const BodyIndex index : {ramp_a_index_, ramp_b_index_}) {
    const Body& ramp = world_.body(index);
    DrawBoxOutline(ramp.position, kRampHalfSize, ramp.rotation,
                   sample_color::kGrey, draw_list);
    const Vec2f normal = ramp.rotation.y_axis();
    const Vec2f face_center = ramp.position + normal * kRampHalfSize.y();
    draw_list.lines.push_back({face_center, face_center + normal * kArrowLength,
                               sample_color::kWhite});
  }
  if (predicted_contact_.has_value()) {
    draw_list.circles.push_back(
        {*predicted_contact_, Ratio(1, 20), sample_color::kGreen});
  }
  const Body& ball = world_.body(ball_index_);
  draw_list.circles.push_back(
      {ball.position, kBallRadius, sample_color::kOrange});
  DrawVelocity(ball, sample_color::kSky, draw_list);
}

void TwoRampsSample::End() {
  world_.Clear();
  steps_since_bounce_ = 0;
  predicted_bounce_.reset();
  measured_bounce_.reset();
  predicted_flight_.reset();
  measured_flight_.reset();
  predicted_contact_.reset();
  second_normal_.reset();
  second_ratio_.reset();
  for (SampleReadout& readout : readouts_) {
    readout.is_valid = false;
  }
}

Vec2f TwoRampsSample::normal_a() const {
  return world_.body(ramp_a_index_).rotation.y_axis();
}

Vec2f TwoRampsSample::normal_b() const {
  return world_.body(ramp_b_index_).rotation.y_axis();
}

}  // namespace neko::tutorial
