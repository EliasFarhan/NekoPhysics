#include "tutorials/04_off_center_impact/solutions/ball_hits_box_sample.h"

#include "neko/math/func.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Vec2f kBoxHalfSize{Scalar{1}, Ratio(1, 2)};
constexpr Scalar kBallRadius = Ratio(1, 4);
// Gap between the ball and the box's top at the start, in meters
constexpr Scalar kStartGap{1};
// Impulse drawn as an arrow of this many meters per N s
constexpr Scalar kImpulseDrawScale = Ratio(1, 2);
}  // namespace

void BallHitsBoxSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();

  box_index_ = world_.AddBody();
  {
    Body& box = world_.body(box_index_);
    box.inverse_mass = Scalar{1};
    box.position = center - Vec2f{Scalar{0}, Scalar{1}};
    const ColliderIndex collider_index =
        world_.AddOrientedBoxCollider(box_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    // A contact uses the larger restitution: 0 lets the ball's decide
    collider.restitution = Scalar{0};
    collider.friction = Scalar{0};
    world_.oriented_box(collider.shape_index).half_size = kBoxHalfSize;
  }

  ball_index_ = world_.AddBody();
  {
    const Vec2f box_position = world_.body(box_index_).position;
    Body& ball = world_.body(ball_index_);
    ball.inverse_mass = Scalar{1} / parameters_[kBallMass].ToScalar();
    ball.position = {
        box_position.x() + parameters_[kOffset].ToScalar(),
        box_position.y() + kBoxHalfSize.y() + kBallRadius + kStartGap};
    ball.velocity = {Scalar{0}, -parameters_[kSpeed].ToScalar()};
    const ColliderIndex collider_index = world_.AddCircleCollider(ball_index_);
    Collider& collider = world_.collider(collider_index);
    collider.is_trigger = false;
    collider.restitution = parameters_[kRestitution].ToScalar();
    collider.friction = Scalar{0};
    world_.circle(collider.shape_index).radius = kBallRadius;
  }
  readouts_[0].value = ToDecimal(predicted_impulse(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(predicted_ball_velocity(), readouts_[1].scale);
  readouts_[3].value = ToDecimal(predicted_box_velocity(), readouts_[3].scale);
  readouts_[5].value = ToDecimal(predicted_box_spin(), readouts_[5].scale);
  for (const std::size_t i : {0u, 1u, 3u, 5u}) {
    readouts_[i].is_valid = true;
  }
}

void BallHitsBoxSample::FixedUpdate() {
  const Vec2f box_before = world_.body(box_index_).position;
  world_.Step();
  if (box_spin_.has_value() || world_.touching_pairs().empty()) {
    return;
  }
  const Body& box = world_.body(box_index_);
  const Body& ball = world_.body(ball_index_);
  ball_velocity_ = ball.velocity.y();
  box_velocity_ = box.velocity.y();
  box_drift_ = box.velocity.x();
  box_spin_ = box.angular_velocity;
  impact_center_ = box_before;
  impact_point_ =
      box_before + Vec2f{parameters_[kOffset].ToScalar(), kBoxHalfSize.y()};

  readouts_[2].value = ToDecimal(*ball_velocity_, readouts_[2].scale);
  readouts_[4].value = ToDecimal(*box_velocity_, readouts_[4].scale);
  readouts_[6].value = ToDecimal(*box_spin_, readouts_[6].scale);
  readouts_[7].value = ToDecimal(*point_rebound(), readouts_[7].scale);
  for (const std::size_t i : {2u, 4u, 6u, 7u}) {
    readouts_[i].is_valid = true;
  }
}

Scalar BallHitsBoxSample::box_inertia() const {
  const Vec2f size = kBoxHalfSize * Scalar{2};
  return (size.x() * size.x() + size.y() * size.y()) / Scalar{12};  // 1 kg
}

Scalar BallHitsBoxSample::predicted_impulse() const {
  const Scalar offset = parameters_[kOffset].ToScalar();
  const Scalar k = Scalar{1} + Scalar{1} / parameters_[kBallMass].ToScalar() +
                   offset * offset / box_inertia();
  return (Scalar{1} + parameters_[kRestitution].ToScalar()) *
         parameters_[kSpeed].ToScalar() / k;
}

Scalar BallHitsBoxSample::predicted_ball_velocity() const {
  return -parameters_[kSpeed].ToScalar() +
         predicted_impulse() / parameters_[kBallMass].ToScalar();
}

Scalar BallHitsBoxSample::predicted_box_velocity() const {
  return -predicted_impulse();  // M = 1 kg
}

Scalar BallHitsBoxSample::predicted_box_spin() const {
  return -parameters_[kOffset].ToScalar() * predicted_impulse() / box_inertia();
}

std::optional<Scalar> BallHitsBoxSample::point_rebound() const {
  if (!box_spin_.has_value()) {
    return std::nullopt;
  }
  // The box's material point under the ball moves at v + w x r, r = (x, 0.5):
  // its vertical part is v.y + w x
  const Scalar box_point =
      *box_velocity_ + *box_spin_ * parameters_[kOffset].ToScalar();
  return (*ball_velocity_ - box_point) / parameters_[kSpeed].ToScalar();
}

void BallHitsBoxSample::Draw(DrawList& draw_list) const {
  const Body& box = world_.body(box_index_);
  const Body& ball = world_.body(ball_index_);
  DrawBoxOutline(box.position, kBoxHalfSize, box.rotation,
                 sample_color::kOrange, draw_list);
  DrawOrientation(box, kBoxHalfSize.x(), sample_color::kWhite, draw_list);
  DrawVelocity(box, sample_color::kSky, draw_list);
  draw_list.circles.push_back(
      {ball.position, kBallRadius, sample_color::kPink});
  if (box_spin_.has_value()) {
    draw_list.lines.push_back(
        {impact_center_, impact_point_, sample_color::kGrey});
    draw_list.lines.push_back(
        {impact_point_,
         impact_point_ +
             Vec2f::Down() * (predicted_impulse() * kImpulseDrawScale),
         sample_color::kRed});
  }
}

void BallHitsBoxSample::End() {
  world_.Clear();
  ball_velocity_.reset();
  box_velocity_.reset();
  box_spin_.reset();
  box_drift_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
