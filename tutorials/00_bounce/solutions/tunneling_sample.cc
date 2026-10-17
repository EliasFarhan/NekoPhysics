#include "tutorials/00_bounce/solutions/tunneling_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kRadius = Ratio(1, 4);
constexpr Scalar kGroundHalfWidth{6};
// The ball's bottom starts 2 m above the ground's top
constexpr Scalar kStartGap{2};
}  // namespace

void TunnelingSample::Begin(const SampleContext& context) {
  const Aabbf& bounds = context.world_bounds;
  const Scalar center_x = bounds.GetCenter().x();
  const Scalar half_thickness = thickness() / Scalar{2};
  ground_ = world_.AddBody();
  {
    Body& ground = world_.body(ground_);
    ground.type = BodyType::kStatic;
    ground.inverse_mass = Scalar{0};
    ground.position = {center_x,
                       bounds.min_bound().y() + Scalar{2} + half_thickness};
    ground_top_ = ground.position.y() + half_thickness;
    Collider& collider = world_.collider(world_.AddAabbCollider(ground_));
    collider.is_trigger = false;
    collider.restitution = Scalar{0};
    world_.aabb(collider.shape_index).half_size = {kGroundHalfWidth,
                                                   half_thickness};
  }
  ball_ = world_.AddBody();
  {
    Body& ball = world_.body(ball_);
    ball.inverse_mass = Scalar{1};
    ball.gravity_scale = Scalar{0};
    ball.position = {center_x, ground_top_ + kRadius + kStartGap};
    ball.velocity = {Scalar{0}, -parameters_[kSpeed].ToScalar()};
    Collider& collider = world_.collider(world_.AddCircleCollider(ball_));
    collider.is_trigger = false;
    collider.restitution = Ratio(1, 2);
    world_.circle(collider.shape_index).radius = kRadius;
  }
  step_ = 0;
  first_depth_.reset();
  passed_.reset();
  readouts_[0].value = ToDecimal(safe_speed(), readouts_[0].scale);
  readouts_[1].value = ToDecimal(step_length(), readouts_[1].scale);
  readouts_[0].is_valid = true;
  readouts_[1].is_valid = true;
}

Scalar TunnelingSample::safe_speed() const {
  return (kRadius + thickness() / Scalar{2}) / world_.dt();
}

Scalar TunnelingSample::step_length() const {
  return parameters_[kSpeed].ToScalar() * world_.dt();
}

void TunnelingSample::FixedUpdate() {
  if (step_ >= kMeasuredSteps) {
    return;
  }
  // The world tests the overlap at the positions of the start of the step
  const Scalar center_before = world_.body(ball_).position.y();
  world_.Step();
  ++step_;
  if (!first_depth_.has_value() && !world_.touching_pairs().empty()) {
    first_depth_ = ground_top_ + kRadius - center_before;
    readouts_[2].value = ToDecimal(*first_depth_, readouts_[2].scale);
    readouts_[2].is_valid = true;
  }
  if (step_ == kMeasuredSteps) {
    const Scalar middle = ground_top_ - thickness() / Scalar{2};
    passed_ = world_.body(ball_).position.y() < middle;
    readouts_[3].value = *passed_ ? 1 : 0;
    readouts_[3].is_valid = true;
  }
}

void TunnelingSample::Draw(DrawList& draw_list) const {
  const Body& ground = world_.body(ground_);
  const Body& ball = world_.body(ball_);
  const Scalar half_thickness = thickness() / Scalar{2};
  draw_list.boxes.push_back({ground.position,
                             {kGroundHalfWidth, half_thickness},
                             sample_color::kDarkGrey});
  const Scalar left = ground.position.x() - kGroundHalfWidth;
  const Scalar right = ground.position.x() + kGroundHalfWidth;
  const auto draw_level = [&](Scalar y, ColorId color) {
    draw_list.lines.push_back({{left, y}, {right, y}, color});
  };
  // The centers that overlap the ground lie between these two lines
  draw_level(ground_top_ + kRadius, sample_color::kGrey);
  draw_level(ground_top_ - thickness() - kRadius, sample_color::kGrey);
  draw_level(ground.position.y(), sample_color::kYellow);
  draw_list.circles.push_back({ball.position, kRadius, sample_color::kOrange});
  DrawVelocity(ball, sample_color::kSky, draw_list);
}

void TunnelingSample::End() {
  world_.Clear();
  step_ = 0;
  first_depth_.reset();
  passed_.reset();
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
