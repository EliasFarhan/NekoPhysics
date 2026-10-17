#include "tutorials/00_bounce/solutions/head_on_sample.h"

#include "main/samples/body_draw.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kRadius = Ratio(1, 4);
// Ball A starts 3 m left of the center, B 1 m right of it
constexpr Scalar kStartA{-3};
constexpr Scalar kStartB{1};
// PhysicsWorldConfig::restitution_threshold, left at its default
constexpr Scalar kThreshold{1};
// One second of steps after the first contact
constexpr int kMeasureAfter = 60;
}  // namespace

void HeadOnSample::Begin(const SampleContext& context) {
  const Vec2f center = context.world_bounds.GetCenter();
  const auto add_ball = [this](Vec2f position, Scalar inverse_mass) {
    const BodyIndex index = world_.AddBody();
    Body& body = world_.body(index);
    body.position = position;
    body.inverse_mass = inverse_mass;
    Collider& collider = world_.collider(world_.AddCircleCollider(index));
    collider.is_trigger = false;
    collider.restitution = restitution();
    world_.circle(collider.shape_index).radius = kRadius;
    return index;
  };
  ball_a_ = add_ball(center + Vec2f{kStartA, Scalar{0}}, Scalar{1});
  ball_b_ = add_ball(center + Vec2f{kStartB, Scalar{0}}, Scalar{1} / mass_b());
  world_.body(ball_a_).velocity = {speed(), Scalar{0}};
  step_ = 0;
  contact_step_.reset();
  is_measured_ = false;
  readouts_[0].value = ToDecimal(predicted_velocity_a(), readouts_[0].scale);
  readouts_[2].value = ToDecimal(predicted_velocity_b(), readouts_[2].scale);
  readouts_[4].value = ToDecimal(initial_momentum(), readouts_[4].scale);
  readouts_[0].is_valid = true;
  readouts_[2].is_valid = true;
  readouts_[4].is_valid = true;
}

Scalar HeadOnSample::predicted_velocity_a() const {
  // Too slow to bounce: no restitution
  const Scalar e = speed() > kThreshold ? restitution() : Scalar{0};
  return (speed() - e * mass_b() * speed()) / (Scalar{1} + mass_b());
}

Scalar HeadOnSample::predicted_velocity_b() const {
  const Scalar e = speed() > kThreshold ? restitution() : Scalar{0};
  return (Scalar{1} + e) * speed() / (Scalar{1} + mass_b());
}

Scalar HeadOnSample::velocity_a() const {
  return world_.body(ball_a_).velocity.x();
}

Scalar HeadOnSample::velocity_b() const {
  return world_.body(ball_b_).velocity.x();
}

Scalar HeadOnSample::momentum() const {
  return velocity_a() + mass_b() * velocity_b();
}

Scalar HeadOnSample::initial_momentum() const { return speed(); }

void HeadOnSample::FixedUpdate() {
  world_.Step();
  ++step_;
  if (!contact_step_.has_value() && !world_.touching_pairs().empty()) {
    contact_step_ = step_;
  }
  if (!is_measured_ && contact_step_.has_value() &&
      step_ == *contact_step_ + kMeasureAfter) {
    is_measured_ = true;
    readouts_[1].value = ToDecimal(velocity_a(), readouts_[1].scale);
    readouts_[3].value = ToDecimal(velocity_b(), readouts_[3].scale);
    readouts_[5].value = ToDecimal(momentum(), readouts_[5].scale);
    readouts_[1].is_valid = true;
    readouts_[3].is_valid = true;
    readouts_[5].is_valid = true;
  }
}

void HeadOnSample::Draw(DrawList& draw_list) const {
  const Body& a = world_.body(ball_a_);
  const Body& b = world_.body(ball_b_);
  draw_list.circles.push_back({a.position, kRadius, sample_color::kOrange});
  draw_list.circles.push_back({b.position, kRadius, sample_color::kTeal});
  DrawVelocity(a, sample_color::kSky, draw_list);
  DrawVelocity(b, sample_color::kSky, draw_list);
}

void HeadOnSample::End() {
  world_.Clear();
  step_ = 0;
  contact_step_.reset();
  is_measured_ = false;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
