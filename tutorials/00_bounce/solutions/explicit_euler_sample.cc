#include "tutorials/00_bounce/solutions/explicit_euler_sample.h"

namespace neko::tutorial {
namespace {
constexpr Scalar kRadius = Ratio(1, 4);
// The balls 2 m apart, the drops drawn over 6 m of the view
constexpr Scalar kSpacing{2};
constexpr Scalar kDrawnDrop{6};
}  // namespace

void ExplicitEulerSample::Begin(const SampleContext& context) {
  const Aabbf& bounds = context.world_bounds;
  top_ = {bounds.GetCenter().x(), bounds.max_bound().y() - Scalar{1}};
  ball_ = world_.AddBody();
  Body& ball = world_.body(ball_);
  ball.inertia_mode = InertiaMode::kFixedRotation;
  step_ = 0;
  explicit_y_ = Scalar{0};
  explicit_v_ = Scalar{0};
}

int ExplicitEulerSample::fall_steps() const {
  return parameters_[kFallTime].value * 60 / parameters_[kFallTime].scale;
}

Scalar ExplicitEulerSample::engine_drop() const {
  return -world_.body(ball_).position.y();
}

Scalar ExplicitEulerSample::exact_drop() const {
  const Scalar time = static_cast<Scalar>(step_) * world_.dt();
  return -world_.gravity().y() * time * time / Scalar{2};
}

Scalar ExplicitEulerSample::engine_velocity() const {
  return world_.body(ball_).velocity.y();
}

Scalar ExplicitEulerSample::shadow_energy_change() const {
  // v^2 - g dt v + 2 g y with g = 9.81 m/s^2 is, with the gravity vector's
  // y (-9.81), v^2 + g_y dt v - 2 g_y y; it starts at 0, at rest at y = 0
  const Scalar v = engine_velocity();
  const Scalar g_y = world_.gravity().y();
  return v * v + g_y * world_.dt() * v + Scalar{2} * g_y * engine_drop();
}

void ExplicitEulerSample::FixedUpdate() {
  if (step_ >= fall_steps()) {
    return;
  }
  world_.Step();
  // Explicit Euler: the position moves with the velocity of the start of the
  // step, then the velocity changes
  const Scalar dt = world_.dt();
  explicit_y_ += explicit_v_ * dt;
  explicit_v_ += world_.gravity().y() * dt;
  ++step_;
  if (step_ == fall_steps()) {
    readouts_[0].value = ToDecimal(engine_drop(), readouts_[0].scale);
    readouts_[1].value = ToDecimal(explicit_drop(), readouts_[1].scale);
    readouts_[2].value = ToDecimal(exact_drop(), readouts_[2].scale);
    readouts_[3].value = ToDecimal(shadow_energy_change(), readouts_[3].scale);
    for (auto& readout : readouts_) {
      readout.is_valid = true;
    }
  }
}

void ExplicitEulerSample::Draw(DrawList& draw_list) const {
  // The drops scaled so that the exact one at the end is 6 m
  const Scalar time = static_cast<Scalar>(fall_steps()) * world_.dt();
  const Scalar full_drop = -world_.gravity().y() * time * time / Scalar{2};
  const Scalar scale = kDrawnDrop / full_drop;
  const auto draw_ball = [&](Scalar x, Scalar drop, ColorId color) {
    draw_list.circles.push_back({{x, top_.y() - drop * scale}, kRadius, color});
  };
  draw_list.lines.push_back({{top_.x() - kSpacing * Scalar{2}, top_.y()},
                             {top_.x() + kSpacing * Scalar{2}, top_.y()},
                             sample_color::kGrey});
  draw_list.lines.push_back(
      {{top_.x() - kSpacing * Scalar{2}, top_.y() - kDrawnDrop},
       {top_.x() + kSpacing * Scalar{2}, top_.y() - kDrawnDrop},
       sample_color::kGreen});
  draw_ball(top_.x() - kSpacing, engine_drop(), sample_color::kOrange);
  draw_ball(top_.x(), exact_drop(), sample_color::kGreen);
  draw_ball(top_.x() + kSpacing, explicit_drop(), sample_color::kRed);
}

void ExplicitEulerSample::End() {
  world_.Clear();
  step_ = 0;
  for (auto& readout : readouts_) {
    readout.is_valid = false;
  }
}

}  // namespace neko::tutorial
